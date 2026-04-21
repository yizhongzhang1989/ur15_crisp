"""
UR Native Force Mode streamer.

Implements "joystick" style direct wrench control of the UR robot using the
robot's native force_mode() primitive. The host streams wrench vectors at
~100 Hz to a small URScript running on the UR controller; the URScript calls
force_mode() each control cycle (500 Hz) with the last-received wrench.

Key safety features:
- Hard safety limits: wrench is clamped server-side before streaming.
- URScript watchdog: if the host stops sending for >200 ms, the URScript
  zeros the wrench automatically.
- Atexit handler: on Python shutdown, the module sends a final zero-wrench
  packet, closes the socket, and calls resend_robot_program to restore
  External Control.
- Hard URScript duration cap: 10 minutes; after that, the URScript exits
  automatically. The host can restart it with another enable call.
- On disable, the streamer sends multiple zero-wrench packets, closes the
  socket cleanly (so URScript exits), waits briefly, and calls
  resend_robot_program to restore the ros2_control External Control program.

Architecture:
  POST /api/ur_force/enable   -> open TCP server, send URScript, wait for
                                 robot to connect; return OK
  POST /api/ur_force/wrench   -> cache the latest wrench vector (6 floats);
                                 the streamer thread pushes it to the robot
  POST /api/ur_force/disable  -> zero wrench, close socket, resend program
  GET  /api/ur_force/status   -> enabled, connected, last_wrench, uptime, ...
"""

from __future__ import annotations

import atexit
import math
import socket
import threading
import time
from typing import Callable, List, Optional

from std_msgs.msg import String
from std_srvs.srv import Trigger


# ----- Safety limits -----
# Maximum wrench components that will ever be sent to the robot, regardless of
# what the UI requests. Server-side clamps.
MAX_FORCE_N: float = 30.0          # N per translation axis
MAX_TORQUE_NM: float = 3.0         # N*m per rotation axis
# Maximum TCP speeds on compliant axes while force_mode is active.
SPEED_LIMIT_M_S: float = 0.1       # m/s on each translational axis
SPEED_LIMIT_RAD_S: float = 0.5     # rad/s on each rotational axis
# UR force_mode damping factor (0=no friction; 1=strong). 0.5 = moderate.
DAMPING_FACTOR: float = 0.5
# UR force_mode gain scaling (0..2; >1 is aggressive). 0.5 = conservative.
GAIN_SCALING: float = 0.5
# Host-side stream rate (Hz). URScript reads at up to 125 Hz (8 ms steptime).
STREAM_HZ: float = 125.0
# Watchdog: if the host doesn't send for this many ms, URScript zeros wrench.
WATCHDOG_MS: float = 200.0
# Hard cap on URScript runtime (seconds) as a safety net.
MAX_SCRIPT_DURATION_S: float = 600.0

# Default payload to set at URScript startup (kg). 0 = leave URCap setting.
# Override via set_params(payload_kg=...). Tune this to remove any vertical
# asymmetry (i.e. +Fz responsiveness != -Fz responsiveness) — that asymmetry
# means the URCap's configured payload doesn't match the real one.
DEFAULT_PAYLOAD_KG: float = 0.0
DEFAULT_PAYLOAD_COG_Z: float = 0.08   # CoG offset along TCP Z (m)
PAYLOAD_KG_CAP: float = 10.0          # safety cap

# TCP port to listen on for URScript connection. Chosen outside URCL's range.
STREAMER_PORT: int = 50011


def _clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def _make_urscript(host_ip: str, port: int,
                   payload_kg: float = 0.0,
                   payload_cog_z: float = 0.08) -> str:
    """Generate the URScript that runs on the UR controller.

    If `payload_kg > 0` the script calls `set_target_payload(...)` at
    startup so `force_mode`'s gravity compensation matches reality. Without
    this, a mis-configured payload causes a vertical asymmetry (pushing
    against the gravity-comp bias is slower than pushing with it).

    Protocol: the URScript connects back to the host on `port` and reads
    11 ASCII floats per packet:
        (fx, fy, fz, tx, ty, tz, timeout_ms, damping, gain_scaling,
         vmax_lin, vmax_rot)
    `timeout_ms` sets the host-requested watchdog (8-ms granularity).
    `damping` is the force_mode damping factor (0..1).
    `gain_scaling` is the force_mode gain scaling (0..2).
    `vmax_lin` is the per-axis linear speed limit (m/s) on compliant axes.
    `vmax_rot` is the per-axis rotational speed limit (rad/s).
    Packet format must be parenthesized and newline-terminated, e.g.
      "(1.0,2.0,3.0,0.0,0.0,0.0,200.0,0.5,0.5,0.1,0.5)\\n"
    """
    max_cycles = int(MAX_SCRIPT_DURATION_S * 125)  # 125 Hz rough cap
    # set_target_payload block — emitted only if payload_kg > 0. Otherwise
    # we leave whatever URCap/Installation settings currently hold.
    if payload_kg > 0:
        payload_block = (
            f'  set_target_payload({payload_kg}, [0.0, 0.0, {payload_cog_z}])\n'
            f'  textmsg("ur_force_mode: set_target_payload({payload_kg} kg)")\n'
        )
    else:
        payload_block = '  textmsg("ur_force_mode: using URCap payload setting")\n'
    return f"""def ur_native_force_joystick():
  textmsg("ur_force_mode: start")
{payload_block}  socket_open("{host_ip}", {port}, "wrench_socket")

  # Selection vector is ALWAYS all-ones: every axis is compliant, so a zero
  # wrench means "zero-gravity / free-float" (robot yields freely to external
  # forces while gravity is compensated via set_target_payload). Non-zero
  # wrench components push the robot along those axes. Small wrench noise is
  # rejected via a deadband below.
  local sel = [1, 1, 1, 1, 1, 1]
  local task_frame = p[0, 0, 0, 0, 0, 0]
  local wrench = [0, 0, 0, 0, 0, 0]
  local limits = [{SPEED_LIMIT_M_S}, {SPEED_LIMIT_M_S}, {SPEED_LIMIT_M_S}, {SPEED_LIMIT_RAD_S}, {SPEED_LIMIT_RAD_S}, {SPEED_LIMIT_RAD_S}]

  force_mode_set_damping({DAMPING_FACTOR})
  force_mode_set_gain_scaling({GAIN_SCALING})
  # Deadband for the wrench values themselves (not the selection vector):
  # small components are snapped to exactly 0 to reject floating-point noise
  # from the streamed packet. Selection stays all-ones so the robot remains
  # compliant (zero-gravity) even with zero wrench.
  local f_db = 0.05
  local t_db = 0.01
  # Track whether force_mode is currently engaged so we only call
  # end_force_mode() on the active->idle transition (watchdog / shutdown).
  local in_force_mode = False

  local running = True
  local timeout_count = 0
  local watchdog_limit = ceil({WATCHDOG_MS} / 8.0)
  local cycle = 0

  while running:
    local rx = socket_read_ascii_float(11, "wrench_socket", 0.008)
    if rx[0] >= 11:
      wrench[0] = rx[1]
      wrench[1] = rx[2]
      wrench[2] = rx[3]
      wrench[3] = rx[4]
      wrench[4] = rx[5]
      wrench[5] = rx[6]
      local tm = rx[7]
      if tm > 0:
        watchdog_limit = ceil(tm / 8.0)
      end
      local vmax_lin = rx[10]
      local vmax_rot = rx[11]
      if vmax_lin > 0:
        limits[0] = vmax_lin
        limits[1] = vmax_lin
        limits[2] = vmax_lin
      end
      if vmax_rot > 0:
        limits[3] = vmax_rot
        limits[4] = vmax_rot
        limits[5] = vmax_rot
      end
      if rx[8] >= 0:
        force_mode_set_damping(rx[8])
      end
      if rx[9] >= 0:
        force_mode_set_gain_scaling(rx[9])
      end

      # Deadband only zeroes tiny wrench components (noise rejection). The
      # selection vector stays all-ones so zero wrench = free-float.
      if wrench[0] < f_db and wrench[0] > -f_db:
        wrench[0] = 0
      end
      if wrench[1] < f_db and wrench[1] > -f_db:
        wrench[1] = 0
      end
      if wrench[2] < f_db and wrench[2] > -f_db:
        wrench[2] = 0
      end
      if wrench[3] < t_db and wrench[3] > -t_db:
        wrench[3] = 0
      end
      if wrench[4] < t_db and wrench[4] > -t_db:
        wrench[4] = 0
      end
      if wrench[5] < t_db and wrench[5] > -t_db:
        wrench[5] = 0
      end
      timeout_count = 0

      # Always run force_mode while streaming is live; zero wrench on all
      # axes gives compliant zero-gravity behaviour (gravity is compensated
      # by set_target_payload).
      force_mode(task_frame, sel, wrench, 2, limits)
      in_force_mode = True
    elif rx[0] == 0:
      timeout_count = timeout_count + 1
      if timeout_count > watchdog_limit:
        wrench = [0, 0, 0, 0, 0, 0]
        if in_force_mode:
          end_force_mode()
          in_force_mode = False
        end
      end
      sleep(0.002)
    else:
      running = False
    end
    cycle = cycle + 1
    if cycle > {max_cycles}:
      running = False
    end
    sleep(0.002)
  end

  if in_force_mode:
    end_force_mode()
  end
  socket_close("wrench_socket")
  textmsg("ur_force_mode: end")
end
"""


def _get_local_ip_reachable_to_robot(robot_ip: str) -> str:
    """Find the local IP the robot can route back to."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect((robot_ip, 30002))
        return s.getsockname()[0]
    finally:
        s.close()


class URForceModeStreamer:
    """Host-side manager for the UR native force-mode control session."""

    def __init__(self, node, robot_ip: str = "192.168.1.15",
                 on_before_resend: Optional[Callable[[], None]] = None,
                 on_after_resend: Optional[Callable[[], None]] = None):
        self._node = node
        self._robot_ip = robot_ip
        self._on_before_resend = on_before_resend
        self._on_after_resend = on_after_resend
        self._script_pub = node.create_publisher(String, "/urscript_interface/script_command", 1)
        self._resend_cli = node.create_client(Trigger, "/io_and_status_controller/resend_robot_program")

        self._lock = threading.Lock()
        self._enabled = False
        self._connected = False
        self._server: Optional[socket.socket] = None
        self._conn: Optional[socket.socket] = None
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._wrench: List[float] = [0.0] * 6
        self._last_send_ts: Optional[float] = None
        self._start_ts: Optional[float] = None
        self._last_error: Optional[str] = None
        self._host_ip: Optional[str] = None

        # Live-tunable params (streamed to URScript each cycle).
        self._damping: float = DAMPING_FACTOR
        self._gain_scaling: float = GAIN_SCALING
        self._vmax_lin: float = SPEED_LIMIT_M_S
        self._vmax_rot: float = SPEED_LIMIT_RAD_S
        # Host-side slider range (used by UI; does not go to URScript).
        self._max_force: float = MAX_FORCE_N
        self._max_torque: float = MAX_TORQUE_NM
        # Payload (applied at URScript startup via set_target_payload).
        # Changes require re-enable to take effect.
        self._payload_kg: float = DEFAULT_PAYLOAD_KG
        self._payload_cog_z: float = DEFAULT_PAYLOAD_COG_Z

        atexit.register(self._atexit_cleanup)

    # --------- public API ---------

    @property
    def enabled(self) -> bool:
        return self._enabled

    def status(self) -> dict:
        with self._lock:
            uptime = (time.time() - self._start_ts) if self._start_ts else 0.0
            return {
                "enabled": self._enabled,
                "connected": self._connected,
                "wrench": list(self._wrench),
                "uptime_s": round(uptime, 1),
                "host_ip": self._host_ip,
                "port": STREAMER_PORT,
                # Live-tunable params.
                "max_force_n": self._max_force,
                "max_torque_nm": self._max_torque,
                "speed_limit_m_s": self._vmax_lin,
                "speed_limit_rad_s": self._vmax_rot,
                "damping": self._damping,
                "gain_scaling": self._gain_scaling,
                # Payload (applied at next enable).
                "payload_kg": self._payload_kg,
                "payload_cog_z": self._payload_cog_z,
                # Hard safety caps (UI should not let sliders exceed these).
                "max_force_n_cap": MAX_FORCE_N,
                "max_torque_nm_cap": MAX_TORQUE_NM,
                "payload_kg_cap": PAYLOAD_KG_CAP,
                "last_error": self._last_error,
            }

    def set_params(self, damping: Optional[float] = None,
                   gain_scaling: Optional[float] = None,
                   speed_limit_m_s: Optional[float] = None,
                   speed_limit_rad_s: Optional[float] = None,
                   max_force_n: Optional[float] = None,
                   max_torque_nm: Optional[float] = None,
                   payload_kg: Optional[float] = None,
                   payload_cog_z: Optional[float] = None) -> dict:
        """Update tunable params at runtime. Streamer picks them up on next cycle.

        Ranges enforced server-side:
          damping         : 0..1
          gain_scaling    : 0..2
          speed_limit_m_s : 0.01..0.5
          speed_limit_rad_s: 0.05..2.0
          max_force_n     : 1..MAX_FORCE_N
          max_torque_nm   : 0.1..MAX_TORQUE_NM
          payload_kg      : 0..PAYLOAD_KG_CAP   (requires re-enable)
          payload_cog_z   : 0..0.3              (requires re-enable)
        """
        with self._lock:
            if damping is not None:
                self._damping = _clamp(float(damping), 0.0, 1.0)
            if gain_scaling is not None:
                self._gain_scaling = _clamp(float(gain_scaling), 0.0, 2.0)
            if speed_limit_m_s is not None:
                self._vmax_lin = _clamp(float(speed_limit_m_s), 0.01, 0.5)
            if speed_limit_rad_s is not None:
                self._vmax_rot = _clamp(float(speed_limit_rad_s), 0.05, 2.0)
            if max_force_n is not None:
                self._max_force = _clamp(float(max_force_n), 1.0, MAX_FORCE_N)
            if max_torque_nm is not None:
                self._max_torque = _clamp(float(max_torque_nm), 0.1, MAX_TORQUE_NM)
            if payload_kg is not None:
                self._payload_kg = _clamp(float(payload_kg), 0.0, PAYLOAD_KG_CAP)
            if payload_cog_z is not None:
                self._payload_cog_z = _clamp(float(payload_cog_z), 0.0, 0.3)
            return {
                "damping": self._damping,
                "gain_scaling": self._gain_scaling,
                "speed_limit_m_s": self._vmax_lin,
                "speed_limit_rad_s": self._vmax_rot,
                "max_force_n": self._max_force,
                "max_torque_nm": self._max_torque,
                "payload_kg": self._payload_kg,
                "payload_cog_z": self._payload_cog_z,
            }

    def set_wrench(self, fx: float, fy: float, fz: float,
                   tx: float, ty: float, tz: float) -> List[float]:
        """Clamp and cache the latest commanded wrench. Returns the clamped vector."""
        with self._lock:
            mf = self._max_force
            mt = self._max_torque
        clamped = [
            _clamp(float(fx), -mf, mf),
            _clamp(float(fy), -mf, mf),
            _clamp(float(fz), -mf, mf),
            _clamp(float(tx), -mt, mt),
            _clamp(float(ty), -mt, mt),
            _clamp(float(tz), -mt, mt),
        ]
        with self._lock:
            self._wrench = clamped
        return clamped

    def enable(self) -> dict:
        """Start the URScript session. Returns a dict with 'success' and info."""
        with self._lock:
            if self._enabled:
                return {"success": True, "already": True}
            self._last_error = None
            self._stop_event.clear()
            self._wrench = [0.0] * 6

        # Resolve host IP reachable from the robot.
        try:
            host_ip = _get_local_ip_reachable_to_robot(self._robot_ip)
        except Exception as e:
            err = f"Failed to find local IP: {e}"
            self._record_error(err)
            return {"success": False, "error": err}

        # Start TCP server first.
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            server.bind(("0.0.0.0", STREAMER_PORT))
            server.listen(1)
            server.settimeout(10.0)
        except OSError as e:
            server.close()
            err = f"Could not bind TCP port {STREAMER_PORT}: {e}"
            self._record_error(err)
            return {"success": False, "error": err}

        # Publish URScript (twice for DDS discovery reliability).
        with self._lock:
            payload_kg = self._payload_kg
            payload_cog_z = self._payload_cog_z
        script = _make_urscript(host_ip, STREAMER_PORT,
                                payload_kg=payload_kg,
                                payload_cog_z=payload_cog_z)
        # Save the generated script for debugging (overwrites each enable).
        try:
            with open("/tmp/ur_force_mode_last_script.urscript", "w") as f:
                f.write(script)
        except Exception:
            pass
        # Publish URScript. The publisher was created at startup so DDS
        # discovery to /urscript_interface/script_command has long since
        # completed by the time the user clicks Enable.
        msg = String()
        msg.data = script
        self._script_pub.publish(msg)

        # Wait for robot to connect back.
        try:
            conn, addr = server.accept()
            conn.settimeout(0.1)
        except socket.timeout:
            server.close()
            err = "Robot did not connect within 10 s (is the driver running?)"
            self._record_error(err)
            return {"success": False, "error": err}

        with self._lock:
            self._enabled = True
            self._connected = True
            self._server = server
            self._conn = conn
            self._host_ip = host_ip
            self._start_ts = time.time()
            self._wrench = [0.0] * 6

        # Start streamer thread.
        self._thread = threading.Thread(target=self._stream_loop, daemon=True)
        self._thread.start()
        return {"success": True, "host_ip": host_ip, "port": STREAMER_PORT,
                "robot_addr": f"{addr[0]}:{addr[1]}"}

    def disable(self) -> dict:
        """Stop the URScript session and restore External Control.

        Sequence (avoids shaking / protective stop on exit):
          1. Zero the commanded wrench so the streamer sends zeros.
          2. Keep streaming zeros for ~0.4 s so the robot decelerates under
             force_mode with the same damping it was just using.
          3. Stop the streamer thread and close the socket.
          4. Publish a tiny preempt URScript that calls `end_force_mode()`
             and `stopj()` to cleanly exit force_mode and brake the arm.
          5. Wait briefly so the preempt script actually runs.
          6. Call `resend_robot_program` to restore External Control.
        """
        with self._lock:
            if not self._enabled:
                return {"success": True, "already_disabled": True}
            # Zero the commanded wrench immediately; streamer loop picks this up.
            self._wrench = [0.0] * 6

        # Let the streamer push zero wrench while force_mode damps residual
        # motion. This is the critical step that prevents jerk on exit.
        time.sleep(0.4)

        # Signal streamer to stop and wait for it to exit.
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)

        # Close socket (force_mode URScript may or may not exit on read error,
        # so we explicitly preempt it below).
        with self._lock:
            try:
                if self._conn:
                    try:
                        self._conn.shutdown(socket.SHUT_RDWR)
                    except Exception:
                        pass
                    self._conn.close()
            finally:
                self._conn = None
            try:
                if self._server:
                    self._server.close()
            finally:
                self._server = None
            self._enabled = False
            self._connected = False

        # Preempt the running force-mode URScript with a small one that
        # properly exits force_mode and brakes the joints. Publishing to
        # /urscript_interface/script_command replaces the currently running
        # program on the UR controller.
        stop_script = (
            "def ur_force_mode_stop():\n"
            "  end_force_mode()\n"
            "  stopj(2.0)\n"
            "  textmsg(\"ur_force_mode: stopped\")\n"
            "end\n"
        )
        try:
            stop_msg = String()
            stop_msg.data = stop_script
            self._script_pub.publish(stop_msg)
        except Exception as e:
            self._record_error(f"failed to publish stop script: {e}")

        # Give the preempt script time to run end_force_mode() and stopj().
        time.sleep(0.8)

        # Critical: snap the CRISP/ros2_control targets to the CURRENT actual
        # joint positions BEFORE resending the External Control program.
        # Without this, the trajectory controller has a stale target from
        # before force_mode was enabled and will drive the robot back to it
        # as soon as External Control reconnects.
        if self._on_before_resend is not None:
            try:
                self._on_before_resend()
            except Exception as e:
                self._record_error(f"on_before_resend hook error: {e}")

        # Resend the External Control program so ros2_control reconnects.
        resend_ok = self._call_resend()

        # Post-resend hook: reactivate any controllers that were deactivated
        # by the before-resend hook. Runs even if resend failed so we leave
        # the controller state consistent.
        if self._on_after_resend is not None:
            try:
                self._on_after_resend()
            except Exception as e:
                self._record_error(f"on_after_resend hook error: {e}")

        return {"success": True, "resend_robot_program": resend_ok}

    # --------- internal ---------

    def _record_error(self, msg: str) -> None:
        with self._lock:
            self._last_error = msg

    def _call_resend(self) -> bool:
        try:
            if not self._resend_cli.wait_for_service(timeout_sec=2.0):
                self._record_error("resend_robot_program service unavailable")
                return False
            fut = self._resend_cli.call_async(Trigger.Request())
            # We cannot spin the node ourselves here (owned by the executor in
            # web_server.py). Just poll the future for a short while.
            t0 = time.time()
            while not fut.done() and time.time() - t0 < 3.0:
                time.sleep(0.05)
            if not fut.done():
                self._record_error("resend_robot_program call timed out")
                return False
            res = fut.result()
            if res is None or not res.success:
                self._record_error(f"resend_robot_program failed: {getattr(res, 'message', '?')}")
                return False
            return True
        except Exception as e:
            self._record_error(f"resend_robot_program error: {e}")
            return False

    def _stream_loop(self) -> None:
        """Thread body: stream the latest wrench to the robot at STREAM_HZ."""
        period = 1.0 / STREAM_HZ
        watchdog_ms = int(WATCHDOG_MS)
        consecutive_errors = 0
        while not self._stop_event.is_set():
            t0 = time.time()
            try:
                with self._lock:
                    w = list(self._wrench)
                    conn = self._conn
                    damping = self._damping
                    gain_scaling = self._gain_scaling
                    vmax_lin = self._vmax_lin
                    vmax_rot = self._vmax_rot
                if conn is None:
                    break
                # Final safety clamp (should already be clamped in set_wrench).
                w = [_clamp(w[i], -MAX_FORCE_N, MAX_FORCE_N) for i in range(3)] + \
                    [_clamp(w[i], -MAX_TORQUE_NM, MAX_TORQUE_NM) for i in range(3, 6)]
                pkt = (
                    "({:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},"
                    "{:.1f},{:.3f},{:.3f},{:.4f},{:.4f})\n"
                ).format(
                    w[0], w[1], w[2], w[3], w[4], w[5],
                    float(watchdog_ms),
                    damping, gain_scaling, vmax_lin, vmax_rot,
                ).encode("ascii")
                conn.sendall(pkt)
                with self._lock:
                    self._last_send_ts = time.time()
                consecutive_errors = 0
            except (BrokenPipeError, ConnectionResetError, OSError) as e:
                consecutive_errors += 1
                self._record_error(f"stream error: {e}")
                if consecutive_errors > 3:
                    with self._lock:
                        self._connected = False
                    break
            except Exception as e:
                self._record_error(f"unexpected stream error: {e}")
                break
            dt = time.time() - t0
            if dt < period:
                time.sleep(period - dt)

        # On loop exit: try to send a final zero wrench so the robot stops.
        with self._lock:
            conn = self._conn
        if conn is not None:
            try:
                with self._lock:
                    damping = self._damping
                    gain_scaling = self._gain_scaling
                    vmax_lin = self._vmax_lin
                    vmax_rot = self._vmax_rot
                pkt = (
                    "(0.000,0.000,0.000,0.000,0.000,0.000,"
                    "{:.1f},{:.3f},{:.3f},{:.4f},{:.4f})\n"
                ).format(
                    float(watchdog_ms), damping, gain_scaling, vmax_lin, vmax_rot
                ).encode("ascii")
                for _ in range(3):
                    try:
                        conn.sendall(pkt)
                    except Exception:
                        break
                    time.sleep(0.01)
            except Exception:
                pass

    def _atexit_cleanup(self) -> None:
        """Called on Python interpreter shutdown. Best-effort safe stop."""
        try:
            if self._enabled:
                self.disable()
        except Exception:
            pass
