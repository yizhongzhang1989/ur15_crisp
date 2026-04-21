#!/usr/bin/env python3
"""
Verify joystick-style wrench streaming:
- Persistent URScript: opens socket, reads wrench each cycle, applies force_mode
- Host: sends wrench sequence [+Fz 1s] -> [0 1s] -> [-Fz 1s] -> [0 1s] -> stop
- Measure: motion during nonzero wrench, deceleration when wrench->0

This verifies the final architecture for the "Native Force Mode" UI panel.
"""
import socket
import threading
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped


HOST_IP = "192.168.1.101"   # will be replaced with robot's view of host
PORT = 50013
SOCKET_NAME = "wrench_rx"

def make_script(host, port):
    return f"""def joystick_wrench():
  textmsg("UR_TEST: joystick start")
  socket_open("{host}", {port}, "wrench_socket")

  local sel = [1, 1, 1, 1, 1, 1]
  local task_frame = p[0, 0, 0, 0, 0, 0]
  local wrench = [0, 0, 0, 0, 0, 0]
  local limits = [0.1, 0.1, 0.1, 0.3, 0.3, 0.3]

  force_mode_set_damping(0.5)
  force_mode_set_gain_scaling(0.5)
  force_mode(task_frame, sel, wrench, 2, limits)

  local running = True
  local timeout_count = 0
  local watchdog_limit = 0
  while running:
    local rx = socket_read_ascii_float(7, "wrench_socket", 0.008)
    if rx[0] >= 7:
      wrench[0] = rx[1]
      wrench[1] = rx[2]
      wrench[2] = rx[3]
      wrench[3] = rx[4]
      wrench[4] = rx[5]
      wrench[5] = rx[6]
      local timeout_ms = rx[7]
      if timeout_ms > 0:
        watchdog_limit = ceil(timeout_ms / 8.0)
      else:
        watchdog_limit = 0
      end
      timeout_count = 0
      force_mode(task_frame, sel, wrench, 2, limits)
    elif rx[0] == 0:
      timeout_count = timeout_count + 1
      if watchdog_limit > 0 and timeout_count > watchdog_limit:
        wrench = [0, 0, 0, 0, 0, 0]
        force_mode(task_frame, sel, wrench, 2, limits)
      end
      sleep(0.002)
    else:
      running = False
    end
    sleep(0.002)
  end

  end_force_mode()
  socket_close("wrench_socket")
  textmsg("UR_TEST: joystick end")
end
"""


class T(Node):
    def __init__(self):
        super().__init__("tj")
        self.pub = self.create_publisher(String, "/urscript_interface/script_command", 1)
        self.resend = self.create_client(Trigger, "/io_and_status_controller/resend_robot_program")
        self.tcp = None
        self.create_subscription(PoseStamped, "/tcp_pose_broadcaster/pose", self._cb, 10)

    def _cb(self, m):
        p = m.pose.position
        self.tcp = (p.x, p.y, p.z, time.time())


def get_local_ip_reachable_to_robot(robot_ip="192.168.1.15"):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect((robot_ip, 30002))
        return s.getsockname()[0]
    finally:
        s.close()


def main():
    # Patch HOST_IP to the actual interface the robot can reach
    global HOST_IP
    HOST_IP = get_local_ip_reachable_to_robot()
    print(f"Using host IP (reachable from robot): {HOST_IP}")

    script = make_script(HOST_IP, PORT)

    # Start TCP server first, then send URScript
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("0.0.0.0", PORT))
    server.listen(1)
    server.settimeout(10.0)

    rclpy.init()
    n = T()
    # Wait for TCP pose
    t0 = time.time()
    while n.tcp is None and time.time() - t0 < 3:
        rclpy.spin_once(n, timeout_sec=0.1)
    p0 = n.tcp
    print(f"TCP start: x={p0[0]:.4f} y={p0[1]:.4f} z={p0[2]:.4f}")

    n.pub.publish(String(data=script))
    time.sleep(0.5)
    n.pub.publish(String(data=script))  # republish for DDS discovery
    print("URScript sent; waiting for robot to connect...")
    server.settimeout(15.0)
    try:
        conn, addr = server.accept()
    except socket.timeout:
        print("FAIL: robot never connected")
        return 1
    print(f"Robot connected from {addr}")

    # Recorder thread
    samples = []  # (t, z, phase)
    stop_rec = threading.Event()

    def recorder():
        while not stop_rec.is_set():
            rclpy.spin_once(n, timeout_sec=0.05)
            if n.tcp is not None:
                samples.append((time.time(), n.tcp[2]))

    rec_thread = threading.Thread(target=recorder, daemon=True)
    rec_thread.start()

    def send(f):
        """f: tuple(fx,fy,fz,tx,ty,tz); adds watchdog_ms=200 at end -> 7 floats."""
        msg = "({:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.1f})\n".format(*f, 200.0).encode()
        try:
            conn.sendall(msg)
        except (BrokenPipeError, ConnectionResetError) as e:
            print(f"send failed: {e}")
            raise

    def phase(label, wrench, duration, rate=50):
        print(f"  {label}: wrench={wrench} for {duration}s")
        t_end = time.time() + duration
        while time.time() < t_end:
            send(wrench)
            time.sleep(1.0 / rate)

    phase("push +Z", (0, 0, 15, 0, 0, 0), 1.0)
    z_peak_up = n.tcp[2]
    phase("zero", (0, 0, 0, 0, 0, 0), 1.0)
    z_after_zero1 = n.tcp[2]
    phase("push -Z", (0, 0, -15, 0, 0, 0), 1.0)
    z_peak_down = n.tcp[2]
    phase("zero", (0, 0, 0, 0, 0, 0), 1.0)
    z_after_zero2 = n.tcp[2]

    # close socket cleanly so URScript exits
    try:
        conn.shutdown(socket.SHUT_RDWR)
    except Exception:
        pass
    conn.close()
    server.close()
    stop_rec.set()
    rec_thread.join(timeout=1.0)

    # Wait for URScript to exit and recover
    time.sleep(1.5)
    fut = n.resend.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(n, fut, timeout_sec=5)
    print(f"resend: {fut.result().success if fut.result() else 'fail'}")
    time.sleep(1.0)

    # Analysis
    print("\n--- ANALYSIS ---")
    print(f"z_start            = {p0[2]*1000:.2f} mm")
    print(f"z after +Fz 1s     = {z_peak_up*1000:.2f} mm  (dz = {(z_peak_up-p0[2])*1000:+.2f} mm)")
    print(f"z after zero 1s    = {z_after_zero1*1000:.2f} mm  (drift = {(z_after_zero1-z_peak_up)*1000:+.2f} mm)")
    print(f"z after -Fz 1s     = {z_peak_down*1000:.2f} mm  (dz from zero = {(z_peak_down-z_after_zero1)*1000:+.2f} mm)")
    print(f"z after zero 1s    = {z_after_zero2*1000:.2f} mm  (drift = {(z_after_zero2-z_peak_down)*1000:+.2f} mm)")
    print(f"z final vs start   = {(z_after_zero2-p0[2])*1000:+.2f} mm")
    print(f"samples captured   = {len(samples)}")

    # Compute velocity during zero-phase (should be near 0)
    if len(samples) > 50:
        # Find samples in the zero phase after +Z (roughly seconds 1-2 of the run)
        t_first = samples[0][0]
        zero_phase_samples = [(t-t_first, z) for t, z in samples if 1.2 < (t-t_first) < 1.9]
        if len(zero_phase_samples) >= 2:
            dt = zero_phase_samples[-1][0] - zero_phase_samples[0][0]
            dz = zero_phase_samples[-1][1] - zero_phase_samples[0][1]
            v = dz / dt if dt > 0 else 0
            print(f"velocity during zero-wrench phase: {v*1000:.3f} mm/s over {dt:.2f}s")

    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
