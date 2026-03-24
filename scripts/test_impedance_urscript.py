"""URScript impedance controller with real-time wrench streaming via socket.

Architecture:
  Python (this script)                    UR Controller
  ┌──────────────────┐                   ┌──────────────────┐
  │ TCP Server :30010 │◄── socket ──────│ URScript client   │
  │ Sends wrench at   │                  │ Reads wrench      │
  │ 125 Hz             │                  │ Applies force_mode│
  │ Listens to ROS     │                  │ at 500 Hz loop    │
  │ /target_pose       │                  │                    │
  └──────────────────┘                   └──────────────────┘

The URScript runs a loop:
  1. Connect to Python TCP server
  2. Read 6 floats (fx, fy, fz, tx, ty, tz) each cycle
  3. Apply force_mode with the received wrench
  4. On disconnect or zero wrench -> end_force_mode

Python side:
  1. Starts a TCP server
  2. Sends URScript program to UR via /urscript_interface
  3. Accepts UR connection
  4. Computes impedance wrench: F = K * (target_pose - current_pose)
  5. Streams wrench to UR at 125 Hz
  6. Reads target from /target_pose topic (or keyboard)

Usage:
    # Terminal 1: Launch UR driver
    ros2 launch ur15_bringup ur15_crisp.launch.py robot_ip:=192.168.1.15

    # Terminal 2: Run this script
    python3 scripts/test_impedance_urscript.py

    # Terminal 3 (optional): Send target poses
    ros2 topic pub /target_pose geometry_msgs/msg/PoseStamped \
      "{header: {frame_id: 'base_link'}, pose: {position: {x: 0.0, y: 0.92, z: 0.60}, orientation: {x: -0.17, y: -0.26, z: -0.75, w: 0.58}}}"
"""

import socket
import struct
import threading
import time
import signal
import sys
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, WrenchStamped


# === Configuration ===
HOST_IP = "192.168.1.142"   # This PC's IP on the robot subnet
PORT = 30010                 # Port for wrench streaming (must not conflict with UR's 30001-30004)
RATE_HZ = 125               # Wrench update rate
WATCHDOG_MS = 500            # ms — default watchdog. 0 = hold forever. Sent with each wrench command.

# Impedance gains (Cartesian spring)
K_POS = np.array([300.0, 300.0, 300.0])   # N/m — translational stiffness
K_ROT = np.array([10.0, 10.0, 10.0])      # Nm/rad — rotational stiffness
D_POS = np.array([50.0, 50.0, 50.0])      # Ns/m — translational damping (not used in force_mode, for future)

# Force limits
MAX_FORCE = 60.0   # N per axis
MAX_TORQUE = 10.0   # Nm per axis
SPEED_LIMIT = 0.1   # m/s on compliant axes


class ImpedanceController(Node):
    def __init__(self):
        super().__init__("impedance_urscript")

        self._target_pos = None  # Will be set from /target_pose or current pose
        self._target_quat = None
        self._current_pos = np.zeros(3)
        self._current_quat = np.array([0, 0, 0, 1.0])
        self._ft_force = np.zeros(3)
        self._running = True
        self._connected = False

        # ROS publishers/subscribers
        self._script_pub = self.create_publisher(String, "/urscript_interface/script_command", 10)
        self.create_subscription(PoseStamped, "current_pose", self._pose_cb, 10)
        self.create_subscription(PoseStamped, "target_pose", self._target_cb, 10)
        self.create_subscription(WrenchStamped, "force_torque_sensor_broadcaster/wrench",
                                 self._ft_cb, qos_profile_sensor_data)

        # Spin ROS in background
        self._spin_thread = threading.Thread(target=self._spin, daemon=True)
        self._spin_thread.start()

        signal.signal(signal.SIGINT, self._signal_handler)

    def _spin(self):
        while self._running:
            rclpy.spin_once(self, timeout_sec=0.01)

    def _pose_cb(self, msg):
        self._current_pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        self._current_quat = np.array([msg.pose.orientation.x, msg.pose.orientation.y,
                                        msg.pose.orientation.z, msg.pose.orientation.w])
        # Initialize target to current on first data
        if self._target_pos is None:
            self._target_pos = self._current_pos.copy()
            self._target_quat = self._current_quat.copy()

    def _target_cb(self, msg):
        self._target_pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        self._target_quat = np.array([msg.pose.orientation.x, msg.pose.orientation.y,
                                       msg.pose.orientation.z, msg.pose.orientation.w])

    def _ft_cb(self, msg):
        self._ft_force = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z])

    def _signal_handler(self, sig, frame):
        print("\n\n  Ctrl+C — stopping...")
        self._running = False

    def compute_wrench(self):
        """Compute impedance wrench: F = K * (target - current)"""
        if self._target_pos is None:
            return np.zeros(6)

        # Position error
        pos_error = self._target_pos - self._current_pos
        force = K_POS * pos_error

        # Clamp forces
        force = np.clip(force, -MAX_FORCE, MAX_FORCE)

        # Rotation error (simplified — just zero torque for now)
        torque = np.zeros(3)

        return np.concatenate([force, torque])

    def generate_urscript(self):
        """Generate the URScript program that runs on the UR controller."""
        return f"""def impedance_receiver():
  # Connect to Python server to receive wrench commands
  socket_open("{HOST_IP}", {PORT}, "wrench_socket")

  # All 6 axes are force-controlled
  local sel = [1, 1, 1, 1, 1, 1]
  local task_frame = p[0, 0, 0, 0, 0, 0]
  local wrench = [0, 0, 0, 0, 0, 0]
  local limits = [{SPEED_LIMIT}, {SPEED_LIMIT}, {SPEED_LIMIT}, 1.0, 1.0, 1.0]

  # Start force mode
  force_mode(task_frame, sel, wrench, 2, limits)

  local running = True
  local timeout_count = 0
  local watchdog_limit = 0
  while running:
    # Read 7 floats: fx, fy, fz, tx, ty, tz, timeout_ms
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

      # Re-apply force_mode with updated wrench
      force_mode(task_frame, sel, wrench, 2, limits)
    elif rx[0] == 0:
      # Timeout — no data received
      timeout_count = timeout_count + 1
      if watchdog_limit > 0 and timeout_count > watchdog_limit:
        # Watchdog triggered — zero the wrench for safety
        wrench = [0, 0, 0, 0, 0, 0]
        force_mode(task_frame, sel, wrench, 2, limits)
      end
      sleep(0.002)
    else:
      # Disconnect or error
      running = False
    end

    sleep(0.002)
  end

  end_force_mode()
  socket_close("wrench_socket")
end
"""

    def run(self):
        print(f"\n{'='*65}")
        print(f"  URScript Impedance Controller")
        print(f"{'='*65}")
        print(f"  Host:           {HOST_IP}:{PORT}")
        print(f"  Stiffness:      K_pos={K_POS[0]:.0f} N/m, K_rot={K_ROT[0]:.1f} Nm/rad")
        print(f"  Max force:      {MAX_FORCE:.0f} N")
        print(f"  Speed limit:    {SPEED_LIMIT:.2f} m/s")
        print(f"  Rate:           {RATE_HZ} Hz")
        print(f"  Watchdog:       {'off (hold forever)' if WATCHDOG_MS == 0 else f'{WATCHDOG_MS} ms'}")
        print(f"{'='*65}")
        print(f"\n  Send targets via: ros2 topic pub /target_pose ...")
        print(f"  Press Ctrl+C to stop.\n")

        # Wait for current pose
        print("  Waiting for /current_pose...")
        while self._running and self._target_pos is None:
            time.sleep(0.1)
        if not self._running:
            return
        print(f"  Initial pose: ({self._current_pos[0]:.3f}, {self._current_pos[1]:.3f}, {self._current_pos[2]:.3f})")

        input("\n  Press Enter to start impedance control...")

        # Start TCP server
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.settimeout(30.0)
        server.bind((HOST_IP, PORT))
        server.listen(1)
        print(f"\n  TCP server listening on {HOST_IP}:{PORT}")

        # Send URScript to the robot
        print("  Sending URScript to UR...")
        script = self.generate_urscript()
        msg = String()
        msg.data = script
        self._script_pub.publish(msg)
        time.sleep(1.0)
        self._script_pub.publish(msg)  # publish twice for DDS discovery

        # Wait for UR to connect
        print("  Waiting for UR to connect...")
        try:
            conn, addr = server.accept()
            conn.settimeout(0.1)
            self._connected = True
            print(f"  UR connected from {addr}")
        except socket.timeout:
            print("  ERROR: UR did not connect within 30s. Is the driver running?")
            server.close()
            return

        # Main control loop
        dt = 1.0 / RATE_HZ
        print(f"\n  Running impedance control at {RATE_HZ} Hz...")
        print(f"  Target: current pose. Send /target_pose to move.\n")

        try:
            while self._running:
                wrench = self.compute_wrench()

                # Send as ASCII floats (URScript socket_read_ascii_float format)
                # Format: "(fx, fy, fz, tx, ty, tz, timeout_ms)\n"
                data = f"({wrench[0]:.2f},{wrench[1]:.2f},{wrench[2]:.2f},{wrench[3]:.2f},{wrench[4]:.2f},{wrench[5]:.2f},{WATCHDOG_MS:.0f})\n"
                try:
                    conn.sendall(data.encode())
                except (BrokenPipeError, ConnectionResetError):
                    print("\n  UR disconnected.")
                    break

                # Print status
                pos_err = np.linalg.norm(self._target_pos - self._current_pos) * 1000 if self._target_pos is not None else 0
                ft_mag = np.linalg.norm(self._ft_force)
                print(
                    f"\r  F:({wrench[0]:6.1f},{wrench[1]:6.1f},{wrench[2]:6.1f})N  |  "
                    f"FT:({self._ft_force[0]:5.1f},{self._ft_force[1]:5.1f},{self._ft_force[2]:5.1f})N  |  "
                    f"err:{pos_err:5.1f}mm  |  "
                    f"pos:({self._current_pos[0]:.3f},{self._current_pos[1]:.3f},{self._current_pos[2]:.3f})",
                    end="", flush=True,
                )

                time.sleep(dt)

        finally:
            # Send stop signal (zero wrench)
            try:
                conn.sendall(b"(0,0,0,0,0,0,0)\n")
                time.sleep(0.1)
            except Exception:
                pass
            conn.close()
            server.close()

            # Send end_force_mode via URScript
            print("\n\n  Sending end_force_mode()...")
            stop_msg = String()
            stop_msg.data = "end_force_mode()"
            self._script_pub.publish(stop_msg)
            time.sleep(0.5)
            self._script_pub.publish(stop_msg)
            print("  Done.")
            print(f"\n  To restore ros2_control:")
            print(f"  ros2 service call /dashboard_client/play std_srvs/srv/Trigger\n")


def main():
    rclpy.init()
    node = ImpedanceController()
    try:
        node.run()
    except Exception as e:
        print(f"\n  Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        node._running = False
        time.sleep(0.3)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
