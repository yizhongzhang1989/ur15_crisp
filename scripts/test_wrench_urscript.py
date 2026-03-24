"""Direct wrench control via URScript force_mode with socket streaming.

Reads target wrench from /target_wrench topic and streams it to the UR
via a TCP socket. The UR runs force_mode and applies the wrench directly.

Architecture:
  Python (this script)                    UR Controller
  ┌──────────────────┐                   ┌──────────────────┐
  │ TCP Server :30010 │◄── socket ──────│ URScript client   │
  │ Forwards wrench   │                  │ Applies force_mode│
  │ from /target_wrench│                 │ with received     │
  │ at 125 Hz          │                 │ wrench            │
  └──────────────────┘                   └──────────────────┘

Usage:
    # Terminal 1: Launch UR driver
    ros2 launch ur15_bringup ur15_crisp.launch.py robot_ip:=192.168.1.15

    # Terminal 2: Run this script
    python3 scripts/test_wrench_urscript.py

    # Terminal 3: Send target wrench

    # Apply 20N downward (uses default watchdog from script config)
    ros2 topic pub /target_wrench geometry_msgs/msg/WrenchStamped \
      "{wrench: {force: {x: 0, y: 0, z: -20}, torque: {x: 0, y: 0, z: 0}}}"

    # Apply 50N downward with 1000ms watchdog (hold for 1s after last msg)
    ros2 param set /wrench_urscript watchdog_ms 1000
    ros2 topic pub --once /target_wrench geometry_msgs/msg/WrenchStamped \
      "{wrench: {force: {z: -50}}}"
    # (with --once, the watchdog will zero the force after 1000ms)

    # Apply 30N and hold forever (watchdog=0), even if publisher stops
    ros2 param set /wrench_urscript watchdog_ms 0
    ros2 topic pub /target_wrench geometry_msgs/msg/WrenchStamped \
      "{wrench: {force: {z: -30}}}"

    # Restore default watchdog
    ros2 param set /wrench_urscript watchdog_ms 500

    # Stop force (zero wrench)
    ros2 topic pub /target_wrench geometry_msgs/msg/WrenchStamped \
      "{wrench: {force: {x: 0, y: 0, z: 0}, torque: {x: 0, y: 0, z: 0}}}"
"""

import socket
import threading
import time
import signal
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from geometry_msgs.msg import WrenchStamped


# === Configuration ===
ROBOT_IP = "192.168.1.15"    # UR robot IP
HOST_IP = "192.168.1.142"   # This PC's IP on the robot subnet
PORT = 30010                 # Port for wrench streaming
RATE_HZ = 10                 # Streaming rate (limited by URScript sleep(0.1))
WATCHDOG_MS = 500            # Default watchdog (ms). 0 = hold forever.
SPEED_LIMIT = 0.1            # m/s max on compliant axes
MAX_FORCE = 80.0             # N per axis safety clamp
MAX_TORQUE = 15.0            # Nm per axis safety clamp


class WrenchForwarder(Node):
    def __init__(self):
        super().__init__("wrench_urscript")

        self._target_wrench = np.zeros(6)  # fx, fy, fz, tx, ty, tz
        self._ft_force = np.zeros(3)
        self._running = True
        self._watchdog_ms = WATCHDOG_MS

        # Declare ROS parameter so it can be changed at runtime
        self.declare_parameter("watchdog_ms", WATCHDOG_MS)

        # ROS
        self._script_pub = self.create_publisher(String, "/urscript_interface/script_command", 10)
        self.create_subscription(WrenchStamped, "target_wrench", self._wrench_cb, 10)
        self.create_subscription(WrenchStamped, "force_torque_sensor_broadcaster/wrench",
                                 self._ft_cb, qos_profile_sensor_data)

        self._spin_thread = threading.Thread(target=self._spin, daemon=True)
        self._spin_thread.start()

        signal.signal(signal.SIGINT, self._signal_handler)

    def _spin(self):
        while self._running:
            rclpy.spin_once(self, timeout_sec=0.01)

    def _wrench_cb(self, msg):
        self._target_wrench = np.array([
            msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z,
            msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z,
        ])
        # Safety clamp
        self._target_wrench[:3] = np.clip(self._target_wrench[:3], -MAX_FORCE, MAX_FORCE)
        self._target_wrench[3:] = np.clip(self._target_wrench[3:], -MAX_TORQUE, MAX_TORQUE)

    def _ft_cb(self, msg):
        self._ft_force = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z])

    def _signal_handler(self, sig, frame):
        print("\n\n  Ctrl+C — stopping...")
        self._running = False

    def generate_urscript(self):
        return f"""def wrench_receiver():
  socket_open("{HOST_IP}", {PORT}, "wrench_socket")

  sel = [1, 1, 1, 1, 1, 1]
  task_frame = p[0, 0, 0, 0, 0, 0]
  wrench = [0, 0, 0, 0, 0, 0]
  limits = [{SPEED_LIMIT}, {SPEED_LIMIT}, {SPEED_LIMIT}, 1.0, 1.0, 1.0]

  force_mode(task_frame, sel, wrench, 2, limits)

  running = True
  timeout_count = 0
  watchdog_limit = 0
  while running:
    rx = socket_read_ascii_float(7, "wrench_socket", 0.008)

    if rx[0] >= 7:
      wrench[0] = rx[1]
      wrench[1] = rx[2]
      wrench[2] = rx[3]
      wrench[3] = rx[4]
      wrench[4] = rx[5]
      wrench[5] = rx[6]
      timeout_ms = rx[7]
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
end
"""

    def send_urscript_to_robot(self, script: str):
        """Send URScript to the UR's secondary interface (port 30002)."""
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(5.0)
            s.connect((ROBOT_IP, 30002))
            s.sendall((script + "\n").encode("utf-8"))
            time.sleep(0.1)
            s.close()
            return True
        except Exception as e:
            print(f"  Failed to send URScript to {ROBOT_IP}:30002 — {e}")
            return False

    def run(self):
        print(f"\n{'='*65}")
        print(f"  Direct Wrench Control via URScript")
        print(f"{'='*65}")
        print(f"  Rate:           {RATE_HZ} Hz")
        print(f"  Max force:      {MAX_FORCE:.0f} N")
        print(f"  Speed limit:    {SPEED_LIMIT:.2f} m/s")
        print(f"{'='*65}")
        print(f"\n  Send wrench via: ros2 topic pub /target_wrench ...")
        print(f"  Press Ctrl+C to stop.\n")

        input("  Press Enter to start...")

        # Main loop — send force_mode commands via urscript_interface
        dt = 1.0 / RATE_HZ
        print(f"\n  Streaming force_mode at {RATE_HZ} Hz via /urscript_interface.\n")

        try:
            while self._running:
                self._watchdog_ms = self.get_parameter("watchdog_ms").value
                w = self._target_wrench

                # Send force_mode wrapped in def...end (required by urscript_interface)
                script = (
                    f"def wrench_cmd():\n"
                    f"  force_mode(p[0,0,0,0,0,0], [1,1,1,1,1,1], "
                    f"[{w[0]:.2f},{w[1]:.2f},{w[2]:.2f},{w[3]:.2f},{w[4]:.2f},{w[5]:.2f}], "
                    f"2, [{SPEED_LIMIT},{SPEED_LIMIT},{SPEED_LIMIT},1.0,1.0,1.0])\n"
                    f"  sleep(0.1)\n"
                    f"end\n"
                )
                msg = String()
                msg.data = script
                self._script_pub.publish(msg)

                ft_mag = np.linalg.norm(self._ft_force)
                print(
                    f"\r  Cmd:({w[0]:6.1f},{w[1]:6.1f},{w[2]:6.1f})N  |  "
                    f"FT:({self._ft_force[0]:5.1f},{self._ft_force[1]:5.1f},{self._ft_force[2]:5.1f})N  |  "
                    f"|FT|:{ft_mag:5.1f}N",
                    end="", flush=True,
                )

                time.sleep(dt)

        finally:
            # Stop force mode
            print("\n\n  Sending end_force_mode()...")
            stop = String()
            stop.data = "def stop_force():\n  end_force_mode()\nend\n"
            for _ in range(5):
                self._script_pub.publish(stop)
                time.sleep(0.05)
            print("  Done.\n")


def main():
    rclpy.init()
    node = WrenchForwarder()
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
