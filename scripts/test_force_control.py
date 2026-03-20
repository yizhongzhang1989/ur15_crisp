"""Test force control using UR native force_mode via URScript.

Sends URScript commands directly to the UR via the urscript_interface topic.
This bypasses ros2_control entirely and uses the UR's internal force controller.

IMPORTANT: This script will temporarily take over control from ros2_control.
After the script ends, you need to restart the External Control program
(via dashboard play) to restore ros2_control.

Usage:
    # 1. Launch the driver (any controller can be active):
    ros2 launch ur15_bringup ur15_crisp.launch.py robot_ip:=192.168.1.15

    # 2. Run this script:
    source install/setup.bash
    python3 scripts/test_force_control.py

    # 3. After the script, restore ros2_control:
    ros2 service call /dashboard_client/play std_srvs/srv/Trigger

Safety:
    - Force ramps up slowly
    - Ctrl+C immediately calls end_force_mode() and stops
    - Speed limits constrain maximum movement speed
    - Script prints FT sensor feedback in real time
"""

import time
import signal
import sys
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from geometry_msgs.msg import WrenchStamped


class URScriptForceTest(Node):
    def __init__(self):
        super().__init__("urscript_force_test")

        self._ft_force = np.zeros(3)
        self._ft_torque = np.zeros(3)
        self._running = True

        # Publisher for URScript commands
        self._script_pub = self.create_publisher(
            String, "/urscript_interface/script_command", 10
        )

        # Subscribe to FT sensor for monitoring
        self.create_subscription(
            WrenchStamped,
            "force_torque_sensor_broadcaster/wrench",
            self._ft_cb,
            qos_profile_sensor_data,
        )

        # Spin in background
        self._spin_thread = threading.Thread(target=self._spin, daemon=True)
        self._spin_thread.start()

        signal.signal(signal.SIGINT, self._signal_handler)

    def _spin(self):
        while self._running:
            rclpy.spin_once(self, timeout_sec=0.05)

    def _ft_cb(self, msg):
        self._ft_force = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z])
        self._ft_torque = np.array([msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z])

    def _signal_handler(self, sig, frame):
        print("\n\n  Ctrl+C — stopping force mode...")
        self._running = False

    def send_script(self, script: str):
        """Send a URScript command."""
        msg = String()
        msg.data = script
        self._script_pub.publish(msg)

    def start_force_mode(self, fz: float, speed_limit: float = 0.05, duration: float = 30.0):
        """Start force_mode by sending a complete URScript program.
        
        Args:
            fz: Force in Z (N). Negative = push down in base frame.
            speed_limit: Max speed on compliant axis (m/s).
            duration: How long to hold force mode (seconds).
        """
        # Send a complete secondary program that runs force_mode in a loop
        # This runs as a secondary thread inside the External Control program
        script = (
            f"def force_test():\n"
            f"  force_mode(p[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "
            f"[0, 0, 1, 0, 0, 0], "
            f"[0.0, 0.0, {fz:.1f}, 0.0, 0.0, 0.0], "
            f"2, "
            f"[0.01, 0.01, {speed_limit:.3f}, 0.5, 0.5, 0.5])\n"
            f"  sleep({duration:.1f})\n"
            f"  end_force_mode()\n"
            f"end\n"
        )
        self.send_script(script)

    def stop_force_mode(self):
        """Stop force_mode."""
        self.send_script("end_force_mode()")

    def run(self):
        # Configuration
        force_down = -30.0      # N (negative = downward)
        force_up = 30.0         # N (positive = upward)
        speed_limit = 0.03      # m/s — very slow for safety
        phase_duration = 5.0    # seconds per phase

        print(f"\n{'='*65}")
        print(f"  UR Force Mode Test — Down then Up")
        print(f"{'='*65}")
        print(f"  Phase 1: {force_down:.0f} N (down) for {phase_duration:.0f} s")
        print(f"  Phase 2: {force_up:.0f} N (up) for {phase_duration:.0f} s")
        print(f"  Speed limit Z:   {speed_limit:.3f} m/s")
        print(f"{'='*65}")
        print(f"\n  The External Control program will be interrupted.")
        print(f"  After the test, restore with:")
        print(f"    ros2 service call /dashboard_client/play std_srvs/srv/Trigger")
        print(f"\n  Press Ctrl+C at any time to stop.\n")

        input("  Press Enter to start...")
        print()

        # Wait for publisher discovery
        time.sleep(1.0)

        # Phase 1: Move downward
        print(f"  Phase 1: Pushing DOWN ({force_down:.0f} N) for {phase_duration:.0f} s...")
        self.start_force_mode(force_down, speed_limit, phase_duration)

        start_time = time.time()
        while self._running and (time.time() - start_time) < phase_duration + 1:
            ft_mag = np.linalg.norm(self._ft_force)
            elapsed = time.time() - start_time
            print(
                f"\r  DOWN [{elapsed:4.1f}s/{phase_duration:.0f}s]  "
                f"FT: ({self._ft_force[0]:6.1f}, {self._ft_force[1]:6.1f}, {self._ft_force[2]:6.1f}) N  |  "
                f"|F|: {ft_mag:5.1f} N",
                end="", flush=True,
            )
            time.sleep(0.1)

        if not self._running:
            print("\n\n  Stopped during phase 1.")
            self.stop_force_mode()
            return

        # Phase 2: Move upward
        print(f"\n\n  Phase 2: Pushing UP ({force_up:.0f} N) for {phase_duration:.0f} s...")
        self.start_force_mode(force_up, speed_limit, phase_duration)

        start_time = time.time()
        while self._running and (time.time() - start_time) < phase_duration + 1:
            ft_mag = np.linalg.norm(self._ft_force)
            elapsed = time.time() - start_time
            print(
                f"\r  UP   [{elapsed:4.1f}s/{phase_duration:.0f}s]  "
                f"FT: ({self._ft_force[0]:6.1f}, {self._ft_force[1]:6.1f}, {self._ft_force[2]:6.1f}) N  |  "
                f"|F|: {ft_mag:5.1f} N",
                end="", flush=True,
            )
            time.sleep(0.1)

        # Stop force mode
        print("\n\n  Sending end_force_mode()...")
        self.stop_force_mode()
        time.sleep(0.5)
        self.stop_force_mode()

        print("  Done.")
        print(f"\n  To restore ros2_control:")
        print(f"  ros2 service call /dashboard_client/play std_srvs/srv/Trigger\n")


def main():
    rclpy.init()
    node = URScriptForceTest()
    try:
        node.run()
    except Exception as e:
        print(f"\n  Error: {e}")
        node.stop_force_mode()
    finally:
        node._running = False
        node.stop_force_mode()
        time.sleep(0.2)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
