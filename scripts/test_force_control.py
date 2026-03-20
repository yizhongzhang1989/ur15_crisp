"""Test force control using UR native force_mode via URScript.

Sends URScript commands directly to the UR via the urscript_interface topic.
This temporarily takes over from External Control to run force_mode.

After force mode ends, the script automatically restores External Control
using the resend_robot_program service (headless mode), which is handled
entirely by the driver without needing the teach pendant.

Usage:
    # 1. Launch the driver:
    ros2 launch ur15_bringup ur15_crisp.launch.py robot_ip:=192.168.1.15

    # 2. Run this script:
    source install/setup.bash
    python3 scripts/test_force_control.py

Safety:
    - Ctrl+C immediately calls end_force_mode() and restores control
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
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
from geometry_msgs.msg import WrenchStamped
from controller_manager_msgs.srv import ListControllers, SwitchController


class URScriptForceTest(Node):
    def __init__(self):
        super().__init__("urscript_force_test")

        self._ft_force = np.zeros(3)
        self._ft_torque = np.zeros(3)
        self._running = True
        self._program_running = False
        self._previous_controller = None

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

        # Subscribe to robot program state
        self.create_subscription(
            Bool,
            "/io_and_status_controller/robot_program_running",
            self._program_cb,
            10,
        )

        # Service clients
        self._resend_program_client = self.create_client(
            Trigger, "/io_and_status_controller/resend_robot_program"
        )
        self._list_ctrl_client = self.create_client(
            ListControllers, "/controller_manager/list_controllers"
        )
        self._switch_ctrl_client = self.create_client(
            SwitchController, "/controller_manager/switch_controller"
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

    def _program_cb(self, msg):
        self._program_running = msg.data

    def _signal_handler(self, sig, frame):
        print("\n\n  Ctrl+C -- stopping force mode...")
        self._running = False

    def call_service(self, client, request, timeout=5.0):
        """Call a ROS 2 service and wait for the response."""
        if not client.wait_for_service(timeout_sec=timeout):
            self.get_logger().warn(f"Service {client.srv_name} not available after {timeout}s")
            return None
        future = client.call_async(request)
        start = time.time()
        while not future.done() and (time.time() - start) < timeout:
            time.sleep(0.05)
        if future.done():
            return future.result()
        self.get_logger().warn(f"Service {client.srv_name} timed out")
        return None

    def get_active_crisp_controller(self):
        """Get the currently active CRISP controller name, if any."""
        CRISP_CONTROLLERS = {
            "gravity_compensation",
            "cartesian_impedance_controller",
            "joint_impedance_controller",
        }
        resp = self.call_service(self._list_ctrl_client, ListControllers.Request())
        if resp is None:
            return None
        for c in resp.controller:
            if c.name in CRISP_CONTROLLERS and c.state == "active":
                return c.name
        return None

    def send_script(self, script: str):
        """Send a URScript command."""
        msg = String()
        msg.data = script
        self._script_pub.publish(msg)

    def start_force_mode(self, fz: float, speed_limit: float = 0.05, duration: float = 30.0):
        """Start force_mode by sending a URScript program.

        This replaces External Control temporarily.
        """
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
        """Stop force_mode via URScript."""
        script = "def stop_fm():\n  end_force_mode()\nend\n"
        self.send_script(script)

    def restore_control(self):
        """Restore External Control using resend_robot_program (headless mode)."""
        print("\n  Restoring External Control (resend_robot_program)...")

        # Wait a moment for force mode program to complete
        time.sleep(1.0)

        resp = self.call_service(self._resend_program_client, Trigger.Request(), timeout=10.0)
        if resp is None:
            print("  WARNING: resend_robot_program service not available.")
            return False
        if not resp.success:
            print(f"  WARNING: resend_robot_program failed: {resp.message}")
            return False

        print(f"  Program resent: {resp.message}")

        # Wait for External Control to reconnect
        print("  Waiting for External Control to connect...", end="", flush=True)
        for i in range(50):  # 5 seconds max
            if self._program_running:
                print(f" OK ({(i+1)*0.1:.1f}s)")
                break
            time.sleep(0.1)
        else:
            print(" timeout (program_running not received)")

        # Re-activate the previous CRISP controller
        if self._previous_controller:
            time.sleep(0.5)
            print(f"  Re-activating controller: {self._previous_controller}...")
            req = SwitchController.Request()
            req.activate_controllers = [self._previous_controller]
            req.strictness = SwitchController.Request.BEST_EFFORT
            resp = self.call_service(self._switch_ctrl_client, req, timeout=10.0)
            if resp and resp.ok:
                print(f"  Controller {self._previous_controller} activated.")
            else:
                print(f"  WARNING: Could not re-activate {self._previous_controller}.")

        print("  Recovery complete.")
        return True

    def run(self):
        # Configuration
        force_down = -30.0      # N (negative = downward)
        force_up = 30.0         # N (positive = upward)
        speed_limit = 0.03      # m/s -- very slow for safety
        phase_duration = 5.0    # seconds per phase

        print(f"\n{'='*65}")
        print(f"  UR Force Mode Test -- Down then Up")
        print(f"{'='*65}")
        print(f"  Phase 1: {force_down:.0f} N (down) for {phase_duration:.0f} s")
        print(f"  Phase 2: {force_up:.0f} N (up) for {phase_duration:.0f} s")
        print(f"  Speed limit Z:   {speed_limit:.3f} m/s")
        print(f"  Method:          URScript + auto-recovery")
        print(f"{'='*65}")
        print(f"\n  External Control will be interrupted temporarily.")
        print(f"  It will be restored automatically after force mode.\n")
        print(f"  Press Ctrl+C at any time to stop.\n")

        # Record active controller before starting
        self._previous_controller = self.get_active_crisp_controller()
        if self._previous_controller:
            print(f"  Active controller: {self._previous_controller}")
        else:
            print(f"  No CRISP controller active")

        input("\n  Press Enter to start...")
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
            time.sleep(0.5)
            self.restore_control()
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

        # Restore External Control
        print("\n\n  Force mode complete.")
        self.stop_force_mode()
        time.sleep(0.5)
        self.restore_control()


def main():
    rclpy.init()
    node = URScriptForceTest()
    try:
        node.run()
    except Exception as e:
        print(f"\n  Error: {e}")
        import traceback
        traceback.print_exc()
        node.stop_force_mode()
        time.sleep(0.5)
        node.restore_control()
    finally:
        node._running = False
        time.sleep(0.2)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
