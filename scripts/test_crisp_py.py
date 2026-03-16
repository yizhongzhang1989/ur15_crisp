"""Test crisp_py with UR15.

Usage:
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    python3 scripts/test_crisp_py.py

Requires the CRISP launch to be running:
    ros2 launch crisp_ur15_bringup ur15_crisp.launch.py robot_ip:=192.168.1.15
"""
import time

import numpy as np

from crisp_py.robot import make_robot
from crisp_py.utils.geometry import Pose


def main():
    # --- Step 1: Connect ---
    print("Creating UR robot interface...")
    robot = make_robot("ur")
    robot.wait_until_ready()

    print(f"  EE pose:     {robot.end_effector_pose}")
    print(f"  Joint values: {np.round(robot.joint_values, 3)}")

    # --- Step 2: Switch to Cartesian impedance ---
    input("\nPress Enter to switch to cartesian_impedance_controller...")
    robot.controller_switcher_client.switch_controller("cartesian_impedance_controller")
    print("Switched to cartesian_impedance_controller.")

    time.sleep(1.0)
    start_pose = robot.end_effector_pose.copy()
    print(f"  Start pose: pos={np.round(start_pose.position, 4)}")

    # --- Step 3: Small figure-eight (5 cm amplitude, 6 seconds) ---
    input("\nPress Enter to start a small figure-eight (5 cm, 6 s)...")
    amplitude = 0.05  # 5 cm — safe small motion
    duration = 6.0
    rate = 20.0
    start_time = time.time()

    while time.time() - start_time < duration:
        t = time.time() - start_time
        x = amplitude * np.sin(2 * np.pi * t / duration)
        z = amplitude * np.sin(4 * np.pi * t / duration)

        target_pose = Pose(
            position=start_pose.position + np.array([x, 0.0, z]),
            orientation=start_pose.orientation,
        )
        robot.set_target(pose=target_pose)
        time.sleep(1.0 / rate)

    print("Figure-eight complete. Returning to start pose...")
    robot.set_target(pose=start_pose)
    time.sleep(2.0)

    # --- Step 4: Done ---
    print("Shutting down in 2 seconds...")
    time.sleep(2.0)
    robot.shutdown()
    print("Done.")


if __name__ == "__main__":
    main()
