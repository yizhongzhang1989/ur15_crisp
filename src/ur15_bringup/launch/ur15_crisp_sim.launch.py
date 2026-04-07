"""Load CRISP controllers into a running Gazebo simulation.

Prerequisites:
  The ur_simulator must already be running in effort mode:
    source install/setup.bash
    cd src/ur_simulator && ./launch_all.sh --control_mode effort

This launch file spawns CRISP controllers into the existing controller_manager.
Controller types and parameters are defined in ur15_sim_controllers.yaml.

After launch, activate a CRISP controller with:
  ros2 control switch_controllers \
      --deactivate forward_effort_controller \
      --activate crisp_gravity_compensation

Usage:
  ros2 launch ur15_bringup ur15_crisp_sim.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # CRISP broadcasters - active (read-only state interfaces, no conflict)
    twist_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "crisp_twist_broadcaster",
            "-c", "/controller_manager",
        ],
    )
    pose_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "crisp_pose_broadcaster",
            "-c", "/controller_manager",
        ],
    )

    # CRISP controllers - inactive (activate manually, see docstring above)
    gravity_comp_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "crisp_gravity_compensation",
            "-c", "/controller_manager",
            "--inactive",
        ],
    )
    cartesian_impedance_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "cartesian_impedance_controller",
            "-c", "/controller_manager",
            "--inactive",
        ],
    )
    joint_impedance_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_impedance_controller",
            "-c", "/controller_manager",
            "--inactive",
        ],
    )

    return LaunchDescription([
        twist_broadcaster_spawner,
        pose_broadcaster_spawner,
        gravity_comp_spawner,
        cartesian_impedance_spawner,
        joint_impedance_spawner,
    ])
