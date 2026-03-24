import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from common.workspace import get_config_path


def generate_launch_description():
    joint_config = get_config_path("joint_config.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("rate", default_value="50.0"),
        DeclareLaunchArgument("trajectory_time", default_value="0.1"),

        # Alicia leader arm driver
        Node(
            package="alicia_duo_leader_driver",
            executable="alicia_driver_node.py",
            name="alicia_driver",
            output="screen",
            parameters=[{
                "port": "ttyACM0",
                "baudrate": 1000000,
                "query_rate": 200.0,
                "joint_config": joint_config,
            }],
        ),

        # Alicia dashboard (web UI)
        Node(
            package="alicia_duo_leader_dashboard",
            executable="dashboard_node.py",
            name="alicia_duo_leader_dashboard",
            output="screen",
            parameters=[{
                "port": 8090,
            }],
        ),

        # Teleop node
        Node(
            package="alicia_teleop",
            executable="teleop_node",
            name="alicia_teleop",
            output="screen",
            parameters=[{
                "rate": LaunchConfiguration("rate"),
                "trajectory_time": LaunchConfiguration("trajectory_time"),
                "joint_scale": [1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
                "joint_offset": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            }],
        ),
    ])
