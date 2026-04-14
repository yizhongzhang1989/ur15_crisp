from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="joint_history_vla_control",
            executable="vla_server",
            name="joint_history_vla_control",
            output="screen",
        ),
    ])
