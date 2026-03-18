from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="web_control",
            executable="web_server",
            name="web_control",
            output="screen",
        ),
    ])
