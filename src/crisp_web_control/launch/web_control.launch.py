from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="crisp_web_control",
            executable="web_server",
            name="crisp_web_control",
            output="screen",
        ),
    ])
