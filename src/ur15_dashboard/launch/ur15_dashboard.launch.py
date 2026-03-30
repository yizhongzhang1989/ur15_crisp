from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port_arg = DeclareLaunchArgument(
        "web_port", default_value="8085", description="Web dashboard port"
    )

    dashboard_node = Node(
        package="ur15_dashboard",
        executable="dashboard_node.py",
        name="ur15_dashboard",
        output="screen",
        parameters=[
            {
                "port": LaunchConfiguration("web_port"),
            }
        ],
    )

    return LaunchDescription([
        port_arg,
        dashboard_node,
    ])
