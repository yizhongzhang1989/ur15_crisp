"""Launch RViz with interactive target pose marker for CRISP control."""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("web_control"), "config", "rviz_control.rviz"]
    )

    target_pose_marker = Node(
        package="web_control",
        executable="target_pose_marker",
        name="target_pose_marker",
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
    )

    return LaunchDescription([
        target_pose_marker,
        rviz,
    ])
