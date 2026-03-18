import os
import shutil

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from common.workspace import get_config_path


def _get_controllers_file() -> str:
    """Return the path to the active controllers YAML.

    Uses config/ur15_controllers.yaml in the workspace root if it exists,
    otherwise creates it from the template.
    """
    from ament_index_python.packages import get_package_share_directory

    template_path = os.path.join(
        get_package_share_directory("ur15_bringup"), "config", "ur15_controllers_template.yaml"
    )

    try:
        user_config = get_config_path("ur15_controllers.yaml")
    except RuntimeError:
        print("[ur15_crisp] Cannot find workspace root, using template directly.")
        return template_path

    if os.path.isfile(user_config):
        print(f"[ur15_crisp] Using config: {user_config}")
    else:
        shutil.copy2(template_path, user_config)
        print(f"[ur15_crisp] Created config from template: {user_config}")

    return user_config


def generate_launch_description():
    robot_ip_arg = DeclareLaunchArgument(
        "robot_ip",
        default_value="192.168.1.15",
        description="IP address of the UR15 robot.",
    )
    use_mock_hardware_arg = DeclareLaunchArgument(
        "use_mock_hardware",
        default_value="false",
        description="Start robot with mock hardware.",
    )
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="false",
        description="Start RViz for visualization.",
    )

    robot_ip = LaunchConfiguration("robot_ip")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    use_rviz = LaunchConfiguration("use_rviz")

    controllers_file = _get_controllers_file()

    ur_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur_robot_driver"), "launch", "ur_control.launch.py"]
            )
        ),
        launch_arguments={
            "ur_type": "ur15",
            "robot_ip": robot_ip,
            "use_mock_hardware": use_mock_hardware,
            "controllers_file": controllers_file,
            "launch_rviz": use_rviz,
            "initial_joint_controller": "scaled_joint_trajectory_controller",
            "headless_mode": "true",
        }.items(),
    )

    # Spawn CRISP broadcasters (active immediately)
    crisp_broadcasters_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "--controller-manager",
            "/controller_manager",
            "crisp_twist_broadcaster",
            "crisp_pose_broadcaster",
        ],
        output="screen",
    )

    # Spawn CRISP controllers (inactive — activate manually for safety)
    crisp_controllers_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "--controller-manager",
            "/controller_manager",
            "--inactive",
            "gravity_compensation",
            "cartesian_impedance_controller",
            "joint_impedance_controller",
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            robot_ip_arg,
            use_mock_hardware_arg,
            use_rviz_arg,
            ur_control,
            crisp_broadcasters_spawner,
            crisp_controllers_spawner,
        ]
    )
