from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description for gripper web interface."""
    
    # Declare launch arguments
    host_arg = DeclareLaunchArgument(
        'host',
        default_value='0.0.0.0',
        description='Web server host address'
    )
    
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='5000',
        description='Web server port'
    )
    
    # Create node
    gripper_web_node = Node(
        package='robotiq_2f140_gripper_web',
        executable='gripper_web_node',
        name='gripper_web_node',
        output='screen',
        parameters=[{
            'host': LaunchConfiguration('host'),
            'port': LaunchConfiguration('port'),
        }]
    )
    
    return LaunchDescription([
        host_arg,
        port_arg,
        gripper_web_node,
    ])
