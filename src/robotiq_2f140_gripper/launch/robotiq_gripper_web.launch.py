from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description for gripper driver and web interface."""
    
    # Declare launch arguments for gripper driver
    device_id_arg = DeclareLaunchArgument(
        'device_id',
        default_value='9',
        description='Modbus device ID of the gripper'
    )
    
    rs485_host_arg = DeclareLaunchArgument(
        'rs485_host',
        default_value='192.168.1.15',
        description='IP address of the RS485 gateway'
    )
    
    rs485_port_arg = DeclareLaunchArgument(
        'rs485_port',
        default_value='54321',
        description='Port number of the RS485 gateway'
    )
    
    status_rate_arg = DeclareLaunchArgument(
        'status_publish_rate',
        default_value='10.0',
        description='Rate (Hz) for publishing gripper status'
    )
    
    # Declare launch arguments for web interface
    web_host_arg = DeclareLaunchArgument(
        'web_host',
        default_value='0.0.0.0',
        description='Web server host address'
    )
    
    web_port_arg = DeclareLaunchArgument(
        'web_port',
        default_value='9000',
        description='Web server port'
    )
    
    # Create the gripper driver node
    gripper_node = Node(
        package='robotiq_2f140_gripper',
        executable='robotiq_gripper_node',
        name='robotiq_gripper_node',
        output='screen',
        parameters=[{
            'device_id': LaunchConfiguration('device_id'),
            'rs485_host': LaunchConfiguration('rs485_host'),
            'rs485_port': LaunchConfiguration('rs485_port'),
            'status_publish_rate': LaunchConfiguration('status_publish_rate'),
        }]
    )
    
    # Create the web interface node
    gripper_web_node = Node(
        package='robotiq_2f140_gripper_web',
        executable='gripper_web_node',
        name='gripper_web_node',
        output='screen',
        parameters=[{
            'host': LaunchConfiguration('web_host'),
            'port': LaunchConfiguration('web_port'),
        }]
    )
    
    return LaunchDescription([
        device_id_arg,
        rs485_host_arg,
        rs485_port_arg,
        status_rate_arg,
        web_host_arg,
        web_port_arg,
        gripper_node,
        gripper_web_node,
    ])
