#!/usr/bin/env python3
"""
ROS2 Node for Robotiq 2F-140 Gripper Control

Actions:
    /gripper/activate (robotiq_gripper_msgs/action/GripperActivate): Activate the gripper
    /gripper/control (robotiq_gripper_msgs/action/GripperControl): Control gripper with feedback

Published Topics:
    /gripper/status (robotiq_gripper_msgs/GripperStatus): Current gripper status
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Header
import time

from robotiq_gripper_msgs.msg import GripperStatus
from robotiq_gripper_msgs.action import GripperControl, GripperActivate
from .robotiq_gripper import Robotiq2f140Gripper


class RobotiqGripperNode(Node):
    """ROS2 Node for controlling Robotiq 2F-140 Gripper."""

    def __init__(self):
        super().__init__('robotiq_gripper_node')
        
        # Declare parameters
        self.declare_parameter('device_id', 9)
        self.declare_parameter('rs485_host', '192.168.1.15')
        self.declare_parameter('rs485_port', 54321)
        self.declare_parameter('status_publish_rate', 10.0)  # Hz
        
        # Get parameters
        device_id = self.get_parameter('device_id').value
        rs485_host = self.get_parameter('rs485_host').value
        rs485_port = self.get_parameter('rs485_port').value
        status_rate = self.get_parameter('status_publish_rate').value
        
        self.get_logger().info('Initializing Robotiq Gripper Node')
        self.get_logger().info(f'  Device ID: {device_id}')
        self.get_logger().info(f'  RS485 Gateway: {rs485_host}:{rs485_port}')
        
        # Initialize gripper controller
        self.gripper = Robotiq2f140Gripper(
            device_id=device_id,
            host=rs485_host,
            port=rs485_port
        )
        
        # Create callback group for status timer (separate from action servers)
        self.status_callback_group = ReentrantCallbackGroup()
        
        # Create action server for gripper activation
        self._activate_action_server = ActionServer(
            self,
            GripperActivate,
            '/gripper/activate',
            self.execute_activate,
            callback_group=ReentrantCallbackGroup()
        )
        
        # Create action server for gripper control
        self._control_action_server = ActionServer(
            self,
            GripperControl,
            '/gripper/control',
            self.execute_control,
            callback_group=ReentrantCallbackGroup()
        )
        
        # Create publisher for gripper status
        self.status_pub = self.create_publisher(
            GripperStatus,
            '/gripper/status',
            10
        )
        
        # Create timer for status publishing with separate callback group
        timer_period = 1.0 / status_rate if status_rate > 0 else 0.1
        self.status_timer = self.create_timer(
            timer_period, 
            self.publish_status,
            callback_group=self.status_callback_group
        )
        
        self.get_logger().info('Robotiq Gripper Node initialized successfully')
        self.get_logger().info('Waiting for activation... Send goal to /gripper/activate action')

    def execute_activate(self, goal_handle):
        """Execute gripper activation action."""
        self.get_logger().info('Activating gripper...')
        
        # Create feedback message
        feedback_msg = GripperActivate.Feedback()
        
        # Send activation status
        feedback_msg.status = 'Starting activation sequence...'
        feedback_msg.is_activated = False
        goal_handle.publish_feedback(feedback_msg)
        
        # Call activation
        success = self.gripper.activate(timeout=5.0)
        
        # Create result
        result = GripperActivate.Result()
        result.success = success
        
        if success:
            feedback_msg.status = 'Gripper activated successfully'
            feedback_msg.is_activated = True
            goal_handle.publish_feedback(feedback_msg)
            
            result.message = 'Gripper activated successfully'
            self.get_logger().info('Gripper activated successfully')
            goal_handle.succeed()
        else:
            feedback_msg.status = 'Failed to activate gripper'
            feedback_msg.is_activated = False
            goal_handle.publish_feedback(feedback_msg)
            
            result.message = 'Failed to activate gripper'
            self.get_logger().error('Failed to activate gripper')
            goal_handle.abort()
        
        return result

    def execute_control(self, goal_handle):
        """Execute gripper control action."""
        self.get_logger().info(
            f'Executing gripper control: position={goal_handle.request.position}, '
            f'speed={goal_handle.request.speed}, force={goal_handle.request.force}'
        )
        
        # Create feedback message
        feedback_msg = GripperControl.Feedback()
        
        # Send movement command
        success = self.gripper.move_to_position(
            position=goal_handle.request.position,
            speed=goal_handle.request.speed,
            force=goal_handle.request.force,
            wait=False
        )
        
        if not success:
            self.get_logger().error('Failed to send gripper command')
            goal_handle.abort()
            result = GripperControl.Result()
            result.success = False
            result.final_position = 0
            result.object_detected = False
            return result
        
        # Monitor progress and publish feedback
        timeout = 10.0  # 10 second timeout
        start_time = time.time()
        
        while True:
            status_dict = self.gripper.get_status()
            
            if status_dict is None:
                if time.time() - start_time > timeout:
                    self.get_logger().error('Timeout waiting for gripper status')
                    goal_handle.abort()
                    result = GripperControl.Result()
                    result.success = False
                    result.final_position = 0
                    result.object_detected = False
                    return result
                time.sleep(0.1)
                continue
            
            # Update feedback
            feedback_msg.current_position = status_dict['position']
            feedback_msg.is_moving = status_dict['moving']
            goal_handle.publish_feedback(feedback_msg)
            
            self.get_logger().debug(
                f"Status: position={status_dict['position']}, moving={status_dict['moving']}, "
                f"raw_registers={[hex(r) for r in status_dict['raw_registers']]}"
            )
            
            # Check if motion is complete
            if not status_dict['moving']:
                break
            
            # Timeout check
            if time.time() - start_time > timeout:
                self.get_logger().warn('Gripper motion timeout')
                break
            
            time.sleep(0.1)
        
        # Motion complete, set result
        goal_handle.succeed()
        result = GripperControl.Result()
        result.success = True
        result.final_position = status_dict['position']
        result.object_detected = status_dict['object_detected']
        
        self.get_logger().info(
            f'Gripper control completed: position={result.final_position}, '
            f'object_detected={result.object_detected}'
        )
        
        return result
    
    def publish_status(self):
        """Periodically publish gripper status."""
        try:
            status_dict = self.gripper.get_status()
            
            if status_dict is None:
                self.get_logger().warn('Failed to read gripper status (returned None)', throttle_duration_sec=5.0)
                return
        except Exception as e:
            self.get_logger().error(f'Exception while reading gripper status: {e}', throttle_duration_sec=5.0)
            return
        
        # Create and populate status message
        msg = GripperStatus()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gripper'
        
        msg.is_activated = status_dict['activated']
        msg.is_moving = status_dict['moving']
        msg.object_detected = status_dict['object_detected']
        msg.fault = status_dict['fault']
        msg.position = status_dict['position']
        msg.force = status_dict['force']
        msg.raw_registers = status_dict['raw_registers']
        
        self.status_pub.publish(msg)
    
    def destroy_node(self):
        """Clean up resources."""
        self.get_logger().info('Shutting down Robotiq Gripper Node...')
        self.gripper.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = RobotiqGripperNode()
    
    # Use MultiThreadedExecutor to allow parallel callback execution
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
