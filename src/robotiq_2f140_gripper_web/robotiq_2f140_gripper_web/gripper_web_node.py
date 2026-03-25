#!/usr/bin/env python3
"""
Web interface node for Robotiq 2F-140 gripper monitoring and control.
Provides a Flask-based web UI for debugging purposes.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from robotiq_gripper_msgs.msg import GripperStatus
from robotiq_gripper_msgs.action import GripperActivate, GripperControl
from flask import Flask, render_template, jsonify, request
from threading import Thread, Lock
import os
from ament_index_python.packages import get_package_share_directory


class GripperWebNode(Node):
    """ROS2 node that provides web interface for gripper control."""
    
    def __init__(self):
        super().__init__('gripper_web_node')
        
        # Gripper status storage
        self.status_lock = Lock()
        self.latest_status = None
        
        # Active goal tracking
        self.active_goal_lock = Lock()
        self.active_goal_future = None
        self.active_result_future = None
        
        # Subscribe to gripper status
        self.status_sub = self.create_subscription(
            GripperStatus,
            '/gripper/status',
            self.status_callback,
            10
        )
        
        # Action clients
        self.activate_client = ActionClient(
            self,
            GripperActivate,
            '/gripper/activate'
        )
        
        self.control_client = ActionClient(
            self,
            GripperControl,
            '/gripper/control'
        )
        
        self.get_logger().info('Gripper Web Node initialized')
        self.get_logger().info('Waiting for action servers...')
        
    def status_callback(self, msg):
        """Update latest status from gripper."""
        with self.status_lock:
            self.latest_status = msg
    
    def get_status(self):
        """Get the latest gripper status (thread-safe)."""
        with self.status_lock:
            return self.latest_status
    
    def activate_gripper(self):
        """Send activation goal to gripper (async, executor handles spinning)."""
        if not self.activate_client.wait_for_server(timeout_sec=0.5):
            return {'success': False, 'message': 'Activate action server not available'}
        
        goal = GripperActivate.Goal()
        goal_future = self.activate_client.send_goal_async(goal)
        
        # Add callback to handle goal response
        import time
        max_wait = 12.0  # Total timeout
        start_time = time.time()
        
        # Wait for goal to be sent and accepted (executor handles spinning)
        while not goal_future.done() and (time.time() - start_time) < 2.0:
            time.sleep(0.01)
        
        if not goal_future.done():
            return {'success': False, 'message': 'Goal send timeout'}
        
        goal_handle = goal_future.result()
        if not goal_handle.accepted:
            return {'success': False, 'message': 'Goal rejected'}
        
        # Wait for result (executor handles spinning)
        result_future = goal_handle.get_result_async()
        
        while not result_future.done() and (time.time() - start_time) < max_wait:
            time.sleep(0.01)
        
        if not result_future.done():
            return {'success': False, 'message': 'Activation timeout'}
        
        result = result_future.result().result
        return {
            'success': result.success,
            'message': result.message
        }
    
    def control_gripper(self, position, speed, force):
        """Send control goal to gripper (async, executor handles spinning)."""
        if not self.control_client.wait_for_server(timeout_sec=0.5):
            return {'success': False, 'message': 'Control action server not available'}
        
        goal = GripperControl.Goal()
        goal.position = int(position)
        goal.speed = int(speed)
        goal.force = int(force)
        
        goal_future = self.control_client.send_goal_async(goal)
        
        # Wait for goal to be sent and accepted (executor handles spinning)
        import time
        max_wait = 17.0  # Total timeout
        start_time = time.time()
        
        while not goal_future.done() and (time.time() - start_time) < 2.0:
            time.sleep(0.01)
        
        if not goal_future.done():
            return {'success': False, 'message': 'Goal send timeout'}
        
        goal_handle = goal_future.result()
        if not goal_handle.accepted:
            return {'success': False, 'message': 'Goal rejected'}
        
        # Wait for result (executor handles spinning)
        result_future = goal_handle.get_result_async()
        
        while not result_future.done() and (time.time() - start_time) < max_wait:
            time.sleep(0.01)
        
        if not result_future.done():
            return {'success': False, 'message': 'Control timeout'}
        
        result = result_future.result().result
        return {
            'success': result.success,
            'final_position': result.final_position,
            'object_detected': result.object_detected
        }


# Global node instance
gripper_node = None

# Flask app
app = Flask(__name__)

# Get package share directory for templates and static files
try:
    pkg_share = get_package_share_directory('robotiq_2f140_gripper_web')
    app.template_folder = os.path.join(pkg_share, 'templates')
    app.static_folder = os.path.join(pkg_share, 'static')
except Exception:
    # Fallback to local directories during development
    app.template_folder = os.path.join(os.path.dirname(__file__), 'templates')
    app.static_folder = os.path.join(os.path.dirname(__file__), 'static')


@app.route('/')
def index():
    """Serve the main web interface."""
    return render_template('index.html')


@app.route('/api/status')
def api_status():
    """Return current gripper status as JSON."""
    if gripper_node is None:
        return jsonify({'error': 'Node not initialized'}), 500
    
    status = gripper_node.get_status()
    if status is None:
        return jsonify({'error': 'No status available'}), 503
    
    return jsonify({
        'timestamp': status.header.stamp.sec + status.header.stamp.nanosec * 1e-9,
        'is_activated': status.is_activated,
        'is_moving': status.is_moving,
        'object_detected': status.object_detected,
        'fault': status.fault,
        'position': status.position,
        'force': status.force,
        'raw_registers': list(status.raw_registers)
    })


@app.route('/api/activate', methods=['POST'])
def api_activate():
    """Activate the gripper."""
    if gripper_node is None:
        return jsonify({'error': 'Node not initialized'}), 500
    
    result = gripper_node.activate_gripper()
    return jsonify(result)


@app.route('/api/control', methods=['POST'])
def api_control():
    """Control the gripper position."""
    if gripper_node is None:
        return jsonify({'error': 'Node not initialized'}), 500
    
    data = request.get_json()
    position = data.get('position', 128)
    speed = data.get('speed', 128)
    force = data.get('force', 128)
    
    # Validate parameters
    if not (0 <= position <= 255):
        return jsonify({'error': 'Position must be 0-255'}), 400
    if not (0 <= speed <= 255):
        return jsonify({'error': 'Speed must be 0-255'}), 400
    if not (0 <= force <= 255):
        return jsonify({'error': 'Force must be 0-255'}), 400
    
    result = gripper_node.control_gripper(position, speed, force)
    return jsonify(result)


def run_flask(host='0.0.0.0', port=5000):
    """Run Flask app in a separate thread."""
    import logging
    log = logging.getLogger('werkzeug')
    log.setLevel(logging.ERROR)  # Only show errors, suppress access logs
    app.run(host=host, port=port, debug=False, use_reloader=False)


def main(args=None):
    """Main entry point."""
    global gripper_node
    
    rclpy.init(args=args)
    
    gripper_node = GripperWebNode()
    
    # Get parameters for web server
    gripper_node.declare_parameter('host', '0.0.0.0')
    gripper_node.declare_parameter('port', 5000)
    
    host = gripper_node.get_parameter('host').value
    port = gripper_node.get_parameter('port').value
    
    gripper_node.get_logger().info(f'Starting web server at http://{host}:{port}')
    
    # Start Flask in a separate thread
    flask_thread = Thread(target=run_flask, args=(host, port), daemon=True)
    flask_thread.start()
    
    # Create executor and spin
    executor = MultiThreadedExecutor()
    executor.add_node(gripper_node)
    
    try:
        gripper_node.get_logger().info('Web interface ready')
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        gripper_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
