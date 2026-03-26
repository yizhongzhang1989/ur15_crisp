# Robotiq 2F-140 Gripper ROS2 Driver

ROS2 driver for controlling the Robotiq 2F-140 gripper via RS485/Modbus interface using ROS2 actions.

## Packages

- **robotiq_gripper_msgs**: Custom messages and actions for gripper control
  - `GripperStatus.msg`: Status message for continuous monitoring
  - `GripperActivate.action`: Action for gripper activation
  - `GripperControl.action`: Action for gripper position control
- **robotiq_2f140_gripper**: ROS2 node for gripper control
- **robotiq_2f140_gripper_web**: Web interface for gripper monitoring and control

## Installation

```bash
cd ~/Documents/robot_dc/colcon_ws
colcon build --packages-select robotiq_gripper_msgs robotiq_2f140_gripper robotiq_2f140_gripper_web
source install/setup.bash
```

## Usage

### 1. Launch the Gripper

**Option A: Driver Only**

```bash
# Default configuration (device_id=9, host=192.168.1.15, port=54321)
ros2 launch robotiq_2f140_gripper robotiq_gripper.launch.py

# Custom configuration
ros2 launch robotiq_2f140_gripper robotiq_gripper.launch.py \
    device_id:=9 \
    rs485_host:=192.168.1.15 \
    rs485_port:=54321 \
    status_publish_rate:=10.0
```

**Option B: With Web Interface**

Launch the gripper driver with web interface in a single command:

```bash
# Default configuration
ros2 launch robotiq_2f140_gripper robotiq_gripper_web.launch.py

# Custom configuration
ros2 launch robotiq_2f140_gripper robotiq_gripper_web.launch.py \
    device_id:=9 \
    rs485_host:=192.168.1.15 \
    rs485_port:=54321 \
    status_publish_rate:=10.0 \
    web_host:=0.0.0.0 \
    web_port:=9000
```

This starts both the gripper driver and web server. Open `http://localhost:9000` in your browser to access the web interface, which provides:

### 2. Activate the Gripper

Before using the gripper, it must be activated:

```bash
# Send activation action goal with feedback
ros2 action send_goal /gripper/activate robotiq_gripper_msgs/action/GripperActivate "{}" --feedback
```

### 3. Control the Gripper

Send control commands via action:

```bash
# Close the gripper (position=255, speed=128, force=128)
ros2 action send_goal /gripper/control robotiq_gripper_msgs/action/GripperControl \
    "{position: 255, speed: 128, force: 128}" --feedback

# Open the gripper (position=0)
ros2 action send_goal /gripper/control robotiq_gripper_msgs/action/GripperControl \
    "{position: 0, speed: 128, force: 128}" --feedback

# Move to 50% position with slow speed and low force
ros2 action send_goal /gripper/control robotiq_gripper_msgs/action/GripperControl \
    "{position: 128, speed: 50, force: 100}" --feedback

# Fast close with high force
ros2 action send_goal /gripper/control robotiq_gripper_msgs/action/GripperControl \
    "{position: 255, speed: 255, force: 255}" --feedback
```

### 4. Monitor Gripper Status

```bash
# Echo status messages (published continuously at 10 Hz)
ros2 topic echo /gripper/status

# View status publishing rate
ros2 topic hz /gripper/status
```

## Actions

### `/gripper/activate` (`robotiq_gripper_msgs/action/GripperActivate`)

Activates the gripper (must be called once before control).

**Goal**: Empty (no parameters needed)

**Result**:
- **success** (bool): True if activation succeeded
- **message** (string): Status message

**Feedback**:
- **status** (string): Current activation status
- **is_activated** (bool): Activation state

### `/gripper/control` (`robotiq_gripper_msgs/action/GripperControl`)

Controls gripper position with real-time feedback.

**Goal**:
- **position** (uint8): 0 (fully open) to 255 (fully closed)
- **speed** (uint8): 0 (slowest) to 255 (fastest)
- **force** (uint8): 0 (weakest) to 255 (strongest)

**Result**:
- **success** (bool): True if motion completed successfully
- **final_position** (uint8): Final gripper position
- **object_detected** (bool): True if object was grasped

**Feedback**:
- **current_position** (uint8): Current position during motion
- **is_moving** (bool): True while gripper is moving

## Topics

### Published Topics

- `/gripper/status` (`robotiq_gripper_msgs/GripperStatus`)
  - **header**: Timestamp and frame_id
  - **is_activated**: Gripper activation status (gSTA == 0x03)
  - **is_moving**: True if gripper is in motion (gOBJ == 0x00)
  - **object_detected**: True if object detected during grasp (gOBJ == 0x01 or 0x02)
  - **fault**: Fault status from gripper
  - **position**: Current position (0-255)
  - **force**: Current force/current reading
  - **raw_registers**: Raw Modbus register values for debugging

## Parameters

- **device_id** (int, default: 9): Modbus device ID
- **rs485_host** (string, default: "192.168.1.15"): RS485 gateway IP address
- **rs485_port** (int, default: 54321): RS485 gateway port
- **status_publish_rate** (double, default: 10.0): Status publishing rate in Hz

## Python API Example

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from robotiq_gripper_msgs.action import GripperActivate, GripperControl

class GripperController(Node):
    def __init__(self):
        super().__init__('gripper_controller')
        
        # Create action clients
        self.activate_client = ActionClient(
            self, GripperActivate, '/gripper/activate')
        self.control_client = ActionClient(
            self, GripperControl, '/gripper/control')
    
    def activate_gripper(self):
        """Activate the gripper."""
        goal = GripperActivate.Goal()
        
        self.activate_client.wait_for_server()
        future = self.activate_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        
        result = future.result().get_result()
        return result.success
    
    def close_gripper(self, speed=128, force=128):
        """Close the gripper."""
        goal = GripperControl.Goal()
        goal.position = 255
        goal.speed = speed
        goal.force = force
        
        self.control_client.wait_for_server()
        future = self.control_client.send_goal_async(
            goal, feedback_callback=self.feedback_callback)
        rclpy.spin_until_future_complete(self, future)
        
        result = future.result().get_result()
        return result.success, result.final_position, result.object_detected
    
    def open_gripper(self, speed=128):
        """Open the gripper."""
        goal = GripperControl.Goal()
        goal.position = 0
        goal.speed = speed
        goal.force = 0
        
        self.control_client.wait_for_server()
        future = self.control_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        
        result = future.result().get_result()
        return result.success, result.final_position
    
    def feedback_callback(self, feedback_msg):
        """Handle feedback during gripper motion."""
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Position: {feedback.current_position}, '
            f'Moving: {feedback.is_moving}')

def main():
    rclpy.init()
    controller = GripperController()
    
    # Activate gripper
    if controller.activate_gripper():
        print('Gripper activated successfully')
        
        # Close gripper
        success, position, detected = controller.close_gripper()
        print(f'Close: success={success}, position={position}, object={detected}')
        
        # Open gripper
        success, position = controller.open_gripper()
        print(f'Open: success={success}, position={position}')
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Troubleshooting

### Connection Issues

1. Check RS485 gateway connectivity:
   ```bash
   ping 192.168.1.15
   ```

2. Verify port is accessible:
   ```bash
   nc -zv 192.168.1.15 54321
   ```

3. Check node logs:
   ```bash
   ros2 run robotiq_2f140_gripper robotiq_gripper_node --ros-args --log-level debug
   ```

### Gripper Not Responding

1. Make sure gripper is activated:
   ```bash
   ros2 action send_goal /gripper/activate robotiq_gripper_msgs/action/GripperActivate "{}"
   ```

2. Check device ID matches your hardware configuration (default: 9)

3. Verify Modbus communication by monitoring status:
   ```bash
   ros2 topic echo /gripper/status
   ```

4. Check action server availability:
   ```bash
   ros2 action list
   ros2 action info /gripper/activate
   ros2 action info /gripper/control
   ```

### Action Timeouts

If actions time out during motion:
- Increase the timeout in the action client code
- Check if gripper is physically obstructed
- Verify status readings show `is_moving` changing to `False`
- Monitor feedback during execution with `--feedback` flag

## Hardware Setup

- **RS485 Gateway**: Connect UR robot's RS485 port to TCP/IP gateway
- **Gripper**: Connect Robotiq 2F-140 to RS485 bus
- **Device ID**: Configure gripper Modbus device ID (default: 9)

## License

MIT
