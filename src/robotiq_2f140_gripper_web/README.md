# Robotiq 2F-140 Gripper Web Interface

Web-based monitoring and control interface for the Robotiq 2F-140 gripper. Provides real-time status visualization and interactive control for debugging purposes.

## Features

- **Real-time Status Monitoring**
  - Connection status
  - Activation state
  - Motion status
  - Object detection
  - Current position and force
  - Visual gripper representation

- **Interactive Control**
  - Activate/deactivate gripper
  - Quick preset buttons (Open, Half, Close)
  - Custom position control with sliders
  - Adjustable speed and force parameters

- **Debug Information**
  - Raw register values
  - Timestamp information
  - Detailed status breakdown

## Dependencies

- Flask: `pip install flask`
- ROS2 Humble
- robotiq_gripper_msgs package
- robotiq_2f140_gripper package (must be running)

## Installation

```bash
cd /path/to/colcon_ws
colcon build --packages-select robotiq_2f140_gripper_web
source install/setup.bash
```

## Usage

### Launch the Web Interface

```bash
ros2 launch robotiq_2f140_gripper_web gripper_web.launch.py
```

With custom host and port:

```bash
ros2 launch robotiq_2f140_gripper_web gripper_web.launch.py host:=0.0.0.0 port:=8080
```

### Access the Interface

Open your web browser and navigate to:
- Local: http://localhost:5000
- Network: http://<your-ip>:5000

### Prerequisites

The gripper driver node must be running:

```bash
ros2 launch robotiq_2f140_gripper robotiq_gripper.launch.py
```

## Web Interface Guide

### Status Panel
- **Connection**: Shows if the web interface is receiving status updates
- **Activated**: Indicates if gripper is activated and ready
- **Moving**: Shows if gripper is currently in motion
- **Object Detected**: Indicates if an object is gripped
- **Fault**: Error indicator
- **Position**: Current gripper position (0=open, 255=closed)
- **Force**: Current force reading

### Visual Indicator
- Animated gripper fingers showing current opening
- Position bar showing percentage closed
- Updates in real-time as gripper moves

### Control Panel
- **Activate Gripper**: Initialize the gripper (required before use)
- **Open/Half/Close**: Quick preset positions
- **Position Slider**: Set custom target position (0-255)
- **Speed Slider**: Adjust movement speed (0-255)
- **Force Slider**: Adjust gripping force (0-255)
- **Move to Position**: Execute movement with current slider values

### Debug Panel
- Raw register values in hexadecimal
- Detailed status information
- Timestamp of last update

## API Endpoints

The web server provides REST API endpoints:

### GET /api/status
Returns current gripper status as JSON.

### POST /api/activate
Activates the gripper.

### POST /api/control
Controls gripper position.

Request body:
```json
{
  "position": 128,
  "speed": 128,
  "force": 128
}
```

## Parameters

- `host` (default: "0.0.0.0"): Web server host address
- `port` (default: 5000): Web server port

## Troubleshooting

### Web Interface Shows "Disconnected"
- Check if gripper driver node is running
- Verify `/gripper/status` topic is publishing: `ros2 topic hz /gripper/status`

### Action Commands Fail
- Ensure gripper is activated first
- Check action servers are available: `ros2 action list`
- Verify gripper hardware connection

### Port Already in Use
- Change port: `ros2 launch robotiq_2f140_gripper_web gripper_web.launch.py port:=8080`
- Or kill process using port: `lsof -ti:5000 | xargs kill -9`

## License

MIT
