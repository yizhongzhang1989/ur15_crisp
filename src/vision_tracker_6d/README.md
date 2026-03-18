# Vision Tracker 6D

Real-time 6D chessboard pose tracking using one or more cameras.  
Publishes `geometry_msgs/PoseStamped` on a ROS 2 topic for robot control.

## Architecture

```
[usb_cam / any camera node]          [vision_tracker_6d]
        |                                     |
  /cam0/image_raw  ───────────────►  subscribes to image topics
                                      detects chessboard (OpenCV)
                                      computes 6D pose (solvePnP)
                                      applies temporal filter
                                              |
                                    /vision_tracker/chessboard_pose
                                    /vision_tracker/detection_active
                                              |
                                      Flask web dashboard (:8090)
                                      live MJPEG + pose display
```

Camera nodes and the tracker are **separate processes** communicating via ROS topics.
This means you can use any camera driver (usb\_cam, realsense, etc.) or even
publish images from a bag file.

---

## Quick Start

```bash
# 1. Build
cd ~/Documents/ur15_crisp
source /opt/ros/humble/setup.bash
colcon build --packages-select vision_tracker_6d

# 2. Source
source install/setup.bash

# 3. Launch everything (usb_cam + tracker + web UI)
ros2 launch vision_tracker_6d vision_tracker.launch.py
```

Open **http://\<host-ip\>:8090** in a browser.

---

## Testing Components Separately

### Step 1 — Start a camera node alone

```bash
source /opt/ros/humble/setup.bash

# Single USB camera on /dev/video0, publishing to /cam0/image_raw
ros2 run usb_cam usb_cam_node_exe --ros-args \
  -p video_device:=/dev/video0 \
  -p framerate:=30.0 \
  -p image_width:=1280 \
  -p image_height:=720 \
  -p pixel_format:=mjpeg2rgb \
  -r image_raw:=/cam0/image_raw
```

To add a second camera:

```bash
ros2 run usb_cam usb_cam_node_exe --ros-args \
  -p video_device:=/dev/video2 \
  -p framerate:=30.0 \
  -r image_raw:=/cam1/image_raw
```

### Step 2 — Verify images are published

```bash
# List topics — you should see /cam0/image_raw
ros2 topic list | grep image

# Check publish rate
ros2 topic hz /cam0/image_raw

# View the image (requires rqt_image_view)
ros2 run rqt_image_view rqt_image_view
```

### Step 3 — Start the tracker (web mode)

```bash
source install/setup.bash
ros2 run vision_tracker_6d web_server
```

Or headless (no web UI):

```bash
ros2 run vision_tracker_6d tracker_node
```

### Step 4 — Verify pose output

```bash
ros2 topic echo /vision_tracker/chessboard_pose
ros2 topic hz /vision_tracker/chessboard_pose
ros2 topic echo /vision_tracker/detection_active
```

---

## Launch File Options

```bash
# Default: launch usb_cam nodes + web server
ros2 launch vision_tracker_6d vision_tracker.launch.py

# Headless: launch usb_cam nodes + tracker only (no web)
ros2 launch vision_tracker_6d vision_tracker.launch.py headless:=true
```

The launch file reads `config/vision_tracker.yaml` and automatically spawns
a `usb_cam` node for each camera that has a `video_device` field.

---

## Configuration

Edit `config/vision_tracker.yaml` (auto-created from template on first launch):

```yaml
chessboard:
  rows: 7               # inner corners per row
  cols: 9               # inner corners per column
  square_size: 0.025    # square size in meters

cameras:
  - name: cam0
    image_topic: /cam0/image_raw      # ROS topic to subscribe to
    video_device: /dev/video0          # usb_cam will open this device
    framerate: 30
    image_width: 640
    image_height: 480
    calibration_file: ""               # path to calibration_data.json
    enabled: true

filter:
  enabled: true
  alpha: 0.5            # EMA smoothing (0=very smooth, 1=no filter)

web:
  port: 8090
```

### Camera entries

| Field | Required | Description |
|-------|----------|-------------|
| `name` | yes | Unique camera identifier |
| `image_topic` | yes | ROS image topic to subscribe to |
| `video_device` | no | If set, launch file spawns usb\_cam for this device |
| `framerate` | no | Capture frame rate (default 30) |
| `image_width` | no | Capture width (default 1280) |
| `image_height` | no | Capture height (default 720) |
| `calibration_file` | no | Path to `calibration_data.json` (from camera\_calibration\_toolkit) |
| `enabled` | no | Set `false` to skip this camera |

If `video_device` is **omitted**, no usb\_cam node is spawned — the tracker
just subscribes to whatever external node publishes on `image_topic` (e.g.
RealSense, IP camera, rosbag replay).

### Calibration

Use the bundled [camera\_calibration\_toolkit](ThirdParty/camera_calibration_toolkit/) to
calibrate each camera. Point `calibration_file` to the resulting
`calibration_data.json`.  Without calibration, the camera feed is shown but
the chessboard pose cannot be computed.

---

## Web Dashboard

**http://\<host-ip\>:8090**

- Live MJPEG camera feed with detected corners and coordinate axes overlay
- Real-time 6D pose display (XYZ + quaternion)
- Start / Stop tracking buttons
- Chessboard parameter editor (rows, cols, square size)
- Camera status indicators (receiving / calibrated)
- FPS counter and detection status

### REST API

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/api/status` | GET | Full tracker status JSON |
| `/api/stream` | GET | SSE stream at ~10 Hz |
| `/api/tracking` | POST | `{"enabled": true/false}` |
| `/api/chessboard` | GET/POST | Get or set chessboard params |
| `/api/cameras` | GET | Camera status |
| `/api/video_feed/<name>` | GET | MJPEG stream |
| `/api/snapshot/<name>` | GET | Single JPEG frame |

---

## ROS Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/vision_tracker/chessboard_pose` | `geometry_msgs/PoseStamped` | Detected 6D pose (filtered) |
| `/vision_tracker/detection_active` | `std_msgs/Bool` | Whether a chessboard is currently detected |

---

## Package Structure

```
src/vision_tracker_6d/
├── package.xml
├── setup.py / setup.cfg
├── config/
│   └── vision_tracker_config_template.yaml
├── launch/
│   └── vision_tracker.launch.py
├── vision_tracker_6d/
│   ├── tracker_node.py          # Backend: image subscription + detection + ROS publish
│   ├── web_server.py            # Flask web UI + REST API
│   ├── chessboard_detector.py   # OpenCV findChessboardCorners + solvePnP
│   ├── pose_filter.py           # EMA + SLERP temporal smoothing
│   ├── calibration_loader.py    # Load camera_calibration_toolkit JSON output
│   ├── camera_manager.py        # (legacy) Direct OpenCV camera capture
│   └── static/
│       └── index.html           # Web dashboard
└── ThirdParty/
    └── camera_calibration_toolkit/  # Git submodule
```
