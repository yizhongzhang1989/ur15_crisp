# joint_vla_control

ROS 2 package for controlling the UR15 robot arm using a VLA (Visual Language Action) model. The VLA takes a camera image, current joint state, and a natural language instruction, then predicts the next 30 frames (1 second at 30 fps) of joint positions and gripper commands.

## Architecture

```
┌──────────────────────────────────────────────────────────────┐
│  External VLA Model Server (HTTP)                            │
│  e.g. http://10.190.172.212:4500/api/inference               │
└───────────────────────┬──────────────────────────────────────┘
                        │  POST: images + state + instruction
                        │  Response: {ROBOT_RIGHT_JOINTS: (30,6),
                        │             ROBOT_RIGHT_GRIPPER: (30,1)}
                        ▼
┌──────────────────────────────────────────────────────────────┐
│  joint_vla_control  (ROS 2 node + Flask web server :8091)    │
│                                                              │
│  Subscribes:                                                 │
│    /ur15_camera/image_raw  (sensor_msgs/Image)               │
│    /joint_states           (sensor_msgs/JointState)          │
│                                                              │
│  Publishes:                                                  │
│    /arm_joint_state        (ArmJointState)  ← same as        │
│                             Alicia leader arm teleop          │
│                                                              │
│  Web dashboard: http://localhost:8091                         │
└──────────────────────────────────────────────────────────────┘
                        │
                        │  /arm_joint_state (30 fps)
                        ▼
┌──────────────────────────────────────────────────────────────┐
│  web_control (existing teleop pipeline)                      │
│    _alicia_cb → _set_cmd_target → smooth → controllers       │
└──────────────────────────────────────────────────────────────┘
```

### Key design decisions

- **Publishes to `/arm_joint_state`** (not `/cmd_target_joint`) so the predicted actions flow through the existing Alicia teleop pipeline in `web_control`, including smoothing and controller feeding.
- **VLA inference is HTTP-based** — the model runs on a separate GPU server. The `vla_predict()` function sends JPEG-encoded images + JSON state via multipart POST.
- **Chunk-based execution** — each prediction returns 30 frames. Execution sends one frame per tick at 30 Hz (1 second per chunk).

## Package Structure

```
joint_vla_control/
├── package.xml
├── setup.py / setup.cfg
├── launch/
│   └── joint_vla_control.launch.py
├── joint_vla_control/
│   ├── __init__.py
│   ├── vla_server.py              # Main node: ROS 2 + Flask + VLA client
│   └── static/
│       └── index.html             # Web dashboard (single-file)
├── resource/
│   └── joint_vla_control
└── test/
    └── __init__.py
```

## Usage

```bash
# Build
cd ~/Documents/ur15_crisp
colcon build --symlink-install --packages-select joint_vla_control
source install/setup.bash

# Launch (requires ur15_bringup + web_control already running)
ros2 launch joint_vla_control joint_vla_control.launch.py

# Dashboard
open http://localhost:8091
```

## Web Dashboard (port 8091)

The dashboard provides:

### Left column
1. **Configuration** — Edit VLA server URL and instruction text. Click "Save Config" to apply, "Run Prediction" to trigger one inference.
2. **Camera** — Live image from `/ur15_camera/image_raw` (auto-refreshes every 2s).
3. **Execution Control**:
   - **Stop** — halt execution
   - **Step Once** — send exactly 1 predicted frame to `/arm_joint_state`
   - **Continuous** — send all predicted frames at 30 fps (auto-stops at end)
   - **Reset Step** — rewind to frame 0 for replay
   - **Auto Run** — loop: predict → execute all frames → predict → ... (click again to stop)
4. **Predicted Trajectory** — table showing all 30 predicted frames with joint angles (degrees) and gripper. Rows are color-coded: dim=sent, blue=current, white=pending.

### Right column
- **Joint Plot** — 2×4 grid of canvases (6 joints + 1 gripper):
  - **Blue line + dots** = predicted trajectory (static, frame-indexed)
  - **Green line + dots** = actual robot position (fills in during execution)
  - **White vertical line** = current execution step
  - All joint canvases share the same Y-axis span for easy comparison
  - Gripper canvas has fixed range [-50, 1050]
  - X-axis shows frame number (0–29) and duration in seconds

## VLA Inference API

The `vla_predict()` function in `vla_server.py` calls the external model server:

**Request** (multipart POST):
- `image_0`: JPEG file (camera image, RGB)
- `json`: JSON with `task_description`, `state`, `use_state`, `has_left`, `has_right`, `has_progress`

**State format**:
```python
{
    "ROBOT_RIGHT_JOINTS": [q1, q2, q3, q4, q5, q6],  # radians
}
```

**Response** (JSON):
```python
{
    "ROBOT_RIGHT_JOINTS": [[q1..q6], ...],  # (chunk_size, 6) radians
    "ROBOT_RIGHT_GRIPPER": [[g], ...],       # (chunk_size, 1)
}
```

The reference client is in `scripts/ur15_client_dummy.py`.

## REST API Endpoints

| Endpoint | Method | Description |
|---|---|---|
| `/` | GET | Dashboard HTML |
| `/api/status` | GET | Current state (joints, config, prediction status) |
| `/api/config` | POST | Set `instruction` and/or `vla_url` |
| `/api/predict` | POST | Run VLA inference (blocking, ~5-30s) |
| `/api/exec` | POST | Set execution mode: `stopped`, `step`, `continuous` |
| `/api/step` | POST | Send one frame (sets mode to `step`) |
| `/api/reset_step` | POST | Reset step counter to 0 |
| `/api/image` | GET | Latest camera image as JPEG |
| `/api/prediction` | GET | Full predicted trajectory as JSON |
| `/api/stream` | GET | SSE stream (~10 Hz) of status + plot data |

## Internal State

Key variables in `VLAControlServer`:

| Variable | Type | Description |
|---|---|---|
| `_latest_image` | ndarray (H,W,3) BGR | Latest camera frame |
| `_latest_joints` | ndarray (6,) | Current joint positions (rad) |
| `_predicted_joints` | ndarray (30,6) or None | Last VLA prediction (rad) |
| `_predicted_gripper` | ndarray (30,1) or None | Last gripper prediction |
| `_pred_step` | int | Current frame index in chunk (0–29) |
| `_exec_mode` | str | `"stopped"`, `"step"`, or `"continuous"` |
| `_instruction` | str | Natural language task instruction |
| `_vla_url` | str | VLA model server endpoint |

## Dependencies

- `rclpy`, `sensor_msgs`, `std_msgs`, `cv_bridge`
- `flask`, `requests`, `Pillow`, `opencv-python`, `numpy`
- `alicia_duo_leader_driver` (for `ArmJointState` message type)

## Port Assignment

This package uses port **8091**. See main README for full port map.
