# UR15 CRISP Controller Workspace

ROS 2 workspace for running [CRISP controllers](https://github.com/utiasDSL/crisp_controllers) on the **UR15** robot arm — compliant, torque-based Cartesian and joint-space controllers for `ros2_control`.

## Prerequisites

- **Ubuntu 22.04** with **ROS 2 Humble** (`ros-humble-desktop`)
- **UR robot driver**: `sudo apt install -y ros-humble-ur-robot-driver`
- **16+ GB RAM** recommended (CRISP compiles heavy C++ templates; use `-j1` if RAM < 16 GB)
- **Network**: Direct Ethernet to UR15 at `192.168.1.15` (host IP: `192.168.1.2` on same subnet)
- **UR15 teach pendant**: "External Control" URCap installed, Host IP set to `192.168.1.2`, port `50002`

## Quick Start

```bash
# 1. Clone with submodules
git clone --recurse-submodules --shallow-submodules https://github.com/yizhongzhang1989/ur15_crisp.git
cd ur15_crisp

# 2. Source ROS 2
source /opt/ros/humble/setup.bash

# 3. Install system dependencies
sudo apt install -y ros-humble-ur-robot-driver ros-humble-pinocchio ros-humble-generate-parameter-library
rosdep install --from-paths src --ignore-src -r -y

# 4. Build (use -j1 if RAM < 16 GB to avoid OOM)
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
source install/setup.bash

# 5. Install crisp_py (Python interface)
pip3 install src/crisp_py

# 6. Launch with mock hardware (verify build works)
ros2 launch ur15_bringup ur15_crisp.launch.py use_mock_hardware:=true

# 7. Launch with real UR15 (see "Launching on Real Hardware" below)
ros2 launch ur15_bringup ur15_crisp.launch.py robot_ip:=192.168.1.15
```

## Launching on Real Hardware

If the robot is in **remote control mode** (no access to teach pendant), use the following sequence:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

# 1. Launch dashboard client (for remote power/brake/program control)
ros2 launch ur_robot_driver ur_dashboard_client.launch.py robot_ip:=192.168.1.15 &
sleep 3

# 2. Power on and release brakes
ros2 service call /dashboard_client/power_on std_srvs/srv/Trigger
sleep 10
ros2 service call /dashboard_client/brake_release std_srvs/srv/Trigger
sleep 10

# 3. Kill the standalone dashboard client (the launch file includes its own)
kill %1

# 4. Launch CRISP controllers
ros2 launch ur15_bringup ur15_crisp.launch.py robot_ip:=192.168.1.15
```

If the teach pendant is accessible, simply start the **External Control** program on the pendant, then run step 4.

> **Note**: Do not run a separate `ur_dashboard_client.launch.py` at the same time as `ur15_crisp.launch.py` — the launch file already includes a dashboard client. Having two will cause an RTDE conflict (`speed_slider_mask is currently controlled by another RTDE client`).

## Activating CRISP Controllers

After launch, CRISP controllers are loaded but **inactive** (for safety). Only one effort-claiming controller can be active at a time.

```bash
# Gravity compensation (safest — robot becomes "weightless", like UR freedrive)
ros2 control switch_controllers --activate gravity_compensation --deactivate scaled_joint_trajectory_controller

# Cartesian impedance control (tracks /target_pose with spring-damper behavior)
ros2 control switch_controllers --activate cartesian_impedance_controller --deactivate scaled_joint_trajectory_controller

# Joint impedance control (tracks /target_joint with per-joint stiffness)
ros2 control switch_controllers --activate joint_impedance_controller --deactivate scaled_joint_trajectory_controller

# Return to trajectory control
ros2 control switch_controllers --activate scaled_joint_trajectory_controller --deactivate <active_crisp_controller>
```

## Commanding the Robot

With `cartesian_impedance_controller` active, send target poses via the `/target_pose` topic. Read the current pose from `/current_pose`.

> **Important**: Both topics use the `base_link` frame. The UR driver's `/tcp_pose_broadcaster/pose` uses the `base` frame (180° rotated around Z) — do not mix them.

```bash
# Read current EE pose
ros2 topic echo /current_pose --once

# Send a target pose (continuous publish — needed for DDS discovery)
ros2 topic pub /target_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'base_link'}, pose: {position: {x: 0.0, y: 0.92, z: 0.69}, orientation: {x: -0.168, y: -0.257, z: -0.755, w: 0.579}}}"
# Press Ctrl+C once the robot reaches the target

# Single-shot publish (use -w 1 to wait for subscriber discovery)
ros2 topic pub -w 1 --once /target_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'base_link'}, pose: {position: {x: 0.0, y: 0.92, z: 0.69}, orientation: {x: -0.168, y: -0.257, z: -0.755, w: 0.579}}}"
```

> **Tip**: `--once` without `-w 1` may fail silently due to DDS discovery delay — the message is sent before the controller's subscriber discovers the new publisher.

## Live Parameter Tuning

CRISP uses `generate_parameter_library` — parameters can be adjusted at runtime without restarting:

```bash
# Adjust Cartesian stiffness
ros2 param set /cartesian_impedance_controller task.k_pos_x 800.0

# Adjust damping
ros2 param set /cartesian_impedance_controller task.d_pos_x 250.0

# Adjust nullspace stiffness
ros2 param set /joint_impedance_controller nullspace.stiffness 15.0
```

You can also use `rqt_reconfigure` for a graphical parameter editor:

```bash
ros2 run rqt_reconfigure rqt_reconfigure
```

Select the controller node (e.g., `cartesian_impedance_controller`) from the tree to view and adjust all parameters with sliders and input fields.

## Critical Notes for UR Robots

### 1. Gravity & Coriolis Compensation — MUST be disabled

> **`use_gravity_compensation` and `use_coriolis_compensation` must be `false` for all CRISP controllers on UR robots.**

The UR robot's internal controller **already compensates for gravity and Coriolis forces**. Effort commands sent via `ros2_control` are **additive** — they are applied on top of the UR's internal compensation. If CRISP also computes gravity torques via Pinocchio and sends them, the robot receives **double gravity compensation**, effectively pushing it with ~1× gravity force. This causes dangerous, fast, uncontrolled motion.

This applies to **all UR robots** (UR3, UR5, UR10, UR15, UR16e, UR20, etc.), not just UR15.

### 2. Joint Impedance — Effective gains are multiplied by weights

The joint impedance controller's actual per-joint gains are:

```
effective_stiffness[i] = nullspace.stiffness × nullspace.weights.<joint>.value
effective_damping[i]   = nullspace.damping   × nullspace.weights.<joint>.value
```

**Common mistake**: setting `stiffness: 90` with `weight: 300` gives an effective stiffness of **27,000 Nm/rad** — orders of magnitude too high. For the UR15:

| Joint group | Inertia | Recommended effective K | Recommended effective D |
|---|---|---|---|
| Shoulder / elbow | ~10–30 kg·m² | 30–100 Nm/rad | 30–80 Nm·s/rad |
| Wrist | ~0.1–0.5 kg·m² | 5–20 Nm/rad | 5–15 Nm·s/rad |

Start conservative and increase gradually.

### 3. Damping — Always set explicit values

CRISP's auto-damping formula assumes unit inertia: $d = 2\sqrt{k}$. On a heavy robot like the UR15 (~50 kg + payload), this is **severely underdamped** and causes oscillation/overshoot that can trigger protective stops. Always set explicit `d_pos_*`, `d_rot_*`, and `nullspace.damping` values.

### 4. Output torque filter — Avoid aggressive smoothing

The `filter.output_torque` EMA parameter (range 0–1, lower = more smoothing) introduces **phase lag** in the control loop. Values below ~0.3 can destabilize the controller by delaying the damping response. Keep at 0.5 unless you have a specific reason.

### 5. Safe testing workflow

1. **Always test in mock hardware first**: `use_mock_hardware:=true`
2. **Start with gravity_compensation** (safest mode) before impedance controllers
3. **Use `stop_commands: true`** for dry-run: computes torques but doesn't send them. Monitor logs, then enable live:
   ```bash
   ros2 param set /gravity_compensation stop_commands false
   ```
4. **Keep the e-stop within reach** when testing any torque-based controller
5. **Only one CRISP controller at a time** — deactivate the current one before activating another

## Verified Controller Configurations

The current `ur15_controllers.yaml` has been tested on real UR15 hardware with the following validated settings:

| Controller | Key Parameters | Behavior |
|---|---|---|
| `gravity_compensation` | k=0, d_pos=80, d_rot=15 | Freedrive-like, viscous damping resists drift |
| `cartesian_impedance_controller` | k_pos=600, d_pos=200, k_rot=30, d_rot=30 | Holds EE pose, compliant to perturbation |
| `joint_impedance_controller` | k_eff=50/10 (shoulder/wrist), d_eff=40/8 | Holds joint config, smooth return |

## Using `crisp_py` (Python Interface)

[crisp_py](https://github.com/utiasDSL/crisp_py) provides a high-level Python API for controlling the robot. It is included as a submodule at `src/crisp_py`.

```python
from crisp_py.robot import make_robot
from crisp_py.utils.geometry import Pose
import numpy as np

robot = make_robot("ur")
robot.wait_until_ready()

# Read state
print(robot.end_effector_pose)
print(robot.joint_values)

# Switch to Cartesian impedance control
robot.controller_switcher_client.switch_controller("cartesian_impedance_controller")

# Send a target pose
target = robot.end_effector_pose.copy()
target.position[2] += 0.05  # move up 5 cm
robot.set_target(pose=target)

# Home the robot (uses joint_trajectory_controller)
robot.home()

robot.shutdown()
```

See [scripts/test_crisp_py.py](scripts/test_crisp_py.py) for a complete figure-eight example.

> **Note**: `crisp_py` requires `>=3.11` in its pyproject.toml, but works on Python 3.10 (ROS Humble). The submodule has this constraint relaxed to `>=3.10`.

## RViz Interactive Control

You can control the robot's Cartesian target pose by dragging a 6-DOF interactive marker in RViz.

### Launch

With CRISP controllers already running:

```bash
source install/setup.bash
ros2 launch web_control rviz_control.launch.py
```

This starts RViz with the robot model, TF frames, and an interactive marker at the current end-effector pose.

### Usage

1. **Add the interactive marker display** (first time only):
   - In RViz, click **Add** (bottom-left) → **By topic** → expand `/target_pose_marker` → select **InteractiveMarkers** → click **OK**
   - The 6-DOF marker (red sphere with arrows and rings) appears at the EE pose
2. **Activate `cartesian_impedance_controller`** (if not already active):
   ```bash
   ros2 control switch_controllers --activate cartesian_impedance_controller --deactivate scaled_joint_trajectory_controller
   ```
3. **Switch to Interact mode**: press **I** or click the hand icon in the RViz toolbar
4. **Drag the marker**:
   - **Arrows** (red/green/blue along X/Y/Z) — translate the target pose
   - **Rings** (circles around X/Y/Z) — rotate the target orientation
5. The robot follows the marker in real time via `/target_pose`
6. **Switch to MoveCamera mode**: press **M** to rotate/pan/zoom the 3D view

> **Safety**: Start with small movements. The marker publishes immediately — keep within ~10–20 cm of the current pose to avoid hitting torque limits.

## Alicia Leader Arm Teleoperation

The workspace supports teleoperation of the UR15 using an [Alicia-D leader arm](https://github.com/yizhongzhang1989/Alicia-D-Leader-ROS) (6-DOF). The leader arm's joint angles are read via serial and forwarded as joint targets to the UR15's joint impedance controller.

### Setup

1. Connect the Alicia leader arm via USB (typically `/dev/ttyACM0`)
2. Build the packages (first time):
   ```bash
   colcon build --symlink-install --packages-select alicia_duo_leader_driver alicia_duo_leader_dashboard alicia_teleop
   source install/setup.bash
   ```

### Launch

```bash
# Terminal 1: UR15 driver + controllers
ros2 launch ur15_bringup ur15_crisp.launch.py robot_ip:=192.168.1.15

# Terminal 2: Web dashboard (port 8080)
ros2 launch web_control web_control.launch.py

# Terminal 3: Alicia driver + dashboard (port 8090) + teleop node
ros2 launch alicia_teleop alicia_teleop.launch.py
```

### Controlling the Robot

**Option A — Web dashboard (recommended):**
1. Open `http://localhost:8080` in a browser
2. In the **Joint** tab, click **Enable Joint Controller** (activates `joint_impedance_controller`)
3. Click **Start Teleop** in the Alicia Teleop section
4. Move the leader arm — the UR15 mirrors the joint angles in real time
5. Click **Stop Teleop** to pause

**Option B — Standalone teleop node:**
The `alicia_teleop` launch also starts a standalone teleop node that publishes to `/scaled_joint_trajectory_controller/joint_trajectory`. Activate it by pressing the sync button on the leader arm (`but1 = 0x10`).

### Joint Configuration

The leader arm joint mapping is configured in `config/joint_config.yaml` (auto-created from template on first launch). Edit this file to adjust:
- `direction` — flip joint rotation (`1.0` or `-1.0`)
- `zero_offset` — align leader arm zero with UR15 zero (radians)
- `continuous` — enable angle unwrapping for wrist joints

### Alicia Dashboard

The Alicia leader arm has its own web dashboard at `http://localhost:8090` for monitoring raw joint angles, zero calibration, and torque enable/disable.

## Structure

```
ur15_crisp/
├── src/
│   ├── crisp_controllers/        # [submodule] utiasDSL/crisp_controllers v2.1.0
│   ├── crisp_py/                 # [submodule] utiasDSL/crisp_py v3.4.0
│   └── ur15_bringup/       # UR15-specific bringup (config + launch)
│       ├── config/
│       │   └── ur15_controllers.yaml
│       └── launch/
│           └── ur15_crisp.launch.py
├── scripts/
│   └── test_crisp_py.py          # crisp_py figure-eight test
└── README.md
```

## Quick Reference

| Action | Command |
|---|---|
| Launch (mock) | `ros2 launch ur15_bringup ur15_crisp.launch.py use_mock_hardware:=true` |
| Launch (real) | `ros2 launch ur15_bringup ur15_crisp.launch.py robot_ip:=192.168.1.15` |
| List controllers | `ros2 control list_controllers` |
| List HW interfaces | `ros2 control list_hardware_interfaces` |
| Check joint states | `ros2 topic echo /joint_states --once` |
| Check EE pose | `ros2 topic echo /current_pose --once` |
| Check FT sensor | `ros2 topic echo /ft_data --once` |
| Record rosbag | `ros2 bag record /joint_states /ft_data /current_pose /target_pose -o test_bag` |

## Quick Reference

| Action | Command |
|---|---|
| Power on (remote) | `ros2 service call /dashboard_client/power_on std_srvs/srv/Trigger` |
| Brake release | `ros2 service call /dashboard_client/brake_release std_srvs/srv/Trigger` |
| Check robot mode | `ros2 service call /dashboard_client/get_robot_mode ur_dashboard_msgs/srv/GetRobotMode` |

## References

- [CRISP Controllers](https://github.com/utiasDSL/crisp_controllers) — controller source & docs
- [crisp_py](https://github.com/utiasDSL/crisp_py) — Python interface for CRISP
- [pixi_ur_ros2](https://github.com/lvjonok/pixi_ur_ros2) — community UR + CRISP integration reference
