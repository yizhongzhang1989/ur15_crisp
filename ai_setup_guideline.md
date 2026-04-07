# AI Setup Guideline — UR15 CRISP Controller Workspace

This file is intended for AI agents. When a user clones this repo and asks an AI
to set up the environment, the AI should follow these instructions.

## Quick Reference

```
Workspace root: the directory containing this file
ROS distro:     Humble (Ubuntu 22.04)
Build system:   colcon
Submodules:     crisp_controllers, crisp_py, alicia_leader, ur_simulator
Modes:          real robot (ur15_crisp.launch.py) or simulation (ur15_crisp_sim.launch.py)
```

## Setup Steps

Run these in order. All commands assume the working directory is the repo root.

### Step 1: Verify prerequisites

```bash
# Check OS
lsb_release -a   # Expect: Ubuntu 22.04

# Check ROS 2
source /opt/ros/humble/setup.bash
ros2 --help       # Should work

# Check tools
which colcon git xacro   # All must exist
```

If ROS 2 Humble is not installed, install it first per
https://docs.ros.org/en/humble/Installation.html

### Step 2: Initialize submodules

```bash
git submodule update --init --recursive
```

Verify:
- `src/crisp_controllers/` should be non-empty (C++ controllers)
- `src/crisp_py/` should be non-empty (Python interface)
- `src/alicia_leader/` should be non-empty (leader arm driver)
- `src/ur_simulator/` should be non-empty (Gazebo simulation)

### Step 3: Install apt dependencies

```bash
sudo apt-get update
sudo apt-get install -y \
  ros-humble-ur-robot-driver \
  ros-humble-pinocchio \
  ros-humble-generate-parameter-library \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-rosbridge-suite \
  ros-humble-ur-description \
  ros-humble-ros-gz \
  ros-humble-gz-ros2-control \
  ros-humble-usb-cam \
  python3-pip \
  python3-flask \
  python3-serial \
  python3-opencv \
  python3-numpy
```

Then install any remaining rosdep keys:

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

### Step 4: Install pip dependencies

```bash
pip3 install --upgrade "pip>=22" "setuptools>=59,<70"
pip3 install flask scipy numpy h5py
pip3 install src/crisp_py
```

Note: setuptools must be `>=59,<70` — versions >=70 break colcon's `--symlink-install`
on Ubuntu 22.04. The pip upgrade is required so `pyproject.toml` is parsed correctly
(the default Ubuntu 22.04 pip installs `crisp_py` as `UNKNOWN`).

### Step 5: Handle ur_simulator submodule

The ur_simulator submodule contains its own nested submodules and packages.
Only `ur_sim_config` needs to be built from source — the rest use system packages.

```bash
# Create symlink so colcon finds ur_sim_config
ln -sf ur_simulator/src/ur_sim_config src/ur_sim_config

# Prevent colcon from scanning inside ur_simulator (uses system packages)
touch src/ur_simulator/COLCON_IGNORE
```

### Step 6: Enable serial devices (for leader arm / gripper)

USB serial devices (e.g. Alicia leader arm, Robotiq gripper) appear as `/dev/ttyACM0`.
The device is owned by the `dialout` group. Add the current user to it:

```bash
sudo usermod -aG dialout $USER
```

Verify:

```bash
groups $USER          # Should include 'dialout'
ls -la /dev/ttyACM0   # Should show group 'dialout', crw-rw----
```

**Note**: The group change requires logging out and back in (or `newgrp dialout`)
to take effect in existing terminal sessions.

### Step 7: Build

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

**Memory note**: The `crisp_controllers` package compiles heavy C++ templates
(Pinocchio + Eigen). If RAM < 8 GB, the compiler may be OOM-killed. Solutions:
- Increase VM RAM to 8+ GB
- Add swap: `sudo fallocate -l 8G /swapfile && sudo chmod 600 /swapfile && sudo mkswap /swapfile && sudo swapon /swapfile`
- Limit parallelism: `MAKEFLAGS="-j1" colcon build --symlink-install --parallel-workers 1`

Build takes 2-5 minutes with 8+ GB RAM.

### Step 8: Verify build

```bash
source install/setup.bash
ros2 pkg list | grep -E "crisp|ur15|web_control|alicia"
```

Expected packages:
- `crisp_controllers`
- `ur15_bringup`
- `ur15_dashboard`
- `web_control`
- `alicia_duo_leader_driver`
- `alicia_duo_leader_dashboard`
- `alicia_teleop`
- `common`
- `camera_node`
- `data_collection`
- `robotiq_2f140_gripper`
- `robotiq_2f140_gripper_web`
- `robotiq_gripper_msgs`
- `vision_tracker_6d`
- `ur_sim_config`

## Launch Modes

### Mode A: Simulation (no real robot needed)

**Terminal 1 — Launch everything:**

```bash
source install/setup.bash
ros2 launch ur15_bringup ur15_crisp_sim.launch.py
```

This starts Gazebo (headless), gravity compensation, and loads CRISP controllers.
CRISP broadcasters are active; CRISP controllers are inactive.

Arguments: `ur_type:=ur15` (default), `gazebo_gui:=true`, `launch_rviz:=true`

**Terminal 2 — Activate a controller:**

```bash
# Activate CRISP gravity compensation (freedrive mode)
ros2 control switch_controllers \
    --deactivate forward_effort_controller \
    --activate crisp_gravity_compensation

# Or activate Cartesian impedance control
ros2 control switch_controllers \
    --deactivate forward_effort_controller \
    --activate cartesian_impedance_controller
```

**Terminal 2 — Launch web_control dashboard (optional):**

```bash
source install/setup.bash
ros2 launch web_control web_control.launch.py
```

Open `http://localhost:8080` for the robot control dashboard.

**Important simulation notes:**
- Controller names differ from real robot: use `crisp_gravity_compensation` (not `gravity_compensation`)
- The sim's `gravity_compensation.py` node occupies the name `gravity_compensation`
- `use_gravity_compensation` and `use_coriolis_compensation` are `true` in sim (Gazebo has no firmware compensation)
- `max_delta_tau` and `filter.output_torque` are relaxed in sim config (no hardware safety concern)

### Mode B: Real robot

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

# If teach pendant is NOT accessible (remote power on):
ros2 launch ur_robot_driver ur_dashboard_client.launch.py robot_ip:=192.168.1.15 &
sleep 3
ros2 service call /dashboard_client/power_on std_srvs/srv/Trigger
sleep 10
ros2 service call /dashboard_client/brake_release std_srvs/srv/Trigger
sleep 10
kill %1

# Launch CRISP
ros2 launch ur15_bringup ur15_crisp.launch.py robot_ip:=192.168.1.15
```

If teach pendant IS accessible: start External Control program on pendant, then
run the launch command directly.

**Activate controllers:**

```bash
ros2 control switch_controllers \
    --activate gravity_compensation \
    --deactivate scaled_joint_trajectory_controller
```

### Mode C: Mock hardware (build verification, no sim or robot)

```bash
ros2 launch ur15_bringup ur15_crisp.launch.py use_mock_hardware:=true
```

## Controller Names

| Controller | Real robot | Simulation |
|---|---|---|
| Freedrive / gravity comp | `gravity_compensation` | `crisp_gravity_compensation` |
| Cartesian impedance | `cartesian_impedance_controller` | `cartesian_impedance_controller` |
| Joint impedance | `joint_impedance_controller` | `joint_impedance_controller` |
| Position trajectory | `scaled_joint_trajectory_controller` | `joint_trajectory_controller` |

## ROS 2 Topics

| Topic | Type | Description |
|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | Joint positions, velocities, efforts |
| `/current_pose` | `geometry_msgs/PoseStamped` | EE pose (base_link frame) |
| `/target_pose` | `geometry_msgs/PoseStamped` | Cartesian target (base_link frame) |
| `/target_joint` | `sensor_msgs/JointState` | Joint target angles |
| `/commanded_torques` | `sensor_msgs/JointState` | CRISP-computed torques |
| `/ft_data` | `geometry_msgs/WrenchStamped` | FT sensor (real robot only) |

## Joint Names

`shoulder_pan_joint`, `shoulder_lift_joint`, `elbow_joint`,
`wrist_1_joint`, `wrist_2_joint`, `wrist_3_joint`

## Web Dashboards

| URL | Package | Description |
|---|---|---|
| `http://localhost:8000` | ur_simulator | Sim 3D viewer (sim mode only) |
| `http://localhost:8080` | web_control | Robot control UI |
| `http://localhost:8085` | ur15_dashboard | UR15 3D viewer |
| `http://localhost:8086` | data_collection | Data collection UI |
| `http://localhost:8088` | robotiq_2f140_gripper_web | Gripper control |
| `http://localhost:8090` | alicia_duo_leader_dashboard | Leader arm status |

## Key Configuration Files

| File | Purpose |
|---|---|
| `config/robot_config.yaml` | Robot IP, tool offset, camera config |
| `config/joint_config.yaml` | Leader arm joint mapping |
| `src/ur15_bringup/config/ur15_controllers_template.yaml` | Real robot controller params |
| `src/ur_simulator/src/ur_sim_config/config/ur_effort_controllers.yaml` | Sim controller params |
| `src/ur_simulator/config/config.yaml` | Simulator config (auto-generated) |

## Architecture

```
Real robot:
  ur_robot_driver → controller_manager → CRISP controllers → /effort interfaces
                                       → joint_state_broadcaster → /joint_states

Simulation:
  Gazebo + gz_ros2_control → controller_manager → CRISP controllers → /effort interfaces
  gravity_compensation.py → forward_effort_controller (when CRISP inactive)
  rosbridge → web dashboard (port 8000)
```

## Critical Safety Notes (real robot only)

1. `use_gravity_compensation: false` — UR firmware compensates internally
2. `use_coriolis_compensation: false` — same reason
3. Only ONE effort controller active at a time
4. Start with `gravity_compensation` before other controllers
5. Keep e-stop within reach

## Troubleshooting

| Problem | Solution |
|---|---|
| OOM during build | Increase VM RAM to 8+ GB or add swap (see Step 6) |
| `gravity_compensation` collapses in sim | Use `crisp_gravity_compensation` (not `gravity_compensation`) |
| Flask not found | `pip3 install flask` or `sudo apt install python3-flask` |
| crisp_py editable install fails | Use `pip3 install src/crisp_py` (no `-e` flag) |
| crisp_py installs as UNKNOWN | Run `pip3 install --upgrade pip setuptools` first |
| Controller not found in sim | Ensure simulator is running with `--control_mode effort` |
| Stale sim processes | `pkill -9 -f "ign gazebo"; pkill -f rosbridge; pkill -f server.py` |
| rosdep update fails | Skip it — cached data is sufficient |
| Gazebo GUI won't start in VM | Headless is default. Set `gazebo_gui: false` in sim config |
| Port already in use | Kill stale processes (see above) |
