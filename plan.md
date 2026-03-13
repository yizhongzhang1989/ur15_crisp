# Plan: Testing CRISP Controllers on UR15

## References

- **CRISP Controllers**: https://github.com/utiasDSL/crisp_controllers (v2.1.0)
- **CRISP Controllers Demos**: https://github.com/utiasDSL/crisp_controllers_demos
- **CRISP Documentation**: https://utiasdsl.github.io/crisp_controllers/
- **pixi_ur_ros2** (community UR integration): https://github.com/lvjonok/pixi_ur_ros2
- **Paper**: [arXiv:2509.06819](https://arxiv.org/abs/2509.06819)

## System Inventory

| Item | Detail |
|---|---|
| OS | Ubuntu 22.04.5 LTS (Jammy), kernel 6.8.0 |
| ROS | ROS 2 Humble (installed at `/opt/ros/humble`) |
| Python | 3.10.12 |
| Robot | Universal Robots UR15 |
| Robot IP | `192.168.1.15` |
| Host IP | `192.168.1.2` (interface `enp2s0`) |
| UR Driver | `ur_robot_driver 2.12.0` (apt-installed, supports UR15) |
| MoveIt 2 | Installed (not needed — CRISP replaces MoveIt for servoing) |
| ros2_control | Installed (controller_manager, joint_trajectory_controller, etc.) |
| UR15 Launch | `/opt/ros/humble/share/ur_robot_driver/launch/ur15.launch.py` |

## What is CRISP?

**CRISP** (Compliant ROS2 Controllers for Learning-Based Manipulation Policies) is a collection of real-time, C++ torque-based controllers for `ros2_control`. It provides:

- **Cartesian Impedance Controller** — virtual spring-damper in Cartesian space: $\tau_{\text{cmd}} = J^T F_{\text{desired}} + \tau_{\text{nullspace}}$
- **Joint Impedance Controller** — PD control in joint space: $\tau_{\text{cmd}} = K_p e + K_d \dot{e}$
- **Gravity Compensation** — hold position with zero Cartesian stiffness
- **Twist / Pose Broadcasters** — publish end-effector twist and pose from joint states + kinematics

CRISP is robot-agnostic and works with any manipulator exposing an **effort (torque) command interface** — which the UR driver provides via `forward_effort_controller`.

CRISP controllers have been validated on UR5 hardware by the community ([pixi_ur_ros2](https://github.com/lvjonok/pixi_ur_ros2)). UR15 uses the same driver and interface, so the integration pattern is identical.

---

## Phase 0 — Prerequisites & Safety Checks

### 0.1 Verify UR15 network connectivity
```bash
ping -c 3 192.168.1.15
```
- Confirm steady response (< 1 ms typical on direct Ethernet).

### 0.2 Verify URCap "External Control" is installed on the teach pendant
- On the UR15 teach pendant: **Settings → URCaps** → confirm **External Control** URCap is listed and enabled.
- Set **Host IP** inside the URCap to `192.168.1.2` (this machine's IP on `enp2s0`).
- Set **Custom port** to `50002` (default for `ur_robot_driver`).

### 0.3 Confirm UR Polyscope version compatibility
- `ur_robot_driver 2.12.0` supports UR15 natively (launch file exists).
- UR15 is a new-series robot → confirm teach pendant runs a compatible firmware version.

### 0.4 Verify ROS 2 environment
```bash
source /opt/ros/humble/setup.bash
ros2 doctor --report | head -40
```

### 0.5 Check effort command interface availability
The CRISP controllers are **torque-based** and require the `effort` command interface. The UR driver (`ur_robot_driver`) exposes `effort` as a command interface. Verify after launching:
```bash
ros2 control list_hardware_interfaces | grep effort
```
Expected: `shoulder_pan_joint/effort [available] [claimed]` (etc. for all 6 joints).

---

## Phase 1 — Bring Up UR15 Driver (Baseline, No CRISP)

Goal: confirm the stock UR15 driver launches and communicates before adding CRISP controllers.

### 1.1 Launch UR15 driver in mock mode first (no robot motion)
```bash
source /opt/ros/humble/setup.bash
ros2 launch ur_robot_driver ur15.launch.py \
  robot_ip:=192.168.1.15 \
  use_mock_hardware:=true
```
- Verify no launch errors.
- In another terminal:
```bash
ros2 topic list
ros2 topic echo /joint_states --once
ros2 control list_controllers
ros2 control list_hardware_interfaces   # confirm effort interface exists
```

### 1.2 Launch UR15 driver against real hardware
```bash
ros2 launch ur_robot_driver ur15.launch.py \
  robot_ip:=192.168.1.15
```
- On the teach pendant, start the **External Control** program.
- Verify:
  - `/joint_states` publishes real joint positions.
  - `scaled_joint_trajectory_controller` is active.
  - `/ft_data` (force/torque) publishes readings.
```bash
ros2 topic hz /joint_states          # expect ~500 Hz
ros2 topic echo /ft_data --once
ros2 control list_controllers
```

### 1.3 Send a trivial test trajectory (small safe motion)
```bash
ros2 launch ur_robot_driver test_scaled_joint_trajectory_controller.launch.py
```
- Confirm the robot moves as expected.
- **SAFETY**: Ensure e-stop is within reach; robot should be in a clear workspace.

---

## Phase 2 — Set Up Workspace & Clone CRISP Controllers

### 2.1 Target workspace layout
```
ur15_crisp/                              # ← this repo (colcon workspace root)
├── src/
│   ├── crisp_controllers/               # [git submodule] utiasDSL/crisp_controllers
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── crisp_controllers.xml        # pluginlib plugin descriptor
│   │   ├── include/crisp_controllers/
│   │   ├── src/
│   │   └── ...
│   └── crisp_ur15_bringup/              # custom bringup package for UR15
│       ├── package.xml
│       ├── setup.py / setup.cfg
│       ├── config/
│       │   └── ur15_controllers.yaml    # controller_manager config with CRISP
│       └── launch/
│           └── ur15_crisp.launch.py     # launch file
├── install/                             # (generated)
├── build/                               # (generated)
├── log/                                 # (generated)
├── plan.md
└── README.md
```

### 2.2 Clone repo with CRISP controllers submodule
```bash
# Fresh clone (new machine):
git clone --recurse-submodules --shallow-submodules https://github.com/yizhongzhang1989/ur15_crisp.git
cd ur15_crisp

# Or if already cloned without submodules:
git submodule update --init --depth 1
```

> **Note**: `crisp_controllers` is pinned as a git submodule at `src/crisp_controllers` (v2.1.0).
> To update to a newer version: `cd src/crisp_controllers && git pull origin main && cd ../.. && git add src/crisp_controllers && git commit`

### 2.3 Install dependencies
```bash
source /opt/ros/humble/setup.bash
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```
Key dependencies of `crisp_controllers`:
- `pinocchio` (≥ 3.4.0) — rigid-body dynamics (Jacobians, FK, gravity)
- `generate_parameter_library` — dynamic parameter declaration
- `controller_interface`, `hardware_interface`, `pluginlib`
- `realtime_tools`

If `pinocchio` is not available via rosdep on Humble, install via:
```bash
sudo apt install ros-humble-pinocchio
# or via pip: pip3 install pin
# or via conda/pixi from robostack
```

### 2.4 Create the `crisp_ur15_bringup` package
```bash
cd ~/Documents/ur15_crisp/src
ros2 pkg create crisp_ur15_bringup \
  --build-type ament_python \
  --dependencies ur_robot_driver crisp_controllers
```

---

## Phase 3 — Configure CRISP Controllers for UR15

### 3.1 Create `config/ur15_controllers.yaml`

This file extends the stock UR controller config with CRISP controller definitions.
Based on the [pixi_ur_ros2 config](https://github.com/lvjonok/pixi_ur_ros2/blob/master/src/crisp_ur_bringup/config/ur_controllers.yaml):

```yaml
/**:
  controller_manager:
    ros__parameters:
      update_rate: 500  # Hz — matches UR15 update rate

      # --- Standard UR controllers ---
      joint_state_broadcaster:
        type: joint_state_broadcaster/JointStateBroadcaster
      io_and_status_controller:
        type: ur_controllers/GPIOController
      speed_scaling_state_broadcaster:
        type: ur_controllers/SpeedScalingStateBroadcaster
      force_torque_sensor_broadcaster:
        type: force_torque_sensor_broadcaster/ForceTorqueSensorBroadcaster
      tcp_pose_broadcaster:
        type: pose_broadcaster/PoseBroadcaster
      joint_trajectory_controller:
        type: joint_trajectory_controller/JointTrajectoryController
      scaled_joint_trajectory_controller:
        type: ur_controllers/ScaledJointTrajectoryController
      forward_velocity_controller:
        type: velocity_controllers/JointGroupVelocityController
      forward_effort_controller:
        type: effort_controllers/JointGroupEffortController
      forward_position_controller:
        type: position_controllers/JointGroupPositionController
      force_mode_controller:
        type: ur_controllers/ForceModeController
      freedrive_mode_controller:
        type: ur_controllers/FreedriveModeController
      passthrough_trajectory_controller:
        type: ur_controllers/PassthroughTrajectoryController
      ur_configuration_controller:
        type: ur_controllers/URConfigurationController
      tool_contact_controller:
        type: ur_controllers/ToolContactController

      # --- CRISP controllers ---
      twist_broadcaster:
        type: crisp_controllers/TwistBroadcaster
      pose_broadcaster:
        type: crisp_controllers/PoseBroadcaster
      gravity_compensation:
        type: crisp_controllers/CartesianController
      cartesian_impedance_controller:
        type: crisp_controllers/CartesianController
      joint_impedance_controller:
        type: crisp_controllers/CartesianController

      enforce_command_limits: false

# --- Standard UR controller parameters ---
speed_scaling_state_broadcaster:
  ros__parameters:
    state_publish_rate: 100.0
    tf_prefix: ""

io_and_status_controller:
  ros__parameters:
    tf_prefix: ""

ur_configuration_controller:
  ros__parameters:
    tf_prefix: ""

force_torque_sensor_broadcaster:
  ros__parameters:
    sensor_name: tcp_fts_sensor
    state_interface_names:
      - force.x
      - force.y
      - force.z
      - torque.x
      - torque.y
      - torque.z
    frame_id: tool0
    topic_name: ft_data

tcp_pose_broadcaster:
  ros__parameters:
    frame_id: base
    pose_name: tcp_pose
    tf:
      child_frame_id: tool0_controller

joint_trajectory_controller:
  ros__parameters:
    joints:
      - shoulder_pan_joint
      - shoulder_lift_joint
      - elbow_joint
      - wrist_1_joint
      - wrist_2_joint
      - wrist_3_joint
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
    state_publish_rate: 100.0
    action_monitor_rate: 20.0
    allow_partial_joints_goal: false
    constraints:
      stopped_velocity_tolerance: 0.2
      goal_time: 0.0

scaled_joint_trajectory_controller:
  ros__parameters:
    joints:
      - shoulder_pan_joint
      - shoulder_lift_joint
      - elbow_joint
      - wrist_1_joint
      - wrist_2_joint
      - wrist_3_joint
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
    state_publish_rate: 100.0
    action_monitor_rate: 20.0
    allow_partial_joints_goal: false
    constraints:
      stopped_velocity_tolerance: 0.2
      goal_time: 0.0
    speed_scaling_interface_name: speed_scaling/speed_scaling_factor

forward_effort_controller:
  ros__parameters:
    joints:
      - shoulder_pan_joint
      - shoulder_lift_joint
      - elbow_joint
      - wrist_1_joint
      - wrist_2_joint
      - wrist_3_joint
    interface_name: effort

forward_velocity_controller:
  ros__parameters:
    joints:
      - shoulder_pan_joint
      - shoulder_lift_joint
      - elbow_joint
      - wrist_1_joint
      - wrist_2_joint
      - wrist_3_joint
    interface_name: velocity

forward_position_controller:
  ros__parameters:
    joints:
      - shoulder_pan_joint
      - shoulder_lift_joint
      - elbow_joint
      - wrist_1_joint
      - wrist_2_joint
      - wrist_3_joint
    interface_name: position

# --- CRISP Broadcaster parameters ---
twist_broadcaster:
  ros__parameters:
    joints:
      - shoulder_pan_joint
      - shoulder_lift_joint
      - elbow_joint
      - wrist_1_joint
      - wrist_2_joint
      - wrist_3_joint
    end_effector_frame: tool0
    base_frame: base_link

pose_broadcaster:
  ros__parameters:
    joints:
      - shoulder_pan_joint
      - shoulder_lift_joint
      - elbow_joint
      - wrist_1_joint
      - wrist_2_joint
      - wrist_3_joint
    end_effector_frame: tool0
    base_frame: base_link

# --- CRISP Controller parameters ---
gravity_compensation:
  ros__parameters:
    joints:
      - shoulder_pan_joint
      - shoulder_lift_joint
      - elbow_joint
      - wrist_1_joint
      - wrist_2_joint
      - wrist_3_joint
    end_effector_frame: tool0
    base_frame: base_link
    task:
      k_pos_x: 0.0
      k_pos_y: 0.0
      k_pos_z: 0.0
      k_rot_x: 30.0
      k_rot_y: 30.0
      k_rot_z: 30.0
    nullspace:
      stiffness: 0.0
    use_friction: true
    use_coriolis_compensation: true
    use_local_jacobian: true

cartesian_impedance_controller:
  ros__parameters:
    joints:
      - shoulder_pan_joint
      - shoulder_lift_joint
      - elbow_joint
      - wrist_1_joint
      - wrist_2_joint
      - wrist_3_joint
    end_effector_frame: tool0
    base_frame: base_link
    filter:
      q: 0.5
      dq: 0.5
      output_torque: 0.5
    task:
      k_pos_x: 600.0
      k_pos_y: 600.0
      k_pos_z: 600.0
      k_rot_x: 0.0
      k_rot_y: 0.0
      k_rot_z: 0.0
    nullspace:
      stiffness: 0.0
      damping: 0.0
    use_friction: false
    use_coriolis_compensation: true
    use_local_jacobian: true
    limit_torques: false

joint_impedance_controller:
  ros__parameters:
    joints:
      - shoulder_pan_joint
      - shoulder_lift_joint
      - elbow_joint
      - wrist_1_joint
      - wrist_2_joint
      - wrist_3_joint
    end_effector_frame: tool0
    base_frame: base_link
    task:
      k_pos_x: 0.0
      k_pos_y: 0.0
      k_pos_z: 0.0
      k_rot_x: 0.0
      k_rot_y: 0.0
      k_rot_z: 0.0
    max_delta_tau: 0.5
    nullspace:
      stiffness: 90.0
      projector_type: none
      damping: 35.0
      max_tau: 1200.0
      regularization: 1.0e-06
      weights:
        shoulder_pan_joint.value: 300.0
        shoulder_lift_joint.value: 300.0
        elbow_joint.value: 300.0
        wrist_1_joint.value: 300.0
        wrist_2_joint.value: 300.0
        wrist_3_joint.value: 300.0
    use_friction: false
    use_coriolis_compensation: false
    use_local_jacobian: true
    limit_error: true
    limit_torques: true
```

### 3.2 Create the launch file `launch/ur15_crisp.launch.py`

This launch file:
1. Includes the stock `ur_control.launch.py` with `ur_type:=ur15`.
2. Overrides `controllers_file` to point to our CRISP-aware config.
3. Spawns CRISP broadcasters (active) and controllers (inactive, for manual activation).

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_ip_arg = DeclareLaunchArgument(
        "robot_ip", default_value="192.168.1.15",
        description="IP address of the UR15 robot.",
    )
    use_mock_hardware_arg = DeclareLaunchArgument(
        "use_mock_hardware", default_value="false",
        description="Start robot with mock hardware.",
    )
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz", default_value="false",
        description="Start RViz for visualization.",
    )

    robot_ip = LaunchConfiguration("robot_ip")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    use_rviz = LaunchConfiguration("use_rviz")

    controllers_file = PathJoinSubstitution(
        [FindPackageShare("crisp_ur15_bringup"), "config", "ur15_controllers.yaml"]
    )

    ur_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur_robot_driver"), "launch", "ur_control.launch.py"]
            )
        ),
        launch_arguments={
            "ur_type": "ur15",
            "robot_ip": robot_ip,
            "use_mock_hardware": use_mock_hardware,
            "controllers_file": controllers_file,
            "launch_rviz": use_rviz,
            "initial_joint_controller": "scaled_joint_trajectory_controller",
            "headless_mode": "true",
        }.items(),
    )

    # Spawn CRISP broadcasters (active immediately)
    crisp_broadcasters_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "--controller-manager", "/controller_manager",
            "twist_broadcaster", "pose_broadcaster",
        ],
        output="screen",
    )

    # Spawn CRISP controllers (inactive — activate manually for safety)
    crisp_controllers_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "--controller-manager", "/controller_manager",
            "--inactive",
            "gravity_compensation",
            "cartesian_impedance_controller",
            "joint_impedance_controller",
        ],
        output="screen",
    )

    return LaunchDescription([
        robot_ip_arg,
        use_mock_hardware_arg,
        use_rviz_arg,
        ur_control,
        crisp_broadcasters_spawner,
        crisp_controllers_spawner,
    ])
```

---

## Phase 4 — Build the Workspace

### 4.1 Build CRISP controllers
```bash
cd ~/Documents/ur15_crisp
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
source install/setup.bash
```

> **OOM Warning**: `crisp_controllers` compiles heavy pinocchio/Eigen templates.
> On machines with < 16 GB RAM, limit parallelism: `colcon build -j1 --cmake-args ...`

### 4.2 Verify CRISP plugin is discoverable
```bash
# Check the plugin is registered
ros2 pkg list | grep crisp
ros2 pkg prefix crisp_controllers
```

### 4.3 (Optional) Run CRISP unit tests
```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test --packages-select crisp_controllers
colcon test-result --verbose
```

---

## Phase 5 — Integration Test with Mock Hardware

### 5.1 Launch with mock hardware
```bash
source ~/Documents/ur15_crisp/install/setup.bash
ros2 launch crisp_ur15_bringup ur15_crisp.launch.py \
  robot_ip:=192.168.1.15 \
  use_mock_hardware:=true
```

### 5.2 Verify controllers are loaded
```bash
ros2 control list_controllers
```
Expected output:
```
joint_state_broadcaster          [active]
scaled_joint_trajectory_controller [active]
twist_broadcaster                [active]
pose_broadcaster                 [active]
gravity_compensation             [inactive]
cartesian_impedance_controller   [inactive]
joint_impedance_controller       [inactive]
...
```

### 5.3 Activate a CRISP controller (mock)
To switch from the default trajectory controller to a CRISP controller:
```bash
# Deactivate the trajectory controller first, then activate CRISP
ros2 control switch_controllers \
  --activate cartesian_impedance_controller \
  --deactivate scaled_joint_trajectory_controller
```

### 5.4 Verify CRISP topics
```bash
ros2 topic list | grep -E "target_pose|target_joint|twist_broadcaster|pose_broadcaster"
ros2 topic echo /cartesian_impedance_controller/target_pose --once
```

### 5.5 Send a test Cartesian target
```bash
ros2 topic pub /cartesian_impedance_controller/target_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'base_link'}, pose: {position: {x: 0.4, y: 0.0, z: 0.5}, orientation: {w: 1.0}}}" --once
```
- Monitor `/joint_states` for commanded motion in mock mode.

---

## Phase 6 — Integration with UR15 Real Hardware

> ⚠️ **SAFETY FIRST**: Keep speed scaling low, stay near the e-stop, and test in a clear workspace.

### 6.1 Launch with real hardware
```bash
# Terminal 1: Launch UR15 driver + CRISP controllers
source ~/Documents/ur15_crisp/install/setup.bash
ros2 launch crisp_ur15_bringup ur15_crisp.launch.py \
  robot_ip:=192.168.1.15

# Terminal 2 (on teach pendant): Start the External Control program
```

### 6.2 Verify hardware interfaces
```bash
ros2 control list_hardware_interfaces | grep effort
```
- Confirm all 6 joints have `effort` command interface `[available]`.

### 6.3 Test gravity compensation first (safest CRISP mode)
Gravity compensation holds the current pose with zero translational stiffness — the robot becomes "weightless" and can be moved by hand.
```bash
ros2 control switch_controllers \
  --activate gravity_compensation \
  --deactivate scaled_joint_trajectory_controller
```
- The robot should hold position softly.
- Gently push the end-effector — it should yield and hold the new position.

### 6.4 Test Cartesian impedance controller
```bash
# First deactivate gravity_compensation, then activate cartesian
ros2 control switch_controllers \
  --activate cartesian_impedance_controller \
  --deactivate gravity_compensation
```
- The robot will track `target_pose` with the configured stiffness (600 N/m in x,y,z by default).
- Send a target close to the current pose:
```bash
# Read current pose first
ros2 topic echo /pose_broadcaster/pose --once

# Then publish a target ~2 mm away
ros2 topic pub /cartesian_impedance_controller/target_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'base_link'}, pose: {position: {x: 0.402, y: 0.0, z: 0.5}, orientation: {w: 1.0}}}" --once
```

### 6.5 Test joint impedance controller
```bash
ros2 control switch_controllers \
  --activate joint_impedance_controller \
  --deactivate cartesian_impedance_controller
```
- Operates in joint space with configurable per-joint stiffness/damping.

### 6.6 Compliance test
- With `cartesian_impedance_controller` active, gently push the end-effector.
- The robot should yield compliantly and return to the target pose when released.

---

## Phase 7 — Tuning & Parameter Adjustment

CRISP controllers use `generate_parameter_library`, so parameters can be modified **live** without restarting:

### 7.1 Adjust Cartesian stiffness
```bash
ros2 param set /cartesian_impedance_controller task.k_pos_x 800.0
ros2 param set /cartesian_impedance_controller task.k_pos_y 800.0
ros2 param set /cartesian_impedance_controller task.k_pos_z 800.0
```

### 7.2 Adjust damping
Damping is typically auto-computed from stiffness. If manual override is needed:
```bash
ros2 param set /cartesian_impedance_controller task.k_rot_x 40.0
```

### 7.3 Enable/disable extras
```bash
# Enable friction compensation
ros2 param set /cartesian_impedance_controller use_friction true

# Enable coriolis compensation
ros2 param set /cartesian_impedance_controller use_coriolis_compensation true
```

### 7.4 Nullspace control
When using Cartesian impedance control, the nullspace term regulates joint posture:
```bash
ros2 param set /cartesian_impedance_controller nullspace.stiffness 10.0
```

---

## Phase 8 — Logging, Monitoring & Data Collection

### 8.1 Record a rosbag for analysis
```bash
ros2 bag record /joint_states /ft_data \
  /cartesian_impedance_controller/target_pose \
  /pose_broadcaster/pose \
  /twist_broadcaster/twist \
  /tf -o crisp_ur15_test_bag
```

### 8.2 Real-time monitoring
```bash
# Joint states plot
ros2 run rqt_plot rqt_plot /joint_states/position[0] /joint_states/velocity[0]

# Force/torque
ros2 run rqt_plot rqt_plot /ft_data/wrench/force/x /ft_data/wrench/force/y /ft_data/wrench/force/z

# End-effector pose
ros2 topic echo /pose_broadcaster/pose
```

### 8.3 Latency and jitter check
```bash
ros2 topic delay /joint_states
ros2 topic hz /joint_states
```
- Expect ~500 Hz, < 2 ms jitter for stable torque control.

---

## Phase 9 — Advanced Tests

### 9.1 Trajectory tracking
- Use `crisp_py` Python interface to send smooth Cartesian trajectories:
```bash
pip3 install crisp-py   # or clone https://github.com/utiasDSL/crisp_py
```
```python
from crisp_py import Robot
robot = Robot(controller_type="cartesian_impedance_controller")
robot.set_target_pose([0.4, 0.0, 0.5, 1.0, 0.0, 0.0, 0.0])  # [x,y,z,qw,qx,qy,qz]
```

### 9.2 Contact tasks
- Move toward a rigid surface with force setpoint.
- Verify stable contact force regulation (no oscillation/chatter).

### 9.3 Stress / edge-case tests
- Singularity behavior near workspace limits.
- Behavior on e-stop and protective stop recovery.
- Speed scaling interaction (UR speed slider < 100%).

### 9.4 (Optional) crisp_gym integration
For data collection and policy deployment:
```bash
pip3 install crisp-gym
```
- Record trajectories in LeRobot format.
- Deploy learning-based policies.

---

## Quick-Reference Commands

| Action | Command |
|---|---|
| Launch UR15 + CRISP (mock) | `ros2 launch crisp_ur15_bringup ur15_crisp.launch.py use_mock_hardware:=true` |
| Launch UR15 + CRISP (real) | `ros2 launch crisp_ur15_bringup ur15_crisp.launch.py` |
| List controllers | `ros2 control list_controllers` |
| List HW interfaces | `ros2 control list_hardware_interfaces` |
| Activate Cartesian impedance | `ros2 control switch_controllers --activate cartesian_impedance_controller --deactivate scaled_joint_trajectory_controller` |
| Activate joint impedance | `ros2 control switch_controllers --activate joint_impedance_controller --deactivate scaled_joint_trajectory_controller` |
| Activate gravity comp | `ros2 control switch_controllers --activate gravity_compensation --deactivate scaled_joint_trajectory_controller` |
| Return to trajectory control | `ros2 control switch_controllers --activate scaled_joint_trajectory_controller --deactivate <active_crisp_ctrl>` |
| Check joint states | `ros2 topic echo /joint_states --once` |
| Check FT sensor | `ros2 topic echo /ft_data --once` |
| Check EE pose | `ros2 topic echo /pose_broadcaster/pose --once` |
| Check EE twist | `ros2 topic echo /twist_broadcaster/twist --once` |
| Record bag | `ros2 bag record /joint_states /ft_data /pose_broadcaster/pose -o test_bag` |
| Dashboard client | `ros2 launch ur_robot_driver ur_dashboard_client.launch.py robot_ip:=192.168.1.15` |
| Power on robot | `ros2 service call /dashboard_client/power_on std_srvs/srv/Trigger` |
| Brake release | `ros2 service call /dashboard_client/brake_release std_srvs/srv/Trigger` |
| Play program | `ros2 service call /dashboard_client/play std_srvs/srv/Trigger` |

---

## Risk Mitigation

| Risk | Mitigation |
|---|---|
| Unstable torque gains → violent motion | Start with gravity_compensation (zero stiffness). Gradually increase gains. Test in mock mode first. |
| `pinocchio` version mismatch on Humble | Use `ros-humble-pinocchio` if available, or build from source / use robostack. |
| Effort interface not claimed | Ensure no other effort-claiming controller is active when switching to CRISP. Only one controller can claim effort at a time. |
| CRISP not building on Humble | CRISP uses compile-time macros for ROS version handling. Check `main` branch supports Humble (CI badge exists for Humble). |
| Network latency / packet loss | Use direct Ethernet (no switch). Monitor with `ros2 topic delay`. |
| Singularities | CRISP includes Jacobian regularization. Tune `nullspace.regularization` parameter. |
| E-stop recovery | After e-stop, controllers are deactivated. Re-activate after robot is powered back on. |
| UR15 joint limits differ from UR5 | Gains in config should be validated for UR15's higher payload (15 kg) and torque limits (433/204/70 Nm). |
