# CRISP + Simulator Integration Plan

## Goal

Make `ur15_crisp` work on both the **real UR15 robot** and the **Gazebo Ignition simulator** (`ur_simulator`) with minimal code changes. A single `use_sim` flag selects the backend.

---

## Architecture Overview

### Real Robot (current, unchanged)

```
ur_robot_driver (URPositionHardwareInterface)
  └── controller_manager (500 Hz)
        ├── joint_state_broadcaster          [active]
        ├── scaled_joint_trajectory_controller [active]
        ├── io_and_status_controller         [active]
        ├── force_torque_sensor_broadcaster   [active]
        ├── crisp_pose_broadcaster           [active]
        ├── crisp_twist_broadcaster          [active]
        ├── cartesian_impedance_controller   [inactive]
        ├── joint_impedance_controller       [inactive]
        └── gravity_compensation             [inactive]

Gravity comp: UR firmware (internal) → CRISP use_gravity_compensation = false
```

### Simulator (new)

```
Gazebo Ignition + gz_ros2_control (IgnitionSystem)
  └── controller_manager (500 Hz)
        ├── joint_state_broadcaster          [active]
        ├── joint_trajectory_controller      [inactive]
        ├── crisp_pose_broadcaster           [active]
        ├── crisp_twist_broadcaster          [active]
        ├── cartesian_impedance_controller   [inactive]
        ├── joint_impedance_controller       [inactive]
        └── gravity_compensation             [inactive]

Gravity comp: Pinocchio (inside CRISP) → CRISP use_gravity_compensation = true
```

### Key Difference

| Aspect | Real Robot | Simulator |
|--------|-----------|-----------|
| Hardware interface | `ur_robot_driver` | `gz_ros2_control` (Gazebo) |
| Gravity compensation | UR firmware (always on) | None — CRISP must do it |
| Coriolis compensation | UR firmware (always on) | None — CRISP must do it |
| CRISP `use_gravity_compensation` | `false` | **`true`** |
| CRISP `use_coriolis_compensation` | `false` | **`true`** |
| UR-specific controllers | Yes (scaled_jtc, GPIO, FT, speed_scaling) | No (not available in Gazebo) |
| URDF source | `ur_robot_driver` xacro | `generate_effort_urdf.sh` (patched xacro) |

**No C++ code changes to `crisp_controllers` are needed.** The difference is purely in configuration (YAML parameters) and launch files.

---

## Implementation Steps

### Step 1: Create sim controller config

**File:** `src/ur15_bringup/config/ur15_sim_controllers.yaml`

This is a sim-specific version of `ur15_controllers_template.yaml` with these differences:

1. **Remove UR-driver-only controllers** that don't exist in Gazebo:
   - `io_and_status_controller` (ur_controllers/GPIOController)
   - `speed_scaling_state_broadcaster` (ur_controllers/SpeedScalingStateBroadcaster)
   - `scaled_joint_trajectory_controller` (ur_controllers/ScaledJointTrajectoryController)
   - `force_mode_controller` (ur_controllers/ForceModeController)
   - `freedrive_mode_controller` (ur_controllers/FreedriveModeController)
   - `passthrough_trajectory_controller` (ur_controllers/PassthroughTrajectoryController)
   - `ur_configuration_controller` (ur_controllers/URConfigurationController)
   - `tool_contact_controller` (ur_controllers/ToolContactController)
   - `tcp_pose_broadcaster` (reads from UR's internal `tcp_pose` sensor — not in Gazebo)
   - `force_torque_sensor_broadcaster` (reads from UR's internal `tcp_fts_sensor` — not in Gazebo)

2. **Keep standard ros2_controllers** (available in Gazebo):
   - `joint_state_broadcaster`
   - `joint_trajectory_controller` (replaces `scaled_joint_trajectory_controller`)
   - `forward_effort_controller`
   - `forward_position_controller`
   - `forward_velocity_controller`

3. **Keep CRISP controllers** (loaded as plugins, same as real):
   - `crisp_twist_broadcaster`
   - `crisp_pose_broadcaster`
   - `gravity_compensation`
   - `cartesian_impedance_controller`
   - `joint_impedance_controller`

4. **Flip gravity/Coriolis flags** for all CRISP controllers:
   ```yaml
   use_gravity_compensation: true    # Gazebo has no firmware gravity comp
   use_coriolis_compensation: true   # Gazebo has no firmware Coriolis comp
   ```

5. **Keep everything else the same** (stiffness, damping, filters, joint names, etc.).

### Step 2: Create sim launch file

**File:** `src/ur15_bringup/launch/ur15_crisp_sim.launch.py`

This launch file:

1. **Generates the effort URDF** using `ur_sim_config`'s `generate_effort_urdf.sh`, but passing our `ur15_sim_controllers.yaml` as the controllers file.

2. **Starts Gazebo Ignition** (headless by default, `gazebo_gui` argument available).

3. **Spawns the robot** into Gazebo via `ros_gz_sim/create`.

4. **Starts `robot_state_publisher`** with the generated URDF.

5. **Starts the clock bridge** (`ros_gz_bridge` for `/clock`).

6. **Spawns controllers** into the Gazebo controller_manager:
   - `joint_state_broadcaster` — active
   - `joint_trajectory_controller` — inactive (available for teleop/dashboard)
   - `crisp_pose_broadcaster` — active (after joint_state_broadcaster)
   - `crisp_twist_broadcaster` — active (after joint_state_broadcaster)
   - `gravity_compensation` — inactive (activate manually)
   - `cartesian_impedance_controller` — inactive (activate manually)
   - `joint_impedance_controller` — inactive (activate manually)

7. **Does NOT start** `gravity_compensation.py` (ur_simulator's Python node) — CRISP controllers handle gravity comp internally.

**Launch arguments:**
- `ur_type` (default: `ur15`)
- `gazebo_gui` (default: `false`)
- `launch_rviz` (default: `false`)
- `world_file` (default: `no_ground_collision.sdf` from ur_web_dashboard worlds)

**Usage:**
```bash
# Start simulation with CRISP
ros2 launch ur15_bringup ur15_crisp_sim.launch.py

# With Gazebo GUI
ros2 launch ur15_bringup ur15_crisp_sim.launch.py gazebo_gui:=true

# Different robot type
ros2 launch ur15_bringup ur15_crisp_sim.launch.py ur_type:=ur5e
```

### Step 3: Update `generate_effort_urdf.sh` to accept external controller configs

**File (ur_simulator):** `src/ur_sim_config/urdf/generate_effort_urdf.sh`

Currently this script takes a controllers YAML path as the second argument and passes it to xacro's `simulation_controllers` parameter. This already works for our use case — we pass `ur15_sim_controllers.yaml` from ur15_crisp.

**No changes needed** to this script. It already accepts external controller configs.

### Step 4: Update ur15_bringup's CMakeLists.txt

**File:** `src/ur15_bringup/CMakeLists.txt`

Add `ur_sim_config` as a dependency (for `generate_effort_urdf.sh` and the effort URDF pipeline), and install the sim controller config and sim launch file.

Add to `package.xml`:
```xml
<exec_depend>ur_sim_config</exec_depend>
<exec_depend>ros_gz_sim</exec_depend>
<exec_depend>ros_gz_bridge</exec_depend>
```

### Step 5: Build ur_simulator packages in ur15_crisp workspace

The submodule at `src/ur_simulator/` contains nested packages under `src/ur_simulator/src/`. colcon won't find them by default because they're two levels deep.

**Options (pick one):**

**A. Symlink the needed packages** into the top-level `src/`:
```bash
ln -s ur_simulator/src/ur_sim_config src/ur_sim_config
ln -s ur_simulator/src/ur_description src/ur_description
ln -s ur_simulator/src/ur_simulation_gz src/ur_simulation_gz
```

**B. Add a `colcon.meta` or use `--packages-up-to`** to discover nested paths.

**C. Move ur_simulator submodule to the workspace root** (not in `src/`) and add its `src/` to the colcon paths.

**Recommended: Option A** — simple symlinks, colcon discovers them naturally.

> **Note:** `ur_description` and `ur_simulation_gz` are likely already installed system-wide via `ros-humble-ur-description` and `ros-humble-ros-gz`. Only `ur_sim_config` needs to be built from source. If system packages are present, skip symlinking `ur_description` and `ur_simulation_gz`.

### Step 6: Test the integration

**Test sequence:**

```bash
# Terminal 1: Start simulation
source install/setup.bash
ros2 launch ur15_bringup ur15_crisp_sim.launch.py

# Terminal 2: Verify controllers
ros2 control list_controllers
# Expected:
#   joint_state_broadcaster    [active]
#   crisp_pose_broadcaster     [active]
#   crisp_twist_broadcaster    [active]
#   gravity_compensation       [inactive]
#   cartesian_impedance_controller [inactive]
#   joint_impedance_controller [inactive]
#   joint_trajectory_controller [inactive]

# Terminal 2: Verify topics
ros2 topic list | grep -E "joint_states|current_pose|target_pose"
# Expected: /joint_states, /current_pose, /target_pose

# Terminal 2: Activate gravity comp
ros2 control switch_controllers --activate gravity_compensation

# Robot should hold position under gravity (Pinocchio gravity comp + damping)

# Terminal 2: Activate impedance controller
ros2 control switch_controllers --deactivate gravity_compensation \
                                --activate cartesian_impedance_controller

# Terminal 3: Send a target pose
ros2 topic pub --once /target_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'base_link'}, pose: {position: {x: 0.5, y: 0.0, z: 0.5}, orientation: {w: 1.0}}}"
```

### Step 7: Web dashboard integration

The web dashboards (`web_control`, `ur15_dashboard`) communicate via standard ROS topics:
- `/joint_states` — available in sim
- `/current_pose` — published by `crisp_pose_broadcaster` (works in sim)
- `/target_pose`, `/target_joint` — subscribed by CRISP controllers (works in sim)
- `/computed_torques` — published by CRISP controllers (works in sim)

**Changes needed in `web_control`:**
- The `web_server.py` lists `_CRISP_CONTROLLERS` and switches between them — this works unchanged.
- Controller switching commands are the same.
- The FT sensor topic (`/ft_data`) won't be available in sim (no FT sensor in Gazebo). The dashboard should handle this gracefully (already does — just shows zeros/stale data).

**No code changes needed** for dashboards in the initial integration. FT sensor simulation can be added later.

---

## File Summary

| File | Action | Repo |
|------|--------|------|
| `src/ur15_bringup/config/ur15_sim_controllers.yaml` | **Create** | ur15_crisp |
| `src/ur15_bringup/launch/ur15_crisp_sim.launch.py` | **Create** | ur15_crisp |
| `src/ur15_bringup/CMakeLists.txt` | **Edit** — add sim config install | ur15_crisp |
| `src/ur15_bringup/package.xml` | **Edit** — add sim dependencies | ur15_crisp |
| `src/ur_sim_config` | **Symlink** from `src/ur_simulator/src/ur_sim_config` | ur15_crisp |
| `src/ur_simulator/` | Already added as submodule | ur_simulator |

**Zero changes to:** `crisp_controllers` (C++), `crisp_py`, `web_control`, `ur15_dashboard`, `alicia_teleop`, `common`.

---

## Future Enhancements (not in scope for initial integration)

1. **Simulated FT sensor** — add a Gazebo FT sensor plugin at the wrist and broadcast on `/ft_data`.
2. **Simulated gripper** — add Robotiq 2F-140 model to the simulation URDF.
3. **Simulated camera** — add a Gazebo camera plugin for vision testing.
4. **Tuned sim dynamics** — adjust Gazebo joint friction/damping to better match real UR15 behavior.
5. **Unified launch** — single `ur15_crisp.launch.py` with `use_sim:=true/false` that picks the right backend.
6. **config/robot_config.yaml** — add `simulation: true/false` field for other nodes to detect mode.
