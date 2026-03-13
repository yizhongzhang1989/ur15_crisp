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

# 5. Launch with mock hardware (verify build works)
ros2 launch crisp_ur15_bringup ur15_crisp.launch.py use_mock_hardware:=true

# 6. Launch with real UR15 (start External Control on teach pendant first)
ros2 launch crisp_ur15_bringup ur15_crisp.launch.py robot_ip:=192.168.1.15
```

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

## Structure

```
ur15_crisp/
├── src/
│   ├── crisp_controllers/        # [submodule] utiasDSL/crisp_controllers v2.1.0
│   └── crisp_ur15_bringup/       # UR15-specific bringup (config + launch)
│       ├── config/
│       │   └── ur15_controllers.yaml
│       └── launch/
│           └── ur15_crisp.launch.py
└── README.md
```

## Quick Reference

| Action | Command |
|---|---|
| Launch (mock) | `ros2 launch crisp_ur15_bringup ur15_crisp.launch.py use_mock_hardware:=true` |
| Launch (real) | `ros2 launch crisp_ur15_bringup ur15_crisp.launch.py robot_ip:=192.168.1.15` |
| List controllers | `ros2 control list_controllers` |
| List HW interfaces | `ros2 control list_hardware_interfaces` |
| Check joint states | `ros2 topic echo /joint_states --once` |
| Check EE pose | `ros2 topic echo /current_pose --once` |
| Check FT sensor | `ros2 topic echo /ft_data --once` |
| Record rosbag | `ros2 bag record /joint_states /ft_data /current_pose /target_pose -o test_bag` |

## References

- [CRISP Controllers](https://github.com/utiasDSL/crisp_controllers) — controller source & docs
- [pixi_ur_ros2](https://github.com/lvjonok/pixi_ur_ros2) — community UR + CRISP integration reference
