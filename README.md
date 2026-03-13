# UR15 CRISP Controller Workspace

ROS 2 workspace for testing the UR15 robot arm with [CRISP controllers](https://github.com/utiasDSL/crisp_controllers) — compliant, torque-based Cartesian and joint-space controllers for `ros2_control`.

## Prerequisites (New Machine)

- **Ubuntu 22.04** with **ROS 2 Humble** installed (`ros-humble-desktop`)
- **UR robot driver**: `sudo apt install -y ros-humble-ur-robot-driver`
- **16+ GB RAM** recommended (CRISP compiles heavy C++ templates; use `-j1` if RAM < 16 GB)
- **Network**: Direct Ethernet to UR15 at `192.168.1.15` (host IP: `192.168.1.2` on same subnet)
- **UR15 teach pendant**: "External Control" URCap installed, Host IP set to `192.168.1.2`, port `50002`

## Quick Start (New Machine)

```bash
# 1. Clone with submodules (shallow to skip large media history)
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

After launch, CRISP controllers are loaded but **inactive** (for safety). Activate one:

```bash
# Gravity compensation (safest — robot becomes "weightless")
ros2 control switch_controllers --activate gravity_compensation --deactivate scaled_joint_trajectory_controller

# Cartesian impedance control (tracks target_pose)
ros2 control switch_controllers --activate cartesian_impedance_controller --deactivate scaled_joint_trajectory_controller

# Joint impedance control (tracks target_joint)
ros2 control switch_controllers --activate joint_impedance_controller --deactivate scaled_joint_trajectory_controller

# Return to trajectory control
ros2 control switch_controllers --activate scaled_joint_trajectory_controller --deactivate <active_crisp_controller>
```

## Structure

```
ur15_crisp/
├── src/
│   ├── crisp_controllers/        # [submodule] utiasDSL/crisp_controllers
│   └── crisp_ur15_bringup/       # UR15-specific bringup (config + launch)
│       ├── config/
│       │   └── ur15_controllers.yaml
│       └── launch/
│           └── ur15_crisp.launch.py
├── plan.md                       # Detailed testing plan
└── README.md
```

## References

- [CRISP Controllers](https://github.com/utiasDSL/crisp_controllers) — controller source
- [CRISP Documentation](https://utiasdsl.github.io/crisp_controllers/) — guides & API
- [pixi_ur_ros2](https://github.com/lvjonok/pixi_ur_ros2) — community UR integration reference
- [Plan](plan.md) — full phase-by-phase testing plan
