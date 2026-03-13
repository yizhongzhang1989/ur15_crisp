# UR15 CRISP Controller Workspace

ROS 2 workspace for testing the UR15 robot arm with the CRISP controller.

## Setup

```bash
# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --symlink-install

# Source
source install/setup.bash
```

## Structure

```
ur15_crisp/
├── src/          # ROS 2 packages
├── install/      # (generated) installed packages
├── build/        # (generated) build artifacts
└── log/          # (generated) build/run logs
```
