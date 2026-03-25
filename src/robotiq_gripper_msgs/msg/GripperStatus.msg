# Status message for Robotiq 2F-140 Gripper

# Header for timestamp
std_msgs/Header header

# Gripper activation status
bool is_activated

# Gripper is currently moving
bool is_moving

# Object detected between fingers
bool object_detected

# Fault status
bool fault

# Current position: 0 = fully open, 255 = fully closed
uint8 position

# Current force (if available)
uint8 force

# Raw register values for debugging
uint16[] raw_registers
