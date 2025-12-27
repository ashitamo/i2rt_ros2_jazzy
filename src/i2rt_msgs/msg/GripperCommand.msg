# Gripper control command
# Supports both position and force control modes

std_msgs/Header header

# Control mode
uint8 POSITION_CONTROL = 0   # Position-based control (default)
uint8 FORCE_CONTROL = 1      # Force-limited control

uint8 mode                   # Control mode (see constants above)

# Position command
float32 position             # Gripper position (0.0 = open, 1.0 = closed)
                             # Normalized to gripper stroke range

# Force limit
float32 max_force            # Maximum gripping force in Newtons
                             # Only used in FORCE_CONTROL mode
                             # Set to 0.0 to disable force limiting

# Timing
float32 duration             # Time to reach target position (seconds)
                             # 0.0 = immediate, >0.0 = smooth trajectory
