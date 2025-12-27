# YAM teaching handle state
# Includes trigger and button inputs from the leader arm teaching handle

std_msgs/Header header

# Trigger position (gripper control)
float32 trigger_position     # Raw trigger position (0.0 - 1.0)
                             # 0.0 = fully released, 1.0 = fully squeezed

# Button states
bool button1_pressed         # Top button state
bool button2_pressed         # Bottom button state

# Button events (edge detection)
bool button1_rising_edge     # True on button press
bool button1_falling_edge    # True on button release
bool button2_rising_edge
bool button2_falling_edge
