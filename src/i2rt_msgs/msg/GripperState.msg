# Current gripper state feedback
std_msgs/Header header

float32 position             # Current gripper position (0.0 - 1.0)
float32 velocity             # Gripper velocity (normalized units/s)
float32 force                # Estimated gripping force in Newtons
bool is_moving               # True if gripper is in motion
bool is_blocked              # True if gripper detected blocking/clogging
bool is_calibrated           # True if gripper has been calibrated (linear grippers)

string gripper_type          # Gripper type: crank_4310, linear_3507, linear_4310, yam_teaching_handle
