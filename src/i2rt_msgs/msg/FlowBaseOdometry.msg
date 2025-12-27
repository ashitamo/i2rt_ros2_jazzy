# Flow Base odometry with additional diagnostics
# Extends standard nav_msgs/Odometry with motor-specific data

std_msgs/Header header

# Standard odometry
geometry_msgs/Pose pose              # Position and orientation
geometry_msgs/Twist twist            # Linear and angular velocities

# Odometry covariance (row-major 6x6)
float64[36] pose_covariance
float64[36] twist_covariance

# Flow Base specific diagnostics
float32[8] motor_temperatures        # Temperature for each of 8 motors (steering + drive)
float32[4] caster_angles             # Current caster steering angles (radians)
float32[4] wheel_velocities          # Current wheel velocities (rad/s)

bool odometry_valid                  # False if odometry has drifted significantly
                                     # or casters are misaligned
