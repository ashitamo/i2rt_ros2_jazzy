# Individual DM motor feedback data
# Provides detailed status for each motor in the robot chain

std_msgs/Header header

uint8 motor_id              # Motor ID on CAN bus (1-7 for YAM arm)
string motor_type           # Motor type: DM4340, DM4310, DM3507

# Position and velocity
float32 position            # Joint position in radians
float32 velocity            # Joint velocity in rad/s
float32 torque              # Motor torque in Nm

# Temperatures
float32 mos_temperature     # MOS (MOSFET) temperature in Celsius
float32 rotor_temperature   # Rotor temperature in Celsius

# Error status
uint8 error_code            # Motor error code (0 = no error)
bool has_error              # True if any error present
string error_description    # Human-readable error description

# Error code definitions
uint8 ERROR_NONE = 0
uint8 ERROR_OVER_VOLTAGE = 1
uint8 ERROR_UNDER_VOLTAGE = 2
uint8 ERROR_OVER_CURRENT = 4
uint8 ERROR_MOS_OVER_TEMP = 8
uint8 ERROR_ROTOR_OVER_TEMP = 16
uint8 ERROR_COMMUNICATION = 32
uint8 ERROR_ENCODER = 64
