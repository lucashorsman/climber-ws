# Command sent to a single arm MCU
# Subscribed by each micro-ROS node

# Actuator position setpoint (m)
# 0.0 = wheel touching surface, <0 = pressing in, >0 = pulling away
float32 actuator_setpoint

# Wheel angular velocity setpoint (rad/s)
float32 wheel_velocity

# Operating mode
# 0 = NORMAL        — track setpoints from ROS 2
# 1 = EMERGENCY_GRIP — override: clamp in immediately (firmware safety)
# 2 = RELEASE        — override: retract actuator fully
# 3 = CLEAR_FAULT    — reset fault latch, return to NORMAL
uint8 mode

uint8 NORMAL         = 0
uint8 EMERGENCY_GRIP = 1
uint8 RELEASE        = 2
uint8 CLEAR_FAULT    = 3
