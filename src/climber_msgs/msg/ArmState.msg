# State feedback from a single arm MCU
# Published by each micro-ROS node at ~50 Hz

std_msgs/Header header

# Time-of-flight sensor array — distances to climbing surface (metres)
float32[] tof_distances

# Actuator (prismatic joint) feedback - physical stroke coordinates
float32 actuator_position       # Current position (m). 0 = fully retracted at limit switch, positive = extending toward cylinder
float32 actuator_velocity       # Current velocity (m/s)

# Wheel encoder feedback
float32 wheel_position          # Accumulated angle (rad)
float32 wheel_velocity          # Angular velocity (rad/s)

# Contact / safety state
# 0 = UNKNOWN, 1 = GRIPPING, 2 = RELEASED, 3 = FAULT
uint8 contact_state

uint8 UNKNOWN  = 0
uint8 GRIPPING = 1
uint8 RELEASED = 2
uint8 FAULT    = 3
