# Aggregated state of the entire climbing robot
# Published by the main climb controller node

std_msgs/Header header

# Per-arm states (order: NE, NW, SW, SE)
climber_msgs/ArmState[4] arms

# Overall robot state machine
# 0 = IDLE, 1 = APPROACHING, 2 = GRIPPING, 3 = CLIMBING,
# 4 = ORBITING, 5 = STOPPED, 6 = FAULT
uint8 robot_state

uint8 IDLE        = 0
uint8 APPROACHING = 1
uint8 GRIPPING    = 2
uint8 CLIMBING    = 3
uint8 ORBITING    = 4
uint8 STOPPED     = 5
uint8 FAULT       = 6
