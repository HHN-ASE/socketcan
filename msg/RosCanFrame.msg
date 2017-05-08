# Message for CAN frames to be published by the ROS framework

# ROS header file
Header header


# 32 bit CAN_ID + EFF/RTR/ERR flags
uint32 can_id

# frame payload length in byte
uint8 can_dlc

# user data
uint8[8] data
