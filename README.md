# SocketCAN-Node


## Description

The SocketCAN node uses the SocketCAN interface of the Linux kernel to access the CAN bus. It provides two topics. The topic `ros_to_can` sends messages on the CAN bus and the topic  `can_to_ros` publishes messages from the CAN bus.

### Subscribed Topics

**`/ros_to_can`** (`RosCanFrame`)

Other nodes can publish to this topic and the messages are sent on the CAN bus

### Published Topics

**`/can_to_ros`** (`RosCanFrame`)

This topic publishes all messages received on the CAN bus

### Parameters

**`~interface_name`** (`string`, default: `can0`)

Sets the SocketCAN interface. By default `can0` is used.

## RosCanFrame Definition

    # Message for CAN frames to be published by the ROS framework

    # ROS header file
    Header header


    # 32 bit CAN_ID + EFF/RTR/ERR flags
    uint32 can_id

    # frame payload length in byte
    uint8 can_dlc

    # user data
    uint8[8] data
