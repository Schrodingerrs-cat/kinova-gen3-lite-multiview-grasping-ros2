#!/usr/bin/env bash
# Run the whole pipeline as ROS 2 nodes. Needs the driver, MoveIt and the camera up in other terminals (all with RMW_IMPLEMENTATION=rmw_cyclonedds_cpp).
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
cd "$(dirname "$0")"
exec ros2 launch ros2_iface/launch/pick.launch.py extra_args:="${*:---no-delay}"
