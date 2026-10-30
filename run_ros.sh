#!/usr/bin/env bash
# Run the pick pipeline on the ROS 2 stack. Needs (other terminals): kortex_bringup gen3_lite.launch.py, the MoveIt launch (start_arm.sh) and realsense2_camera
# (every terminal: RMW_IMPLEMENTATION=rmw_cyclonedds_cpp).
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp HW6_BACKEND=ros2
cd "$(dirname "$0")"
/usr/bin/python3 ros2_iface/keepalive.py &
KEEP=$!
.venv/bin/python ros2_iface/viz_node.py &      # point clouds + grasp markers for RViz (rviz2 -d ros2_iface/hw7.rviz)
VIZ=$!
trap 'kill $KEEP $VIZ 2>/dev/null' EXIT
echo "waiting for camera streams (colour + aligned depth)..."
for i in $(seq 1 90); do [ -f /tmp/hw6_camera_ready ] && break; sleep 1; done
[ -f /tmp/hw6_camera_ready ] || { echo "camera streams did not start: is realsense2_camera running with align_depth.enable:=true ?"; exit 1; }
echo "camera streams are flowing"
.venv/bin/python run_pick.py "$@"
