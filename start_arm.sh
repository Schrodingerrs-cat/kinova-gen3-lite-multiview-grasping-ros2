#!/usr/bin/env bash
# One-shot: install deps, configure USB link to Kinova Gen3 Lite, launch MoveIt + RViz.
set -e

ROBOT_IP=192.168.1.10
HOST_IP=192.168.1.11

# 1. Install packages if missing
PKGS="ros-humble-kinova-gen3-lite-moveit-config ros-humble-rmw-cyclonedds-cpp ros-humble-moveit-ros-visualization ros-humble-ros2controlcli"
MISSING=""
for p in $PKGS; do
  dpkg -s "$p" >/dev/null 2>&1 || MISSING="$MISSING $p"
done
if [ -n "$MISSING" ]; then
  echo ">> Installing:$MISSING"
  sudo apt update
  sudo apt install -y $MISSING
fi

# 2. Find the arm's RNDIS interface
IFACE=""
for d in /sys/class/net/*; do
  n=$(basename "$d")
  if [ -L "$d/device/driver" ] && [ "$(basename "$(readlink -f "$d/device/driver")")" = "rndis_host" ]; then
    IFACE=$n
  fi
done
if [ -z "$IFACE" ]; then
  echo "!! Arm USB interface not found. Check the micro-B cable and that the arm is powered on, then re-run."
  exit 1
fi
echo ">> Arm interface: $IFACE"

# 3. Assign IP if not set
if ! ip -4 addr show dev "$IFACE" | grep -q "$HOST_IP"; then
  sudo ip addr add "$HOST_IP/24" dev "$IFACE"
fi
sudo ip link set "$IFACE" up

# 4. Check connectivity
if ! ping -c 2 -W 2 "$ROBOT_IP" >/dev/null 2>&1; then
  echo "!! Cannot ping $ROBOT_IP. Check power/cable and re-run."
  exit 1
fi
echo ">> Arm reachable at $ROBOT_IP"

# 5. Launch MoveIt + RViz (real hardware)
echo ">> Launching. Keep the e-stop within reach and the workspace clear."
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
exec ros2 launch kinova_gen3_lite_moveit_config robot.launch.py robot_ip:=$ROBOT_IP
