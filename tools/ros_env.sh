# Source this in EVERY terminal that touches the robot: ./start_arm.sh, the driver, the camera, ./run_ros.sh, rviz2 and
# the scripts in tools/. All ROS 2 terminals must use identical DDS settings, otherwise nodes silently fail to see each
# other. Easiest: put these lines in ~/.bashrc instead (then close all ROS terminals and run `ros2 daemon stop`).
# Why these lines exist: docs/TROUBLESHOOTING.md sections 3.1 and 3.2.
#
#   source tools/ros_env.sh
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOCALHOST_ONLY=1
export CYCLONEDDS_URI='<CycloneDDS><Domain><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>120</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'
