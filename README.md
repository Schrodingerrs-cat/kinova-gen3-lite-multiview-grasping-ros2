# HW7: grasping with multi-view eye-in-hand point clouds on a Kinova Gen3 Lite (ROS 2)

Wrist-mounted RealSense D405, Kinova Gen3 Lite, ROS 2 Humble (`ros2_kortex` driver, MoveIt, `realsense2_camera`). The perception and grasp code is plain
Python (NumPy / SciPy / Open3D); `ros2_iface/` is the hardware layer.

## Pipeline

1. **Scan once** - the arm visits 10 fixed viewpoints (5 straight down, yawed -45..+45 deg about the base; 5 oblique, 25 deg off vertical) stored as joint angles in `calibration/table_views.json`. Each view's depth image becomes a point cloud in the base frame: `p_base = T_base_tool @ X(hand-eye) @ p_cam`.
2. **Perceive** (`perceive.py`, HW 6 steps 1-3) - per-view table alignment, ICP, merge + voxel downsample, workspace crop, RANSAC table plane, points above it, Euclidean clusters (ghost rejection), then for the chosen object: centroid, normals toward the centroid, antipodal pair inside the friction cone nearest the centroid, approach vector orthogonal to the PCA major axis and the grasp axis.
3. **Plan** (`grasp_plan.py`) - pre-grasp / grasp / lift poses, approach tilt and jaw flip searched with IK.
4. **Execute** (`grasp_exec.py`) - pre-grasp, close-range re-synthesis on a dense cloud, descend, gripper close with contact check, lift, put the object back where it was.
5. The loop picks every object from the one scan (`run_pick.py`).

Works for any object shape / size / colour; there are no dimension or colour assumptions. Parameters are in `config.yaml`.

## Run (ROS 2)

Every terminal: `source /opt/ros/humble/setup.bash && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` (all processes must use the same DDS).

```bash
./start_arm.sh                      # (repo parent) USB link + MoveIt/RViz
ros2 launch kortex_bringup gen3_lite.launch.py robot_ip:=192.168.1.10 launch_rviz:=false     # arm driver
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true pointcloud.enable:=true  # camera
./run_ros.sh --no-delay             # full pick; output in runs/<timestamp>/
```

Keep the e-stop in reach. Read-only check of the ROS side (no motion): `python3 ros2_iface/smoke_test.py`.

## Point clouds in ROS (RViz)

`ros2_iface/viz_node.py` publishes the pipeline's clouds and grasp as latched topics in `base_link`: `/hw7/scan_views`, `/hw7/merged`, `/hw7/table`, `/hw7/objects` (one colour per cluster), `/hw7/target`, and `/hw7/grasp` (centroid, grasp points, approach, pre-grasp/grasp/lift path). `run_ros.sh` starts it for a live run; open RViz with `rviz2 -d ros2_iface/hw7.rviz`.

No robot needed: replay a saved run with `.venv/bin/python ros2_iface/viz_node.py --run "runs/old runs/20261029_210833" --object 1`.

## Layout

```
run_ros.sh            starts the camera keepalive and run_pick.py with the ROS backend
run_pick.py           orchestrator: scan once, then perceive/plan/execute per object
scan.py perceive.py grasp_plan.py grasp_exec.py     pipeline stages
common.py cfg.py      shared helpers, config loader (HW6_BACKEND=ros2 switches the hardware layer)
ros2_iface/           robot_ros.py (joint states, TF, FollowJointTrajectory, MoveIt IK/Cartesian path, gripper, camera topics), backend.py, keepalive.py, viz_node.py + hw7.rviz (point clouds in RViz), smoke_test.py
calibration/          hand_eye.json, table_views.json, scan_joints.json (home pose; old ones commented), tag_in_base.json, tip_offset_measured.json
runs/old runs/<ts>/   one demo run: video, point clouds, figures, per-step images, log
archive/              the earlier Kortex-API version and old calibration (not used)
```

Notes: the camera node publishes aligned depth only while subscribed, so `run_ros.sh` runs `ros2_iface/keepalive.py` first. The arm's own link is USB (RNDIS); if the interface disappears the driver crashes with `RefreshFeedback` timeouts.
