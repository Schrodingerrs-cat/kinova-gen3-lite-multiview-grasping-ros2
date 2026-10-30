# HW7: grasping with multi-view eye-in-hand point clouds on a Kinova Gen3 Lite (ROS 2)

Wrist-mounted RealSense D405, Kinova Gen3 Lite, ROS 2 Humble (`ros2_kortex` driver, MoveIt, `realsense2_camera`). The perception and grasp code is plain
Python (NumPy / SciPy / Open3D); `ros2_iface/` is the hardware layer.

## Pipeline (ROS 2 nodes)

`ros2 launch ros2_iface/launch/pick.launch.py` starts one node per stage plus an orchestrator; stages are `std_srvs/Trigger` services, results go out on latched `/hw7/*` topics.

| node / service | what it does |
|---|---|
| `hw7_scan` `/hw7/scan` | go to the scan pose, visit the 10 fixed viewpoints (5 straight down, yawed -45..+45 deg; 5 oblique), publish each view's cloud on `/hw7/view_cloud` and the accumulated cloud on `/hw7/scan_views` (base frame: `p_base = T_base_tool @ X(hand-eye) @ p_cam`) |
| `hw7_perception` `/hw7/perceive` (param `target`) | `perceive.py` (HW 6 steps 1-3): per-view table alignment, ICP, merge + voxel, crop, RANSAC table plane, clusters, then centroid, normals, antipodal pair inside the friction cone, PCA approach; publishes `/hw7/merged /hw7/table /hw7/objects /hw7/target /hw7/grasp` |
| `hw7_planner` `/hw7/plan` | `grasp_plan.py`: pre-grasp / grasp / lift poses, approach tilt and jaw flip searched with MoveIt IK |
| `hw7_executor` `/hw7/execute`, `/hw7/home` | `grasp_exec.py`: pre-grasp, close-range re-synthesis, descend, close, lift, put back |
| `run_pick.py` (manager) | scans once, then per object: perceive, plan, execute; run folder, video, log, summary |

The service response message is JSON `{"info": ..., "log": ...}`. Works for any object shape / size / colour; parameters are in `config.yaml`.

## Run

Every terminal: `source /opt/ros/humble/setup.bash && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` (all processes must use the same DDS).

```bash
./start_arm.sh                      # (repo parent) USB link + MoveIt/RViz
ros2 launch kortex_bringup gen3_lite.launch.py robot_ip:=192.168.1.10 launch_rviz:=false     # arm driver
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true pointcloud.enable:=true  # camera
./run_ros.sh                        # the pipeline nodes + orchestrator; output in runs/<timestamp>/
```

Keep the e-stop in reach. Read-only check of the ROS side (no motion): `python3 ros2_iface/smoke_test.py`.

## Point clouds in ROS (RViz)

The nodes publish the pipeline's clouds and grasp as latched topics in `base_link`: `/hw7/scan_views`, `/hw7/merged`, `/hw7/table`, `/hw7/objects` (one colour per cluster), `/hw7/target`, and `/hw7/grasp` (centroid, grasp points, approach, pre-grasp/grasp/lift path). The pipeline nodes publish these themselves; open RViz with `rviz2 -d ros2_iface/hw7.rviz`.

No robot needed: replay a saved run with `.venv/bin/python ros2_iface/viz_node.py --run "runs/old runs/20261029_210833" --object 1`.

## Layout

```
run_ros.sh            ros2 launch of the whole pipeline
run_pick.py           orchestrator node
scan.py perceive.py grasp_plan.py grasp_exec.py     stage code used by the nodes
common.py cfg.py      ROS arm/camera/gripper helpers, config loader
ros2_iface/           hw7_nodes.py (the 4 stage nodes), launch/pick.launch.py, robot_ros.py (joint states, TF, FollowJointTrajectory, MoveIt IK/Cartesian path, gripper, camera topics), keepalive.py, viz_node.py + hw7.rviz, smoke_test.py
calibration/          hand_eye.json, table_views.json, scan_joints.json (home pose; old ones commented), tag_in_base.json, tip_offset_measured.json
runs/old runs/<ts>/   one demo run: video, point clouds, figures, per-step images, log
archive/              the earlier Kortex-API version and old calibration (not used)
```

Notes: the camera node publishes aligned depth only while subscribed, so `run_ros.sh` runs `ros2_iface/keepalive.py` first. The arm's own link is USB (RNDIS); if the interface disappears the driver crashes with `RefreshFeedback` timeouts.
