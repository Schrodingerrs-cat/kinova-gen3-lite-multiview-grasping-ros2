# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

Repo `kinova-gen3-lite-multiview-grasping-ros2` (RBE 595 HW6/HW7): grasping with multi-view eye-in-hand point clouds on a real Kinova Gen3 Lite with a wrist RealSense D405. Runs on ROS 2 Humble (`ros2_kortex` driver, MoveIt, `realsense2_camera`) through `ros2_iface/`; the Kortex-API code was removed (git tag `pre-ros-nodes` has the last version before the ROS node split). Open3D/NumPy/SciPy for point clouds. It must work for ANY object shape/size/colour and several objects anywhere on the table: no dimension or colour assumptions.

There is no build, lint or test suite. Everything is scripts run against the real arm (venv: `source .venv/bin/activate`; arm at 192.168.1.10).

## Commands

Everything is ROS 2 (Humble): `./run_ros.sh` = `ros2 launch ros2_iface/launch/pick.launch.py` (driver, MoveIt and camera must be up in other terminals, all with `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`; `extra_args` for run_pick: `--no-delay`, `--single`). Read-only check: `python3 ros2_iface/smoke_test.py`. Replay a saved run's clouds in RViz without the robot: `.venv/bin/python ros2_iface/viz_node.py --run <run> --object k`, then `rviz2 -d ros2_iface/hw7.rviz`.

Perception offline on saved scans: copy a run's `scan/02_pointclouds/raw_views_npz` into a scratch dir as `scans/scan_k.npz`, then `python perceive.py --target 0` there with `PYTHONPATH` set to this directory (or start `ros2_iface/hw7_nodes.py perception` there and call `/hw7/perceive`).

## Architecture

`run_pick.py` is the orchestrator node. The stages run as nodes (`ros2_iface/hw7_nodes.py`: scan, perception, planner, executor) exposing Trigger services; parameters (`target`, `media_dir`) are set through each node's standard parameter service; the response message is JSON with the stage's printed log. Nodes share the working folder's scratch files (`scans/`, `grasp.json`, `plan.json`, `figures/`, `pointclouds/`, cleared per run and collected into `runs/<ts>/{scan,object_k,video,config}`) and publish clouds/markers on latched `/hw7/*` topics (`ros2_iface/viz_node.py`).

Flow: `/hw7/scan` once → for each object k: `/hw7/perceive` (target=k) → `/hw7/plan` → `/hw7/execute` → put back. Later objects reuse the same scan.

- **`scan.py`**: fixed views from `calibration/table_views.json` (stored as joint angles, executed with no runtime IK). Cloud per view → base frame: `T_base_cam = T_base_tool @ X` (hand-eye `calibration/hand_eye.json`).
- **`perceive.py`**: HW6 steps 1.1–3: trimmed ICP between views, merge/voxel (4 mm), workspace crop, RANSAC table plane, points above plane, DBSCAN clusters (with ghost-cluster rejection of floating clusters and `min_cluster_points`), then centroid → normals toward centroid → antipodal pair under friction cone nearest centroid → approach orthogonal to PCA major axis. Writes `grasp.json`.
- **`grasp_plan.py`**: builds pre-grasp/grasp/lift poses; searches approach tilt about the jaw axis and jaw flip, with multi-seed IK chain (pre-grasp→grasp→lift). Writes `plan.json` (incl. `jaw_flip`).
- **`grasp_exec.py`**: moves to pre-grasp, re-synthesises the grasp on a dense close-range cloud (`refine_grasp`, trusts close-range table height), approaches, closes the gripper with a stall-detecting ramp, lifts, releases (`execution.release: place_back`). `GraspAbort` = object-specific abort that skips to the next object.
- **`common.py`**: ROS-only helpers: `connect()`/`RosBase` (joint states, TF, MoveIt IK via `ros2_iface/robot_ros.py`), `D405` (camera topics), joint moves, gripper ramp, `lift_up`. Global speed scaling via `motion.speed_factor`. `cfg.py` loads `config.yaml` (override path with `HW7_CONFIG`).

## Gotchas

- All tunables are in `config.yaml` (perception filters, grasp params, `execution.max_correction_m`, `safety`, `motion`). `safety.enabled: false` is the user's deliberate setting (disables software bounds/joint-step caps, so IK can accept very large joint swings; the arm's own firmware limits remain).
- NumPy 2.x vs Open3D: pass contiguous float64 arrays; normals are computed with SciPy.
- Fingertip offset 3.5 cm from the tool frame (`calibration/tip_offset_measured.json`); hand-eye from own Park–Martin solver with an ArUco tag.
- The camera node publishes aligned depth only while the raw depth topic is also subscribed, so `ros2_iface/keepalive.py` (started by the launch file) holds those subscriptions.
- The old `viewpoints.planner` block in `config.yaml` is unused. The scan is 10 fixed views from `calibration/table_views.json` (5 straight-down sweep, 5 oblique). Earlier Kortex-only tools (hand-eye calibration, make_views, report) are in `archive/`.
- ROS: every terminal needs the same `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`; the camera node only publishes aligned depth while subscribed, so `run_ros.sh` starts `ros2_iface/keepalive.py` first; the arm's USB link dropping crashes the driver (`RefreshFeedback` timeout).

## Host requirements and failure triage (read before debugging a failed run)

Most failures seen when porting this repo to a second laptop were host problems that look like hardware faults. Details, measurements and
fixes are in `docs/TROUBLESHOOTING.md`; this section is the short version for an assistant.

Required on every machine, in EVERY terminal (identical settings, or nodes cannot see each other; measured 0.0 Hz when they differ):

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOCALHOST_ONLY=1
export CYCLONEDDS_URI='<CycloneDDS><Domain><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>120</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'
```

`tools/ros_env.sh` sets them. After changing them, every ROS terminal must be restarted and `ros2 daemon stop` run. Never edit `~/.bashrc` for the user
without asking; give them the lines.

Triage order when something fails. Do these before touching code or concluding that a cable or the camera is faulty:

1. `camera stalled: no frames for 4 s (3 tries)` with a camera that shows up in `lsusb` and in `ros2 topic hz`: DDS settings (section 3.1). Check
   the environment of the RUNNING processes with `tr '\0' '\n' < /proc/<pid>/environ | grep -E 'ROS_LOCALHOST_ONLY|CYCLONEDDS_URI'`.
2. Repeating `No controller active in LOW_LEVEL_SERVOING mode !`: the controller spawners died, usually the participant-index limit (3.2). Look at
   `~/.ros/log/<newest>/launch.log` for `Failed to find a free participant index`.
3. D405 slow, 10 FPS or `Frames didn't arrived`: `rs-enumerate-devices | grep "Usb Type"` must say 3.x (3.3). A USB disconnect/reconnect pair in
   `journalctl -k` usually means the camera node was restarted, not a cable fault.
4. RViz arm in the wrong pose: the driver is not running, only `start_arm.sh` is (3.5). The orange arm is the MoveIt query goal state. Never press Plan
   or Plan & Execute there.
5. Gripper `contact=False` although the object is held: see below.

Facts about the gripper that are easy to get wrong:
- The Gen3 Lite gripper is underactuated: the motor drives only the proximal finger links, the distal segments are passive. The only channel reported
  for the finger is POSITION (about 990 Hz). Velocity is always exactly 0.0 and effort is nan. Do not build a stall test on velocity or effort.
- The position reading is about 1.00 both for an empty closed gripper and for a thin or small object held at the tips. So `contact = pos < GRIP_EMPTY`
  gives false negatives for objects narrower than about 40 mm. `grasp_exec.py` therefore has a rescue: when the finger check says empty, it lifts and
  `object_in_hand(cam)` decides from the wrist camera (depth between the fingertips below 0.20 m and the table in the lower corners more than 0.08 m
  farther). The camera check is only valid AFTER a lift. Switch: `execution.vision_grasp_check` in `config.yaml`. Runs where the finger check already
  reports contact are unchanged.
- Start runs with the gripper open (`.venv/bin/python tools/open_gripper.py`). A run that started with it closed aborted once with `only 41 table points
  around the object`; the mechanism is unproven but opening first removed it.
- Thin cables (about 10 mm) are outside what the pipeline can grasp, because the close-range cloud keeps only points above `perception.min_height` (12 mm).

Diagnostic tools (`tools/`, none of them moves the arm): `open_gripper.py`, `gripper_probe.py`, `flight_recorder.py` (passive, run it beside a normal run),
`analyze_flight.py`. Do not use `pgrep -f` or `pkill -f` with a pattern that also occurs in your own command line, it kills your own shell; use
`ps -eo pid=,args=` and kill by PID. Do not actuate the gripper or the arm without the user's go-ahead.

Local changes relative to the original repo: `perceive.py` (matplotlib guard), `grasp_exec.py` (vision rescue and a `diagnostic:` line), `config.yaml`
(`vision_grasp_check`), plus `tools/` and `docs/TROUBLESHOOTING.md`.
