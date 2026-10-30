# Real-robot grasping with multi-view eye-in-hand point clouds (RBE 595 HW 7)

Kinova Gen3 Lite + wrist-mounted RealSense D405, no ROS. The arm is driven through the Kortex Python API,
the camera through `pyrealsense2`, point clouds with Open3D / NumPy / SciPy.

## Run it

```bash
source .venv/bin/activate
python run_pick.py            # full cycle; everything is saved to runs/<timestamp>/
python run_pick.py --exec-only    # re-run only the grasp stage from the last grasp.json / plan.json
python run_pick.py --no-delay     # skip the start countdown (config: pipeline.start_delay_s)
python scan.py --plan             # read-only: which observation viewpoints are reachable (IK only, no motion)
python tools/check_link.py        # read-only: arm link + camera frame check
```

Everything tunable lives in **`config.yaml`** (stages, search sweep, viewpoints, workspace, filters, grasp parameters,
object size, safety switch). Object-dependent filters are derived from `object:`.

## Pipeline (HW 6 / HW 7), for ANY object

No dimension, shape or colour assumptions: any cluster of points above the table plane is an object, and there can be several.
Stages are listed in `pipeline.stages`.

| stage | module | what it does |
|---|---|---|
| search | `search.py` | yaws the base (joint 1) in steps over the table; locates objects in each view (crop, RANSAC table, points above it, clustering) and centres on the first confident one |
| observe | `scan.py` | >= 2 viewpoints aimed at the object (IK-checked), one point cloud each, transformed into the base frame |
| perceive | `perceive.py` | ICP-align, merge, downsample (1.1) - crop to the table region (1.2) - RANSAC table plane (1.3) - objects = points above it (1.4) - Euclidean clusters (1.5) - for the selected object: centroid (2.1), surface normals oriented to the centroid (2.2), antipodal pair inside the friction cone whose grasp points are closest to the centroid (2.3), approach vector orthogonal to the PCA major axis and the grasp axis (3) |
| plan | `grasp_plan.py` | pre-grasp / grasp / lift poses along that approach vector; picks the least-tilted approach that passes IK; no motion |
| execute | `grasp_exec.py` | pre-grasp, close-range refinement (the same grasp synthesis on the dense close-up cloud), move along the approach, smooth gripper close, lift, place back / drop |

* **Several objects:** every cluster gets a centroid (Step 2.1); `object.selection` (nearest to the base | largest | tallest) picks the one to grasp.
* **No usable antipodal pair** (thin or curved objects): the jaws close across the object's body at its middle (local PCA axis of the point nearest the centroid).
* Shared code: `common.py` (arm, camera, motion, gripper ramp), `cfg.py` (config loader), `run_pick.py` (orchestrator).

## Layout

```
config.yaml            all parameters
run_pick.py            orchestrator            search.py scan.py perceive.py grasp_plan.py grasp_exec.py   pipeline modules
common.py cfg.py       shared code
calibration/           hand_eye.py (+ its data: views.json, views/), hand_eye.json, tip_offset_measured.json, calibration tag
tools/                 check_link.py, make_report.py (report generator; still expects the pre-reorganisation run layout)
docs/                  assignment PDF
runs/<timestamp>/      one folder per run:
    summary.md results.json log.txt
    video/  01_search/  02_observation/  03_pointclouds/ (stages/*.ply, raw_views_npz/)  04_perception_figures/  05_execution/  config/
runs/failed_run_logs/  logs of earlier aborted / stopped runs
```
`scans/`, `figures/`, `pointclouds/` are scratch folders the pipeline refills on every run.

## Calibration

`calibration/hand_eye.py` solves the camera-in-tool transform from `views.json` (arm pose + tag pose per view);
the pipeline reads `calibration/hand_eye.json`. The fingertip offset (3.5 cm from the Kortex tool frame) was measured by touching
the table and is set in `config.yaml` (`grasp.tip_offset`).

## Notes

* `safety.enabled: false` in `config.yaml` bypasses the software workspace box, base-zone height rules, path checks and joint-step caps.
  Set it to `true` to restore them. The arm's own firmware protection zone, joint and force limits are independent.
* The close-range refinement at the pre-grasp pose (and the fingertip-above-table check) stay on regardless: the far-range cloud of an
  object near the edge of the workspace can be off by a couple of cm.
* `common.py` still contains a minimum-jerk speed-command executor (`motion.smooth: true`); it measured no smoother than the arm's
  built-in joint moves and is off by default.
