# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

RBE 595 HW6/HW7: grasping with multi-view eye-in-hand point clouds on a real Kinova Gen3 Lite (wrist RealSense D405, **no ROS**). Kortex Python API for the arm, `pyrealsense2` for the camera, Open3D/NumPy/SciPy for point clouds. It must work for ANY object shape/size/colour and several objects anywhere on the table: no dimension or colour assumptions.

There is no build, lint or test suite. Everything is scripts run against the real arm (venv: `source .venv/bin/activate`; arm at 192.168.1.10, admin/admin).

## Commands

```bash
python run_pick.py --no-delay      # full session: scan ONCE, then pick each object and put it back (runs/<timestamp>/)
python run_pick.py --exec-only     # re-run only grasp stage from last grasp.json/plan.json
python run_pick.py --single        # one object only
python scan.py --plan              # read-only: list fixed sweep views (no motion)
python tools/make_views.py         # rebuild calibration/table_views.json (needs arm connection for IK, does not move)
python tools/check_link.py         # read-only arm link + camera check
python perceive.py --target 0      # perception only on scans/scan_*.npz (no robot); target = k-th cluster, nearest-to-base first
```

**`run_pick.py` starts a REAL robot run immediately (even with `--help`).** Run long jobs in the background (`run_in_background`, tail the log). Kill by exact PID, not `pkill -f` (it kills your own shell).

Offline perception testing: copy a run's `scan/02_pointclouds/raw_views_npz` into a scratch dir as `scans/scan_k.npz` plus `config.yaml`, then run `perceive.py` there with `PYTHONPATH` set to this directory.

## Architecture

`run_pick.py` is the orchestrator; each stage is also a standalone module communicating through scratch files (`scans/`, `grasp.json`, `plan.json`, `figures/`, `pointclouds/`) that are cleared per run and then collected into `runs/<ts>/{scan,object_k,video,config}`.

Flow: `observe_once` (go to scan pose, sweep, capture) → for each object k: `perceive.py --target k` → `grasp_plan.py` (no motion, IK check) → `grasp_exec.py` → put back. Scan happens once; later objects reuse the same clouds.

- **`scan.py`**: fixed sweep views from `calibration/table_views.json` (scan pose yawed about base z by +45/0/−45°, i.e. pure joint-1 turns, stored as joint angles and executed with no runtime IK). Cloud per view → base frame: `T_base_cam = T_base_tool @ X` (hand-eye `calibration/hand_eye.json`, from `calibration/hand_eye.py`).
- **`perceive.py`**: HW6 steps 1.1–3: trimmed ICP between views, merge/voxel (4 mm), workspace crop, RANSAC table plane, points above plane, DBSCAN clusters (with ghost-cluster rejection of floating clusters and `min_cluster_points`), then centroid → normals toward centroid → antipodal pair under friction cone nearest centroid → approach orthogonal to PCA major axis. Writes `grasp.json`.
- **`grasp_plan.py`**: builds pre-grasp/grasp/lift poses; searches approach tilt about the jaw axis and jaw flip, with multi-seed IK chain (pre-grasp→grasp→lift). Writes `plan.json` (incl. `jaw_flip`).
- **`grasp_exec.py`**: moves to pre-grasp, re-synthesises the grasp on a dense close-range cloud (`refine_grasp`, trusts close-range table height), approaches, closes the gripper with a stall-detecting ramp, lifts, releases (`execution.release: place_back`). `GraspAbort` = object-specific abort that skips to the next object.
- **`common.py`**: arm connection, D405 wrapper, joint/Cartesian moves, gripper ramp, `lift_up`. Global speed scaling via `motion.speed_factor`. `cfg.py` loads `config.yaml` (override path with `HW7_CONFIG`).

## Gotchas

- All tunables are in `config.yaml` (perception filters, grasp params, `execution.max_correction_m`, `safety`, `motion`). `safety.enabled: false` is the user's deliberate setting (disables software bounds/joint-step caps, so IK can accept very large joint swings; the arm's own firmware limits remain).
- NumPy 2.x vs Open3D: pass contiguous float64 arrays; normals are computed with SciPy.
- Fingertip offset 3.5 cm from the tool frame (`calibration/tip_offset_measured.json`); hand-eye from own Park–Martin solver with an ArUco tag.
- Camera (D405) can stall; `D405.grab` restarts it. Kortex abort code 55 = manual stop/link drop; `SESSION_NOT_IN_CONTROL` when another session holds control; `METHOD_FAILED` = arm can't generate the trajectory (IK seed-dependent).
- `place.py` slot search and the old `viewpoints.planner` block in `config.yaml` are unused by the current flow; `README.md` and `tools/make_report.py` still describe the older search/tilted-view pipeline and old run layout.
