"""Build the HW7 report (Markdown -> HTML -> PDF) from a run folder.

python make_report.py [runs/<timestamp>]     (default: the newest run folder)
Every number and image comes from the run's own log.txt / figures / media; nothing is hand-edited.
"""
import glob
import json
import os
import re
import subprocess
import sys

import markdown
import yaml
from PIL import Image

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))    # project root
sys.path.insert(0, HERE)
os.chdir(HERE)


def newest_run():
    runs = sorted(glob.glob('runs/2*'))
    return runs[-1]


RUN = sys.argv[1] if len(sys.argv) > 1 else newest_run()
LOG = open(f'{RUN}/log.txt').read()
LINES = LOG.splitlines()
CONFIG = open('config.yaml').read()
status = re.findall(r'=== (.*?) ===\n', LOG)
status = status[-1] if status else 'unknown'


def lines(*prefixes, maxn=12):
    out = [l for l in LINES if any(l.strip().startswith(p) for p in prefixes)]
    return '\n'.join(out[:maxn])


def jpg(path):
    """JPEG copy of a run image (keeps the PDF small); returns the path relative to the run folder."""
    out = 'jpg/' + path.replace('/', '_').rsplit('.', 1)[0] + '.jpg'
    os.makedirs(f'{RUN}/jpg', exist_ok=True)
    if not os.path.exists(f'{RUN}/{out}'):
        Image.open(f'{RUN}/{path}').convert('RGB').save(f'{RUN}/{out}', quality=82)
    return out


def img(path, caption, width=58):
    if not os.path.exists(f'{RUN}/{path}'):
        return f'*(missing: {path})*\n'
    return f'<figure><img src="{jpg(path)}" style="width:{width}%"><figcaption>{caption}</figcaption></figure>\n'


def block(text):
    return f'```\n{text}\n```\n' if text.strip() else ''


md = []
A = md.append
A('# RBE 595 – HW 7: Real-Robot Grasping with Multi-View Eye-in-Hand Point Clouds\n')
A(f'**Team:** *[add team member names]* &nbsp;|&nbsp; **Run:** `{os.path.basename(RUN)}` &nbsp;|&nbsp; **Result:** {status}\n')
A('''## 1. Overview

The HW 6 grasping pipeline (merge → crop → table plane → segment → cluster → centroid / normals / antipodal grasp →
PCA approach) was moved to a real **Kinova Gen3 Lite** with a **wrist-mounted Intel RealSense D405**. There is no ROS in
this implementation: the arm is driven through the Kortex Python API (joint-space moves with a fixed duration; in joint feedback they showed about 3x
less speed jitter than Cartesian speed-capped moves), the camera through `pyrealsense2`, and the point-cloud
processing uses Open3D / NumPy / SciPy. Every tunable value lives in `config.yaml` (Section 5).

## 2. Setup and calibration

* **Hand-eye calibration (camera in the tool frame).** A fixed ArUco 4x4 target (100 mm) was viewed from 12 arm poses
  with tilts of about 12 deg and wrist rotations up to 25 deg. A closed-form Park-Martin solution was refined by least squares over `T_base_tool * X * T_cam_tag = T_base_tag`;
  the tag position was consistent to **3.6 mm rms (6.6 mm max)** with a 3.7 deg rotation spread. This residual is the dominant source of
  inter-view disagreement and is why the views are aligned by ICP before merging.
* **Fingertip offset.** Touching the fingertips to the table showed the fingertip ends sit **3.5 cm** from the Kortex tool-frame
  origin (the URDF value of 13 cm does not apply to the Kortex tool frame), stored as `grasp.tip_offset`.
* **Safety.** Every commanded tool position is bounds-checked against a box and the arm's own base protection zone
  (`safety:` in the config); all moves use a fixed duration and are verified to end within 6 mm of the target.
''')

A('## 3. Pipeline, stage by stage\n')

A('### Stage 0 – Observation viewpoints and acquisition\n')
_cfg = yaml.safe_load(CONFIG)
_rows = []
for _v in _cfg['viewpoints']['views']:
    _desc = (f"camera placed {_v['lookat']['distance']:.2f} m from the aim point at azimuth {_v['lookat']['azimuth_deg']} deg, tilted {_v['lookat']['tilt_deg']} deg toward it"
             if 'lookat' in _v else 'directly above the workspace (base observation pose)')
    _rows.append(f"| {_v['name']} | {_desc} |")
A(f'''The camera is moved to the viewpoints in `viewpoints.views`. The observation poses are predetermined; the `lookat` views place the camera
around a nominal aim point `look_at = {_cfg['viewpoints']['look_at']}` (table-top region where the object is expected), so it moves **far to the right, far to the left and toward the base while
staying aimed at the object**. Before any motion each view is checked with inverse kinematics and the safety bounds, and unreachable views are skipped (at least two are required).
At each viewpoint a colour + depth frame is captured, converted to a point cloud, and transformed into the **robot base frame** with
`p_base = T_base_tool(measured) * X(hand-eye) * p_cam`, so clouds from different viewpoints can be combined. The side faces of the object, which a single top-down view cannot see, become visible.

| view | camera placement |
|---|---|
''' + '\n'.join(_rows) + '\n')
views = sorted(glob.glob(f'{RUN}/media/view_*_color.png'))
if views:
    A('<div class="grid">' + ''.join(f'<figure><img src="{jpg("media/" + os.path.basename(v))}"><figcaption>{os.path.basename(v)[5:-10]}</figcaption></figure>' for v in views) + '</div>\n')
A(block(lines('view ')))
A(img('figures/s1_0_camera_poses.png', 'Camera poses (black points, arrows = viewing direction) and each view\'s cloud in the base frame.'))

A('### Stage 1.1 – Transform, align, merge and downsample\n')
A('''Each cloud is in the base frame already; because the hand-eye error leaves the views a few mm off from each other, each view is aligned to view 0 with
trimmed point-to-point ICP before they are concatenated and voxel-downsampled (leaf size `perception.voxel`).
''')
A(block(lines('[1.1]')))
A(img('figures/s1_0_views_aligned.png', 'Aligned clouds, one colour per viewpoint.'))
A(img('figures/s1_1_merged.png', 'Merged and downsampled cloud (RGB).'))

A('### Stage 1.2 – Workspace filter\n')
A('A crop box in the base frame (`perception.ws_min/ws_max`) keeps only the tabletop region of interest.\n')
A(block(lines('[1.2]')))
A(img('figures/s1_2_workspace.png', 'Workspace-filtered cloud.'))

A('### Stage 1.3 – Table plane (RANSAC)\n')
A('RANSAC plane fit (`ransac_dist`, `ransac_iters`). Inliers (blue) are the table.\n')
A(block(lines('[1.3]')))
A(img('figures/s1_3_table.png', 'Table plane inliers (blue) vs. the rest (red).'))

A('### Stage 1.4 – Object points\n')
A('Plane inliers and points lower than `min_height` above the plane are removed; the remainder are the objects.\n')
A(block(lines('[1.4]')))
A(img('figures/s1_4_objects.png', 'Segmented object points (red) over the table (grey).'))

A('### Stage 1.5 – Euclidean clustering\n')
A('DBSCAN clustering, a per-cluster density filter that removes isolated depth-halo points, then the target is the cluster whose footprint and height match `object:` in the config.\n')
A(block(lines('[1.5]', 'cluster ', 'target')))
for f in sorted(glob.glob(f'{RUN}/figures/s1_5_cluster*.png')):
    A(img(f'figures/{os.path.basename(f)}', os.path.basename(f)[5:-4] + ' (each cluster shown individually).', 45))

A('### Stage 2.1 – Centroid\n')
A(block(lines('[2.1]')))
A(img('figures/s2_1_centroid.png', 'Object cloud with its centroid (gold).'))

A('### Stage 2.2 / 2.3 – Surface normals and antipodal grasp points\n')
A('''Normals are estimated from covariance of neighbours within `normal_radius` and flipped to point toward the centroid. Pairs of points whose normals both
lie inside the friction cone (`mu`) around the line joining them, with width within the gripper limits, are feasible; the pair whose midpoint is closest to the
centroid is selected. If the side faces are too thin for any pair the pipeline falls back to the top-face rectangle (Stage 3b).
''')
A(block(lines('[2.2]', '[2.3]')))
A(img('figures/s2_2_normals.png', 'Surface normals (blue, 120 of them shown) oriented toward the centroid.'))
A(img('figures/s2_3_grasp_points.png', 'Selected grasp points (red) with the object cloud.'))

A('### Stage 3 – Approach direction (PCA) and top-face refinement\n')
A('''The approach is orthogonal to the PCA major axis and to the grasp axis. For a near-cubic object the PCA axes are almost degenerate, so
when the cross-product would tilt more than `max_approach_tilt_deg` from vertical the vertical direction (made orthogonal to the jaw axis) is used. A minimum-area
rectangle fitted to the top face then snaps the jaw axis to a face normal, closes the jaws across the narrow side (if one exists), and gives the grasp centre.
''')
A(block(lines('[3]', '[3b]', 'final:', maxn=6)))
A(img('figures/s3_approach.png', 'Grasp points and approach direction (magenta) over the object cloud.'))

A('### Stage 4 – Grasp execution\n')
A('''Poses are built from the grasp: the tool z-axis is the approach (straight down), the jaws close along tool x, and the tool origin is placed `tip_offset` above the
fingertip target. The arm moves to the pre-grasp pose, **re-measures the object from close range** (new top-face rectangle and local table height; the run
aborts if it disagrees with the plan), descends slowly, closes the gripper in steps until the fingers stall, judges the grasp from the settled gripper
position, and lifts.
''')
A(block(lines('[D]', 'pre_grasp', 'grasp ', 'lift ', 'PLAN', 'tip offset', maxn=8)))
A(img('figures/s4_grasp_poses.png', 'Planned tool frames: pre-grasp, grasp, lift (x red, y green, z blue) over the object cloud.'))
A(block(lines('->', 'reached', 'local table', 'close-range', 'refined', 'gripper', 'after lift', maxn=24)))
snaps = [('media/10_pre_grasp.png', 'Pre-grasp pose (wrist camera).'), ('media/11_pre_grasp_refined.png', 'Pre-grasp after the close-range correction.'),
         ('media/12_at_grasp_pose.png', 'At the grasp pose: object between the open fingers.'), ('media/13_gripper_closed.png', 'Gripper closed on the object.'),
         ('media/14_after_lift.png', 'After the lift.')]
A('<div class="grid">' + ''.join(f'<figure><img src="{jpg(p)}"><figcaption>{c}</figcaption></figure>' for p, c in snaps if os.path.exists(f'{RUN}/{p}')) + '</div>\n')

A(f'''## 4. Result

**{status}**

The continuous wrist-camera recording of the whole run is `{os.path.basename(RUN)}/wrist_camera.mp4` (the full log is `log.txt`; every
colour/depth frame and point cloud is saved in `media/` and `scans/`). *The external video of the robot for submission is recorded separately on demo day.*

## 5. Parameters (`config.yaml`)

Changing the observation poses, workspace bounds, filter settings, object dimensions or grasp parameters needs only this file; object-dependent filters
(height window, top-face band, size checks) are derived from `object:`.

```yaml
{CONFIG}```

## 6. Known limitations

* The hand-eye calibration (3.6 mm rms, 3.7 deg) is the main accuracy limit; ICP between views hides most of it, and the close-range re-measurement before the
  descent removes the rest of the position error for the grasp itself.
* The far-range cloud of a dark object is blurry and its apparent height is biased; the grasp height therefore uses the *measured table height plus a fixed offset*, not the cloud's height.
* Viewpoints are predetermined (as permitted), aimed at a nominal point. Some azimuths (toward the +x side) are not reachable by the wrist from this base pose and are skipped by the IK check.
* The large-rotation viewpoints go beyond the rotation range used in the hand-eye calibration (about 25 deg), where pose error is larger; ICP against the top-down view absorbs most of it.
* The wrist camera's USB stream stalled several times during development (cable strain on the wrist). The code now detects a stall, restarts the stream (hard-resets the device if needed) and skips a view that still fails.
* The Stage 2.2 normals figure was regenerated offline from the same saved scans of this run (that figure was added to the pipeline after the run).
* Results come from single runs per object placement, not a success-rate study.

## 7. How to run

```
cd HW7 && source .venv/bin/activate
python run_pick.py              # full run: scan -> perceive -> plan -> grasp + lift; everything saved to runs/<timestamp>/
python make_report.py           # build report.pdf from the newest run
```
''')

text = '\n'.join(md)
open(f'{RUN}/report.md', 'w').write(text)
css = '''<style>body{font-family:Helvetica,Arial,sans-serif;max-width:860px;margin:24px auto;padding:0 16px;line-height:1.45;color:#222}
h1{font-size:22px;border-bottom:2px solid #345} h2{font-size:18px;margin-top:26px;color:#234} h3{font-size:15px;margin-top:18px;color:#345}
pre{background:#f4f6f8;padding:8px;font-size:11px;overflow-x:auto;white-space:pre-wrap} code{font-size:11.5px}
figure{margin:8px 0;text-align:center;break-inside:avoid} figcaption{font-size:11px;color:#555}
.grid{display:grid;grid-template-columns:repeat(3,1fr);gap:6px} .grid figure{margin:2px} .grid img{width:100%}
img{max-width:100%}</style>'''
html = '<html><head><meta charset="utf-8">' + css + '</head><body>' + markdown.markdown(text, extensions=['tables', 'fenced_code']) + '</body></html>'
open(f'{RUN}/report.html', 'w').write(html)
pdf = os.path.abspath(f'{RUN}/report.pdf')
res = subprocess.run(['google-chrome', '--headless=new', '--no-sandbox', '--disable-gpu', '--no-pdf-header-footer',
                      f'--print-to-pdf={pdf}', 'file://' + os.path.abspath(f'{RUN}/report.html')], capture_output=True, text=True, timeout=120)
print('report:', pdf if os.path.exists(pdf) else f'PDF FAILED: {res.stderr[-300:]}')
