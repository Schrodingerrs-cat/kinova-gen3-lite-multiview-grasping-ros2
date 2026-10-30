import datetime
import glob
import json
import os
import shutil
import subprocess
import sys
import time

import cv2
import numpy as np

from cfg import CFG, HAND_EYE, TIP_OFFSET_RECORD, load_json
from common import connect, BaseClient, D405, move_joint, move_to_joints, lift_up, go_home
import grasp_exec
import scan

SCAN_POSE = tuple(scan.SCAN_POSE)     # observation pose above the workspace (config.yaml)
NOISE = ('UserWarning', 'warnings.warn', 'ax.scatter')
SUBS = ('01_observation', '02_pointclouds', '03_perception_figures', '04_execution')
EXCL = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'placed_objects.json')


class Tee:
    def __init__(self, *streams):
        self.streams = streams

    def write(self, x):
        for s in self.streams:
            s.write(x)
        return len(x)

    def flush(self):
        for s in self.streams:
            s.flush()


def sub(cmd):
    """Run a pipeline script."""
    res = subprocess.run([sys.executable, '-u'] + cmd, capture_output=True, text=True)
    keep = lambda t: '\n'.join(l for l in t.splitlines() if not any(n in l for n in NOISE))
    out, err = keep(res.stdout), keep(res.stderr)
    print(out)
    if res.returncode != 0:
        print(err[-1500:])
    return res.returncode, out + '\n' + err


def add_zone(x, y, r):
    """Objects inside a zone (already placed / skipped) are ignored."""
    zones = json.load(open(EXCL)) if os.path.exists(EXCL) else []
    zones.append([float(x), float(y), float(r)])
    json.dump(zones, open(EXCL, 'w'))


def clear_scratch(scans=False):
    for f in glob.glob('figures/*.png') + glob.glob('pointclouds/*.ply') + (glob.glob('scans/*') if scans else []) + ['grasp.json', 'plan.json', 'object_cloud.npy', 'objects.json', 'place_slots.json']:
        if os.path.exists(f):
            os.remove(f)


def collect(od):
    """Copy this cycle's scratch results into its object folder."""
    for src, dst in (('figures', f'{od}/03_perception_figures'), ('pointclouds', f'{od}/02_pointclouds/stages')):
        if os.path.isdir(src):
            shutil.copytree(src, dst, dirs_exist_ok=True)
    for f in ('grasp.json', 'plan.json', 'objects.json', 'place_slots.json'):
        if os.path.exists(f):
            shutil.copy(f, f'{od}/04_execution/')


SCAN_JOINTS_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'calibration', 'scan_joints.json')


def to_scan_pose(b):
    """Get to the observation pose from wherever the arm was left."""
    p = b.GetMeasuredCartesianPose()
    print(f'  arm at ({p.x:.3f}, {p.y:.3f}, {p.z:.3f}) m', flush=True)
    if p.z < 0.15:
        print('  lifting straight up first', flush=True)
        try:
            lift_up(b, dz=0.17 - p.z)
        except Exception as exc:
            print(f'  straight lift failed ({type(exc).__name__}), trying other routes', flush=True)
    steps = []
    if os.path.exists(SCAN_JOINTS_FILE):
        sj = load_json(SCAN_JOINTS_FILE)
        steps.append(('joint-space move to the saved scan joints', lambda: move_to_joints(b, sj, duration=8.0, max_joint_deg=200)))
    def rise_then_ik():
        p2 = b.GetMeasuredCartesianPose()
        if p2.z < 0.26:
            for zt in (0.28, 0.24, 0.20, 0.16):
                if zt <= p2.z + 0.02:
                    continue
                try:
                    move_joint(b, p2.x, p2.y, zt, p2.theta_x, p2.theta_y, p2.theta_z, duration=5.0, max_joint_deg=60)
                    break
                except Exception as exc:
                    print(f'  cannot rise to z={zt} ({type(exc).__name__}), trying lower', flush=True)
        move_joint(b, *SCAN_POSE, duration=8.0, max_joint_deg=200)
    steps.append(('rise, then IK move to the scan pose', rise_then_ik))
    steps.append(('arm Home action, then IK move to the scan pose', lambda: (go_home(b), move_joint(b, *SCAN_POSE, duration=8.0, max_joint_deg=200))))
    for name, fn in steps:
        try:
            fn()
            print(f'  scan pose reached ({name})', flush=True)
            break
        except Exception as exc:
            print(f'  {name}: failed ({type(exc).__name__})', flush=True)
    else:
        raise RuntimeError('could not reach the scan pose from the current arm pose')
    if not os.path.exists(SCAN_JOINTS_FILE):
        json.dump([a.value for a in b.GetMeasuredJointAngles().joint_angles], open(SCAN_JOINTS_FILE, 'w'))
        print(f'  saved the scan joints to {os.path.relpath(SCAN_JOINTS_FILE)}', flush=True)


def observe_once(cam, run):
    """Move above the table and take the point clouds of the fixed viewpoints ONCE."""
    sd = f'{run}/scan'
    for s in ('01_observation', '02_pointclouds/raw_views_npz'):
        os.makedirs(f'{sd}/{s}', exist_ok=True)
    clear_scratch(scans=True)
    print('\n== Scan: moving to the scan pose ==', flush=True)
    with connect() as router:
        to_scan_pose(BaseClient(router))
    cv2.imwrite(f'{sd}/01_observation/overview_scan_pose.png', cam.grab(warmup=8)[0])
    print('\n== Scan: fixed viewpoints, one point cloud each (done once) ==', flush=True)
    scan.main(cam=cam, media_dir=f'{sd}/01_observation')
    shutil.copytree('scans', f'{sd}/02_pointclouds/raw_views_npz', dirs_exist_ok=True)


def pick_object(k, cam, run):
    """Pick the k-th object (nearest to the base first) of the single observation and put it back."""
    od = f'{run}/object_{k}'
    for s in ('03_perception_figures', '02_pointclouds', '04_execution'):
        os.makedirs(f'{od}/{s}', exist_ok=True)
    clear_scratch()
    print(f'\n== Object {k + 1}: perception ==', flush=True)
    rc, out = sub(['perceive.py', '--target', str(k)])
    objs = json.load(open('objects.json')) if os.path.exists('objects.json') else None
    info = dict(object=k, n_objects=None if objs is None else len(objs['objects']))
    if 'no pickable object' in out or 'no object found above the table' in out:
        collect(od)
        return 'NO_MORE_OBJECTS', info
    if 'object too wide' in out:
        collect(od)
        return 'SKIPPED: object too wide for the gripper', info
    if rc != 0 or 'saved grasp.json' not in out:
        collect(od)
        raise RuntimeError('perception failed unexpectedly')
    info['target'] = objs['objects'][k]['centroid']

    print(f'\n== Object {k + 1}: grasp plan check (no motion) ==', flush=True)
    rc, out = sub(['grasp_plan.py'])
    if 'PLAN OK' not in out:
        collect(od)
        return 'SKIPPED: plan check failed (grasp not reachable)', info

    print(f'\n== Object {k + 1}: grasp, lift, put back ==', flush=True)
    try:
        status = grasp_exec.main(cam=cam, media_dir=f'{od}/04_execution')
    except grasp_exec.GraspAbort as exc:
        collect(od)
        return f'SKIPPED: {exc}', info
    collect(od)
    return status, info


def main():
    ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    run = f'runs/{ts}'
    for f in ('video', 'config'):
        os.makedirs(f'{run}/{f}', exist_ok=True)
    logf = open(f'{run}/log.txt', 'w')
    sys.stdout = Tee(sys.__stdout__, logf)
    print(f'Run {ts}  (saved to runs/{ts}/)', flush=True)
    delay = float(CFG.get('pipeline', {}).get('start_delay_s', 0))
    if delay > 0 and '--no-delay' not in sys.argv:
        print(f'Starting in {delay:.0f} s. Place the objects and keep clear of the arm.', flush=True)
        for left in range(int(delay), 0, -5):
            print(f'  {left} s', flush=True)
            time.sleep(min(5, left))
    max_objects = 1 if '--single' in sys.argv else int(CFG.get('pick_all', {}).get('max_objects', 1))
    if os.path.exists(EXCL):
        os.remove(EXCL)
    status, results, picked = 'NOT STARTED', [], 0
    cam = D405()
    cam.start_recording(f'{run}/video/wrist_camera.mp4')
    try:
        observe_once(cam, run)
        for k in range(max_objects):
            st, info = pick_object(k, cam, run)
            info['status'] = st
            results.append(info)
            print(f'\n-> Object {k + 1}: {st}', flush=True)
            if st == 'NO_MORE_OBJECTS':
                break
            if st.startswith('PICK'):
                picked += 1
        with connect() as router:
            to_scan_pose(BaseClient(router))
        status = f'Done: {picked} object(s) picked and put back from one scan'
    except BaseException as exc:
        status = f'Aborted after {picked} object(s): {exc!r}'
        raise
    finally:
        print(f'\n== {status} ==', flush=True)
        cam.close()
        os.makedirs(f'{run}/config/code', exist_ok=True)
        for f in glob.glob('*.py'):
            shutil.copy(f, f'{run}/config/code/')
        for f in ('config.yaml', HAND_EYE, TIP_OFFSET_RECORD):
            if os.path.exists(f):
                shutil.copy(f, f'{run}/config/')
        logf.flush()
        json.dump(dict(run=ts, status=status, picked=picked, cycles=results), open(f'{run}/results.json', 'w'), indent=1,
                  default=lambda o: o.tolist() if hasattr(o, 'tolist') else str(o))
        keys = ('== ', 'view ', '[1.', '[2.', '[3', 'cluster', 'target', 'PLAN', 'close-range', 'refined', 'gripper', 'reached', '->', 'COMPLETE')
        lines = [l for l in open(f'{run}/log.txt').read().splitlines() if any(k in l for k in keys)]
        with open(f'{run}/summary.md', 'w') as f:
            f.write(f'# Run {ts}\n\n**Result:** {status}\n\n```\n' + '\n'.join(lines) + '\n```\n')
        print(f'Saved to {run}/', flush=True)


if __name__ == '__main__':
    main()
