"""Orchestrator node: scan once, then perceive / plan / execute every object through the /hw7/* ROS 2 services. Start the nodes with ros2 launch."""
import datetime
import glob
import json
import os
import shutil
import sys
import time

import cv2
import rclpy
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from std_srvs.srv import Trigger

from cfg import CFG, HAND_EYE, TIP_OFFSET_RECORD
from common import D405

CAMERA_FLAG = '/tmp/hw7_camera_ready'


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


def clear_scratch(scans=False):
    for f in glob.glob('figures/*.png') + glob.glob('pointclouds/*.ply') + (glob.glob('scans/*') if scans else []) + ['grasp.json', 'plan.json', 'object_cloud.npy', 'objects.json']:
        if os.path.exists(f):
            os.remove(f)


def collect(od):
    """Copy this object's scratch results into its folder."""
    for src, dst in (('figures', f'{od}/03_perception_figures'), ('pointclouds', f'{od}/02_pointclouds/stages')):
        if os.path.isdir(src):
            shutil.copytree(src, dst, dirs_exist_ok=True)
    for f in ('grasp.json', 'plan.json', 'objects.json'):
        if os.path.exists(f):
            shutil.copy(f, f'{od}/04_execution/')


class Stages:
    """Client side of the pipeline services."""

    def __init__(self, node, logf):
        self.node, self.logf = node, logf

    def set_param(self, target, name, value):
        cli = self.node.create_client(SetParameters, f'/{target}/set_parameters')
        if not cli.wait_for_service(timeout_sec=30):
            raise RuntimeError(f'node {target} not found: start the pipeline with ros2 launch ros2_iface/launch/pick.launch.py')
        pv = ParameterValue()
        if isinstance(value, int):
            pv.type, pv.integer_value = ParameterType.PARAMETER_INTEGER, value
        else:
            pv.type, pv.string_value = ParameterType.PARAMETER_STRING, str(value)
        fut = cli.call_async(SetParameters.Request(parameters=[Parameter(name=name, value=pv)]))
        rclpy.spin_until_future_complete(self.node, fut)

    def call(self, service):
        """Call a Trigger service; returns (success, info). The stage's printed output goes to the run log."""
        cli = self.node.create_client(Trigger, service)
        if not cli.wait_for_service(timeout_sec=30):
            raise RuntimeError(f'service {service} not available: start the pipeline with ros2 launch ros2_iface/launch/pick.launch.py')
        fut = cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self.node, fut)
        res = fut.result()
        d = json.loads(res.message)
        self.logf.write(d['log'])
        return res.success, d['info']


def observe_once(st, run):
    """Go to the scan pose and visit the fixed viewpoints ONCE (service /hw7/scan)."""
    sd = f'{run}/scan'
    for s in ('01_observation', '02_pointclouds/raw_views_npz'):
        os.makedirs(f'{sd}/{s}', exist_ok=True)
    clear_scratch(scans=True)
    print('\n== Scan: fixed viewpoints, one point cloud each (done once) ==', flush=True)
    st.set_param('hw7_scan', 'media_dir', os.path.abspath(f'{sd}/01_observation'))
    ok, info = st.call('/hw7/scan')
    if not ok:
        raise RuntimeError(f'scan failed: {info}')
    print(f'  {info}', flush=True)
    shutil.copytree('scans', f'{sd}/02_pointclouds/raw_views_npz', dirs_exist_ok=True)


def pick_object(k, st, run):
    """Pick the k-th object (nearest to the base first) of the single observation and put it back."""
    od = f'{run}/object_{k}'
    for s in ('03_perception_figures', '02_pointclouds', '04_execution'):
        os.makedirs(f'{od}/{s}', exist_ok=True)
    clear_scratch()
    print(f'\n== Object {k + 1}: perception ==', flush=True)
    st.set_param('hw7_perception', 'target', k)
    ok, info = st.call('/hw7/perceive')
    objs = json.load(open('objects.json')) if os.path.exists('objects.json') else None
    res = dict(object=k, n_objects=None if objs is None else len(objs['objects']))
    if not ok:
        collect(od)
        if 'no pickable object' in info or 'no object found above the table' in info:
            return 'NO_MORE_OBJECTS', res
        if 'object too wide' in info:
            return 'SKIPPED: object too wide for the gripper', res
        raise RuntimeError(f'perception failed: {info}')
    res['target'] = objs['objects'][k]['centroid']
    print(f'  target at ({res["target"][0]:.3f}, {res["target"][1]:.3f}) of {res["n_objects"]} object(s)', flush=True)

    print(f'\n== Object {k + 1}: grasp plan check (no motion) ==', flush=True)
    ok, info = st.call('/hw7/plan')
    print(f'  {info}', flush=True)
    if not ok:
        collect(od)
        return 'SKIPPED: plan check failed (grasp not reachable)', res

    print(f'\n== Object {k + 1}: grasp, lift, put back ==', flush=True)
    st.set_param('hw7_executor', 'media_dir', os.path.abspath(f'{od}/04_execution'))
    ok, info = st.call('/hw7/execute')
    collect(od)
    if not ok and not info.startswith('SKIPPED'):
        raise RuntimeError(info)
    return info, res


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
    status, results, picked = 'NOT STARTED', [], 0
    rclpy.init()
    node = rclpy.create_node('hw7_manager')
    st = Stages(node, logf)
    for _ in range(90):
        if os.path.exists(CAMERA_FLAG):
            break
        time.sleep(1)
    cam = D405()
    cam.start_recording(f'{run}/video/wrist_camera.mp4')
    try:
        observe_once(st, run)
        for k in range(max_objects):
            stat, info = pick_object(k, st, run)
            info['status'] = stat
            results.append(info)
            print(f'\n-> Object {k + 1}: {stat}', flush=True)
            if stat == 'NO_MORE_OBJECTS':
                break
            if stat.startswith('PICK'):
                picked += 1
        st.call('/hw7/home')
        status = f'Done: {picked} object(s) picked and put back from one scan'
    except BaseException as exc:
        status = f'Aborted after {picked} object(s): {exc!r}'
        raise
    finally:
        print(f'\n== {status} ==', flush=True)
        cam.close()
        os.makedirs(f'{run}/config/code', exist_ok=True)
        for f in glob.glob('*.py') + glob.glob('ros2_iface/*.py'):
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
