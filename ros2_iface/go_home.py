"""Move the arm to the saved home (scan) pose in joint space: python3 ros2_iface/go_home.py   (needs the driver and MoveIt running)."""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from cfg import ROOT, load_json
from common import BaseClient, _wrap180, connect, move_to_joints

home = np.array(load_json(os.path.join(ROOT, 'calibration', 'scan_joints.json')))
with connect() as router:
    base = BaseClient(router)
    q = np.array([a.value for a in base.GetMeasuredJointAngles().joint_angles])
    d = np.abs(_wrap180(home - q))
    print(f'distance to home per joint (deg): {np.round(d, 1)}')
    if d.max() > 0.3:
        dur = max(8.0, float(d.max()) / 10.0)
        print(f'moving to home, {dur:.0f} s ...', flush=True)
        move_to_joints(base, home, duration=dur, max_joint_deg=360)
    q2 = np.array([a.value for a in base.GetMeasuredJointAngles().joint_angles])
    print(f'at home, max joint error {np.abs(_wrap180(home - q2)).max():.2f} deg')
