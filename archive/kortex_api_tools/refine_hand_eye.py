"""Refine the hand-eye transform so the TABLE comes out as ONE plane in every view (no tag needed).

Uses the scans of a run (scan_k.npz: points in base from the OLD X, plus T_base_tool): with a correct X every view must see the same flat table.
Solves a small correction (rotation + translation, in the tool frame) of X; writes calibration/hand_eye.json (backup: hand_eye.pre_refine.json).
python tools/refine_hand_eye.py runs/<run>/scan/02_pointclouds/raw_views_npz [--write]
"""
import glob
import json
import os
import shutil
import sys

import numpy as np
import open3d as o3d
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as R

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
HE = os.path.join(ROOT, 'calibration', 'hand_eye.json')
X0 = np.array(json.load(open(HE)))
files = sorted(glob.glob(sys.argv[1] + '/scan_*.npz'))
views = []
for f in files:
    d = np.load(f)
    Told, Tbt = d['T_base_cam'], d['T_base_tool']
    pc = (d['xyz'].astype(float) - Told[:3, 3]) @ Told[:3, :3]               # back to the camera frame
    pc = pc[::3]
    q = pc @ (Tbt @ X0)[:3, :3].T + (Tbt @ X0)[:3, 3]
    m = (q[:, 0] > 0.20) & (q[:, 0] < 0.50) & (abs(q[:, 1]) < 0.30) & (q[:, 2] > -0.15) & (q[:, 2] < 0.0)
    p = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(np.ascontiguousarray(q[m])))
    _, inl = p.segment_plane(0.008, 3, 500)
    views.append((pc[m][inl][::4], Tbt))
    print(f'{os.path.basename(f)}: {len(inl)} table points')


def corr(v):                                    # v = [rot x, rot y, -, -, -, trans z] in the tool frame; only these are observable from a flat table
    X = np.eye(4); X[:3, :3] = R.from_rotvec([v[0], v[1], 0.0]).as_matrix(); X[:3, 3] = 0.0
    return X0 @ X


def unit(n2):
    n = np.array([n2[0], n2[1], 1.0]); return n / np.linalg.norm(n)


def resid(v):
    X = corr(v[:6]); n = unit(v[6:8]); d = v[8]
    out = []
    for pc, Tbt in views:
        T = Tbt @ X
        out.append((pc @ T[:3, :3].T + T[:3, 3]) @ n + d)
    return np.concatenate(out + [0.05 * v[:6]])                             # weak prior: stay near the tag-based X


def tilt_z(v):
    X = corr(v); res = []
    for pc, Tbt in views:
        T = Tbt @ X; q = pc @ T[:3, :3].T + T[:3, 3]
        A = np.c_[q[:, 0], q[:, 1], np.ones(len(q))]; a, b, c = np.linalg.lstsq(A, q[:, 2], rcond=None)[0]
        res.append((np.degrees(np.arctan(np.hypot(a, b))), a * 0.35 + c, q[:, 2].std() * 1000))
    return res


print('before: tilt deg / table z at x=.35 / flatness mm per view:', [tuple(round(x, 3) for x in r) for r in tilt_z(np.zeros(6))])
sol = least_squares(resid, np.zeros(9)).x
print('correction rot deg', np.degrees(sol[:3]).round(2), 'trans mm', (sol[3:6] * 1000).round(1))
print('after:  tilt deg / table z at x=.35 / flatness mm per view:', [tuple(round(x, 3) for x in r) for r in tilt_z(sol[:6])])
if '--write' in sys.argv:
    shutil.copy(HE, HE.replace('.json', '.pre_refine.json'))
    json.dump(corr(sol[:6]).tolist(), open(HE, 'w'))
    print('wrote', HE)
