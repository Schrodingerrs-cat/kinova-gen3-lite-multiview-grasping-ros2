"""Automated hand-eye calibration (tool -> D405) from a fixed tag, moving the arm above the table.

For tag fixed in the base frame:  T_base_tool_i @ X @ T_cam_tag_i = T_base_tag  (constant)
Solves X = T_tool_cam with cv2.calibrateHandEye. Writes hand_eye.json, tag_in_base.json, views/*.png.
"""
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))      # project modules (common, cfg)
os.chdir(HERE)                                  # views.json, views/, hand_eye.json, tag_in_base.json live here

import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R

from common import connect, BaseClient, D405, detect_tag, move_to, pose_to_matrix, BOUNDS

# (dx, dy, dz in m ; rx, ry, rz in deg about the TOOL axes) relative to the starting pose
DELTAS = [
    (0, 0, 0, 0, 0, 0),
    (0, 0, 0, 20, 0, 0), (0, 0, 0, -20, 0, 0),
    (0, 0, 0, 0, 20, 0), (0, 0, 0, 0, -20, 0),
    (0, 0, 0, 0, 0, 40), (0, 0, 0, 0, 0, -40),
    (0.04, 0, 0, 15, 15, 25), (-0.04, 0, 0, -15, 15, -25),
    (0, 0.05, 0.05, 15, -15, 35), (0, -0.05, -0.04, -15, 15, -35),
    (0.03, 0.04, 0.07, -15, -15, -30), (-0.03, -0.04, 0.03, 15, 15, 30),
    (0.05, -0.03, 0.02, 20, -20, 0), (-0.05, 0.03, 0.02, -20, 20, 0),
]
SPEED, ROT_SPEED = 0.03, 10.0
MAX_ROT_MISMATCH_DEG = 2.0
N_AVG = 8


def euler_deg(Rm):
    return R.from_matrix(Rm).as_euler('xyz', degrees=True)


def tag_avg(cam):
    """Average tag pose over N_AVG frames; None if not seen in most of them."""
    Ts = []
    for i in range(N_AVG):
        color, _ = cam.grab(warmup=1 if i else 6)
        res = detect_tag(color, cam.K, cam.dist)
        if res:
            Ts.append(res[0])
    if len(Ts) < N_AVG - 2:
        return None, color
    t = np.mean([T[:3, 3] for T in Ts], axis=0)
    rot = R.from_matrix([T[:3, :3] for T in Ts]).mean().as_matrix()
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = rot, t
    return T, color


def park_martin(views):
    """Closed-form AX=XB over all view pairs. A = Tbt_i^-1 Tbt_j, B = Tct_i Tct_j^-1. Returns X = T_tool_cam."""
    As, Bs = [], []
    for i in range(len(views)):
        for j in range(i + 1, len(views)):
            As.append(np.linalg.inv(views[i][0]) @ views[j][0])
            Bs.append(views[i][1] @ np.linalg.inv(views[j][1]))
    M = np.zeros((3, 3))
    for A, B in zip(As, Bs):
        M += np.outer(R.from_matrix(B[:3, :3]).as_rotvec(), R.from_matrix(A[:3, :3]).as_rotvec())
    w, V = np.linalg.eigh(M.T @ M)
    Rx = V @ np.diag(w ** -0.5) @ V.T @ M.T
    if np.linalg.det(Rx) < 0:
        Rx = -Rx
    C = np.vstack([A[:3, :3] - np.eye(3) for A in As])
    d = np.concatenate([Rx @ B[:3, 3] - A[:3, 3] for A, B in zip(As, Bs)])
    tx = np.linalg.lstsq(C, d, rcond=None)[0]
    X = np.eye(4); X[:3, :3], X[:3, 3] = Rx, tx
    return X


def refine(views, X0):
    """Jointly refine X and the constant tag pose: Tbt_i X Tct_i = T_base_tag for all i."""
    from scipy.optimize import least_squares
    tag0 = views[0][0] @ X0 @ views[0][1]

    def unpack(v):
        X = np.eye(4); X[:3, :3], X[:3, 3] = R.from_rotvec(v[:3]).as_matrix(), v[3:6]
        G = np.eye(4); G[:3, :3], G[:3, 3] = R.from_rotvec(v[6:9]).as_matrix(), v[9:12]
        return X, G

    def resid(v):
        X, G = unpack(v)
        out = []
        for Tbt, Tct in views:
            E = np.linalg.inv(G) @ Tbt @ X @ Tct
            out += list(E[:3, 3]) + list(0.1 * R.from_matrix(E[:3, :3]).as_rotvec())   # ~10 cm lever arm
        return np.array(out)
    v0 = np.concatenate([R.from_matrix(X0[:3, :3]).as_rotvec(), X0[:3, 3],
                         R.from_matrix(tag0[:3, :3]).as_rotvec(), tag0[:3, 3]])
    X, G = unpack(least_squares(resid, v0).x)
    return X, G


def main():
    os.makedirs('views', exist_ok=True)
    cam = D405()
    views = []
    with connect() as router:
        base = BaseClient(router)
        p0 = base.GetMeasuredCartesianPose()
        T0 = pose_to_matrix(p0)
        print(f'start pose xyz=({p0.x:.3f},{p0.y:.3f},{p0.z:.3f}) theta=({p0.theta_x:.1f},{p0.theta_y:.1f},{p0.theta_z:.1f})')
        try:
            for k, (dx, dy, dz, rx, ry, rz) in enumerate(DELTAS):
                Rt = T0[:3, :3] @ R.from_euler('xyz', [rx, ry, rz], degrees=True).as_matrix()
                e = euler_deg(Rt)
                pos = T0[:3, 3] + [dx, dy, dz]
                move_to(base, *pos, *e, speed=SPEED, rot_speed=ROT_SPEED)
                import time
                time.sleep(1.0)
                T_bt = pose_to_matrix(base.GetMeasuredCartesianPose())
                mism = np.degrees(np.linalg.norm(R.from_matrix(Rt.T @ T_bt[:3, :3]).as_rotvec()))
                if mism > MAX_ROT_MISMATCH_DEG:
                    raise RuntimeError(f'pose {k}: measured rotation differs from commanded by {mism:.1f} deg - '
                                       'Euler convention wrong, stopping')
                try:
                    T_ct, color = tag_avg(cam)
                except RuntimeError as exc:
                    print(f'pose {k}: camera error ({exc}); stopping sweep with {len(views)} views')
                    break
                cv2.imwrite(f'views/pose_{k:02d}.png', color)
                if T_ct is None:
                    print(f'pose {k:2d}: tag NOT detected, skipped (rot mismatch {mism:.2f} deg)')
                    continue
                views.append((T_bt, T_ct))
                json.dump([[a.tolist(), b.tolist()] for a, b in views], open('views.json', 'w'))
                print(f'pose {k:2d}: ok  tag at {T_ct[:3, 3].round(3)} m, rot mismatch {mism:.2f} deg')
        finally:
            print('returning to start pose')
            move_to(base, *T0[:3, 3], p0.theta_x, p0.theta_y, p0.theta_z, speed=SPEED, rot_speed=ROT_SPEED)
    cam.close()
    json.dump([[a.tolist(), b.tolist()] for a, b in views], open('views.json', 'w'))
    solve(views)


def solve(views):
    if len(views) < 6:
        raise SystemExit(f'only {len(views)} usable views, need >= 6')
    X0 = park_martin(views)
    X, G = refine(views, X0)
    for name, XX in (('closed-form (Park-Martin)', X0), ('refined', X)):
        tags = [v[0] @ XX @ v[1] for v in views]          # T_base_tag per view, should be constant
        pos = np.array([T[:3, 3] for T in tags])
        d = np.linalg.norm(pos - pos.mean(0), axis=1)
        rot = np.degrees(np.linalg.norm(
            [R.from_matrix(T[:3, :3] @ tags[0][:3, :3].T).as_rotvec() for T in tags], axis=1)).max()
        print(f'{name:26s} tag-position spread rms={np.sqrt((d**2).mean())*1000:.1f} mm '
              f'max={d.max()*1000:.1f} mm | rot spread max={rot:.2f} deg | t_tool_cam={XX[:3,3].round(4)}')
    print(f'\nT_tool_cam =\n{X.round(4)}')
    print(f'tag centre in base: {G[:3, 3].round(4)} m, tag normal in base: {G[:3, 2].round(3)}')
    json.dump(X.tolist(), open('hand_eye.json', 'w'))
    json.dump(G.tolist(), open('tag_in_base.json', 'w'))


if __name__ == '__main__':
    import sys
    if '--solve' in sys.argv:
        solve([(np.array(a), np.array(b)) for a, b in json.load(open('views.json'))])
    else:
        main()
