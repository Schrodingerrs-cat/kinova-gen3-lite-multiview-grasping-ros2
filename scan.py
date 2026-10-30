"""Visit the fixed viewpoints and save one base-frame point cloud per view (scans/scan_k.npz)."""
import json
import os
import sys
import time

import numpy as np

from cfg import CFG, HAND_EYE, ROOT
from common import connect, BaseClient, D405, move_to_joints, pose_to_matrix, _wrap180

_V = CFG['viewpoints']
SCAN_POSE = _V['scan_pose']
MOVE_S = float(_V.get('move_duration_s', 6.0))
JOINT_DEG_PER_S = 12.0
DEPTH_MIN, DEPTH_MAX = 0.10, 0.70                             # m, valid range from the camera
PIXEL_STRIDE = 2                                              # keep every 2nd pixel


def depth_to_points(depth, color, K):
    h, w = depth.shape
    v, u = np.mgrid[0:h:PIXEL_STRIDE, 0:w:PIXEL_STRIDE]
    z = depth[::PIXEL_STRIDE, ::PIXEL_STRIDE]
    ok = (z > DEPTH_MIN) & (z < DEPTH_MAX)
    x = (u[ok] - K[0, 2]) * z[ok] / K[0, 0]
    y = (v[ok] - K[1, 2]) * z[ok] / K[1, 1]
    rgb = color[::PIXEL_STRIDE, ::PIXEL_STRIDE][ok][:, ::-1] / 255.0
    return np.column_stack([x, y, z[ok]]), rgb


def main(cam=None, media_dir=None, plan_only=False):
    os.makedirs('scans', exist_ok=True)
    X = np.array(json.load(open(HAND_EYE)))
    own_cam = cam is None
    cam = cam or D405()
    names = []
    try:
        with connect() as router:
            base = BaseClient(router)
            fixed = json.load(open(os.path.join(ROOT, 'calibration', 'table_views.json')))
            views, q_park = fixed['views'], fixed['parking_joints']
            for i, v in enumerate(views):
                v['dq'] = float(np.abs(_wrap180(np.array(v['joints']) - np.array(q_park))).max())
                if plan_only:
                    kind = 'sweep %+.0f deg' % v['tilt_deg'] if v.get('kind', 'sweep') == 'sweep' else 'oblique %.0f deg' % v['tilt_deg']
                    print(f'  view {i + 1}/{len(views)} {v["name"]:15s} {kind:16s} tool xyz {np.round(v["tool_xyz"], 3)}  max joint step {v["dq"]:.0f} deg', flush=True)
            if plan_only:
                return views

            def capture(v, k):
                try:
                    T_bt = pose_to_matrix(base.GetMeasuredCartesianPose())
                    color, depth = cam.grab(warmup=6)
                except RuntimeError as exc:                   # camera stalled and could not recover: skip this view
                    print(f'  view {k + 1}/{len(views)} {v["name"]}: skipped, camera error: {exc}', flush=True)
                    return
                if media_dir:
                    import cv2
                    cv2.imwrite(f'{media_dir}/{v["name"]}_color.png', color)
                    np.savez_compressed(f'{media_dir}/{v["name"]}_depth.npz', depth_m=depth.astype(np.float16), T_base_tool=T_bt)
                pts, rgb = depth_to_points(depth, color, cam.K)
                T = T_bt @ X
                np.savez_compressed(f'scans/scan_{len(names)}.npz', xyz=(pts @ T[:3, :3].T + T[:3, 3]).astype(np.float32), rgb=rgb.astype(np.float32),
                                    T_base_cam=T, T_base_tool=T_bt)
                names.append(v['name'])
                if media_dir:
                    pf = f'{media_dir}/camera_poses.json'
                    poses = json.load(open(pf)) if os.path.exists(pf) else []
                    poses.append(dict(index=len(names) - 1, name=v['name'], T_base_cam=T.tolist(), T_base_tool=T_bt.tolist(), tilt_deg=v['tilt_deg'], tilt_axis=v['tilt_axis']))
                    json.dump(poses, open(pf, 'w'), indent=1)
                print(f'  view {len(names)}/{len(views)} {v["name"]}: {len(pts)} points', flush=True)

            try:
                for k, v in enumerate(views):
                    move_to_joints(base, v['joints'], duration=max(MOVE_S, v['dq'] / JOINT_DEG_PER_S), max_joint_deg=360)      # the planner's validated joints: no second IK
                    time.sleep(1.0)
                    capture(v, k)
            finally:
                print('  returning to the scan pose', flush=True)
                move_to_joints(base, q_park, duration=MOVE_S + 3, max_joint_deg=360)
    finally:
        if own_cam:
            cam.close()
    json.dump(names, open('scans/view_names.json', 'w'))
    return names


if __name__ == '__main__':
    main(plan_only='--plan' in sys.argv, media_dir='scan_plan_preview' if '--plan' in sys.argv else None)
