"""Build the FIXED observation viewpoints for the table and save them as joint configurations (calibration/table_views.json).

Each view is the scan (parking) pose translated to another place over the table and tilted by one angle about ONE horizontal base axis, so the camera
looks at the table from a different side: no change of wrist yaw, no combined rotations. The tool poses are converted to joint angles once with the arm's
IK (seeded from the parking joints) and checked: reachable, small joint steps, away from the joint limits. At run time scan.py only moves to those joints.

python tools/make_views.py        # prints every view's checks and writes calibration/table_views.json (needs the arm for IK only; it does not move)
"""
import json
import os
import sys

import numpy as np
from scipy.spatial.transform import Rotation as R

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from cfg import CFG, ROOT, load_json
from common import connect, BaseClient, Base_pb2, _wrap180, joint_margin_deg

SCAN_POSE = CFG['viewpoints']['scan_pose']
OUT = os.path.join(ROOT, 'calibration', 'table_views.json')
Z = SCAN_POSE[2]

# Left-to-right sweep: the scan pose swung about the vertical axis through the robot base by +45 / 0 / -45 deg. Position and camera yaw turn together,
# so every view looks straight down at the table from a different side.
SWEEP_DEG = [('left_45', 45.0), ('left_22', 22.5), ('center', 0.0), ('right_22', -22.5), ('right_45', -45.0)]


def swept_pose(az_deg):
    Rz = R.from_euler('z', az_deg, degrees=True)
    xyz = Rz.apply(np.array(SCAN_POSE[:3]))
    eul = (Rz * R.from_euler('xyz', SCAN_POSE[3:], degrees=True)).as_euler('xyz', degrees=True)
    return tuple(float(v) for v in xyz), eul


# Oblique views: the camera sits on a ring around the table centre C and looks at C from OBLIQUE_TILT_DEG off vertical, from several sides, so the sides
# of the objects (not only their tops) are seen. Same wrist roll as the parking pose, the tool turned minimally to aim the camera at C.
TABLE_CENTRE = np.array([0.42, 0.0])
OBLIQUE_TILT_DEG = 25.0
OBLIQUE = [('obl_near', 180.0), ('obl_near_left', 135.0), ('obl_left', 90.0), ('obl_right', -90.0), ('obl_near_right', -135.0), ('obl_far', 0.0)]


def oblique_pose(phi_deg, cam_z, t_cam, table_z):
    h = cam_z - table_z
    r = h * np.tan(np.radians(OBLIQUE_TILT_DEG))
    cam = np.array([TABLE_CENTRE[0] + r * np.cos(np.radians(phi_deg)), TABLE_CENTRE[1] + r * np.sin(np.radians(phi_deg)), cam_z])
    aim = np.array([TABLE_CENTRE[0], TABLE_CENTRE[1], table_z])
    v1 = (aim - cam) / np.linalg.norm(aim - cam)                       # camera optical axis (= tool z) must point here
    R0 = R.from_euler('xyz', SCAN_POSE[3:], degrees=True).as_matrix()
    v0 = R0[:, 2]
    axis = np.cross(v0, v1)
    ang = np.arctan2(np.linalg.norm(axis), float(v0 @ v1))
    Rn = (R.from_rotvec(axis / np.linalg.norm(axis) * ang) * R.from_matrix(R0)).as_matrix()
    tool = cam - Rn @ t_cam                                           # camera origin = tool origin + R t
    return tuple(float(v) for v in tool), R.from_matrix(Rn).as_euler('xyz', degrees=True)


def main():
    q_park = np.array(load_json(os.path.join(ROOT, 'calibration', 'scan_joints.json')), dtype=float)
    views = []
    with connect() as router:
        base = BaseClient(router)
        print(f'parking joints: {np.round(q_park, 1)}')
        t_cam = np.array(json.load(open(os.path.join(ROOT, 'calibration', 'hand_eye.json'))))[:3, 3]
        p0 = base.GetMeasuredCartesianPose()
        cam_z = float(SCAN_POSE[2] + 0.083)                         # camera height above the base plane at the straight-down views
        table_z = float(CFG['viewpoints'].get('table_z', -0.05))
        jobs = [(n, a, 'sweep') for n, a in SWEEP_DEG] + [(n, p, 'oblique') for n, p in OBLIQUE]
        for name, az, kind in jobs:
            xyz, eul = swept_pose(az) if kind == 'sweep' else oblique_pose(az, cam_z, t_cam, table_z)
            q = None
            for seed in [q_park] + [v['joints'] for v in views]:
                d = Base_pb2.IKData(); c = d.cartesian_pose
                c.x, c.y, c.z, c.theta_x, c.theta_y, c.theta_z = *xyz, *eul
                for v in seed:
                    d.guess.joint_angles.add().value = float(v)
                try:
                    q = np.array([a.value for a in base.ComputeInverseKinematics(d).joint_angles])
                    break
                except Exception:
                    q = None
            if q is None:
                print(f'  {name:9s}: IK FAILED')
                continue
            ang, axis = az, 'z'
            step = np.abs(_wrap180(q - q_park))
            ok = step.max() <= (80 if kind == 'sweep' else 120) and joint_margin_deg(q) >= 6
            print(f'  {name:11s}: tool xyz {xyz}, {kind} {ang:+.0f} | joint steps from parking {np.round(step).astype(int)} | '
                  f'margin to limits {joint_margin_deg(q):.0f} deg -> {"OK" if ok else "CHECK"}')
            if not ok:
                continue                                          # too close to a joint limit or too large a step: not used
            views.append(dict(name=name, tool_xyz=list(xyz), tool_theta_deg=[float(v) for v in eul], tilt_axis='z' if kind == 'sweep' else 'oblique', tilt_deg=ang if kind == 'sweep' else OBLIQUE_TILT_DEG, kind=kind, joints=[float(v) for v in q]))
    views.sort(key=lambda v: -float(_wrap180(np.array(v['joints']) - q_park)[0]))        # one smooth left-to-right pass of joint 1 (fewest long moves)
    json.dump(dict(parking_joints=[float(v) for v in q_park], views=views), open(OUT, 'w'), indent=1)
    print(f'saved {len(views)} fixed views to {os.path.relpath(OUT, ROOT)}')


if __name__ == '__main__':
    main()
