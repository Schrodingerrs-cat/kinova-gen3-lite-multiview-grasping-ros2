"""Step 4 planning (DRY RUN, no motion)."""
import json
import os
import sys

import numpy as np
from scipy.spatial.transform import Rotation as R

from cfg import ROOT, load_json
from common import connect, BaseClient, pose_to_matrix, in_safe_bounds, joint_cap, joint_margin_deg, Base_pb2, _wrap180

P = dict(
    tip_offset=0.035,
    grasp_height=0.015,
    pre_grasp_height=0.10,     # m above the grasp point
    lift_height=0.12,          # m above the grasp point
    max_joint_deg=60.0,
    tilt_candidates_deg=[0, 8, -8, 16, -16, 24, -24, 32, -32, 40, -40, 50, -50],   # approach tilt about the jaw axis, smallest first
)

from cfg import section as _section
P.update(_section('grasp'))
P['max_joint_deg'] = joint_cap(P['max_joint_deg'])


def build(g, T_start, tip_offset=None, table_z=None, tilt_deg=0.0, flip=False):
    """Tool poses for the grasp in grasp.json."""
    tip = P['tip_offset'] if tip_offset is None else tip_offset
    a, b, c, d = g['table_plane']
    mx, my = g['midpoint'][:2]
    if table_z is None:
        table_z = -(a * mx + b * my + d) / c
    app = np.array(g.get('approach', [0.0, 0.0, -1.0]), dtype=float)
    app /= np.linalg.norm(app)
    if abs(app[2]) >= 0.5:
        gh = float(np.clip(0.4 * g['obj_height'], 0.008, 0.03)) if g.get('obj_height') else P['grasp_height']
    else:
        gh = max(float(g['midpoint'][2]) - table_z, 0.012)
    G = np.array([mx, my, table_z + gh])        # grasp point between the pads
    gax = np.array(g['grasp_axis'], dtype=float)
    gax = gax - app * (gax @ app)               # jaw axis made orthogonal to the approach
    gax /= np.linalg.norm(gax)
    best = None
    for x_t in (gax, -gax):
        R_t = np.column_stack([x_t, np.cross(app, x_t), app])
        ang = np.degrees(np.linalg.norm(R.from_matrix(T_start[:3, :3].T @ R_t).as_rotvec()))
        if best is None or ang < best[0]:
            best = (ang, x_t)
    x_t = best[1]
    if flip:
        x_t = -x_t
        best = (np.degrees(np.linalg.norm(R.from_matrix(T_start[:3, :3].T @ np.column_stack([x_t, np.cross(app, x_t), app])).as_rotvec())), x_t)
    y0 = np.cross(app, x_t)
    phi = np.radians(tilt_deg)
    z_a = app * np.cos(phi) + y0 * np.sin(phi)
    R_t = np.column_stack([x_t, np.cross(z_a, x_t), z_a])
    build.approach = z_a
    origin = G - z_a * tip                      # tool origin that puts the fingertip ends on G
    eul = R.from_matrix(R_t).as_euler('xyz', degrees=True)
    poses = {'pre_grasp': (origin - z_a * P['pre_grasp_height'], eul),     # backed off ALONG the approach direction
             'grasp': (origin, eul),
             'lift': (origin + np.array([0, 0, P['lift_height']]), eul)}   # lifted straight up
    return poses, G, table_z, best[0]

def place_poses(poses, G, slot_xy, dz, yaw_deg, extra):
    """Tool poses to carry the held object to a table slot."""
    Rz = R.from_euler('z', yaw_deg, degrees=True)
    origin, eul = poses['grasp']
    eul_new = (Rz * R.from_euler('xyz', eul, degrees=True)).as_euler('xyz', degrees=True)
    off = Rz.apply(origin - G)
    centre = np.array([slot_xy[0], slot_xy[1], G[2] + dz])
    put = centre + off + np.array([0, 0, 0.003])
    above = put + np.array([0, 0, P['lift_height'] + extra])
    return above, put, eul_new


def _load_seeds():
    try:
        return [np.array(load_json(os.path.join(ROOT, 'calibration', 'scan_joints.json')), dtype=float)]
    except Exception:
        return []


_EXTRA_SEEDS = _load_seeds()


def ik_chain(base, poses, q_start, cap=None):
    """IK + bounds + joint-step check of a pose chain (pre-grasp -> grasp -> lift)."""
    qq, rows, ok_all = np.array(q_start, dtype=float).copy(), {}, True
    for name, (pos, eul) in poses.items():
        sol, dq = None, None
        for seed in (qq, np.asarray(q_start, dtype=float), *_EXTRA_SEEDS):
            d = Base_pb2.IKData(); cp = d.cartesian_pose
            cp.x, cp.y, cp.z, cp.theta_x, cp.theta_y, cp.theta_z = *pos, *eul
            for v in seed:
                d.guess.joint_angles.add().value = float(v)
            try:
                sol = np.array([a.value for a in base.ComputeInverseKinematics(d).joint_angles])
                dq = float(np.abs(_wrap180(sol - qq)).max()); qq = sol
                break
            except Exception:
                sol, dq = None, None
        b_ok = in_safe_bounds(*pos)
        ok = b_ok and dq is not None and dq <= (P['max_joint_deg'] if cap is None else cap) and joint_margin_deg(sol) >= P.get('joint_margin_deg', 6.0)
        ok_all &= ok
        rows[name] = (pos, eul, b_ok, dq, ok, sol)
    return ok_all, rows


def plan_tilt(base, g, T_start, q_start, tip=None, table_z=None, verbose=True, cap=None):
    """Search the approach tilt about the jaw axis for the first one whose pose chain passes IK."""
    chosen = None
    for flip in (False, True):                                                  # jaw direction needing the least wrist rotation first
        for tilt in P['tilt_candidates_deg']:
            poses, G, tz, wrist = build(g, T_start, tip, table_z=table_z, tilt_deg=tilt, flip=flip)
            ok_all, rows = ik_chain(base, poses, q_start, cap)
            if chosen is None:
                chosen = (tilt, poses, G, tz, wrist, ok_all, rows)              # keep the first (tilt 0) for the failure report
                plan_tilt.flip = flip
            if ok_all:
                chosen = (tilt, poses, G, tz, wrist, ok_all, rows)
                plan_tilt.flip = flip
                break
            if verbose:
                print(f'  {"flipped " if flip else ""}approach tilt {tilt:+d} deg: not feasible ({", ".join(n for n, r in rows.items() if not r[4])} fail)', flush=True)
        if chosen[5]:
            break
    build(g, T_start, tip, table_z=table_z, tilt_deg=chosen[0], flip=plan_tilt.flip)     # leaves build.approach set for the chosen tilt
    return chosen


def main(tip=None):
    g = json.load(open('grasp.json'))
    with connect() as router:
        base = BaseClient(router)
        T0 = pose_to_matrix(base.GetMeasuredCartesianPose())
        q = np.array([x.value for x in base.GetMeasuredJointAngles().joint_angles])
        q_start = q.copy()
        chosen = plan_tilt(base, g, T0, q_start, tip)
        tilt, poses, G, table_z, wrist, ok_all, rows = chosen
        print(f'tip offset used: {P["tip_offset"] if tip is None else tip:.3f} m (measured)')
        print(f'table z at the cube {table_z:.4f} m; grasp point (between pads) {G.round(4)}; jaw axis = tool x, '
              f'wrist rotation from start {wrist:.0f} deg; approach tilt {tilt:+d} deg ({"straight down" if tilt == 0 else "tilted about the jaw axis"})')
        plan = {}
        for name, (pos, eul, b_ok, dq, ok, sol) in rows.items():
            print(f'{name:9s}: tool xyz {pos.round(3)} theta {eul.round(1)} | bounds {"ok" if b_ok else "OUT"} | '
                  f'IK {"FAIL" if dq is None else f"max joint step {dq:.0f} deg"} -> {"OK" if ok else "NOT OK"}')
            plan[name] = dict(xyz=pos.tolist(), theta=eul.tolist(), joints=None if sol is None else sol.tolist())
        place = None
        if os.path.exists('place_slots.json') and ok_all:
            q_lift = rows['lift'][5]
            a_, b_, c_, d_ = g['table_plane']
            slots = sorted(json.load(open('place_slots.json')), key=lambda s: np.hypot(s[0] - G[0], s[1] - G[1]))      # nearest the pick first
            for k, s in enumerate(slots):
                dz = (-(a_ * s[0] + b_ * s[1] + d_) / c_) - (-(a_ * G[0] + b_ * G[1] + d_) / c_)
                for yaw in P.get('place_yaw_candidates_deg', [0, 90, -90, 180]):
                    above, put, eul_p = place_poses(poses, G, s, dz, yaw, P.get('place_extra_height_m', 0.04))
                    ok_p, _ = ik_chain(base, {'above_slot': (above, eul_p), 'place_slot': (put, eul_p)}, q_lift)
                    if ok_p:
                        place = dict(slot_xy=s, slot_index=k, yaw_deg=yaw, dz=float(dz))
                        break
                print(f'place slot {k} ({s[0]:.3f}, {s[1]:.3f}): {"reachable" if place else "NOT reachable"}' + (f' with wrist turned {place["yaw_deg"]} deg' if place else ''), flush=True)
                if place:
                    break
            if place is None:
                ok_all = False
                print('no reachable free place slot', flush=True)
        json.dump(dict(tip_offset=P['tip_offset'] if tip is None else tip, approach_tilt_deg=tilt, jaw_flip=bool(plan_tilt.flip), grasp_point=G.tolist(), poses=plan, place=place), open('plan.json', 'w'), indent=1)
        try:
            from perceive import save_fig
            from scipy.spatial.transform import Rotation as Rr
            obj = np.load('object_cloud.npy')
            cols = {'pre_grasp': 'tab:orange', 'grasp': 'tab:red', 'lift': 'tab:purple'}
            arrows, marks = [], []
            for name, dd in plan.items():
                pos = np.array(dd['xyz']); Rm = Rr.from_euler('xyz', dd['theta'], degrees=True).as_matrix()
                for i, col in enumerate(('red', 'green', 'blue')):
                    arrows.append((pos, pos + 0.04 * Rm[:, i], col, f'tool {"xyz"[i]} axis' if name == 'grasp' else None))
                marks.append((pos, cols[name], 70, f'{name} (tool origin)'))
            marks.append((np.array(G), 'gold', 90, 'grasp point (fingertips)'))
            path = np.array([plan[n]['xyz'] for n in ('pre_grasp', 'grasp', 'lift')])
            save_fig('s4_grasp_poses', [(obj, 'tab:green', 3, 'object'), (path, 'k', 8, 'tool path')], marks, arrows, view=(20, -50))
            print('saved figures/s4_grasp_poses.png')
        except Exception as exc:
            print('(pose figure skipped:', repr(exc), ')')
        print('PLAN OK' if ok_all else 'PLAN HAS PROBLEMS')
        return ok_all


if __name__ == '__main__':
    main(float(sys.argv[1]) if len(sys.argv) > 1 else None)
