"""Step 4: execute the grasp on the real arm. Pre-grasp -> close-range cube check -> descend -> close -> lift.

Safety: slow joint-space moves, every pose bounds- and IK-checked first, the cube position is re-measured from
the pre-grasp pose and the run aborts if it differs from the plan, the gripper closes in steps until the
fingers stall, and the lift is skipped if the gripper closed on nothing. Ctrl-C / any error stops the arm.
"""
import json
import sys
import time

import cv2
import numpy as np

from common import connect, BaseClient, D405, move_joint, pose_to_matrix, in_safe_bounds, Base_pb2, grip_pos, grip_cmd, grip_ramp
from grasp_plan import build, plan_tilt, place_poses, ik_chain, P
from scan import depth_to_points

from cfg import section as _section, HAND_EYE

_E = _section('execution')
MAX_CORRECTION = _E['max_correction_m']      # m: abort if the close-range block centre differs from the plan by more than this
GRIP_EMPTY = _E['grip_empty']                # settled gripper position above this = closed on nothing
POS_TOL = _E['pos_tol_m']                    # m: abort if a move ends farther than this from its target
DUR = _E['durations_s']
PLACE_EXTRA = float(_E.get('place_extra_height_m', 0.04))   # carry the object this much higher than the normal lift
REFINE_MAX_JOINT_DEG = float(_E.get('refine_max_joint_deg', 70.0))   # the refined pre-grasp must be a small joint move from where the arm is
RELEASE = _E.get('release', 'none')           # place_back | drop | none


class GraspAbort(RuntimeError):
    """A check failed BEFORE the object was touched (not reachable, not where expected, missed grasp): skip this object, carry on."""


def go(base, name, pos, eul, duration=None, soft=False):
    if not in_safe_bounds(*pos):
        raise RuntimeError(f'{name}: {np.round(pos, 3)} outside safe bounds')
    print(f'-> {name}: tool xyz {np.round(pos, 3)}, theta {np.round(eul, 1)}', flush=True)
    try:
        move_joint(base, *pos, *eul, duration=duration, max_joint_deg=70)
    except RuntimeError as exc:
        if soft and 'details=55' not in str(exc):          # the arm could not generate this approach move (and nobody pressed stop): skip the object
            raise GraspAbort(f'{name}: the arm could not generate the move ({exc})')
        raise
    time.sleep(0.6)
    err = np.linalg.norm(pose_to_matrix(base.GetMeasuredCartesianPose())[:3, 3] - pos)
    print(f'   reached, position error {err * 1000:.1f} mm', flush=True)
    if err > POS_TOL:
        raise RuntimeError(f'{name}: ended {err * 1000:.1f} mm from target')


def refine_grasp(base, cam, X, g):
    """Close-range refinement at the pre-grasp pose (any object): re-segment the target from the dense close-up cloud (local table height,
    points above it, the cluster at the planned grasp) and re-run the SAME grasp synthesis on it. Aborts if the result disagrees with
    the far-range plan by more than MAX_CORRECTION. Returns (synthesis dict, local table height, close-up object cloud)."""
    from perceive import pcd, synthesize_grasp, P as PP
    time.sleep(1.0)
    T = pose_to_matrix(base.GetMeasuredCartesianPose()) @ X
    color, depth = cam.grab(warmup=10)
    pts, _ = depth_to_points(depth, color, cam.K)
    pb = pts @ T[:3, :3].T + T[:3, 3]
    plane = np.array(g['table_plane'], dtype=float)
    n, d = plane[:3], plane[3]
    mid = np.array(g['midpoint'])
    ext = np.array(g['obj_max']) - np.array(g['obj_min'])
    rad = float(max(ext[0], ext[1]) / 2 + 0.03)                      # object footprint radius + margin
    r = np.hypot(pb[:, 0] - mid[0], pb[:, 1] - mid[1])
    h = (pb @ n + d) / np.linalg.norm(n)
    ring = (r > rad) & (r < rad + 0.07) & (np.abs(h) < 0.02)         # bare table around the object
    plane_z = -(plane[0] * mid[0] + plane[1] * mid[1] + plane[3]) / plane[2]
    if ring.sum() < 300:
        raise GraspAbort(f'close-range check: only {ring.sum()} table points around the object')
    table_local = float(np.median(pb[ring, 2]))
    print(f'   local table height {table_local:.4f} m (plane fit said {plane_z:.4f}, diff {(table_local - plane_z) * 1000:.1f} mm)', flush=True)
    if abs(table_local - plane_z) > 0.02:                      # no abort: the dense close-up (measured from the pre-grasp pose) is the height the fingertips are planned on
        print(f'   local table height differs from the far-range plane by more than 20 mm: continuing with the close-range height {table_local:.4f} m', flush=True)
    plane2 = plane.copy()
    plane2[3] = -(plane[0] * mid[0] + plane[1] * mid[1] + plane[2] * table_local)       # same orientation, through the local table height
    h2 = (pb @ plane2[:3] + plane2[3]) / np.linalg.norm(plane2[:3])
    lo_b, hi_b = np.array(g['obj_min'][:2]) - 0.03, np.array(g['obj_max'][:2]) + 0.03          # the far-range object's footprint + margin
    inside = np.all((pb[:, :2] >= lo_b) & (pb[:, :2] <= hi_b), axis=1)
    cand = pb[inside & (h2 > PP['min_height'])]
    if len(cand) < 100:
        raise GraspAbort(f'close-range check: only {len(cand)} object points inside the planned object footprint')
    cand = np.asarray(pcd(cand).voxel_down_sample(PP['voxel']).points)                  # same density as the far-range cloud
    lab = np.asarray(pcd(cand).cluster_dbscan(0.015, PP['cluster_min_pts']))
    if lab.max() < 0:
        raise GraspAbort('close-range check: no object cluster inside the planned footprint')
    biggest = int(np.argmax(np.bincount(lab[lab >= 0])))
    obj = cand[lab == biggest]                                                         # the object (stray fragments dropped)
    c_far, c_close = np.array(g['centroid'][:2]), obj[:, :2].mean(0)
    shift = float(np.linalg.norm(c_close - c_far))
    print(f'   close-range object: {len(obj)} points, centroid ({c_close[0]:.4f}, {c_close[1]:.4f}), {shift * 1000:.1f} mm from the far-range centroid', flush=True)
    if shift > MAX_CORRECTION:                                                         # compare WHERE THE OBJECT IS, not the grasp point (the re-synthesis may pick a better one)
        raise GraspAbort('close-range object position disagrees with the far-range one - aborting before the descent')
    res = synthesize_grasp(obj, plane2, figs=False)
    print(f'   close-range grasp ({res["method"]}): grasp point ({res["mid"][0]:.4f}, {res["mid"][1]:.4f}) '
          f'(far-range grasp point was ({mid[0]:.4f}, {mid[1]:.4f}))', flush=True)
    return res, table_local, c_close - c_far


def main(cam=None, media_dir=None):
    own_cam = cam is None
    g = json.load(open('grasp.json'))
    X = np.array(json.load(open(HAND_EYE)))
    cam = cam or D405()

    def snap(name):
        if media_dir:
            cv2.imwrite(f'{media_dir}/{name}.png', cam.grab(warmup=2)[0])
    with connect() as router:
        base = BaseClient(router)
        try:
            T0 = pose_to_matrix(base.GetMeasuredCartesianPose())
            plan0 = json.load(open('plan.json'))
            tilt = plan0.get('approach_tilt_deg', 0.0)                                  # tilt and jaw direction chosen by the plan check (IK)
            poses, G, table_z, wrist = build(g, T0, tilt_deg=tilt, flip=plan0.get('jaw_flip', False))
            print(f'tip offset {P["tip_offset"]} m, table z {table_z:.4f}, grasp point {G.round(4)}, wrist rotation {wrist:.0f} deg', flush=True)
            grip_ramp(base, 0.0, rate=0.3)                    # open smoothly
            go(base, 'pre-grasp', *poses['pre_grasp'], soft=True)
            snap('10_pre_grasp')

            res, table_local, shift_xy = refine_grasp(base, cam, X, g)
            g2 = dict(g)
            g2.update(midpoint=res['mid'].tolist(), grasp_axis=res['gax'].tolist(), approach=res['app'].tolist(), obj_height=res['obj_height'])
            g3 = dict(g)                                       # fallback: the far-range grasp the plan already validated, moved to the measured object position
            g3['midpoint'] = [g['midpoint'][0] + shift_xy[0], g['midpoint'][1] + shift_xy[1], g['midpoint'][2]]
            q_now = np.array([a.value for a in base.GetMeasuredJointAngles().joint_angles])
            T_now = pose_to_matrix(base.GetMeasuredCartesianPose())
            chosen = None
            for label, gg in (('close-range re-synthesis', g2), ('far-range grasp shifted to the close-range object position', g3)):
                cand = plan_tilt(base, gg, T_now, q_now, table_z=table_local, verbose=False, cap=REFINE_MAX_JOINT_DEG)
                print(f'   {label}: approach tilt {cand[0]:+d} deg, {"reachable" if cand[5] else "NOT reachable"}', flush=True)
                if cand[5]:
                    chosen = (cand, gg)
                    break
            if chosen is None:                                  # last resort: the plan's own (validated) orientation, only translated to the measured object position
                gg = g3
                poses3, G3, tz3, wr3 = build(gg, T_now, tilt_deg=tilt, table_z=table_local, flip=plan0.get('jaw_flip', False))
                ok3, rows3 = ik_chain(base, poses3, q_now, REFINE_MAX_JOINT_DEG)
                print(f'   far-range orientation moved to the measured object position: {"reachable" if ok3 else "NOT reachable"}', flush=True)
                if not ok3:
                    raise GraspAbort('neither the refined nor the far-range grasp is comfortably reachable - aborting before the descent')
                chosen = ((tilt, poses3, G3, tz3, wr3, True, rows3), gg)
            (tilt, poses, G, table_z, wrist, ok_all, _), g2 = chosen
            print(f'   refined grasp: approach tilt {tilt:+d} deg (re-searched with IK), wrist rotation {wrist:.0f} deg', flush=True)
            tip_z = poses['grasp'][0][2] + build.approach[2] * P['tip_offset']      # fingertip ends = tool origin + approach * tip offset
            print(f'   refined: table z {table_z:.4f}, fingertip ends will stop at z {tip_z:.4f} ({(tip_z - table_z) * 1000:.1f} mm above the table)', flush=True)
            if tip_z - table_z < 0.004:
                raise GraspAbort('planned fingertip height is < 4 mm above the table - aborting before the descent')
            go(base, 'pre-grasp (refined)', *poses['pre_grasp'], duration=DUR['pre_grasp_refined'], soft=True)

            above = poses['grasp'][0] - build.approach * 0.04            # 4 cm back along the approach direction
            go(base, 'descend (stage 1)', above, poses['grasp'][1], duration=DUR['descend_stage1'])
            snap('11_pre_grasp_refined')
            go(base, 'descend to grasp (slow)', *poses['grasp'], duration=DUR['descend_final'])
            snap('12_at_grasp_pose')

            pos, stalled = grip_ramp(base, 1.0, rate=0.2)      # close smoothly until the fingers stall on the object
            time.sleep(0.8)
            pos = grip_pos(base)
            contact = stalled or pos < GRIP_EMPTY
            print(f'   gripper closed to {pos:.2f}, contact={contact}', flush=True)
            snap('13_gripper_closed')
            if not contact:
                raise GraspAbort('gripper closed on nothing - not lifting')
            go(base, 'lift', *poses['lift'], duration=DUR['lift'])
            snap('14_after_lift')
            held = grip_pos(base)
            print(f'   after lift gripper at {held:.2f} (was {pos:.2f}) -> {"HOLDING" if abs(held - pos) < 0.12 else "LOST?"}', flush=True)
            time.sleep(3.0)
            plan = json.load(open('plan.json'))
            if plan.get('place'):                             # carry the object to a free spot on the table, lower it, let go
                slot = np.array(plan['place']['slot_xy'])
                above, put, eul = place_poses(poses, G, slot, plan['place']['dz'], plan['place'].get('yaw_deg', 0), PLACE_EXTRA)
                go(base, f'carry to the free slot ({slot[0]:.3f}, {slot[1]:.3f})', above, eul, duration=DUR['lift'] + 3.0)
                go(base, 'lower at the slot', put, eul, duration=DUR['descend_final'])
                snap('15_placed_on_table')
                pos_open, _ = grip_ramp(base, 0.0, rate=0.25)
                print(f'   gripper opened smoothly -> {pos_open:.2f}', flush=True)
                snap('16_released')
                go(base, 'retreat up', above, eul, duration=DUR['lift'])
                result = f'PICK + PLACE COMPLETE (slot {slot[0]:.3f}, {slot[1]:.3f})'
                print(result, flush=True)
                return result
            if RELEASE == 'place_back':                       # lower the object back where it was, then let go
                go(base, 'lower to the table (place back)', poses['grasp'][0] + np.array([0, 0, 0.003]), poses['grasp'][1], duration=DUR['descend_final'])
                snap('15_placed_on_table')
            if RELEASE in ('place_back', 'drop'):
                pos_open, _ = grip_ramp(base, 0.0, rate=0.25)       # open smoothly
                print(f'   gripper opened smoothly -> {pos_open:.2f}', flush=True)
                snap('16_released')
                go(base, 'retreat up', *poses['lift'], duration=DUR['lift'])
                result = 'PICK + LIFT + RELEASE COMPLETE (' + RELEASE + ')'
            else:
                result = 'GRASP + LIFT COMPLETE (gripper still closed)'
            print(result, flush=True)
            return result
        except BaseException as exc:
            print(f'!! STOPPING: {exc!r}', flush=True)
            if not isinstance(exc, GraspAbort):
                base.Stop()
            raise
        finally:
            if own_cam:
                cam.close()


if __name__ == '__main__':
    main()
