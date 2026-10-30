"""HW 6 steps 1.1-3: merge the scans, find the table, cluster objects, synthesise a grasp."""
import json
import sys
import math
import os

import matplotlib
matplotlib.use('Agg')
import os as _os
import mpl_toolkits
mpl_toolkits.__path__.insert(0, _os.path.join(_os.path.dirname(_os.path.dirname(matplotlib.__file__)), 'mpl_toolkits'))
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation as R

P = dict(
    n_scans=4,
    voxel=0.004,                       # 1.1 downsample (m)
    ws_min=(0.27, -0.30, -0.12), ws_max=(0.60, 0.30, 0.15),   # 1.2 crop box in base (m)
    ransac_dist=0.004, ransac_iters=500,                      # 1.3 table plane
    min_height=0.015,                                         # 1.4 keep points this far above the plane
    cluster_eps=0.012, cluster_min_pts=40,                    # 1.5 DBSCAN
    obj_dim_min=0.03, obj_dim_max=0.09, obj_h_min=0.040, obj_h_max=0.080,
    normal_radius=0.012, normal_max_nn=40,                    # 2.2
    mu=0.5, width_min=0.02, width_max=0.075, max_pts=1500,    # 2.3 friction coeff, gripper widths
    tag_crosscheck=False,                                     # tag has been removed from the table
    tie_tol=0.003, max_axis_tilt_deg=10.0, max_approach_tilt_deg=5.0, top_min_height=0.035, prefer_short_side=True,   # 3b: top-face points are this far above the table
    denoise_radius=0.008, denoise_min_nb=12,                  # drop isolated halo points per cluster
    icp_align=True, icp_iters=40, icp_max_dist=0.03, icp_trim=0.7,   # align scans to scan 0 before merging
)

from cfg import CFG, section as _section
P.update(_section('perception'))


def save_ply(path, xyz, rgb=None):
    """Write a binary PLY (xyz float32 + rgb uint8)."""
    os.makedirs(os.path.dirname(path), exist_ok=True)
    xyz = np.asarray(xyz, dtype=np.float32).reshape(-1, 3)
    n = len(xyz)
    if rgb is None:
        rgb = np.full((n, 3), 200, np.uint8)
    rgb = np.asarray(rgb)
    if rgb.dtype != np.uint8:
        rgb = (np.clip(rgb, 0, 1) * 255).astype(np.uint8)
    arr = np.empty(n, dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('r', 'u1'), ('g', 'u1'), ('b', 'u1')])
    arr['x'], arr['y'], arr['z'] = xyz.T
    arr['r'], arr['g'], arr['b'] = rgb.T
    with open(path, 'wb') as f:
        f.write(('ply\nformat binary_little_endian 1.0\nelement vertex %d\nproperty float x\nproperty float y\nproperty float z\n'
                 'property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n' % n).encode())
        arr.tofile(f)


def save_fig(name, layers, markers=(), arrows=(), view=(35, -60)):
    """layers: [(xyz, color, size, label)]."""
    os.makedirs('figures', exist_ok=True)
    fig = plt.figure(figsize=(7, 6))
    ax = fig.add_subplot(111, projection='3d')
    allp = []
    for xyz, c, s, lab in layers:
        if len(xyz) > 25000:
            sel = np.random.default_rng(0).choice(len(xyz), 25000, replace=False)
            xyz = xyz[sel]
            if not isinstance(c, str) and not isinstance(c, tuple):
                c = np.asarray(c)[sel]
        ax.scatter(*xyz.T, c=c, s=s, label=lab, depthshade=False)
        allp.append(xyz)
    for xyz, c, s, lab in markers:
        xyz = np.atleast_2d(xyz)
        ax.scatter(*xyz.T, c=c, s=s, label=lab, depthshade=False, edgecolors='k')
    for a, b, c, lab in arrows:
        ax.quiver(*a, *(b - a), color=c, label=lab, arrow_length_ratio=0.2, linewidth=2)
    pts = np.vstack(allp)
    mid, half = (pts.min(0) + pts.max(0)) / 2, (pts.max(0) - pts.min(0)).max() / 2
    for setter, m in zip((ax.set_xlim, ax.set_ylim, ax.set_zlim), mid):
        setter(m - half, m + half)
    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)'); ax.set_zlabel('z (m)')
    ax.view_init(*view)
    ax.legend(loc='upper left', fontsize=7)
    fig.tight_layout()
    fig.savefig(f'figures/{name}.png', dpi=110)
    plt.close(fig)


def pcd(xyz, rgb=None):
    c = o3d.geometry.PointCloud()
    c.points = o3d.utility.Vector3dVector(np.ascontiguousarray(xyz, dtype=np.float64))
    if rgb is not None:
        c.colors = o3d.utility.Vector3dVector(np.ascontiguousarray(rgb, dtype=np.float64))
    return c


def antipodal(pts, nrm, centroid):
    """HW 6 Step 2.3: antipodal pair inside the friction cone nearest the centroid."""
    cos_t = 1.0 / math.sqrt(1.0 + P['mu'] ** 2)
    if len(pts) > P['max_pts']:
        sel = np.random.default_rng(0).choice(len(pts), P['max_pts'], replace=False)
        pts, nrm = pts[sel], nrm[sel]
    best = None
    for s in range(0, len(pts), 256):
        d = pts[None, :, :] - pts[s:s + 256, None, :]
        dist = np.linalg.norm(d, axis=2)
        with np.errstate(invalid='ignore', divide='ignore'):
            dh = d / dist[..., None]
        c1 = np.einsum('cnk,ck->cn', dh, nrm[s:s + 256])
        c2 = -np.einsum('cnk,nk->cn', dh, nrm)
        ok = (dist > P['width_min']) & (dist < P['width_max']) & (c1 > cos_t) & (c2 > cos_t)
        ok &= np.abs(dh[:, :, 2]) < math.sin(math.radians(P['max_axis_tilt_deg']))
        if not ok.any():
            continue
        ci, nj = np.nonzero(ok)
        if P.get('pair_criterion', 'grasp_points') == 'midpoint':
            md = np.linalg.norm((pts[s + ci] + pts[nj]) / 2 - centroid, axis=1)
        else:
            md = 0.5 * (np.linalg.norm(pts[s + ci] - centroid, axis=1) + np.linalg.norm(pts[nj] - centroid, axis=1))
        q = np.minimum(c1[ci, nj], c2[ci, nj])
        k = np.lexsort((-q, md))[0]
        cand = (float(md[k]), -float(q[k]), pts[s + ci[k]], pts[nj[k]])
        if best is None or cand[0] < best[0] - P['tie_tol'] or (abs(cand[0] - best[0]) <= P['tie_tol'] and cand[1] < best[1]):
            best = cand
    return best


def kabsch_T(src, dst):
    sc, dc = src.mean(0), dst.mean(0)
    U, _, Vt = np.linalg.svd((src - sc).T @ (dst - dc))
    Rm = Vt.T @ np.diag([1, 1, np.sign(np.linalg.det(Vt.T @ U.T))]) @ U.T
    T = np.eye(4); T[:3, :3], T[:3, 3] = Rm, dc - Rm @ sc
    return T


def icp_align(src, dst):
    """Trimmed point-to-point ICP."""
    tree, T, cur = cKDTree(dst), np.eye(4), src.copy()
    rms = None
    for _ in range(P['icp_iters']):
        d, idx = tree.query(cur, distance_upper_bound=P['icp_max_dist'])
        ok = np.isfinite(d)
        if ok.sum() < 100:
            break
        keep = np.flatnonzero(ok)[np.argsort(d[ok])[:int(P['icp_trim'] * ok.sum())]]
        dT = kabsch_T(cur[keep], dst[idx[keep]])
        cur = cur @ dT[:3, :3].T + dT[:3, 3]
        T = dT @ T
        rms = float(np.sqrt((d[keep] ** 2).mean()))
    return T, rms


EXCL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'placed_objects.json')


def exclusion_zones():
    """[[x, y, radius], ...]."""
    try:
        return json.load(open(EXCL_PATH))
    except Exception:
        return []


def is_excluded(xy):
    return any(np.hypot(xy[0] - z[0], xy[1] - z[1]) < z[2] for z in exclusion_zones())


def select_target(clusters, heights=None):
    """Which object to grasp when there are several (object.selection)."""
    rule = CFG.get('object', {}).get('selection', 'nearest')
    if rule == 'largest':
        key = [-len(c) for c in clusters]
    elif rule == 'tallest' and heights is not None:
        key = [-h for h in heights]
    else:
        key = [float(np.hypot(*c.mean(0)[:2])) for c in clusters]        # nearest to the robot base in the table plane
    return int(np.argmin(key))


def synthesize_grasp(obj, plane, figs=False):
    """HW 6 grasp synthesis for ONE object cloud (any shape)."""
    plane = np.asarray(plane, dtype=float)
    n_pl, d_pl = plane[:3], plane[3]
    centroid = obj.mean(0)
    h_all = (obj @ n_pl + d_pl) / np.linalg.norm(n_pl)
    obj_height = float(np.percentile(h_all, 95))
    print(f'[2.1] centroid = {centroid.round(4)}, object height {obj_height * 1000:.0f} mm above the table', flush=True)
    if figs:
        save_fig('s2_1_centroid', [(obj, 'tab:green', 5, 'object')], [(centroid, 'gold', 120, 'centroid')])

    # 2.2 surface normals, oriented toward the centroid
    tree = cKDTree(obj)
    nrm = np.full_like(obj, np.nan)
    for i, nb in enumerate(tree.query_ball_point(obj, P['normal_radius'])):
        if len(nb) < 5:
            continue
        q = obj[nb] - obj[nb].mean(0)
        n = np.linalg.eigh(q.T @ q)[1][:, 0]                      # smallest-variance direction = surface normal
        nrm[i] = n if np.dot(n, centroid - obj[i]) >= 0 else -n
    good = np.isfinite(nrm).all(axis=1)
    pts, nrm = obj[good], nrm[good]
    to_c = (centroid - pts) / np.linalg.norm(centroid - pts, axis=1, keepdims=True)
    print(f'[2.2] {len(pts)} normals; mean angle to the centroid direction = '
          f'{np.degrees(np.arccos(np.clip(np.einsum("ij,ij->i", nrm, to_c), -1, 1))).mean():.1f} deg', flush=True)
    if figs:
        sel = np.random.default_rng(1).choice(len(pts), min(120, len(pts)), replace=False)
        save_fig('s2_2_normals', [(obj, 'tab:green', 3, 'object')], [(centroid, 'gold', 100, 'centroid')],
                 [(pts[i], pts[i] + 0.015 * nrm[i], 'tab:blue', 'surface normals (oriented to the centroid)' if j == 0 else None) for j, i in enumerate(sel)])

    best = antipodal(pts, nrm, centroid)
    ev, evec = np.linalg.eigh(np.cov((obj - centroid).T))
    major = evec[:, np.argmax(ev)]
    degenerate = ev[-2] / ev[-1] > P.get('pca_degenerate_ratio', 0.7)       # near-isotropic: the major axis is not well defined
    method, quality, p1, p2 = 'antipodal', None, None, None
    if best is not None:
        md, negq, p1, p2 = best
        mid = (p1 + p2) / 2
        if tree.query(mid)[0] > P.get('mid_body_tol', 0.012):               # curved object: the pair's midpoint is in free air
            print(f'[2.3] antipodal pair rejected: its midpoint is {tree.query(mid)[0] * 1000:.0f} mm from the object body', flush=True)
            best = None
        elif np.linalg.norm(mid - centroid) > P.get('max_pair_midpoint_offset_m', 0.025):   # a stable grasp is near the centre of the object
            print(f'[2.3] antipodal pair rejected: its midpoint is {np.linalg.norm(mid - centroid) * 1000:.0f} mm from the centroid '
                  f'(limit {P.get("max_pair_midpoint_offset_m", 0.025) * 1000:.0f} mm), an off-centre feature', flush=True)
            best = None
    if best is not None:
        quality = -negq
        gax = (p2 - p1) / np.linalg.norm(p2 - p1)
        width = float(np.linalg.norm(p2 - p1))
        print(f'[2.3] antipodal pair: p1={p1.round(4)} p2={p2.round(4)} width={width * 1000:.1f} mm, mean grasp-point distance to the centroid={md * 1000:.1f} mm, cone quality={quality:.3f}', flush=True)
        app = np.cross(major, gax)
        note = 'cross(PCA major axis, grasp axis)'
        if degenerate or np.linalg.norm(app) < 1e-3:
            app = np.array([0, 0, -1.0]) - gax * (-gax[2])
            note = 'vertical made orthogonal to the grasp axis (PCA major axis ill-defined: near-isotropic object)'
        app /= np.linalg.norm(app)
        if abs(app[2]) >= 0.5:
            app = app * (-1.0 if app[2] > 0 else 1.0)                       # from above
        elif np.dot(app[:2], mid[:2]) < 0:
            app = -app                                                       # side approach: moving outward from the robot
    else:
        method = 'across_body'
        xy = obj[:, :2]
        gi = int(np.argmin(np.linalg.norm(xy - xy.mean(0), axis=1)))
        local = xy[np.linalg.norm(xy - xy[gi], axis=1) < P.get('local_radius_m', 0.04)]
        w_, V_ = np.linalg.eigh(np.cov((local - local.mean(0)).T))
        body = V_[:, np.argmax(w_)]
        jaw = np.array([-body[1], body[0]])
        pj = (local - local.mean(0)) @ jaw
        lo_j, hi_j = np.percentile(pj, 3), np.percentile(pj, 97)
        width = float((hi_j - lo_j) / 0.94)
        c_xy = local.mean(0) + jaw * (lo_j + hi_j) / 2
        tz = -(plane[0] * c_xy[0] + plane[1] * c_xy[1] + plane[3]) / plane[2]
        mid = np.array([c_xy[0], c_xy[1], tz + 0.5 * obj_height])
        gax, app = np.array([jaw[0], jaw[1], 0.0]), np.array([0.0, 0.0, -1.0])
        p1, p2 = mid - gax * width / 2, mid + gax * width / 2
        note = 'vertical (grasping across the body)'
        print(f'[2.3] no usable antipodal pair; grasping across the body near its middle at ({mid[0]:.3f}, {mid[1]:.3f}), '
              f'local body direction {np.round(body, 2)}, width {width * 1000:.0f} mm', flush=True)
    print(f'[3] PCA eigenvalues {ev.round(6)}, major axis {major.round(3)}; approach {app.round(3)} via {note}', flush=True)
    if figs:
        save_fig('s2_3_grasp_points', [(obj, 'tab:green', 5, 'object')], [(centroid, 'gold', 100, 'centroid'), (np.array([p1, p2]), 'red', 120, 'grasp points')])
        save_fig('s3_approach', [(obj, 'tab:green', 5, 'object')], [(centroid, 'gold', 100, 'centroid'), (np.array([p1, p2]), 'red', 120, 'grasp points')],
                 [(mid - app * 0.08, mid, 'magenta', 'approach')])
    return dict(centroid=centroid, p1=p1, p2=p2, mid=mid, gax=gax, app=app, width=width, method=method, quality=quality,
                obj_height=obj_height, major=major, eigenvalues=ev)


def main():
    import glob
    o3d.utility.random.seed(0); np.random.seed(0)
    files = sorted(glob.glob('scans/scan_*.npz')) + sorted(glob.glob('scans/obl_*.npz'))
    print(f'[1.1] scans: {[os.path.basename(f) for f in files]}')
    scans = [np.load(f) for f in files]

    # 1.1 merge + voxel downsample
    lo_c, hi_c = np.array(P['ws_min']) - 0.05, np.array(P['ws_max']) + 0.05
    clouds = [s['xyz'].astype(np.float64) for s in scans]
    ref_i = len(clouds) // 2                                              # centre view is the reference
    if P.get('plane_align', True) and len(clouds) > 1:
        def table_plane(x):                                               # RANSAC table plane of one view (normal up)
            x = x[np.all((x >= np.array(P['ws_min'])) & (x <= np.array(P['ws_max'])), axis=1)]
            pl, _ = pcd(x).segment_plane(P['ransac_dist'], 3, P['ransac_iters'])
            pl = np.array(pl, float)
            return -pl if pl[2] < 0 else pl
        planes = [table_plane(x) for x in clouds]
        n0, d0 = planes[ref_i][:3], planes[ref_i][3]
        for k in range(len(clouds)):
            if k == ref_i:
                continue
            n, d = planes[k][:3], planes[k][3]
            rv = np.cross(n, n0); s_, c_ = np.linalg.norm(rv), float(np.dot(n, n0))
            Rk = R.from_rotvec(rv / s_ * np.arctan2(s_, c_)).as_matrix() if s_ > 1e-9 else np.eye(3)
            p0 = np.array([0.4, 0.0, -d / n[2]])                          # a table point of view k near the workspace centre
            p0 = p0 - n * (np.dot(n, p0) + d)                             # exactly on its plane
            q = Rk @ p0
            t = p0 - q + n0 * (-(np.dot(n0, p0) + d0))                    # rotate about p0, then slide onto the reference plane
            clouds[k] = clouds[k] @ Rk.T + t
            print(f'[1.1] table of view {k} tilted {np.degrees(np.arctan2(s_, c_)):.2f} deg, offset {(-(np.dot(n0, p0) + d0)) * 1000:+.1f} mm vs the centre view: aligned')
    if P['icp_align']:
        def region(x):
            x = x[np.all((x >= lo_c) & (x <= hi_c), axis=1)]
            return np.asarray(pcd(x).voxel_down_sample(P['voxel']).points)
        ref = region(clouds[ref_i])
        for k in [i for i in range(len(clouds)) if i != ref_i]:
            T, rms = icp_align(region(clouds[k]), ref)
            rot = np.degrees(np.linalg.norm(R.from_matrix(T[:3, :3]).as_rotvec()))
            print(f'[1.1] ICP scan {k} -> scan {ref_i}: shift {np.linalg.norm(T[:3, 3]) * 1000:.1f} mm, '
                  f'rot {rot:.2f} deg, trimmed rms {rms * 1000:.2f} mm')
            clouds[k] = clouds[k] @ T[:3, :3].T + T[:3, 3]
    try:
        names = json.load(open('scans/view_names.json'))
    except Exception:
        names = [f'view {k}' for k in range(len(clouds))]
    from matplotlib import colors as _mc
    hexes = [_mc.to_hex(plt.get_cmap('tab10')(k % 10)) for k in range(len(clouds))]
    view_layers = []
    for k, x in enumerate(clouds):
        m = np.all((x >= lo_c) & (x <= hi_c), axis=1)
        view_layers.append((x[m], hexes[k], 1, f'view {k}: {names[k] if k < len(names) else k}'))
    save_fig('s1_0_views_aligned', view_layers, view=(40, -60))
    for k, x in enumerate(clouds):                                       # each viewpoint's aligned cloud as a PLY
        m = np.all((x >= lo_c) & (x <= hi_c), axis=1)
        save_ply(f'pointclouds/view_{k}_{names[k] if k < len(names) else k}_aligned.ply', x[m], scans[k]['rgb'][m])
    cams = np.array([sc['T_base_cam'][:3, 3] for sc in scans])
    fw = [sc['T_base_cam'][:3, 2] * 0.10 for sc in scans]
    save_fig('s1_0_camera_poses', [(l[0][::6], l[1], 1, l[3]) for l in view_layers] + [(cams, 'k', 1, 'camera origins')],
             arrows=[(cams[k], cams[k] + fw[k], hexes[k], None) for k in range(len(cams))], view=(25, -50))
    print(f'[1.0] {len(clouds)} viewpoints: ' + ', '.join(f'{names[k] if k < len(names) else k} (camera z {cams[k][2]:.3f} m)' for k in range(len(cams))))
    merged = pcd(np.vstack(clouds), np.vstack([s['rgb'] for s in scans]))
    n_raw = len(merged.points)
    merged = merged.voxel_down_sample(P['voxel'])
    xyz_m = np.asarray(merged.points)
    print(f'[1.1] merged {n_raw} -> {len(xyz_m)} points (voxel {P["voxel"]} m)')
    save_fig('s1_1_merged', [(xyz_m, np.asarray(merged.colors), 1, 'merged + downsampled')], view=(40, -60))
    save_ply('pointclouds/merged_downsampled.ply', xyz_m, np.asarray(merged.colors))

    lo, hi = np.array(P['ws_min'], dtype=float), np.array(P['ws_max'], dtype=float)
    if '--center' in sys.argv and P.get('crop_half_size_m'):
        ci = sys.argv.index('--center')
        cxy, half = np.array([float(sys.argv[ci + 1]), float(sys.argv[ci + 2])]), float(P['crop_half_size_m'])
        lo[:2], hi[:2] = np.maximum(lo[:2], cxy - half), np.minimum(hi[:2], cxy + half)
        print(f'[1.2] cropping to +/-{half:.2f} m around the detected object at ({cxy[0]:.3f}, {cxy[1]:.3f})')
    keep = np.all((xyz_m >= lo) & (xyz_m <= hi), axis=1)
    ws = pcd(xyz_m[keep], np.asarray(merged.colors)[keep])
    xyz_w = np.asarray(ws.points)
    print(f'[1.2] workspace: {len(xyz_w)} points')
    save_fig('s1_2_workspace', [(xyz_w, np.asarray(ws.colors), 1, 'workspace crop')])
    save_ply('pointclouds/workspace_filtered.ply', xyz_w, np.asarray(ws.colors))

    plane, inl = ws.segment_plane(P['ransac_dist'], 3, P['ransac_iters'])
    for _ in range(3):
        q_in = xyz_w[inl]
        c0 = q_in.mean(0)
        nrm = np.linalg.svd(q_in - c0, full_matrices=False)[2][-1]
        plane = (*nrm, -float(nrm @ c0))
        sd = np.abs(xyz_w @ nrm + plane[3])
        inl = np.where(sd < P['ransac_dist'])[0]
    a, b, c, d = plane
    if c < 0:
        a, b, c, d = -a, -b, -c, -d
        plane = (a, b, c, d)
    print(f'[1.3] plane normal=({a:.3f},{b:.3f},{c:.3f}) -> {len(inl)} inliers, z at x,y=0: {-d / c:.4f} m')
    mask = np.zeros(len(xyz_w), bool); mask[inl] = True
    save_fig('s1_3_table', [(xyz_w[mask], 'tab:blue', 1, 'table plane (RANSAC)'),
                            (xyz_w[~mask], 'tab:red', 1, 'not plane')])
    save_ply('pointclouds/table_plane.ply', xyz_w[mask])

    # 1.4 objects: not plane and above the plane
    height = (xyz_w @ np.array([a, b, c]) + d) / np.linalg.norm([a, b, c])
    obj_mask = (~mask) & (height > P['min_height'])
    objs, hobj = xyz_w[obj_mask], height[obj_mask]
    n_before = len(objs)
    if len(objs) > 20:
        nb = np.array([len(v) for v in cKDTree(objs).query_ball_point(objs, P['denoise_radius'])])
        objs, hobj = objs[nb >= P['denoise_min_nb']], hobj[nb >= P['denoise_min_nb']]
    n_dense = len(objs)
    if P.get('min_view_support', 1) > 1 and len(clouds) >= 2 and len(objs) > 50:
        r = float(P.get('view_support_radius', 0.006))
        sup = np.zeros(len(objs), int)
        for cl_v in clouds:
            near = cl_v[np.all((cl_v >= lo) & (cl_v <= hi), axis=1)]
            if len(near):
                sup += (cKDTree(near).query(objs, distance_upper_bound=r)[0] < r)
        keep_v = sup >= int(P['min_view_support'])
        objs, hobj = objs[keep_v], hobj[keep_v]
    print(f'[1.4] object points: {len(objs)} (> {P["min_height"] * 1000:.0f} mm above plane; {n_before - n_dense} noise points and {n_dense - len(objs)} single-view points removed)')
    save_fig('s1_4_objects', [(xyz_w[mask], (0.8, 0.8, 0.8), 0.5, 'table'), (objs, 'tab:red', 3, 'object points')])
    save_ply('pointclouds/objects_above_table.ply', objs)

    # 1.5 DBSCAN clustering
    lab = np.asarray(pcd(objs).cluster_dbscan(P['cluster_eps'], P['cluster_min_pts']))
    clusters, heights, rejected = [], [], []
    for k in range(lab.max() + 1):
        sel = lab == k
        cl = objs[sel]
        nb = np.array([len(v) for v in cKDTree(cl).query_ball_point(cl, P['denoise_radius'])])
        keep = nb >= P['denoise_min_nb']
        if keep.sum() >= P.get('min_cluster_points', P['cluster_min_pts']):
            h_lo, h_hi = float(hobj[sel][keep].min()), float(hobj[sel][keep].max())      # bottom / top of the cluster above the table plane
            if h_lo > P.get('max_bottom_gap_m', 0.04) or h_hi > P.get('max_object_height_m', 0.15):
                rejected.append((cl[keep].mean(0), h_lo, h_hi))
                continue
            clusters.append(cl[keep])
            heights.append(h_hi)                                        # top of the object above the table plane
    for c_, lo_, hi_ in rejected:
        print(f'[1.5] rejected a cluster at {c_.round(3)}: spans {lo_ * 1000:.0f}-{hi_ * 1000:.0f} mm above the table (not resting on it)', flush=True)
    print(f'[1.5] {len(clusters)} cluster(s):')
    for i, cl in enumerate(clusters):
        ext = cl.max(0) - cl.min(0)
        print(f'      cluster {i}: {len(cl)} pts, extent {ext.round(3)}, top {heights[i] * 1000:.0f} mm above table, centroid {cl.mean(0).round(3)}')
        save_fig(f's1_5_cluster{i}', [(cl, 'tab:green', 4, f'object {i}')])
        save_ply(f'pointclouds/cluster_{i}.ply', cl)
    if not clusters:
        raise SystemExit('no object found above the table')
    order = np.argsort([float(np.hypot(*cl.mean(0)[:2])) for cl in clusters])          # nearest to the robot base first
    clusters, heights = [clusters[i] for i in order], [heights[i] for i in order]
    centroids = np.array([cl.mean(0) for cl in clusters])
    print('[2.1] centroids of all objects: ' + '; '.join(f'object {i}: ({c[0]:.3f}, {c[1]:.3f}, {c[2]:.3f})' for i, c in enumerate(centroids)), flush=True)
    t = int(sys.argv[sys.argv.index('--target') + 1]) if '--target' in sys.argv else 0
    if t >= len(clusters):
        raise SystemExit('no pickable object (all objects done)')
    print(f'      target = object {t} of {len(clusters)} on the table (nearest-first order)', flush=True)
    from matplotlib import colors as _mc2
    save_fig('s1_5_all_clusters', [(cl, _mc2.to_hex(plt.get_cmap('tab10')(i % 10)), 4, f'object {i}' + (' (TARGET)' if i == t else '')) for i, cl in enumerate(clusters)])
    save_fig('s2_1_centroids_all', [(cl, _mc2.to_hex(plt.get_cmap('tab10')(i % 10)), 4, f'object {i}') for i, cl in enumerate(clusters)],
             [(centroids, 'gold', 90, 'centroids')])
    obj = clusters[t]
    np.save('object_cloud.npy', obj)
    save_ply('pointclouds/target_object.ply', obj)

    json.dump(dict(target=t, objects=[dict(centroid=cl.mean(0).tolist(), extent=(cl.max(0) - cl.min(0)).tolist(), n=int(len(cl)), top=float(heights[i]))
                                      for i, cl in enumerate(clusters)]), open('objects.json', 'w'), indent=1)
    # 2.1-3 grasp synthesis on the target
    res = synthesize_grasp(obj, plane, figs=True)
    if res['width'] > P.get('gripper_max_width_m', 0.085):
        raise SystemExit(f'object too wide for the gripper: grasp width {res["width"] * 1000:.0f} mm > {P.get("gripper_max_width_m", 0.085) * 1000:.0f} mm')
    json.dump(dict(centroid=res['centroid'].tolist(), p1=res['p1'].tolist(), p2=res['p2'].tolist(), midpoint=res['mid'].tolist(),
                   grasp_axis=res['gax'].tolist(), approach=res['app'].tolist(), table_plane=[float(v) for v in plane], width=res['width'],
                   method=res['method'], quality=res['quality'], obj_height=res['obj_height'],
                   obj_top_z=float(obj[:, 2].max()), obj_min=obj.min(0).tolist(), obj_max=obj.max(0).tolist(), n_objects=len(clusters), target=t),
              open('grasp.json', 'w'), indent=1)
    save_ply('pointclouds/grasp_points.ply', np.array([res['centroid'], res['p1'], res['p2'], res['mid']]),
             np.array([[255, 215, 0], [255, 0, 0], [255, 0, 0], [0, 120, 255]], np.uint8))   # centroid gold, grasp points red, midpoint blue
    print('saved grasp.json and figures/')

if __name__ == '__main__':
    main()
