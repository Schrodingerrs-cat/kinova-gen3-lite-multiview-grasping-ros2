"""Publishes the pipeline's point clouds and grasp to ROS 2 for RViz (frame base_link, latched topics).

Live:    python ros2_iface/viz_node.py                          watches scans/, pointclouds/, grasp.json, plan.json of the working folder
Replay:  python ros2_iface/viz_node.py --run runs/<ts> [--object k]   publishes a saved run (no robot needed)

Topics: /hw7/scan_views (raw clouds of all views), /hw7/merged, /hw7/table, /hw7/objects (clusters, one colour each), /hw7/target, /hw7/grasp (markers).
"""
import argparse
import glob
import json
import os
import sys
import time

import numpy as np
import open3d as o3d
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray

FRAME = 'base_link'
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
PALETTE = np.array([[230, 25, 75], [60, 180, 75], [255, 225, 25], [0, 130, 200], [245, 130, 48], [145, 30, 180], [70, 240, 240], [240, 50, 230]]) / 255.0
MAX_POINTS = 300000


def pack(xyz, rgb):
    """PointCloud2 data from Nx3 xyz (m) and Nx3 rgb (0..1)."""
    xyz = np.asarray(xyz, np.float32)
    c = (np.clip(rgb, 0, 1) * 255).astype(np.uint32)
    packed = ((c[:, 0] << 16) | (c[:, 1] << 8) | c[:, 2]).view(np.float32)
    arr = np.zeros(len(xyz), dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4'), ('rgb', 'f4')])
    arr['x'], arr['y'], arr['z'], arr['rgb'] = xyz[:, 0], xyz[:, 1], xyz[:, 2], packed
    return arr.tobytes()


def cloud_msg(xyz, rgb, stamp):
    m = PointCloud2()
    m.header = Header(frame_id=FRAME, stamp=stamp)
    m.height, m.width = 1, len(xyz)
    m.fields = [PointField(name=n, offset=o, datatype=PointField.FLOAT32, count=1) for n, o in (('x', 0), ('y', 4), ('z', 8), ('rgb', 12))]
    m.is_bigendian, m.point_step, m.row_step, m.is_dense = False, 16, 16 * len(xyz), True
    m.data = pack(xyz, rgb)
    return m


def read_ply(path):
    p = o3d.io.read_point_cloud(path)
    xyz = np.asarray(p.points)
    rgb = np.asarray(p.colors) if p.has_colors() else np.full((len(xyz), 3), 0.8)
    return xyz, rgb


def thin(xyz, rgb):
    if len(xyz) > MAX_POINTS:
        i = np.random.default_rng(0).choice(len(xyz), MAX_POINTS, replace=False)
        return xyz[i], rgb[i]
    return xyz, rgb


class Viz(Node):
    def __init__(self):
        super().__init__('hw7_viz')
        self.pubs = {n: self.create_publisher(PointCloud2, f'/hw7/{n}', LATCHED) for n in ('scan_views', 'merged', 'table', 'objects', 'target')}
        self.grasp_pub = self.create_publisher(MarkerArray, '/hw7/grasp', LATCHED)

    def cloud(self, name, xyz, rgb):
        if len(xyz):
            xyz, rgb = thin(xyz, rgb)
            self.pubs[name].publish(cloud_msg(xyz, rgb, self.get_clock().now().to_msg()))

    def grasp(self, g, plan):
        arr, now = MarkerArray(), self.get_clock().now().to_msg()

        def marker(i, kind, rgba, scale):
            m = Marker(); m.header = Header(frame_id=FRAME, stamp=now); m.ns, m.id, m.type, m.action = 'grasp', i, kind, Marker.ADD
            m.scale.x, m.scale.y, m.scale.z = (float(s) for s in scale); m.color.r, m.color.g, m.color.b, m.color.a = (float(c) for c in rgba)
            m.pose.orientation.w = 1.0
            return m

        def pt(v):
            return Point(x=float(v[0]), y=float(v[1]), z=float(v[2]))
        spheres = [('centroid', g['centroid'], (1, 0.84, 0, 1)), ('p1', g['p1'], (1, 0, 0, 1)), ('p2', g['p2'], (1, 0, 0, 1)), ('mid', g['midpoint'], (0, 0.47, 1, 1))]
        for i, (_, v, c) in enumerate(spheres):
            m = marker(i, Marker.SPHERE, c, (0.014, 0.014, 0.014)); m.pose.position = pt(v); arr.markers.append(m)
        mid, app = np.array(g['midpoint']), np.array(g['approach'])
        a = marker(10, Marker.ARROW, (0, 0.8, 0.8, 1), (0.006, 0.012, 0.0)); a.points = [pt(mid - app * 0.08), pt(mid)]; arr.markers.append(a)
        if plan:
            path = marker(11, Marker.LINE_STRIP, (1, 0.5, 0, 1), (0.004, 0, 0))
            for n in ('pre_grasp', 'grasp', 'lift'):
                path.points.append(pt(plan['poses'][n]['xyz']))
            arr.markers.append(path)
            Rg = R.from_euler('xyz', plan['poses']['grasp']['theta'], degrees=True).as_matrix()
            o = np.array(plan['poses']['grasp']['xyz'])
            for j, col in enumerate(((1, 0, 0, 1), (0, 1, 0, 1), (0, 0, 1, 1))):
                ax = marker(20 + j, Marker.ARROW, col, (0.004, 0.008, 0.0)); ax.points = [pt(o), pt(o + Rg[:, j] * 0.05)]; arr.markers.append(ax)
        self.grasp_pub.publish(arr)


def load_json(path):
    try:
        return json.load(open(path))
    except Exception:
        return None


def publish_set(v, scans, stages, grasp_json, plan_json):
    """Publish whatever exists from the given file locations."""
    if scans:
        xs, cs = [], []
        for f in scans:
            d = np.load(f)
            xs.append(d['xyz']); cs.append(d['rgb'])
        v.cloud('scan_views', np.vstack(xs), np.vstack(cs))
    one = lambda n: os.path.join(stages, n)
    if os.path.exists(one('merged_downsampled.ply')):
        v.cloud('merged', *read_ply(one('merged_downsampled.ply')))
    if os.path.exists(one('table_plane.ply')):
        x, _ = read_ply(one('table_plane.ply')); v.cloud('table', x, np.tile([0.3, 0.5, 0.9], (len(x), 1)))
    cl = sorted(glob.glob(one('cluster_*.ply')))
    if cl:
        xs, cs = [], []
        for i, f in enumerate(cl):
            x, _ = read_ply(f); xs.append(x); cs.append(np.tile(PALETTE[i % len(PALETTE)], (len(x), 1)))
        v.cloud('objects', np.vstack(xs), np.vstack(cs))
    if os.path.exists(one('target_object.ply')):
        x, _ = read_ply(one('target_object.ply')); v.cloud('target', x, np.tile([0, 1, 0], (len(x), 1)))
    g = load_json(grasp_json)
    if g:
        v.grasp(g, load_json(plan_json))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run', help='replay a saved run folder (runs/<ts>)')
    ap.add_argument('--object', type=int, default=0, help='with --run: which object folder (0-based)')
    a = ap.parse_args()
    rclpy.init()
    v = Viz()
    if a.run:
        od = os.path.join(a.run, f'object_{a.object}')
        publish_set(v, sorted(glob.glob(os.path.join(a.run, 'scan/02_pointclouds/raw_views_npz/scan_*.npz'))), os.path.join(od, '02_pointclouds/stages'),
                    os.path.join(od, '04_execution/grasp.json'), os.path.join(od, '04_execution/plan.json'))
        v.get_logger().info(f'replaying {a.run} object {a.object}: topics /hw7/* (frame {FRAME}); Ctrl-C to stop')
        rclpy.spin(v)
        return
    v.get_logger().info('watching scans/, pointclouds/, grasp.json, plan.json; topics /hw7/*')
    seen = None
    while rclpy.ok():
        files = glob.glob('scans/scan_*.npz') + glob.glob('pointclouds/*.ply') + ['grasp.json', 'plan.json']
        sig = tuple(sorted((f, os.path.getmtime(f)) for f in files if os.path.exists(f)))
        if sig != seen:
            seen = sig
            try:
                publish_set(v, sorted(glob.glob('scans/scan_*.npz')), 'pointclouds', 'grasp.json', 'plan.json')
            except Exception as exc:
                v.get_logger().warn(f'skipped a half-written file ({exc})')
        rclpy.spin_once(v, timeout_sec=1.0)


if __name__ == '__main__':
    main()
