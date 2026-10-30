"""ROS 2 nodes of the HW7 pipeline. Each stage is a std_srvs/Trigger service; point clouds, grasp and plan go out on latched /hw7/* topics.

  python ros2_iface/hw7_nodes.py scan|perception|planner|executor     (normally started by ros2 launch ros2_iface/launch/pick.launch.py)

/hw7/scan (go to the scan pose, visit the fixed views, publish each view cloud)   param media_dir
/hw7/perceive (merge, table, clusters, grasp for object `target`)                 param target
/hw7/plan (IK check of pre-grasp / grasp / lift)
/hw7/execute (pre-grasp, close-range refinement, grasp, lift, put back)           param media_dir
/hw7/home (return to the scan pose)
The service response message is JSON: {"info": ..., "log": <the stage's printed output>}.
"""
import contextlib
import glob
import io
import json
import os
import sys
import traceback

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from viz_node import Viz, load_json, publish_set


class _Tee(io.TextIOBase):
    def __init__(self, *streams):
        self.streams = streams

    def write(self, x):
        for s in self.streams:
            s.write(x)
        return len(x)

    def flush(self):
        for s in self.streams:
            s.flush()


def captured(fn):
    """Run fn() printing live and capturing the output; returns (ok, info, log)."""
    buf = io.StringIO()
    ok, info = True, ''
    with contextlib.redirect_stdout(_Tee(sys.__stdout__, buf)):
        try:
            info = fn() or ''
        except SystemExit as exc:
            ok, info = False, str(exc.code or '')
        except Exception as exc:
            ok, info = False, f'{type(exc).__name__}: {exc}'
            traceback.print_exc()
    return ok, info, buf.getvalue()


class StageNode(Node):
    def __init__(self, name, service, handler, params=()):
        super().__init__(name)
        for p, default in params:
            self.declare_parameter(p, default)
        self.viz = Viz(self)
        self.create_service(Trigger, service, self._wrap(handler))
        self.get_logger().info(f'{service} ready')

    @staticmethod
    def _wrap(handler):
        def cb(req, res):
            ok, info, log = captured(handler)
            res.success, res.message = ok, json.dumps({'info': info, 'log': log})
            return res
        return cb


class ScanNode(StageNode):
    def __init__(self):
        super().__init__('hw7_scan', '/hw7/scan', self.run, [('media_dir', '')])
        from common import D405
        self.cam, self.acc = D405(), []

    def run(self):
        import numpy as np
        import scan
        from common import connect, BaseClient
        self.acc = []
        with connect() as router:
            scan.to_scan_pose(BaseClient(router))
        names = scan.main(cam=self.cam, media_dir=self.get_parameter('media_dir').value or None, on_view=self.on_view)
        return f'{len(names)} views'

    def on_view(self, name, xyz, rgb):
        import numpy as np
        self.viz.cloud('view_cloud', xyz, rgb)
        self.acc.append((xyz, rgb))
        self.viz.cloud('scan_views', np.vstack([a[0] for a in self.acc]), np.vstack([a[1] for a in self.acc]))


class PerceptionNode(StageNode):
    def __init__(self):
        super().__init__('hw7_perception', '/hw7/perceive', self.run, [('target', 0)])

    def run(self):
        import perceive
        try:
            perceive.main(target=int(self.get_parameter('target').value))
        finally:
            publish_set(self.viz, sorted(glob.glob('scans/scan_*.npz')), 'pointclouds', 'grasp.json', None)


class PlannerNode(StageNode):
    def __init__(self):
        super().__init__('hw7_planner', '/hw7/plan', self.run)

    def run(self):
        import grasp_plan
        ok = grasp_plan.main()
        g = load_json('grasp.json')
        if g:
            self.viz.grasp(g, load_json('plan.json'))
        if not ok:
            raise SystemExit('PLAN HAS PROBLEMS')
        return 'PLAN OK'


class ExecutorNode(StageNode):
    def __init__(self):
        super().__init__('hw7_executor', '/hw7/execute', self.run, [('media_dir', '')])
        from common import D405
        self.cam = D405()
        self.create_service(Trigger, '/hw7/home', self._wrap(self.home))

    def run(self):
        import grasp_exec
        try:
            return grasp_exec.main(cam=self.cam, media_dir=self.get_parameter('media_dir').value or None)
        except grasp_exec.GraspAbort as exc:
            raise SystemExit(f'SKIPPED: {exc}')

    def home(self):
        import scan
        from common import connect, BaseClient
        with connect() as router:
            scan.to_scan_pose(BaseClient(router))
        return 'at the scan pose'


ROLES = {'scan': ScanNode, 'perception': PerceptionNode, 'planner': PlannerNode, 'executor': ExecutorNode}


def main():
    rclpy.init()
    node = ROLES[sys.argv[1]]()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
