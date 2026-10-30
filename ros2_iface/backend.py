"""ROS 2 backend for common.py."""
import itertools
import os
import sys
import threading
import time
import types

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CameraInfo, Image

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from robot_ros import Robot, COLOR, DEPTH, INFO   # noqa: E402

_NODE_IDS = itertools.count()
GRIP_CLOSED = 0.81


def _init():
    if not rclpy.ok():
        rclpy.init()


def _deg360(q):
    return np.mod(np.asarray(q, dtype=float), 360.0)


class _Val:
    def __init__(self, v=0.0): self.value = v


class _List(list):
    def add(self):
        v = _Val(); self.append(v); return v


class _Guess:
    def __init__(self): self.joint_angles = _List()
    def Clear(self): self.joint_angles = _List()


class IKData:
    def __init__(self):
        self.cartesian_pose = types.SimpleNamespace(x=0, y=0, z=0, theta_x=0, theta_y=0, theta_z=0)
        self.guess = _Guess()


Base_pb2 = types.SimpleNamespace(IKData=IKData)


class RosBase:
    """The handful of BaseClient calls the pipeline makes, answered through ROS 2."""
    def __init__(self, robot): self.r = robot

    def GetMeasuredCartesianPose(self):
        T = self.r.tool_matrix()
        e = R.from_matrix(T[:3, :3]).as_euler('xyz', degrees=True)          # Kortex convention: extrinsic x-y-z
        return types.SimpleNamespace(x=T[0, 3], y=T[1, 3], z=T[2, 3], theta_x=e[0], theta_y=e[1], theta_z=e[2])

    def GetMeasuredJointAngles(self):
        return types.SimpleNamespace(joint_angles=[_Val(float(v)) for v in _deg360(self.r.joints_deg())])

    def ComputeInverseKinematics(self, d):
        c = d.cartesian_pose
        T = np.eye(4); T[:3, :3] = R.from_euler('xyz', [c.theta_x, c.theta_y, c.theta_z], degrees=True).as_matrix(); T[:3, 3] = [c.x, c.y, c.z]
        seed = [a.value for a in d.guess.joint_angles]
        seed = None if len(seed) != 6 else (np.mod(np.array(seed) + 180.0, 360.0) - 180.0)
        q = self.r.ik(T, seed)
        if q is None:
            raise RuntimeError('IK: no solution')
        return types.SimpleNamespace(joint_angles=[_Val(float(v)) for v in _deg360(q)])

    def Stop(self): self.r.stop()
    def ClearFaults(self): pass
    def SetServoingMode(self, *a, **k): pass


class _Conn:
    def __enter__(self):
        _init()
        self.robot = Robot(f'hw6_robot_{next(_NODE_IDS)}')
        if not self.robot.wait_ready(15.0):
            raise RuntimeError('ROS 2 stack not ready: start the kortex_bringup driver (joint_states, controllers) first')
        return self.robot

    def __exit__(self, *a):
        self.robot.destroy_node()


def connect():
    return _Conn()


def BaseClient(router):
    return RosBase(router)


class D405:
    """Camera through the realsense2_camera topics."""
    def __init__(self):
        _init()
        self.node = Node('hw6_camera')
        self._ex = SingleThreadedExecutor(); self._ex.add_node(self.node)
        self._lock, self._color, self._depth, self._seq, self._writer = threading.Lock(), None, None, 0, None
        self.K, self.dist = None, np.zeros(5)
        self.node.create_subscription(Image, COLOR, self._on_color, qos_profile_sensor_data)
        self.node.create_subscription(Image, DEPTH, self._on_depth, qos_profile_sensor_data)
        self.node.create_subscription(CameraInfo, INFO, self._on_info, qos_profile_sensor_data)
        self._stop = False
        self._th = threading.Thread(target=self._spin, daemon=True); self._th.start()
        t0 = time.time()
        while (self.K is None or self._color is None or self._depth is None) and time.time() - t0 < 15:
            time.sleep(0.05)
        if self.K is None or self._color is None or self._depth is None:
            raise RuntimeError('no camera frames: is realsense2_camera running with align_depth.enable:=true ?')

    def _spin(self):
        while not self._stop and rclpy.ok():
            self._ex.spin_once(timeout_sec=0.05)

    def _on_info(self, m): self.K = np.array(m.k).reshape(3, 3)

    def _on_color(self, m):
        c = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)
        c = np.ascontiguousarray(c[:, :, ::-1] if m.encoding == 'rgb8' else c)
        with self._lock:
            self._color, self._seq = c, self._seq + 1
        if self._writer is not None:
            self._writer.write(c)

    def _on_depth(self, m):
        d = np.frombuffer(m.data, np.uint16).reshape(m.height, m.width).astype(np.float32) * 1e-3
        with self._lock:
            self._depth = d

    def grab(self, warmup=10):
        with self._lock:
            target = self._seq + max(1, warmup)
        t0 = time.time()
        while time.time() - t0 < 30:
            with self._lock:
                if self._seq >= target:
                    break
            time.sleep(0.02)
        else:
            raise RuntimeError('camera stalled: no frames for 30 s')
        time.sleep(0.05)
        with self._lock:
            return self._color.copy(), self._depth.copy()

    def start_recording(self, path, fps=15):
        import cv2
        h, w = self._color.shape[:2]
        self._writer = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*'mp4v'), fps, (w, h))
        if not self._writer.isOpened():
            self._writer = cv2.VideoWriter(path.rsplit('.', 1)[0] + '.avi', cv2.VideoWriter_fourcc(*'MJPG'), fps, (w, h))

    def stop_recording(self):
        if self._writer is not None:
            self._writer.release(); self._writer = None

    def close(self):
        self.stop_recording()
        self._stop = True
        self._th.join(timeout=2)
        self.node.destroy_node()


def install(ns):
    """Override the Kortex-based names in common.py's namespace `ns`."""
    from cfg import CFG
    speed = float(CFG.get('motion', {}).get('speed_factor', 1.0))

    def joint_action(base, target, duration, timeout=90):
        q = np.mod(np.asarray(target, dtype=float) + 180.0, 360.0) - 180.0         # ROS joint limits are in (-180, 180]
        base.r.move_joints(q, max(0.5, duration / speed))
        return True

    def grip_pos(base):
        base.r.spin_until(lambda: False, 0.05)
        return float(np.clip((getattr(base.r, 'grip_pos_msr', 0.0) or 0.0) / GRIP_CLOSED, 0.0, 1.0))

    def grip_cmd(base, v):
        from control_msgs.action import GripperCommand
        goal = GripperCommand.Goal(); goal.command.position = float(np.clip(v, 0.0, 1.0)) * GRIP_CLOSED; goal.command.max_effort = 30.0
        base.r.grip.send_goal_async(goal)
        base.r.spin_until(lambda: False, 0.01)

    def lift_up(base, dz=0.12, speed_=0.04, timeout=15.0):
        T = base.r.tool_matrix(); z0 = T[2, 3]
        T[2, 3] += dz
        base.r.move_line(T, max(1.0, dz / 0.05 / speed))
        return base.r.tool_matrix()[2, 3] - z0

    def go_home(base, name='Home', timeout=90):
        from cfg import load_json, ROOT
        q = load_json(os.path.join(ROOT, 'calibration', 'scan_joints.json'))
        joint_action(base, q, 10.0)

    ns.update(connect=connect, BaseClient=BaseClient, D405=D405, Base_pb2=Base_pb2, _joint_action=joint_action, grip_pos=grip_pos, grip_cmd=grip_cmd,
              lift_up=lift_up, go_home=go_home, _fk_ok=lambda base, q: True)
