"""Shared helpers: ROS 2 arm / camera access, poses, safety bounds, motion and gripper (ros2_kortex + MoveIt + realsense2_camera)."""
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

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'ros2_iface'))
from robot_ros import Robot, COLOR, DEPTH, INFO   # noqa: E402

from cfg import CFG as _CFG   # noqa: E402

GRAB_TRIES = 3
STALL_S = 4.0                # a camera grab that gets no frame for this long counts as a stall
GRIP_CLOSED = 0.81           # finger joint (rad) of a fully closed, empty gripper, measured in the air (0 = open)
SPEED = float(_CFG.get('motion', {}).get('speed_factor', 1.0))
_S = _CFG['safety']
SAFETY_ON = bool(_S.get('enabled', True))
_NODE_IDS = itertools.count()


def joint_cap(deg):
    """Per-move joint-step cap; unlimited when software safety is off."""
    return deg if SAFETY_ON else 360.0


BOUNDS = {'x': tuple(_S['bounds']['x']), 'y': tuple(_S['bounds']['y']), 'z': (_S['z_min_near_zone'], _S['bounds']['z_max'])}
ZONE_AXIS_XY, ZONE_RADIUS = tuple(_S['zone']['axis_xy']), _S['zone']['radius']
ZONE_MARGIN = _S['zone']['margin']
Z_MIN_NEAR, Z_MIN_FAR = _S['z_min_near_zone'], _S['z_min_far_from_zone']


def in_safe_bounds(x, y, z):
    if not SAFETY_ON:
        return True
    if not (BOUNDS['x'][0] <= x <= BOUNDS['x'][1] and BOUNDS['y'][0] <= y <= BOUNDS['y'][1] and z <= BOUNDS['z'][1]):
        return False
    far = np.hypot(x - ZONE_AXIS_XY[0], y - ZONE_AXIS_XY[1]) >= ZONE_RADIUS + ZONE_MARGIN
    return z >= (Z_MIN_FAR if far else Z_MIN_NEAR)


def check_bounds(x, y, z):
    if not in_safe_bounds(x, y, z):
        raise ValueError(f'tool position ({x:.3f}, {y:.3f}, {z:.3f}) outside safe bounds {BOUNDS}')


def pose_to_matrix(p):
    """Tool pose (x, y, z m; theta x, y, z deg, extrinsic x-y-z) -> 4x4 base<-tool."""
    T = np.eye(4)
    T[:3, :3] = R.from_euler('xyz', [p.theta_x, p.theta_y, p.theta_z], degrees=True).as_matrix()
    T[:3, 3] = [p.x, p.y, p.z]
    return T


def _wrap180(d):
    return (np.asarray(d) + 180.0) % 360.0 - 180.0


def joint_margin_deg(q_deg):
    """Smallest distance (deg) of any joint from its limit."""
    lim = np.array(_CFG['robot']['joint_limits_deg'], dtype=float)
    return float(np.min(lim[:len(q_deg)] - np.abs(_wrap180(np.asarray(q_deg, dtype=float)))))


# ---- stand-ins for the few message fields the pipeline fills ----
class _Val:
    def __init__(self, v=0.0):
        self.value = v


class _List(list):
    def add(self):
        v = _Val()
        self.append(v)
        return v


class _Guess:
    def __init__(self):
        self.joint_angles = _List()

    def Clear(self):
        self.joint_angles = _List()


class IKData:
    def __init__(self):
        self.cartesian_pose = types.SimpleNamespace(x=0, y=0, z=0, theta_x=0, theta_y=0, theta_z=0)
        self.guess = _Guess()


Base_pb2 = types.SimpleNamespace(IKData=IKData)


def _deg360(q):
    return np.mod(np.asarray(q, dtype=float), 360.0)


class RosBase:
    """Arm state and IK for the pipeline, answered through ROS 2 (joint states, TF, MoveIt /compute_ik)."""

    def __init__(self, robot):
        self.r = robot

    def GetMeasuredCartesianPose(self):
        T = self.r.tool_matrix()
        e = R.from_matrix(T[:3, :3]).as_euler('xyz', degrees=True)
        return types.SimpleNamespace(x=T[0, 3], y=T[1, 3], z=T[2, 3], theta_x=e[0], theta_y=e[1], theta_z=e[2])

    def GetMeasuredJointAngles(self):
        return types.SimpleNamespace(joint_angles=[_Val(float(v)) for v in _deg360(self.r.joints_deg())])

    def ComputeInverseKinematics(self, d):
        c = d.cartesian_pose
        T = np.eye(4)
        T[:3, :3] = R.from_euler('xyz', [c.theta_x, c.theta_y, c.theta_z], degrees=True).as_matrix()
        T[:3, 3] = [c.x, c.y, c.z]
        seed = [a.value for a in d.guess.joint_angles]
        seed = None if len(seed) != 6 else _wrap180(np.array(seed))
        q = self.r.ik(T, seed)
        if q is None:
            raise RuntimeError('IK: no solution')
        return types.SimpleNamespace(joint_angles=[_Val(float(v)) for v in _deg360(q)])

    def Stop(self):
        self.r.stop()


class _Conn:
    def __enter__(self):
        if not rclpy.ok():
            rclpy.init()
        self.robot = Robot(f'hw7_robot_{next(_NODE_IDS)}')
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
    """Camera through the realsense2_camera topics.

    Default (on demand): subscribes only while grab() is running, so idle nodes do not pull full-rate video through DDS.
    continuous=True: keeps the colour stream (and depth with depth=True) subscribed on a background thread, for video recording."""

    def __init__(self, continuous=False, depth=True):
        if not rclpy.ok():
            rclpy.init()
        self.node = Node(f'hw7_camera_{next(_NODE_IDS)}')
        self._ex = SingleThreadedExecutor()
        self._ex.add_node(self.node)
        self._lock, self._color, self._depth, self._seq, self._writer = threading.Lock(), None, None, 0, None
        self.K, self.dist = None, np.zeros(5)
        self.continuous, self._want_depth, self._subs = continuous, depth, []
        if continuous:
            self._subscribe(depth)
            self._stop = False
            self._th = threading.Thread(target=self._spin, daemon=True)
            self._th.start()
            t0 = time.time()
            while self._color is None and time.time() - t0 < 15:
                time.sleep(0.05)
            if self._color is None:
                raise RuntimeError('no camera frames: is realsense2_camera running with align_depth.enable:=true ?')

    def _subscribe(self, depth=True):
        with self._lock:
            self._color, self._depth, self._seq = None, None, 0
        self._subs = [self.node.create_subscription(Image, COLOR, self._on_color, qos_profile_sensor_data)]
        if depth:
            self._subs.append(self.node.create_subscription(Image, DEPTH, self._on_depth, qos_profile_sensor_data))
            self._subs.append(self.node.create_subscription(CameraInfo, INFO, self._on_info, qos_profile_sensor_data))

    def _unsubscribe(self):
        for s in self._subs:
            self.node.destroy_subscription(s)
        self._subs = []

    def _spin(self):
        while not self._stop and rclpy.ok():
            self._ex.spin_once(timeout_sec=0.05)

    def _on_info(self, m):
        self.K = np.array(m.k).reshape(3, 3)

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
        """(color BGR, depth metres) after `warmup` new colour frames."""
        if self.continuous:
            with self._lock:
                target = self._seq + max(1, warmup)
            t0 = time.time()
            while time.time() - t0 < STALL_S:
                with self._lock:
                    if self._seq >= target:
                        break
                time.sleep(0.02)
            else:
                raise RuntimeError(f'camera stalled: no frames for {STALL_S:.0f} s')
            time.sleep(0.05)
        else:
            for attempt in range(GRAB_TRIES):                      # a fresh subscription sometimes takes a while to match: retry with new ones
                self._subscribe(True)
                try:
                    t0 = time.time()
                    while True:
                        self._ex.spin_once(timeout_sec=0.05)
                        with self._lock:
                            if self._seq >= max(1, warmup) and self._depth is not None and self.K is not None:
                                break
                        if time.time() - t0 > STALL_S:
                            raise TimeoutError
                    break
                except TimeoutError:
                    if attempt == GRAB_TRIES - 1:
                        raise RuntimeError(f'camera stalled: no frames for {STALL_S:.0f} s ({GRAB_TRIES} tries)')
                finally:
                    self._unsubscribe()
        with self._lock:
            return self._color.copy(), (self._depth.copy() if self._depth is not None else None)

    def start_recording(self, path, fps=15):
        import cv2
        h, w = self._color.shape[:2]
        self._writer = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*'mp4v'), fps, (w, h))
        if not self._writer.isOpened():
            self._writer = cv2.VideoWriter(path.rsplit('.', 1)[0] + '.avi', cv2.VideoWriter_fourcc(*'MJPG'), fps, (w, h))

    def stop_recording(self):
        if self._writer is not None:
            self._writer.release()
            self._writer = None

    def close(self):
        self.stop_recording()
        if self.continuous:
            self._stop = True
            self._th.join(timeout=2)
        self.node.destroy_node()


def _ik_seeds(cur_deg):
    """IK seeds in order: the current joints, then the saved home (scan) joints."""
    seeds = [cur_deg]
    try:
        from cfg import ROOT, load_json
        p = os.path.join(ROOT, 'calibration', 'scan_joints.json')
        if os.path.exists(p):
            seeds.append(np.array(load_json(p), dtype=float))
    except Exception:
        pass
    return seeds


def move_joint(base, x, y, z, theta_x, theta_y, theta_z, duration=None, max_joint_deg=40.0, timeout=90):
    """IK for the target tool pose, then a joint-space move of fixed duration; refuses joint steps above max_joint_deg."""
    check_bounds(x, y, z)
    max_joint_deg = joint_cap(max_joint_deg)
    cur = base.GetMeasuredJointAngles()
    ik = Base_pb2.IKData()
    t = ik.cartesian_pose
    t.x, t.y, t.z, t.theta_x, t.theta_y, t.theta_z = x, y, z, theta_x, theta_y, theta_z
    cur_deg = np.array([a.value for a in cur.joint_angles])
    sol, last_exc = None, None
    for guess in _ik_seeds(cur_deg):
        ik.guess.Clear()
        for v in guess:
            ik.guess.joint_angles.add().value = float(v)
        try:
            sol = base.ComputeInverseKinematics(ik)
            break
        except Exception as exc:
            last_exc = exc
    if sol is None:
        raise last_exc
    sol_deg = np.array([a.value for a in sol.joint_angles])
    delta = _wrap180(sol_deg - cur_deg)
    if np.abs(delta).max() > max_joint_deg:
        raise RuntimeError(f'IK wants a {np.abs(delta).max():.0f} deg joint move (> {max_joint_deg}); refusing')
    if np.abs(delta).max() < 0.3:
        return True
    if duration is None:
        duration = max(2.0, float(np.abs(delta).max()) / 8.0)
    return _joint_action(base, cur_deg + delta, duration, timeout)


def _joint_action(base, target, duration, timeout=90):
    """Joint-space move to `target` (deg) lasting `duration` s (shortened by motion.speed_factor)."""
    q = _wrap180(np.asarray(target, dtype=float))
    base.r.move_joints(q, max(0.5, duration / SPEED))
    return True


def move_to_joints(base, q_deg, duration=None, max_joint_deg=90.0, timeout=90):
    """Move to explicit joint angles (deg)."""
    cur = np.array([a.value for a in base.GetMeasuredJointAngles().joint_angles])
    delta = _wrap180(np.asarray(q_deg, dtype=float) - cur)
    max_joint_deg = joint_cap(max_joint_deg)
    if np.abs(delta).max() > max_joint_deg:
        raise RuntimeError(f'joint move of {np.abs(delta).max():.0f} deg (> {max_joint_deg}); refusing')
    if np.abs(delta).max() < 0.3:
        return True
    if duration is None:
        duration = max(2.0, float(np.abs(delta).max()) / 10.0)
    return _joint_action(base, cur + delta, duration, timeout)


def lift_up(base, dz=0.12, speed=0.04, timeout=15.0):
    """Straight up by dz metres (MoveIt Cartesian path)."""
    T = base.r.tool_matrix()
    z0 = T[2, 3]
    T[2, 3] += dz
    base.r.move_line(T, max(1.0, dz / 0.05 / SPEED))
    return base.r.tool_matrix()[2, 3] - z0


def go_home(base, name='Home', timeout=90):
    """Move to the saved home (scan) joints."""
    from cfg import ROOT, load_json
    _joint_action(base, load_json(os.path.join(ROOT, 'calibration', 'scan_joints.json')), 10.0)


# ---- gripper: 0 = open, 1 = closed (scaled from the finger joint) ----
def grip_pos(base):
    base.r.spin_until(lambda: False, 0.05)
    return float(np.clip((getattr(base.r, 'grip_pos_msr', 0.0) or 0.0) / GRIP_CLOSED, 0.0, 1.0))


def grip_cmd(base, v):
    from control_msgs.action import GripperCommand
    goal = GripperCommand.Goal()
    goal.command.position = float(np.clip(v, 0.0, 1.0)) * GRIP_CLOSED
    goal.command.max_effort = 30.0
    base.r.grip.send_goal_async(goal)
    base.r.spin_until(lambda: False, 0.01)


def _smoothstep(x):
    return x * x * x * (10 - 15 * x + 6 * x * x)


def grip_ramp(base, target, rate=0.25, dt=0.04, stall_gap=0.05, stall_steps=3):
    """Move the gripper to `target` along a minimum-jerk ramp; while closing, stop and hold if the fingers lag the command (contact). Returns (position, contact)."""
    start = grip_pos(base)
    n = max(2, int(abs(target - start) / (rate * dt)))
    lag = 0
    for i in range(1, n + 1):
        cmd = start + (target - start) * _smoothstep(i / n)
        grip_cmd(base, cmd)
        time.sleep(dt)
        if target > start and i > n * 0.15:
            lag = lag + 1 if cmd - grip_pos(base) > stall_gap else 0
            if lag >= stall_steps:
                pos = grip_pos(base)
                grip_cmd(base, min(1.0, pos + 0.04))
                time.sleep(0.5)
                return grip_pos(base), True
    time.sleep(0.6)
    return grip_pos(base), False
