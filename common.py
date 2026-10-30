"""Shared helpers: arm, camera, motion and gripper (Kortex API; ROS 2 via ros2_iface)."""
import os
import sys

sys.path.insert(0, '/home/aryans-cat/Documents/GitHub/VBRM_insertion/Kinova-kortex2_Gen3_G3L/api_python/examples')

import cv2
import numpy as np
import pyrealsense2 as rs
from scipy.spatial.transform import Rotation as R

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from utilities import DeviceConnection

ARM_IP = '192.168.1.10'
CREDENTIALS = ('admin', 'admin')

TAG_DICT = cv2.aruco.DICT_4X4_50
TAG_ID = 0
TAG_SIZE = 0.100  # m

COLOR_W, COLOR_H, COLOR_FPS = 1280, 720, 15


def connect():
    return DeviceConnection(ARM_IP, port=DeviceConnection.TCP_PORT, credentials=CREDENTIALS)


def pose_to_matrix(p):
    """Kortex tool pose."""
    T = np.eye(4)
    T[:3, :3] = R.from_euler('xyz', [p.theta_x, p.theta_y, p.theta_z], degrees=True).as_matrix()
    T[:3, 3] = [p.x, p.y, p.z]
    return T


def measured_tool_matrix(base: BaseClient):
    return pose_to_matrix(base.GetMeasuredCartesianPose())


class D405:
    """Color + aligned depth stream with the colour intrinsics."""

    def __init__(self):
        for fps in (COLOR_FPS, 5):                                    # USB 2.1 links only carry 1280x720 at 5 fps
            self.pipe = rs.pipeline()
            cfg = rs.config()
            cfg.enable_stream(rs.stream.color, COLOR_W, COLOR_H, rs.format.bgr8, fps)
            cfg.enable_stream(rs.stream.depth, COLOR_W, COLOR_H, rs.format.z16, fps)
            self.cfg, self.fps = cfg, fps
            try:
                prof = self.pipe.start(cfg)
                break
            except RuntimeError:
                if fps == 5:
                    raise
        self.align = rs.align(rs.stream.color)
        self.depth_scale = prof.get_device().first_depth_sensor().get_depth_scale()
        i = prof.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        self.K = np.array([[i.fx, 0, i.ppx], [0, i.fy, i.ppy], [0, 0, 1.0]])
        self.dist = np.array(i.coeffs, dtype=np.float64)

    def _grab_direct(self, warmup=10):
        for _ in range(warmup):
            frames = self.align.process(self.pipe.wait_for_frames())
        color = np.asanyarray(frames.get_color_frame().get_data())
        depth = np.asanyarray(frames.get_depth_frame().get_data()).astype(np.float32) * self.depth_scale
        return color, depth

    def start_recording(self, path, fps=None):
        fps = fps or self.fps
        import threading as _th
        self._lock, self._latest, self._seq, self._stop = _th.Lock(), None, 0, False
        self._writer = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*'mp4v'), fps, (COLOR_W, COLOR_H))
        if not self._writer.isOpened():
            path = path.rsplit('.', 1)[0] + '.avi'
            self._writer = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*'MJPG'), fps, (COLOR_W, COLOR_H))
        self.video_path = path
        self._thread = _th.Thread(target=self._rec_loop, daemon=True)
        self._thread.start()

    def _restart(self, hard=False):
        """Restart the stream; with hard=True also reset the device (equivalent to a replug)."""
        import time as _t
        try:
            self.pipe.stop()
        except Exception:
            pass
        if hard:
            try:
                rs.context().query_devices()[0].hardware_reset()
                _t.sleep(6)
            except Exception:
                pass
        self.pipe = rs.pipeline()
        for _ in range(5):
            try:
                prof = self.pipe.start(self.cfg)
                self.depth_scale = prof.get_device().first_depth_sensor().get_depth_scale()
                return True
            except Exception:
                _t.sleep(1.5)
        return False

    def _rec_loop(self):
        import time as _t
        fails = restarts = 0
        while not self._stop:
            try:
                frames = self.align.process(self.pipe.wait_for_frames(3000))
                fails = restarts = 0
            except RuntimeError:
                fails += 1
                if fails >= 2:                                    # ~6 s without a frame: the stream is stalled
                    print(f'[camera] stream stalled, restarting (hard reset: {restarts >= 1})', flush=True)
                    self._restart(hard=restarts >= 1)
                    restarts, fails = restarts + 1, 0
                continue
            color = np.asanyarray(frames.get_color_frame().get_data()).copy()
            depth = np.asanyarray(frames.get_depth_frame().get_data()).astype(np.float32) * self.depth_scale
            with self._lock:
                self._latest, self._seq = (color, depth), self._seq + 1
            self._writer.write(color)

    def grab(self, warmup=10):
        if getattr(self, '_thread', None) is None:
            return self._grab_direct(warmup)
        import time as _t
        target = self._seq + warmup
        deadline = _t.time() + 30.0
        while self._seq < target:
            if _t.time() > deadline:
                raise RuntimeError('camera stalled: no frames for 30 s')
            _t.sleep(0.02)
        with self._lock:
            return self._latest

    def stop_recording(self):
        if getattr(self, '_thread', None) is not None:
            self._stop = True
            self._thread.join(timeout=5)
            self._writer.release()
            self._thread = None

    def close(self):
        self.stop_recording()
        self.pipe.stop()


def detect_tag(color, K, dist):
    """Return (T_cam_tag 4x4, corners 4x2) for TAG_ID, or None."""
    det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(TAG_DICT))
    corners, ids, _ = det.detectMarkers(cv2.cvtColor(color, cv2.COLOR_BGR2GRAY))
    if ids is None or TAG_ID not in ids.ravel():
        return None
    c = corners[list(ids.ravel()).index(TAG_ID)].reshape(4, 2).astype(np.float64)
    h = TAG_SIZE / 2
    obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float64)
    ok, rvec, tvec = cv2.solvePnP(obj, c, K, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok:
        return None
    T = np.eye(4)
    T[:3, :3] = cv2.Rodrigues(rvec)[0]
    T[:3, 3] = tvec.ravel()
    return T, c


import threading
import time

from kortex_api.autogen.messages import Base_pb2

# x, y, z limits (m) for any commanded tool position
from cfg import CFG as _CFG
_S = _CFG['safety']
SAFETY_ON = bool(_S.get('enabled', True))


def joint_cap(deg):
    """Per-move joint-step cap."""
    return deg if SAFETY_ON else 360.0


BOUNDS = {'x': tuple(_S['bounds']['x']), 'y': tuple(_S['bounds']['y']), 'z': (_S['z_min_near_zone'], _S['bounds']['z_max'])}


ZONE_AXIS_XY, ZONE_RADIUS = tuple(_S['zone']['axis_xy']), _S['zone']['radius']      # arm's base protection zone (cylinder)
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
        raise ValueError(f'tool position ({x:.3f}, {y:.3f}, {z:.3f}) outside safe bounds '
                         f'{BOUNDS}, z >= {Z_MIN_FAR} only when >= {ZONE_RADIUS + ZONE_MARGIN:.2f} m from the zone axis')


def move_to(base, x, y, z, theta_x, theta_y, theta_z, speed=0.03, rot_speed=10.0, timeout=90):
    """Blocking Cartesian move of the tool to a pose."""
    check_bounds(x, y, z)
    base.SetServoingMode(Base_pb2.ServoingModeInformation(servoing_mode=Base_pb2.SINGLE_LEVEL_SERVOING))
    done, result = threading.Event(), {}

    def cb(n):
        if n.action_event in (Base_pb2.ACTION_END, Base_pb2.ACTION_ABORT):
            result['event'] = n.action_event
            result['abort'] = n.abort_details
            done.set()
    handle = base.OnNotificationActionTopic(cb, Base_pb2.NotificationOptions())
    a = Base_pb2.Action()
    a.name = 'move_to'
    c = a.reach_pose.target_pose
    c.x, c.y, c.z, c.theta_x, c.theta_y, c.theta_z = x, y, z, theta_x, theta_y, theta_z
    a.reach_pose.constraint.speed.translation = speed
    a.reach_pose.constraint.speed.orientation = rot_speed
    try:
        base.ExecuteAction(a)
        if not done.wait(timeout):
            base.Stop()
            raise TimeoutError('move timed out; arm stopped')
    finally:
        base.Unsubscribe(handle)
    if result['event'] != Base_pb2.ACTION_END:
        raise RuntimeError(f'move aborted, details={result["abort"]}')
    return True


def _wrap180(d):
    return (np.asarray(d) + 180.0) % 360.0 - 180.0


def joint_margin_deg(q_deg):
    """Smallest distance (deg) of any joint from its limit."""
    lim = np.array(_CFG['robot']['joint_limits_deg'], dtype=float)
    signed = (np.asarray(q_deg, dtype=float) + 180.0) % 360.0 - 180.0
    return float(np.min(lim[:len(signed)] - np.abs(signed)))


def _ik_seeds(cur_deg):
    """IK seeds in order: the current joints, then the saved parking (scan) joints if they exist."""
    seeds = [cur_deg]
    try:
        from cfg import ROOT
        import json as _json
        p = os.path.join(ROOT, 'calibration', 'scan_joints.json')
        if os.path.exists(p):
            from cfg import load_json
            seeds.append(np.array(load_json(p), dtype=float))
    except Exception:
        pass
    return seeds


def move_joint(base, x, y, z, theta_x, theta_y, theta_z, duration=None, max_joint_deg=40.0, timeout=90):
    """Smooth move: IK for the target tool pose, then a joint-space move with a fixed duration."""
    check_bounds(x, y, z)
    max_joint_deg = joint_cap(max_joint_deg)
    cur = base.GetMeasuredJointAngles()
    ik = Base_pb2.IKData()
    t = ik.cartesian_pose
    t.x, t.y, t.z, t.theta_x, t.theta_y, t.theta_z = x, y, z, theta_x, theta_y, theta_z
    for a in cur.joint_angles:
        ik.guess.joint_angles.add().value = a.value
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
    target = cur_deg + delta
    if duration is None:
        duration = max(2.0, float(np.abs(delta).max()) / 8.0)       # <= 8 deg/s on the fastest joint
    if _SMOOTH:
        return smooth_joint_trajectory(base, target, duration)

    return _joint_action(base, target, duration, timeout)


SPEED = float(_CFG.get('motion', {}).get('speed_factor', 1.0))


def _joint_action(base, target, duration, timeout=90):
    """Run a joint-space move to `target` (deg) lasting `duration` s."""
    duration = duration / SPEED
    base.SetServoingMode(Base_pb2.ServoingModeInformation(servoing_mode=Base_pb2.SINGLE_LEVEL_SERVOING))
    done, result = threading.Event(), {}

    def cb(n):
        if n.action_event in (Base_pb2.ACTION_END, Base_pb2.ACTION_ABORT):
            result['event'], result['abort'] = n.action_event, n.abort_details
            done.set()
    handle = base.OnNotificationActionTopic(cb, Base_pb2.NotificationOptions())
    a = Base_pb2.Action()
    a.name = 'move_joint'
    for i, v in enumerate(target):
        j = a.reach_joint_angles.joint_angles.joint_angles.add()
        j.joint_identifier, j.value = i, float(v % 360.0)
    a.reach_joint_angles.constraint.type = Base_pb2.JOINT_CONSTRAINT_DURATION
    a.reach_joint_angles.constraint.value = float(duration)
    try:
        base.ExecuteAction(a)
        if not done.wait(timeout):
            base.Stop()
            raise TimeoutError('move timed out; arm stopped')
    finally:
        base.Unsubscribe(handle)
    if result['event'] != Base_pb2.ACTION_END:
        raise RuntimeError(f'move aborted, details={result["abort"]}')
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
    for tau in np.linspace(0.25, 1.0, 4):
        if not _fk_ok(base, cur + delta * tau):
            raise RuntimeError(f'joint move refused: path point at {tau:.2f} leaves the safe bounds')
    if duration is None:
        duration = max(2.0, float(np.abs(delta).max()) / 10.0)
    if _SMOOTH:
        return smooth_joint_trajectory(base, cur + delta, duration)
    return _joint_action(base, cur + delta, duration, timeout)


def grip_pos(base):
    req = Base_pb2.GripperRequest(); req.mode = Base_pb2.GRIPPER_POSITION
    return base.GetMeasuredGripperMovement(req).finger[0].value


def grip_cmd(base, v):
    cmd = Base_pb2.GripperCommand(); cmd.mode = Base_pb2.GRIPPER_POSITION
    f = cmd.gripper.finger.add(); f.finger_identifier = 1; f.value = float(np.clip(v, 0.0, 1.0))
    base.SendGripperCommand(cmd)


def _smoothstep(x):
    return x * x * x * (10 - 15 * x + 6 * x * x)          # minimum-jerk blend 0 -> 1


def grip_ramp(base, target, rate=0.25, dt=0.04, stall_gap=0.05, stall_steps=3):
    """Move the gripper to `target` along a minimum-jerk ramp at about `rate`."""
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


import signal

from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient

_M = _CFG.get('motion', {})
_SMOOTH = bool(_M.get('smooth', False))
_KP, _VMAX, _TOL = float(_M.get('kp', 2.5)), float(_M.get('vmax_deg_s', 30.0)), float(_M.get('tol_deg', 0.2))
_CYC = {}


def _cyclic():
    """One cached UDP cyclic client (fast joint feedback), closed at interpreter exit."""
    if 'c' not in _CYC:
        import atexit
        conn = DeviceConnection(ARM_IP, port=DeviceConnection.UDP_PORT, credentials=CREDENTIALS)
        router = conn.__enter__()
        _CYC['conn'], _CYC['c'] = conn, BaseCyclicClient(router)
        atexit.register(lambda: conn.__exit__(None, None, None))
    return _CYC['c']


def _speeds(v):
    js = Base_pb2.JointSpeeds()
    for i, x in enumerate(v):
        s = js.joint_speeds.add()
        s.joint_identifier, s.value, s.duration = i, float(x), 1       # 1 = shortest expiry the arm accepts
    return js


def _fk_ok(base, q_deg):
    """Tool-origin bounds check of a joint configuration via forward kinematics."""
    if not SAFETY_ON:
        return True
    try:
        ja = Base_pb2.JointAngles()
        for v in q_deg:
            ja.joint_angles.add().value = float(v % 360.0)
        p = base.ComputeForwardKinematics(ja)
        return in_safe_bounds(p.x, p.y, p.z)
    except Exception:
        return True                                                        # FK unavailable: endpoint checks still apply


def smooth_joint_trajectory(base, q_target_deg, T, dt=0.02):
    """Move all joints from the measured configuration to q_target along a minimum-jerk profile."""
    cyc = _cyclic()
    q_now = lambda: np.array([a.position for a in cyc.RefreshFeedback().actuators])
    q0 = q_now()
    delta = _wrap180(np.asarray(q_target_deg) - q0)
    for tau in np.linspace(0.1, 1.0, 10):                                  # path check before moving
        s = 10 * tau ** 3 - 15 * tau ** 4 + 6 * tau ** 5
        if not _fk_ok(base, q0 + delta * s):
            raise RuntimeError(f'smooth move refused: path point at {tau:.1f} leaves the safe bounds')
    base.SetServoingMode(Base_pb2.ServoingModeInformation(servoing_mode=Base_pb2.SINGLE_LEVEL_SERVOING))

    def _term(signum, frame):
        raise SystemExit('SIGTERM during a move')
    old = signal.signal(signal.SIGTERM, _term)
    trace = []
    t0 = time.perf_counter()
    try:
        while True:
            t = time.perf_counter() - t0
            tau = min(t / T, 1.0)
            s = 10 * tau ** 3 - 15 * tau ** 4 + 6 * tau ** 5
            ds = (30 * tau ** 2 - 60 * tau ** 3 + 30 * tau ** 4) / T
            q = q_now()
            err = _wrap180(q0 + delta * s - q)
            v = np.clip(delta * ds + _KP * err, -_VMAX, _VMAX)
            base.SendJointSpeedsCommand(_speeds(v))
            trace.append((t, q.copy(), v.copy()))
            if tau >= 1.0 and np.abs(_wrap180(np.asarray(q_target_deg) - q)).max() < _TOL:
                break
            if t > T + 3.0:
                raise TimeoutError(f'smooth move did not converge (residual {np.abs(_wrap180(np.asarray(q_target_deg) - q)).max():.2f} deg)')
            time.sleep(max(0.0, dt - (time.perf_counter() - t0 - t)))
    except BaseException:
        base.Stop()
        raise
    finally:
        try:
            base.SendJointSpeedsCommand(_speeds(np.zeros(len(q0))))
        finally:
            signal.signal(signal.SIGTERM, old)
    smooth_joint_trajectory.last_trace = trace
    return True


def lift_up(base, dz=0.12, speed=0.04 * SPEED, timeout=15.0):
    """Straight up in the base frame by dz metres using a Cartesian twist."""
    base.SetServoingMode(Base_pb2.ServoingModeInformation(servoing_mode=Base_pb2.SINGLE_LEVEL_SERVOING))
    z0 = base.GetMeasuredCartesianPose().z
    t = Base_pb2.TwistCommand()
    t.reference_frame = Base_pb2.CARTESIAN_REFERENCE_FRAME_BASE
    t.twist.linear_z = speed
    t.duration = 0
    t0 = time.time()
    try:
        base.SendTwistCommand(t)
        while time.time() - t0 < timeout:
            z = base.GetMeasuredCartesianPose().z
            if z - z0 >= dz:
                break
            if time.time() - t0 > 3.0 and z - z0 < 0.005:
                raise RuntimeError('lift_up: the arm is not moving')
            time.sleep(0.05)
    finally:
        base.Stop()
        time.sleep(0.5)
    return base.GetMeasuredCartesianPose().z - z0


def go_home(base, name='Home', timeout=90):
    """Run one of the arm's own built-in joint-angle actions (Home, Retract, Packaging, Zero)."""
    t = Base_pb2.RequestedActionType()
    t.action_type = Base_pb2.REACH_JOINT_ANGLES
    handle = next(a.handle for a in base.ReadAllActions(t).action_list if a.name == name)
    base.SetServoingMode(Base_pb2.ServoingModeInformation(servoing_mode=Base_pb2.SINGLE_LEVEL_SERVOING))
    done, result = threading.Event(), {}

    def cb(n):
        if n.action_event in (Base_pb2.ACTION_END, Base_pb2.ACTION_ABORT):
            result['event'], result['abort'] = n.action_event, n.abort_details
            done.set()
    sub = base.OnNotificationActionTopic(cb, Base_pb2.NotificationOptions())
    try:
        base.ExecuteActionFromReference(handle)
        if not done.wait(timeout):
            base.Stop()
            raise TimeoutError(f'{name} timed out; arm stopped')
    finally:
        base.Unsubscribe(sub)
    if result['event'] != Base_pb2.ACTION_END:
        raise RuntimeError(f'{name} aborted, details={result["abort"]}')
    return True


if os.environ.get('HW6_BACKEND', _CFG.get('backend', 'kortex')) == 'ros2':
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'ros2_iface'))
    import backend as _ros_backend
    _ros_backend.install(globals())
