#!/usr/bin/env python3
# Gripper-only measurement. The ARM NEVER MOVES here; only the two fingers open and close.
# Uses his own closing routine (grip_ramp) unchanged, and records what the gripper really reports
# while it closes: finger position and finger velocity at the joint_states rate.
#
#   from the repo root:
#   .venv/bin/python tools/gripper_probe.py empty
#   .venv/bin/python tools/gripper_probe.py banana_middle
#
# Hold the object between the OPEN pads with your hand (fingers away from the pads), press Enter, and keep holding
# it through the close. Results are saved in tools/results/<label>_<time>.json

import json
import os
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from rclpy.qos import qos_profile_sensor_data

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)
sys.path.insert(0, os.path.join(REPO, 'ros2_iface'))
from common import BaseClient, D405, GRIP_CLOSED, connect, grip_pos, grip_ramp   # noqa: E402

FINGER = 'right_finger_bottom_joint'
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'results')


class Logger(Node):
    def __init__(self):
        super().__init__('gripper_probe_logger')
        self.rows = []
        self.on = False
        self.create_subscription(JointState, '/joint_states', self.cb, 50)
        # the camera node only publishes aligned depth while raw depth is subscribed (his keepalive does the same)
        self.create_subscription(Image, '/camera/camera/depth/image_rect_raw', lambda m: None, qos_profile_sensor_data)
        self.ex = SingleThreadedExecutor()
        self.ex.add_node(self)
        self.stop = False
        self.th = threading.Thread(target=self.spin, daemon=True)
        self.th.start()

    def spin(self):
        while not self.stop and rclpy.ok():
            self.ex.spin_once(timeout_sec=0.02)

    def cb(self, m):
        if not self.on or FINGER not in m.name:
            return
        i = m.name.index(FINGER)
        vel = m.velocity[i] if i < len(m.velocity) else float('nan')
        self.rows.append((time.monotonic(), float(m.position[i]), float(vel)))


def summarise(rows, t_start):
    t = np.array([r[0] for r in rows]) - t_start
    pos = np.array([r[1] for r in rows]) / GRIP_CLOSED
    vel = np.array([r[2] for r in rows]) / GRIP_CLOSED
    moving = np.where(np.abs(vel) > 0.02)[0]
    out = {'samples': len(rows), 'rate_hz': round(len(rows) / max(t[-1] - t[0], 1e-6), 1)}
    tail = pos[t > t[-1] - 0.5]
    out['final_position'] = round(float(np.median(tail)), 4)
    out['final_position_spread'] = round(float(tail.max() - tail.min()), 4)
    out['peak_speed'] = round(float(np.nanmax(np.abs(vel))), 3)
    if len(moving):
        k = moving[-1]
        out['last_motion_at_s'] = round(float(t[k]), 2)
        out['position_when_motion_stopped'] = round(float(pos[k]), 4)
    return out


def depth_grid(depth, rows=(240, 320, 400, 480), cols=(0, 170, 340, 510, 680, 848)):
    # median valid depth (m) in a grid over the LOWER half of the image, where the fingertips and anything they hold appear
    out = []
    for r0, r1 in zip(rows[:-1], rows[1:]):
        line = []
        for c0, c1 in zip(cols[:-1], cols[1:]):
            d = depth[r0:r1, c0:c1]
            d = d[(d > 0.05) & (d < 1.5)]
            line.append(round(float(np.median(d)), 3) if len(d) > 200 else None)
        out.append(line)
    return out


def snapshot(cam, tag, stem):
    import cv2
    color, depth = cam.grab(warmup=6)
    cv2.imwrite(stem + f'_{tag}.png', color)
    np.savez_compressed(stem + f'_{tag}_depth.npz', depth_m=depth.astype(np.float16))
    grid = depth_grid(depth)
    print(f'  depth in the lower half of the image, {tag} (m; rows top to bottom, columns left to right):')
    for row in grid:
        print('    ' + '  '.join(f'{v:5.3f}' if v is not None else '  ---' for v in row))
    return grid


def main():
    label = sys.argv[1] if len(sys.argv) > 1 else 'test'
    rclpy.init()
    logger = Logger()
    cam = D405()
    stem = os.path.join(OUT, f'{label}_{time.strftime("%H%M%S")}')
    with connect() as router:
        base = BaseClient(router)
        print('opening the gripper ...', flush=True)
        grip_ramp(base, 0.0, rate=0.3)
        print(f'open reading: {grip_pos(base):.3f}')
        input(f'[{label}] Put the object between the OPEN pads (or leave it empty), keep fingers clear, then press Enter to CLOSE ... ')
        grid_open = snapshot(cam, 'open', stem)
        logger.rows.clear()
        t0 = time.monotonic()
        logger.on = True
        pos, stalled = grip_ramp(base, 1.0, rate=0.2)
        time.sleep(1.5)
        logger.on = False
        grid_closed = snapshot(cam, 'closed', stem)
        s = summarise(logger.rows, t0)
        s.update(label=label, his_routine_position=round(pos, 4), his_routine_stalled=bool(stalled),
                 his_rule_says_contact=bool(stalled or pos < 0.975), empty_closed_reading_in_his_code=1.0)
        path = stem + '.json'
        json.dump({'summary': s, 'rows': logger.rows, 'depth_grid_open': grid_open, 'depth_grid_closed': grid_closed},
                  open(path, 'w'))
        print('\n' + json.dumps(s, indent=1))
        print(f'raw log: {path}')
        input('\nHold the object, then press Enter to OPEN the gripper ... ')
        grip_ramp(base, 0.0, rate=0.3)
    cam.close()
    logger.stop = True
    logger.th.join(timeout=2.0)
    logger.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
