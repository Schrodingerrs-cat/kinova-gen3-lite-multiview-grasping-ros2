#!/usr/bin/env python3
# Passive flight recorder. Run it in its own terminal BEFORE ./run_ros.sh and leave it running. It never commands
# anything. It only listens and records, at full rate:
#   - the finger joint position and the six arm joints from /joint_states  (about 1 kHz)
#   - median depth in a few regions of the wrist camera's aligned depth image  (about 30 Hz)
# Stop it with Ctrl-C after the run; it saves tools/results/flight_<time>.npz and prints a short summary.
#
#   from the repo root:  .venv/bin/python tools/flight_recorder.py

import os
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'results')
FINGER = 'right_finger_bottom_joint'
GRIP_CLOSED = 0.81
ARM = [f'joint_{i}' for i in range(1, 7)]
# regions of the 848x480 image, (row0, row1, col0, col1)
ROIS = {'tips_centre': (400, 480, 340, 510), 'low_centre': (320, 480, 255, 595),
        'low_left': (320, 480, 0, 255), 'low_right': (320, 480, 595, 848), 'whole': (0, 480, 0, 848)}


class Recorder(Node):
    def __init__(self):
        super().__init__('flight_recorder')
        self.t_js, self.finger, self.arm_t, self.arm_q = [], [], [], []
        self.t_d, self.roi = [], {k: [] for k in ROIS}
        self.n_js = 0
        self.create_subscription(JointState, '/joint_states', self.on_js, 200)
        self.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw', self.on_depth, qos_profile_sensor_data)

    def on_js(self, m):
        t = time.time()
        if FINGER in m.name:
            self.t_js.append(t)
            self.finger.append(m.position[m.name.index(FINGER)] / GRIP_CLOSED)
        self.n_js += 1
        if self.n_js % 20 == 0 and all(j in m.name for j in ARM):
            self.arm_t.append(t)
            self.arm_q.append([m.position[m.name.index(j)] for j in ARM])

    def on_depth(self, m):
        d = np.frombuffer(m.data, np.uint16).reshape(m.height, m.width)
        self.t_d.append(time.time())
        for k, (r0, r1, c0, c1) in ROIS.items():
            v = d[r0:r1, c0:c1]
            v = v[(v > 50) & (v < 1500)]
            self.roi[k].append(float(np.median(v)) * 1e-3 if len(v) > 200 else np.nan)

    def save(self):
        path = os.path.join(OUT, f'flight_{time.strftime("%H%M%S")}.npz')
        np.savez_compressed(path, t_js=np.array(self.t_js), finger=np.array(self.finger), arm_t=np.array(self.arm_t),
                            arm_q=np.array(self.arm_q), t_d=np.array(self.t_d), **{f'roi_{k}': np.array(v) for k, v in self.roi.items()})
        return path


def main():
    os.makedirs(OUT, exist_ok=True)
    rclpy.init()
    r = Recorder()
    print('recording (passive). Start ./run_ros.sh in another terminal. Ctrl-C here when it has finished.', flush=True)
    last = time.time()
    try:
        while rclpy.ok():
            rclpy.spin_once(r, timeout_sec=0.02)
            if time.time() - last > 5:
                f = r.finger[-1] if r.finger else float('nan')
                d = r.roi['tips_centre'][-1] if r.roi['tips_centre'] else float('nan')
                print(f'  {len(r.finger)} finger samples, {len(r.t_d)} depth frames | finger {f:.3f} | bottom-centre depth {d:.3f} m', flush=True)
                last = time.time()
    except KeyboardInterrupt:
        pass
    path = r.save()
    print(f'saved {path}')
    r.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
