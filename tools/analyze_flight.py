#!/usr/bin/env python3
# Reads a flight_*.npz and prints, around every gripper CLOSE event, a table of what the gripper and the wrist
# camera reported, once per second: finger position, depth between the fingertips, and how much the arm moved.
#   from the repo root:  .venv/bin/python tools/analyze_flight.py [file]   (default: newest)
import glob
import os
import sys

import numpy as np

here = os.path.dirname(os.path.abspath(__file__))
f = sys.argv[1] if len(sys.argv) > 1 else sorted(glob.glob(os.path.join(here, 'results', 'flight_*.npz')))[-1]
d = np.load(f)
t0 = d['t_js'][0]
tj, fin = d['t_js'] - t0, d['finger']
td = d['t_d'] - t0
print(f'{os.path.basename(f)}: {len(tj)} finger samples over {tj[-1]:.0f} s, {len(td)} depth frames')

# a close event = the finger position rises through 0.3 from below
idx = np.where((fin[1:] >= 0.3) & (fin[:-1] < 0.3))[0]
if not len(idx):
    print('no gripper close event in this recording'); sys.exit()
arm_t, arm_q = d['arm_t'] - t0, d['arm_q']
for n, i in enumerate(idx, 1):
    tc = tj[i]
    after = (tj > tc) & (tj < tc + 40)
    plateau = tj[after][np.argmax(fin[after] > fin[after].max() - 0.004)] if after.any() else tc
    print(f'\n=== close event {n}: finger passes 0.3 at t = {tc:.1f} s;  reaches its final value {fin[after].max():.4f} at t = {plateau:.1f} s ===')
    print(' t_rel(s)  finger   tips_centre  low_centre  low_left  low_right   arm moved(deg)')
    for ts in np.arange(-2, 30, 1.0):
        t = tc + ts
        if t < tj[0] or t > tj[-1]: continue
        fv = fin[np.argmin(np.abs(tj - t))]
        k = np.argmin(np.abs(td - t))
        r = [d[f'roi_{x}'][k] for x in ('tips_centre', 'low_centre', 'low_left', 'low_right')]
        m = (arm_t > t - 0.5) & (arm_t < t + 0.5)
        mv = np.degrees(np.abs(np.diff(arm_q[m], axis=0)).sum()) if m.sum() > 1 else 0.0
        print(f' {ts:+6.0f}   {fv:6.3f}   ' + '   '.join(f'{x:9.3f}' for x in r) + f'    {mv:6.1f}')
