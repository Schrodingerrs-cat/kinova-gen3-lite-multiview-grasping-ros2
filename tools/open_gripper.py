#!/usr/bin/env python3
# Opens the gripper. Fingers only, the arm is never commanded. Run before ./run_ros.sh so the scan sees an open hand.
#   from the repo root:  .venv/bin/python tools/open_gripper.py
import os
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)
sys.path.insert(0, os.path.join(REPO, 'ros2_iface'))
from common import BaseClient, connect, grip_pos, grip_ramp   # noqa: E402

with connect() as router:
    base = BaseClient(router)
    print(f'gripper reading before: {grip_pos(base):.3f}')
    grip_ramp(base, 0.0, rate=0.3)
    print(f'gripper reading now:    {grip_pos(base):.3f}  (0.000 = open)')
