"""Loads config.yaml (override the path with HW7_CONFIG=...)."""
import os

import yaml

PATH = os.environ.get('HW7_CONFIG', os.path.join(os.path.dirname(os.path.abspath(__file__)), 'config.yaml'))
CFG = yaml.safe_load(open(PATH))

ROOT = os.path.dirname(os.path.abspath(__file__))
HAND_EYE = os.path.join(ROOT, CFG['calibration']['hand_eye'])
TIP_OFFSET_RECORD = os.path.join(ROOT, CFG['calibration']['tip_offset_record'])


def section(name):
    return dict(CFG.get(name, {}))


def load_json(path):
    """json.load that ignores whole-line // comments (calibration/scan_joints.json keeps the old home configs commented out)."""
    import json
    return json.loads('\n'.join(l for l in open(path).read().splitlines() if not l.lstrip().startswith('//')))
