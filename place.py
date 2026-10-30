"""Free spots on the table for placing picked objects (HW 6: 'pick the objects and place them at target locations')."""
import numpy as np

from cfg import CFG

PL = CFG.get('place', {})


def candidate_slots(objects, used=(), n=10):
    """objects: [{centroid, extent}] currently on the table; used: [[x, y], ...] spots already taken.
    Returns up to n [x, y] spots, best first: inside the configured region, away from the robot base, clear of every object (its radius + clearance)
    and of used spots, nearest the anchor first, successive spots spaced apart so a failed one is followed by a genuinely different one."""
    reg = PL.get('region', {'x': [0.28, 0.50], 'y': [-0.30, 0.30]})
    anchor = np.array(PL.get('anchor_xy', [0.32, 0.22]), dtype=float)
    clearance = float(PL.get('clearance_m', 0.07))
    spacing = float(PL.get('slot_spacing_m', 0.10))
    base_min = float(PL.get('min_base_distance_m', 0.38))
    axis = np.array(CFG['safety']['zone']['axis_xy'], dtype=float)
    cands = []
    for x in np.arange(reg['x'][0], reg['x'][1] + 1e-9, 0.02):
        for y in np.arange(reg['y'][0], reg['y'][1] + 1e-9, 0.02):
            p = np.array([x, y])
            if np.linalg.norm(p - axis) < base_min:
                continue
            if any(np.linalg.norm(p - np.array(o['centroid'][:2])) < 0.5 * max(o['extent'][0], o['extent'][1]) + clearance for o in objects):
                continue
            if any(np.linalg.norm(p - np.array(s)) < spacing for s in used):
                continue
            cands.append(p)
    cands.sort(key=lambda p: np.linalg.norm(p - anchor))
    out = []
    for p in cands:                                  # successive candidates are spaced apart
        if all(np.linalg.norm(p - q) >= 0.05 for q in out):
            out.append(p)
        if len(out) >= n:
            break
    return [[float(q[0]), float(q[1])] for q in out]
