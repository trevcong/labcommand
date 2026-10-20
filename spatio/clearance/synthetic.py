"""One small scene, generated from analytic first intersections with solid boxes."""

from pathlib import Path

import numpy as np

from .model import load_json


def first_hit(origin, target, solids):
    """First positive surface intersection on a finite aimed ray."""
    origin, target = np.array(origin, float), np.array(target, float)
    delta = target - origin
    nearest = None
    for low, high in solids:
        enter, leave = 0.0, 1.0
        for axis in range(3):
            if delta[axis] == 0:
                if not low[axis] <= origin[axis] <= high[axis]:
                    leave = -1
                    break
            else:
                a, b = sorted(((low[axis] - origin[axis]) / delta[axis],
                               (high[axis] - origin[axis]) / delta[axis]))
                enter, leave = max(enter, a), min(leave, b)
        if 0 < enter <= leave and (nearest is None or enter < nearest):
            nearest = enter
    if nearest is None:
        raise ValueError("synthetic ray has no first return")
    return (origin + nearest * delta).tolist()


def scene(scale=1.0, translation=(0.0, 0.0, 0.0), start_time=100.0):
    """Transform the whole independent scene; values are fixture geometry only."""
    if not np.isfinite(scale) or scale <= 0:
        raise ValueError("synthetic scale must be positive and finite")
    shift = np.asarray(translation, float)
    zones = load_json(Path(__file__).parent / "fixtures" / "example_zones.json")
    solids = [
        ([2.0, 0.0, 0.0], [2.05, 0.6, 0.6]),  # panel face
        ([0.9, 0.1, 0.1], [1.2, 0.5, 0.5]),   # box: 2.0 - 1.2 = 0.80 m
        ([0.6, 1.9, 0.0], [0.7, 2.5, 0.6]),  # hides the second zone
        ([0.6, 3.9, 0.0], [0.7, 4.2, 0.6]),  # left gap edge
        ([0.6, 4.3, 0.0], [0.7, 4.6, 0.6]),  # right gap edge
        ([2.0, 3.9, 0.0], [2.1, 4.6, 0.6]),  # wall behind gap
    ]
    aimed = [
        ([1.85, 0.25, 0.25], [[0.5, 0.25, 0.25], [2.5, 0.25, 0.25]]),
        ([0.15, 2.25, 0.25], [[2.0, 2.25, 0.25]]),
        ([0.15, 4.25, 0.25], [[2.5, 4.25, 0.25],
                              [2.5, 3.95, 0.25], [2.5, 4.55, 0.25]]),
    ]
    scans = {"schema_version": 1, "frame_id": zones["frame_id"], "scans": []}
    for i, (origin, targets) in enumerate(aimed):
        hits = [first_hit(origin, target, solids) for target in targets]
        scans["scans"].append({
            "timestamp": start_time + i * 0.75,
            "origin": (np.array(origin) * scale + shift).tolist(),
            "hits": (np.array(hits) * scale + shift).tolist(),
        })
    for zone in zones["zones"]:
        v = zone["required_volume"]
        if zone["shape"] == "box":
            for key in ("min", "max"):
                v[key] = (np.array(v[key]) * scale + shift).tolist()
        else:
            v["plane_point"] = (np.array(v["plane_point"]) * scale + shift).tolist()
            for key in ("u_min", "u_max", "v_min", "v_max", "depth_m"):
                v[key] *= scale
        zone["provision"] += f" Synthetic geometry scale: {scale:g}."
    return zones, scans
