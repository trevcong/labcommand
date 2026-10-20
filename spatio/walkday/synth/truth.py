#!/usr/bin/env python3
"""Compare a session's trajectory.tum with a synthetic recording's ground truth.

    truth.py RECORDING_DIR SESSION_DIR [--json OUT]

The estimate lives in FAST-LIO's map frame (the IMU pose at start-up). Ground
truth is shifted so the true start position is the origin; the synthetic loop
starts heading +x with zero roll and pitch, so no rotation is applied, and no
alignment is fitted. Diagnostic only: this is not a pass/fail gate.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from scene import Path, SceneParams  # noqa: E402


def load_params(rec):
    with open(os.path.join(rec, "synth_params.json")) as f:
        raw = json.load(f)
    raw["boxes"] = [(tuple(a), tuple(b)) for a, b in raw["boxes"]]
    for k in ("centre_xy", "semi_axes", "end_offset_m", "room_min", "room_max", "elevation_deg"):
        raw[k] = tuple(raw[k])
    return SceneParams(**raw)


def compare(rec, session):
    p = load_params(rec)
    path = Path(p)
    rows = [l.split() for l in open(os.path.join(session, "trajectory.tum")) if l.strip() and not l.startswith("#")]
    d = np.array(rows, dtype=float)
    t = d[:, 0]
    est = d[:, 1:4]
    pos, _, _, _ = path.state(t)
    gt = pos - path.state(np.array([p.t0]))[0][0]
    err = np.linalg.norm(est - gt, axis=1)
    still = t < path.t_move0
    out = {
        "poses": int(len(t)),
        "position_error_m": {"median": float(np.median(err)), "max": float(err.max()),
                             "first": float(err[0]), "last": float(err[-1])},
        "drift_while_parked_at_start_m": float(np.linalg.norm(est[still][-1] - est[still][0])) if still.sum() > 1 else None,
        "true_end_offset_m": list(p.end_offset_m),
        "estimated_last_xyz_m": est[-1].tolist(),
        "true_last_xyz_m": gt[-1].tolist(),
    }
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("recording")
    ap.add_argument("session")
    ap.add_argument("--json")
    a = ap.parse_args()
    out = compare(a.recording, a.session)
    print(json.dumps(out, indent=2))
    if a.json:
        with open(a.json, "w") as f:
            json.dump(out, f, indent=2)


if __name__ == "__main__":
    main()
