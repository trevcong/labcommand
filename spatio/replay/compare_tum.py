#!/usr/bin/env python3
"""Compare two TUM trajectories pose-by-pose at matching timestamps.

No alignment: both trajectories are compared in the frames they were written
in. For two FAST-LIO runs on the same input that is meaningful, because each
run's map frame (camera_init) is fixed at its first processed scan. If the two
runs initialised on different scans the offset shows up as error -- reported,
not removed.

    compare_tum.py A.tum B.tum [--max-dt 0.005] [--json OUT]
"""

from __future__ import annotations

import argparse
import bisect
import json
import math
import os
import statistics
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import tum  # noqa: E402


def match(a, b, max_dt):
    """Pairs (pa, pb) where pb is the pose in b nearest in time to pa, within max_dt."""
    bt = [p.t for p in b]
    pairs = []
    for pa in a:
        i = bisect.bisect_left(bt, pa.t)
        best = None
        for j in (i - 1, i):
            if 0 <= j < len(b) and abs(bt[j] - pa.t) <= max_dt:
                if best is None or abs(bt[j] - pa.t) < abs(bt[best] - pa.t):
                    best = j
        if best is not None:
            pairs.append((pa, b[best]))
    return pairs


def compare(a, b, max_dt=0.005):
    pairs = match(a, b, max_dt)
    out = {"poses_a": len(a), "poses_b": len(b), "matched": len(pairs), "max_dt_s": max_dt}
    if not pairs:
        return out
    d = [math.dist((p.x, p.y, p.z), (q.x, q.y, q.z)) for p, q in pairs]
    r = [math.degrees(tum.rotation_angle_between(p, q)) for p, q in pairs]
    out.update({
        "position_diff_m": {"median": statistics.median(d), "max": max(d), "last": d[-1]},
        "rotation_diff_deg": {"median": statistics.median(r), "max": max(r), "last": r[-1]},
        "first_matched_t": pairs[0][0].t,
        "last_matched_t": pairs[-1][0].t,
    })
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("a")
    ap.add_argument("b")
    ap.add_argument("--max-dt", type=float, default=0.005)
    ap.add_argument("--json", default=None)
    args = ap.parse_args()
    res = compare(tum.read_tum(args.a), tum.read_tum(args.b), args.max_dt)
    res.update({"a": args.a, "b": args.b})
    text = json.dumps(res, indent=2)
    if args.json:
        with open(args.json, "w") as f:
            f.write(text + "\n")
    print(text)


if __name__ == "__main__":
    main()
