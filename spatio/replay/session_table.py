#!/usr/bin/env python3
"""Markdown table of session results, one row per session folder.

    python3 spatio/replay/session_table.py SESSION_DIR [SESSION_DIR ...]

Reads each session's trajectory.tum and, if present, the walk checker's
walkcheck.json. Nothing is aligned or recomputed beyond tum.py's plain summary.
"""
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import tum  # noqa: E402

HEADER = (
    "| Session | Poses | Duration (s) | Path (m) | First→last pose 3D (m) "
    "| Mean of first/last 4 s, 3D (m) | Walkcheck | Walkcheck 5 s-window return 3D (m) "
    "| Wall p95 (cm) |\n"
    "|---|---|---|---|---|---|---|---|---|"
)


def _fmt(v, nd):
    return "—" if v is None else f"{v:.{nd}f}"


def row(session_dir):
    name = os.path.basename(os.path.normpath(session_dir))
    poses = tum.read_tum(os.path.join(session_dir, "trajectory.tum"))
    s = tum.summarize(poses)
    w = tum.window_closure(poses, 4.0)
    verdict, ret, p95 = "not run", None, None
    wc_path = os.path.join(session_dir, "walkcheck.json")
    if os.path.exists(wc_path):
        with open(wc_path) as f:
            wc = json.load(f)
        verdict = wc.get("result", "?")
        reasons = wc.get("reasons") or []
        if reasons:
            verdict += " (" + ", ".join(reasons) + ")"
        ret = (wc.get("return_to_start") or {}).get("error_3d_m")
        p95 = (wc.get("walls") or {}).get("p95_point_to_plane_cm")
    return (
        f"| `{name}` | {s['poses']} | {s['duration_s']:.1f} | {s['path_length_m']:.1f} "
        f"| {s['start_end_3d_m']:.3f} | {w['closure_3d_m']:.3f} | {verdict} "
        f"| {_fmt(ret, 3)} | {_fmt(p95, 2)} |"
    )


def main(argv):
    if not argv:
        print(__doc__.strip(), file=sys.stderr)
        return 2
    print(HEADER)
    for d in argv:
        print(row(d))
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
