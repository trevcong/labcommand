"""Read-only smoke of explicit session folders, maps, and trajectories.

Pass ``--session`` (a directory with ``map.pcd`` and ``trajectory.tum``),
``--map``, and ``--trajectory``. Nothing is read unless it was named.
A missing requested file, or a map or trajectory that does not parse,
exits 1. ``check_trajectory`` issues are reported and do not change the
exit code.

The JSON report is written to ``--out``. When ``--out`` is omitted it
goes to ``~/Developer/spatio-data/derived/g1-io-fixtures/real_pcd_smoke.json``.
Recordings and sessions are not modified.
"""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import numpy as np

from spatio.io.pcd import PcdFormatError, read_pcd, read_pcd_header
from spatio.io.tum import TumFormatError, check_trajectory, read_tum

DEFAULT_OUT = Path.home() / "Developer/spatio-data/derived/g1-io-fixtures/real_pcd_smoke.json"


def main(argv=None) -> int:
    args = _parser().parse_args(argv)
    map_paths = [Path(value) for value in (args.map or [])]
    traj_paths = [Path(value) for value in (args.trajectory or [])]
    for session in args.session or []:
        root = Path(session)
        map_paths.append(root / "map.pcd")
        traj_paths.append(root / "trajectory.tum")
    report = {"maps": [], "trajectories": []}
    if not map_paths and not traj_paths:
        report["status"] = "missing required inputs"
    else:
        for path in map_paths:
            report["maps"].append(_map_report(path))
        for path in traj_paths:
            report["trajectories"].append(_trajectory_report(path))
    text = json.dumps(report, indent=2) + "\n"
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(text, encoding="utf-8")
    print(text, end="")
    return _exit_code(report)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="python3 -m spatio.io.smoke_real",
        description=(
            "Read the maps and trajectories you name and write a JSON report. "
            "There is no default recording."
        ),
    )
    parser.add_argument(
        "--session",
        action="append",
        default=None,
        metavar="DIR",
        help="Session directory. Checks map.pcd and trajectory.tum in that directory.",
    )
    parser.add_argument(
        "--map",
        action="append",
        default=None,
        metavar="PATH",
        help="PCD file to read. Repeat for more than one file.",
    )
    parser.add_argument(
        "--trajectory",
        action="append",
        default=None,
        metavar="PATH",
        help="TUM file to read. Repeat for more than one file.",
    )
    parser.add_argument(
        "--out",
        default=str(DEFAULT_OUT),
        help="JSON report path (default: %(default)s).",
    )
    return parser


def _exit_code(report: dict) -> int:
    """0 only when every requested file was read. Anything else is 1."""

    if report.get("status"):
        return 1
    items = list(report["maps"]) + list(report["trajectories"])
    if not items:
        return 1
    for item in items:
        if item.get("status") != "read":
            return 1
    return 0


def _map_report(path: Path) -> dict:
    if not path.is_file():
        return {"path": str(path), "status": "missing"}
    t0 = time.perf_counter()
    try:
        header = read_pcd_header(str(path))
        t1 = time.perf_counter()
        cloud = read_pcd(str(path))
    except PcdFormatError as exc:
        return {"path": str(path), "status": "PcdFormatError", "error": str(exc)}
    bbox = {}
    nonfinite = {}
    for key in ("x", "y", "z"):
        values = cloud[key]
        lo = float(np.min(values))
        hi = float(np.max(values))
        if math.isfinite(lo) and math.isfinite(hi):
            bad = 0
        else:
            bad = int(values.size - np.isfinite(values).sum())
            finite = np.isfinite(values)
            if finite.any():
                lo = float(values[finite].min())
                hi = float(values[finite].max())
            else:
                lo = None
                hi = None
        bbox[key] = [lo, hi]
        nonfinite[key] = bad
    t2 = time.perf_counter()
    count = int(cloud["x"].shape[0])
    del cloud
    return {
        "path": str(path),
        "status": "read",
        "bytes": path.stat().st_size,
        "points_header": int(header.points),
        "points_read": count,
        "width": int(header.width),
        "height": int(header.height),
        "data": header.data,
        "fields": list(header.fields),
        "bbox_min_m": [bbox["x"][0], bbox["y"][0], bbox["z"][0]],
        "bbox_max_m": [bbox["x"][1], bbox["y"][1], bbox["z"][1]],
        "nonfinite": nonfinite,
        "header_seconds": t1 - t0,
        "read_and_bbox_seconds": t2 - t1,
    }


def _trajectory_report(path: Path) -> dict:
    if not path.is_file():
        return {"path": str(path), "status": "missing"}
    try:
        traj = read_tum(str(path))
    except TumFormatError as exc:
        return {"path": str(path), "status": "TumFormatError", "error": str(exc)}
    issues = check_trajectory(traj)
    duration = float(traj.t[-1] - traj.t[0]) if len(traj) else None
    return {
        "path": str(path),
        "status": "read",
        "n_poses": len(traj),
        "duration_s": duration,
        "issues": [
            {"code": issue.code, "index": issue.index, "dt": issue.dt, "gap_s": issue.gap_s}
            for issue in issues
        ],
    }


if __name__ == "__main__":
    raise SystemExit(main())
