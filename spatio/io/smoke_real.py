"""Read-only smoke of the copied historical maps and any session trajectories.

Run under the shared big-I/O lock. This writes a JSON report under
``~/Developer/spatio-data/derived/g1-io-fixtures/`` and does not modify
recordings or sessions.
"""

from __future__ import annotations

import json
import math
import time
from pathlib import Path

import numpy as np

from spatio.io.pcd import read_pcd, read_pcd_header
from spatio.io.tum import TumFormatError, check_trajectory, read_tum

MAPS = (
    Path.home() / "Developer/spatio-data/recordings/session_20260808_211213/map.pcd",
    Path.home()
    / "Developer/spatio-data/recordings/20260826T221936Z_JT128_room_walk/jt128_map.pcd",
)
SESSIONS = Path.home() / "Developer/spatio-data/sessions"
OUT = Path.home() / "Developer/spatio-data/derived/g1-io-fixtures/real_pcd_smoke.json"


def main() -> int:
    report = {"maps": [], "trajectories": []}
    for path in MAPS:
        report["maps"].append(_map_report(path))
    if SESSIONS.is_dir():
        paths = sorted(SESSIONS.rglob("trajectory.tum"))
    else:
        paths = []
    if not paths:
        report["trajectories"].append(
            {
                "path": str(SESSIONS),
                "status": "no trajectory.tum files found",
            }
        )
    for path in paths:
        report["trajectories"].append(_trajectory_report(path))
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0


def _map_report(path: Path) -> dict:
    if not path.is_file():
        return {"path": str(path), "status": "missing"}
    t0 = time.perf_counter()
    header = read_pcd_header(str(path))
    t1 = time.perf_counter()
    cloud = read_pcd(str(path))
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
