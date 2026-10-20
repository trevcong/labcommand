"""Read-only smoke of explicit session folders, maps, and trajectories.

Pass ``--session`` (a directory with ``map.pcd`` and ``trajectory.tum``),
``--map``, and ``--trajectory``. Nothing is read unless it was named.
A missing requested file, a map or trajectory that does not parse, or a
map with no points exits 1. In those cases the report is still written,
and the other named files are still checked. ``check_trajectory`` issues
are reported and do not change the exit code.

The JSON report is written to ``--out``. When ``--out`` is omitted it
goes to ``~/Developer/spatio-data/derived/g1-io-fixtures/real_pcd_smoke.json``.
``--out`` must not be an input file. The same path, a symlink, and a hard
link are rejected before any write, and the process exits 1.
Recordings and sessions are not modified.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from pathlib import Path
from typing import Optional, Sequence

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
    out = Path(args.out)
    try:
        clash = _aliased_input(out, map_paths + traj_paths)
    except OSError as exc:
        print("refusing to write %s: %s" % (out, exc), file=sys.stderr)
        return 1
    if clash is not None:
        print(
            "refusing to write %s: it is the same file as input %s" % (out, clash),
            file=sys.stderr,
        )
        return 1
    report = {"maps": [], "trajectories": []}
    if not map_paths and not traj_paths:
        report["status"] = "missing required inputs"
    else:
        for path in map_paths:
            report["maps"].append(_map_report(path))
        for path in traj_paths:
            report["trajectories"].append(_trajectory_report(path))
    text = json.dumps(report, indent=2) + "\n"
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
        help=(
            "JSON report path (default: %(default)s). "
            "Must not be the same file as an input."
        ),
    )
    return parser


def _aliased_input(out: Path, inputs: Sequence[Path]) -> Optional[Path]:
    """Return the first input that is the same file as ``out``.

    ``resolve`` collapses symlinks, ``..``, and a report path that does not
    exist yet. It does not collapse hard links, so those are compared by
    device and inode. On a case-insensitive directory the resolved path is
    compared case-folded, because ``resolve`` keeps the spelling it was given.
    """

    out_resolved = _resolve_output(out)
    fold = _case_insensitive(out_resolved)
    out_key = _path_key(out_resolved, fold)
    for src in inputs:
        src_resolved = _try_resolve(src)
        if src_resolved is not None:
            src_fold = fold or _case_insensitive(src_resolved)
            if src_fold and not fold:
                fold = True
                out_key = _path_key(out_resolved, True)
            if _path_key(src_resolved, src_fold) == out_key:
                return src
        try:
            if os.path.samefile(out, src):
                return src
        except OSError:
            pass
    return None


def _resolve_output(path: Path) -> Path:
    resolved = _try_resolve(path)
    if resolved is None:
        raise OSError("cannot resolve output path %s" % path)
    return resolved


def _try_resolve(path: Path) -> Optional[Path]:
    try:
        return path.resolve(strict=False)
    except (OSError, RuntimeError):
        return None


def _path_key(resolved: Path, fold: bool) -> str:
    text = os.path.normpath(os.fspath(resolved))
    if fold:
        return text.casefold()
    return text


def _case_insensitive(path: Path) -> bool:
    """True when a directory on ``path`` treats ``A`` and ``a`` as one name."""

    current = path if path.is_dir() else path.parent
    for _ in range(64):
        if current.is_dir():
            flipped = _flip_case(current.name)
            if flipped != current.name:
                other = current.with_name(flipped)
                try:
                    return os.path.samefile(current, other)
                except OSError:
                    return False
        parent = current.parent
        if parent == current:
            return False
        current = parent
    return False


def _flip_case(name: str) -> str:
    chars = []
    changed = False
    for char in name:
        if char.islower():
            chars.append(char.upper())
            changed = True
        elif char.isupper():
            chars.append(char.lower())
            changed = True
        else:
            chars.append(char)
    if not changed:
        return name
    return "".join(chars)


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
    # np.min/np.max raise ValueError on a zero-size array. A cloud this
    # package accepts with POINTS 0 must not abort the rest of the report.
    if any(cloud[key].size == 0 for key in ("x", "y", "z")):
        t2 = time.perf_counter()
        count = int(cloud["x"].shape[0])
        del cloud
        return {
            "path": str(path),
            "status": "empty",
            "error": "point cloud has no points; bounding box is undefined",
            "bytes": path.stat().st_size,
            "points_header": int(header.points),
            "points_read": count,
            "width": int(header.width),
            "height": int(header.height),
            "data": header.data,
            "fields": list(header.fields),
            "bbox_min_m": [None, None, None],
            "bbox_max_m": [None, None, None],
            "nonfinite": {"x": 0, "y": 0, "z": 0},
            "header_seconds": t1 - t0,
            "read_and_bbox_seconds": t2 - t1,
        }
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
