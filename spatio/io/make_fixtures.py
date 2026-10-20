"""Regenerate the golden session fixtures.

``python3 -m spatio.io.make_fixtures --out spatio/io/fixtures`` writes them.
``--check`` writes a fresh copy in a temp directory and compares it with
``--out``. Computed EXPECTED.json floats allow 1e-12 absolute roundoff;
other values and files remain exact. The metric is not imported by the tests.

``--out`` must be missing, empty, or contain only the recognized fixture
directories and ``.DS_Store``. Any other child is refused before the
directory is changed. The repository root, this package directory, ``/``,
and the home directory are refused as well.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import shutil
import sys
import tempfile
from pathlib import Path

import numpy as np

from spatio.io.pcd import read_pcd, write_pcd
from spatio.io.synth import room_map, square_loop, stationary, with_gap, with_reset
from spatio.io.tum import read_tum, write_tum

T0 = 1786223548.0
WINDOW_S = 10.0
EDGE_S = 1e-6
CREATED_AT = "2026-10-19T00:00:00Z"
LABEL = "replay demonstrated; physical accuracy not evaluated"
DEFINITION = (
    "Return error compares the mean position over the first stationary_window_s "
    "seconds with the mean over the last stationary_window_s seconds, including "
    "a 1e-6 s edge so float rounding cannot drop the boundary sample. There is "
    "no alignment. Horizontal error is the XY norm. Vertical error is signed Z, "
    "end minus start. Heading error is the signed yaw change in degrees, wrapped "
    "to (-180, 180]. Yaw is atan2(2*(w*z+x*y), 1-2*(y*y+z*z)) of the xyzw "
    "quaternion. wall_separation_m is the smallest gap from 0.01 m to 0.5 m "
    "between x-planes that each contain at least max(500, 5 percent of the points) "
    "after rounding x to 0.1 mm, or null when no such pair exists. The doubled "
    "wall is the +X face copied along +X."
)

# Each spec is the whole synthetic configuration. The canonical JSON of
# "params" is what config_sha256 covers.
SPECS = [
    {
        "name": "loop_5cm",
        "kind": "loop",
        "loop": {"end_offset_m": (0.03, 0.04, 0.0), "end_yaw_deg": 0.0},
        "map": "corners",
        "blurb": "square loop that stops 0.03 m east and 0.04 m north, a 0.050 m horizontal miss",
        "params": {
            "trajectory": "square_loop",
            "end_offset_m": [0.03, 0.04, 0.0],
            "end_yaw_deg": 0.0,
            "side_m": 4.0,
            "speed_mps": 0.5,
            "rate_hz": 10.0,
            "stationary_s": 10.0,
            "t0": T0,
            "noise_m": 0.0,
            "seed": 0,
            "map": "unit-cube-corners",
        },
    },
    {
        "name": "loop_15cm",
        "kind": "loop",
        "loop": {"end_offset_m": (0.09, 0.12, 0.0), "end_yaw_deg": 0.0},
        "map": "corners",
        "blurb": "square loop that stops 0.09 m east and 0.12 m north, a 0.150 m horizontal miss",
        "params": {
            "trajectory": "square_loop",
            "end_offset_m": [0.09, 0.12, 0.0],
            "end_yaw_deg": 0.0,
            "side_m": 4.0,
            "speed_mps": 0.5,
            "rate_hz": 10.0,
            "stationary_s": 10.0,
            "t0": T0,
            "noise_m": 0.0,
            "seed": 0,
            "map": "unit-cube-corners",
        },
    },
    {
        "name": "loop_vertical_8cm",
        "kind": "loop",
        "loop": {"end_offset_m": (0.0, 0.0, 0.08), "end_yaw_deg": 0.0},
        "map": "corners",
        "blurb": "square loop that stops 0.080 m above the start",
        "params": {
            "trajectory": "square_loop",
            "end_offset_m": [0.0, 0.0, 0.08],
            "end_yaw_deg": 0.0,
            "side_m": 4.0,
            "speed_mps": 0.5,
            "rate_hz": 10.0,
            "stationary_s": 10.0,
            "t0": T0,
            "noise_m": 0.0,
            "seed": 0,
            "map": "unit-cube-corners",
        },
    },
    {
        "name": "loop_heading_10deg",
        "kind": "loop",
        "loop": {"end_offset_m": (0.0, 0.0, 0.0), "end_yaw_deg": 10.0},
        "map": "corners",
        "blurb": "closed square loop whose final hold is yawed 10 degrees",
        "params": {
            "trajectory": "square_loop",
            "end_offset_m": [0.0, 0.0, 0.0],
            "end_yaw_deg": 10.0,
            "side_m": 4.0,
            "speed_mps": 0.5,
            "rate_hz": 10.0,
            "stationary_s": 10.0,
            "t0": T0,
            "noise_m": 0.0,
            "seed": 0,
            "map": "unit-cube-corners",
        },
    },
    {
        "name": "stationary_60s",
        "kind": "stationary",
        "duration_s": 60.0,
        "map": "corners",
        "blurb": "60 second hold at the origin",
        "params": {
            "trajectory": "stationary",
            "duration_s": 60.0,
            "rate_hz": 10.0,
            "t0": T0,
            "noise_m": 0.0,
            "seed": 0,
            "map": "unit-cube-corners",
        },
    },
    {
        "name": "timestamp_reset",
        "kind": "reset",
        "map": "corners",
        "blurb": "closed square whose clock restarts at the middle pose",
        "params": {
            "trajectory": "square_loop+with_reset",
            "end_offset_m": [0.0, 0.0, 0.0],
            "end_yaw_deg": 0.0,
            "reset": "suffix rebased so the middle pose receives t0",
            "side_m": 4.0,
            "speed_mps": 0.5,
            "rate_hz": 10.0,
            "stationary_s": 10.0,
            "t0": T0,
            "noise_m": 0.0,
            "seed": 0,
            "map": "unit-cube-corners",
        },
    },
    {
        "name": "gap_2s",
        "kind": "gap",
        "map": "corners",
        "blurb": "closed square with 2.0 s added to timestamps from 15 s onward",
        "params": {
            "trajectory": "square_loop+with_gap",
            "end_offset_m": [0.0, 0.0, 0.0],
            "gap_s": 2.0,
            "at_t_offset_s": 15.0,
            "side_m": 4.0,
            "speed_mps": 0.5,
            "rate_hz": 10.0,
            "stationary_s": 10.0,
            "t0": T0,
            "noise_m": 0.0,
            "seed": 0,
            "map": "unit-cube-corners",
        },
    },
    {
        "name": "map_sharp_wall",
        "kind": "loop",
        "loop": {"end_offset_m": (0.0, 0.0, 0.0), "end_yaw_deg": 0.0},
        "map": "sharp",
        "blurb": "closed square plus a sharp synthetic room, noise 0",
        "params": {
            "trajectory": "square_loop",
            "end_offset_m": [0.0, 0.0, 0.0],
            "end_yaw_deg": 0.0,
            "map": "room_map",
            "size_m": [6.0, 5.0, 2.5],
            "points_per_m2": 400.0,
            "noise_m": 0.0,
            "doubled_wall_offset_m": None,
            "seed": 0,
            "t0": T0,
        },
    },
    {
        "name": "map_doubled_wall_6cm",
        "kind": "loop",
        "loop": {"end_offset_m": (0.0, 0.0, 0.0), "end_yaw_deg": 0.0},
        "map": "doubled",
        "blurb": "closed square plus the same room with the +X wall copied 0.06 m along +X",
        "params": {
            "trajectory": "square_loop",
            "end_offset_m": [0.0, 0.0, 0.0],
            "end_yaw_deg": 0.0,
            "map": "room_map",
            "size_m": [6.0, 5.0, 2.5],
            "points_per_m2": 400.0,
            "noise_m": 0.0,
            "doubled_wall_offset_m": 0.06,
            "seed": 0,
            "t0": T0,
        },
    },
    {
        "name": "pcd_formats",
        "kind": "stationary",
        "duration_s": 10.0,
        "map": "thousand",
        "blurb": "10 second hold and 1000 float32 points stored as ascii, binary, and binary_compressed",
        "params": {
            "trajectory": "stationary",
            "duration_s": 10.0,
            "rate_hz": 10.0,
            "t0": T0,
            "noise_m": 0.0,
            "seed": 0,
            "map": "i*0.125, (i-500)*0.25, -1.5 for i=0..999 float32",
        },
    },
]


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Write or check the golden fixtures.")
    parser.add_argument("--out", required=True, help="fixture directory")
    parser.add_argument(
        "--check",
        action="store_true",
        help="regenerate and compare with --out (EXPECTED.json float tolerance: 1e-12)",
    )
    args = parser.parse_args(argv)
    target = _safe_dir(Path(args.out))
    if args.check:
        if not target.is_dir():
            print("fixture directory does not exist: %s" % target, file=sys.stderr)
            return 1
        with tempfile.TemporaryDirectory(prefix="spatio-g1-fixtures-") as tmp:
            fresh = Path(tmp) / "fixtures"
            write_fixtures(fresh)
            ok = _compare_trees(fresh, target)
        return 0 if ok else 1
    write_fixtures(target)
    return 0


def write_fixtures(out: Path) -> None:
    """Replace recognized fixture sessions under ``out``.

    Refuses, before any write or delete, when ``out`` contains a child that
    is not a recognized fixture directory. ``.DS_Store`` is left in place.
    """

    out = _safe_dir(out)
    if out.exists() and not out.is_dir():
        raise SystemExit("refusing to use a file as the fixture directory: %s" % out)
    unrelated = _unrelated_children(out)
    if unrelated:
        raise SystemExit(
            "refusing to write %s: unrelated contents: %s. "
            "Remove them first. Only recognized fixture directories are replaced."
            % (out, ", ".join(unrelated))
        )
    out.mkdir(parents=True, exist_ok=True)
    for spec in SPECS:
        dest = out / spec["name"]
        if dest.exists():
            shutil.rmtree(dest)
        dest.mkdir()
        _write_one(dest, spec)
    _guard_sizes(out)


def _write_one(dest: Path, spec: dict) -> None:
    name = spec["name"]
    traj = _trajectory(spec)
    arrays = _map_arrays(spec["map"])
    write_tum(
        str(dest / "trajectory.tum"),
        traj,
        comments=("run_id: %s" % name, LABEL),
    )
    write_pcd(str(dest / "map.pcd"), arrays, data="binary")
    if spec["map"] == "thousand":
        write_pcd(str(dest / "points_ascii.pcd"), arrays, data="ascii")
        write_pcd(str(dest / "points_binary.pcd"), arrays, data="binary")
        write_pcd(
            str(dest / "points_binary_compressed.pcd"),
            arrays,
            data="binary_compressed",
        )
    loaded = read_tum(str(dest / "trajectory.tum"))
    cloud = read_pcd(str(dest / "map.pcd"))
    metrics = _metrics(loaded, cloud["x"], WINDOW_S)
    _sanity(name, metrics)
    digest = hashlib.sha256(_canonical(spec["params"])).hexdigest()
    manifest = {
        "run_id": name,
        "source_recording": "synthetic://%s" % name,
        "sensor": "synthetic",
        "fastlio_revision": "none",
        "config_file": "spatio/io/synth.py",
        "config_sha256": digest,
        "image": "none",
        "created_at": CREATED_AT,
        "notes": "%s. Synthetic fixture, not a FAST-LIO replay. %s. config_sha256 is the SHA-256 of the canonical JSON of the synthetic parameters."
        % (LABEL, spec["blurb"]),
        "physical_accuracy_evaluated": False,
    }
    expected = {
        "n_poses": metrics["n_poses"],
        "duration_s": metrics["duration_s"],
        "path_length_m": metrics["path_length_m"],
        "monotonic": metrics["monotonic"],
        "max_gap_s": metrics["max_gap_s"],
        "resets": metrics["resets"],
        "return_error_d3_m": metrics["return_error_d3_m"],
        "return_error_horizontal_m": metrics["return_error_horizontal_m"],
        "return_error_vertical_m": metrics["return_error_vertical_m"],
        "heading_error_deg": metrics["heading_error_deg"],
        "stationary_window_s": WINDOW_S,
        "map_points": metrics["map_points"],
        "wall_separation_m": metrics["wall_separation_m"],
        "notes": "%s %s" % (DEFINITION, spec["blurb"]),
    }
    _dump(dest / "manifest.json", manifest)
    _dump(dest / "EXPECTED.json", expected)


def _trajectory(spec: dict):
    kind = spec["kind"]
    if kind == "loop":
        kwargs = {"noise_m": 0.0, "seed": 0, "t0": T0}
        kwargs.update(spec["loop"])
        return square_loop(**kwargs)
    if kind == "stationary":
        return stationary(spec["duration_s"], 10.0, T0, 0.0, 0)
    if kind == "reset":
        base = square_loop(noise_m=0.0, seed=0, t0=T0)
        return with_reset(base, len(base) // 2)
    if kind == "gap":
        base = square_loop(noise_m=0.0, seed=0, t0=T0)
        return with_gap(base, T0 + 15.0, 2.0)
    raise ValueError("unknown fixture kind %r" % kind)


def _map_arrays(kind: str):
    if kind == "corners":
        pts = np.array(
            [[x, y, z] for x in (0.0, 1.0) for y in (0.0, 1.0) for z in (0.0, 1.0)],
            dtype=np.float32,
        )
        return {"x": pts[:, 0], "y": pts[:, 1], "z": pts[:, 2]}
    if kind == "sharp":
        cloud = room_map(noise_m=0.0, doubled_wall_offset_m=None, seed=0)
    elif kind == "doubled":
        cloud = room_map(noise_m=0.0, doubled_wall_offset_m=0.06, seed=0)
    elif kind == "thousand":
        index = np.arange(1000, dtype=np.float32)
        return {
            "x": index * np.float32(0.125),
            "y": (index - np.float32(500.0)) * np.float32(0.25),
            "z": np.full(1000, np.float32(-1.5), dtype=np.float32),
        }
    else:
        raise ValueError("unknown map kind %r" % kind)
    return {key: np.asarray(cloud[key], dtype=np.float32) for key in ("x", "y", "z")}


def _metrics(traj, x_values, window: float) -> dict:
    t = traj.t
    xyz = traj.xyz
    quat = traj.quat_xyzw
    n = len(traj)
    duration = float(t[-1] - t[0]) if n else 0.0
    if n >= 2:
        path = float(np.linalg.norm(np.diff(xyz, axis=0), axis=1).sum())
        dt = np.diff(t)
        monotonic = bool(np.all(dt > 0.0))
        positive = dt[dt > 0.0]
        max_gap = float(positive.max()) if positive.size else 0.0
        resets = int(np.sum(dt < 0.0))
    else:
        path = 0.0
        monotonic = False
        max_gap = 0.0
        resets = 0
    start = t <= float(t[0]) + window + EDGE_S
    end = t >= float(t[-1]) - window - EDGE_S
    if not np.any(start) or not np.any(end):
        raise RuntimeError("empty return-error window")
    delta = xyz[end].mean(axis=0) - xyz[start].mean(axis=0)
    return {
        "n_poses": n,
        "duration_s": duration,
        "path_length_m": path,
        "monotonic": monotonic,
        "max_gap_s": max_gap,
        "resets": resets,
        "return_error_d3_m": float(np.linalg.norm(delta)),
        "return_error_horizontal_m": float(np.hypot(delta[0], delta[1])),
        "return_error_vertical_m": float(delta[2]),
        "heading_error_deg": _heading_deg(quat[start], quat[end]),
        "map_points": int(np.asarray(x_values).shape[0]),
        "wall_separation_m": _wall_separation(x_values),
        "start_span_m": float(np.max(xyz[start].max(axis=0) - xyz[start].min(axis=0))),
        "end_span_m": float(np.max(xyz[end].max(axis=0) - xyz[end].min(axis=0))),
    }


def _heading_deg(start_q, end_q) -> float:
    delta = _mean_yaw(end_q) - _mean_yaw(start_q)
    wrapped = math.atan2(math.sin(delta), math.cos(delta))
    return float(math.degrees(wrapped))


def _mean_yaw(quat) -> float:
    x = quat[:, 0]
    y = quat[:, 1]
    z = quat[:, 2]
    w = quat[:, 3]
    ang = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return float(math.atan2(float(np.sin(ang).mean()), float(np.cos(ang).mean())))


def _wall_separation(x_values):
    values = np.asarray(x_values, dtype=np.float64).reshape(-1)
    if values.size < 500:
        return None
    rounded = np.round(values, 4)
    planes, counts = np.unique(rounded, return_counts=True)
    threshold = max(500, int(math.ceil(0.05 * float(values.size))))
    peaks = np.sort(planes[counts >= threshold])
    if peaks.size < 2:
        return None
    keep = [float(gap) for gap in np.diff(peaks) if 0.01 <= float(gap) <= 0.5]
    if not keep:
        return None
    return float(min(keep))


def _sanity(name: str, metrics: dict) -> None:
    """Refuse to write a fixture whose known target is off."""

    def near(key, target, tol=1e-9):
        got = metrics[key]
        if abs(got - target) > tol:
            raise RuntimeError("%s %s is %r, expected %r" % (name, key, got, target))

    if name == "loop_5cm":
        near("return_error_d3_m", 0.05)
        near("return_error_horizontal_m", 0.05)
        near("return_error_vertical_m", 0.0)
        near("heading_error_deg", 0.0)
    elif name == "loop_15cm":
        near("return_error_d3_m", 0.15)
        near("return_error_horizontal_m", 0.15)
        near("return_error_vertical_m", 0.0)
    elif name == "loop_vertical_8cm":
        near("return_error_d3_m", 0.08)
        near("return_error_horizontal_m", 0.0)
        near("return_error_vertical_m", 0.08)
    elif name == "loop_heading_10deg":
        near("return_error_d3_m", 0.0)
        near("heading_error_deg", 10.0, tol=1e-6)
        near("path_length_m", 16.0, tol=1e-6)
    elif name == "stationary_60s":
        if metrics["n_poses"] != 601 or abs(metrics["duration_s"] - 60.0) > 1e-9:
            raise RuntimeError("stationary_60s shape is wrong: %r" % metrics)
        near("path_length_m", 0.0)
        near("return_error_d3_m", 0.0)
    elif name == "timestamp_reset":
        if metrics["monotonic"] or metrics["resets"] != 1:
            raise RuntimeError("timestamp_reset was not a single backward jump")
    elif name == "gap_2s":
        if not metrics["monotonic"] or metrics["resets"] != 0:
            raise RuntimeError("gap_2s broke monotonic time")
        if not 2.0 < metrics["max_gap_s"] < 2.2:
            raise RuntimeError("gap_2s max_gap_s is %r" % metrics["max_gap_s"])
        near("path_length_m", 16.0, tol=1e-6)
    elif name == "map_sharp_wall":
        if metrics["wall_separation_m"] is not None or metrics["map_points"] < 10000:
            raise RuntimeError("sharp wall fixture looks wrong: %r" % metrics)
    elif name == "map_doubled_wall_6cm":
        sep = metrics["wall_separation_m"]
        if sep is None or abs(sep - 0.06) > 1e-6:
            raise RuntimeError("doubled wall separation is %r" % sep)
    elif name == "pcd_formats":
        if metrics["map_points"] != 1000:
            raise RuntimeError("pcd_formats has %d points" % metrics["map_points"])
    if name != "timestamp_reset":
        if metrics["start_span_m"] > 1e-6 or metrics["end_span_m"] > 1e-6:
            raise RuntimeError("%s scoring window is not stationary" % name)


def _canonical(params: dict) -> bytes:
    return json.dumps(params, sort_keys=True, separators=(",", ":"), allow_nan=False).encode(
        "utf-8"
    )


def _dump(path: Path, obj: dict) -> None:
    path.write_text(
        json.dumps(obj, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )


def _guard_sizes(out: Path) -> None:
    total = 0
    for path in out.rglob("*"):
        if not path.is_file() or path.name == ".DS_Store":
            continue
        size = path.stat().st_size
        total += size
        if size >= 1_000_000:
            raise RuntimeError("%s is %d bytes, over 1 MB" % (path, size))
    if total >= 5_000_000:
        raise RuntimeError("fixtures total %d bytes, over 5 MB" % total)


def _expected_equal(a, b) -> bool:
    """Only computed float leaves permit roundoff, never counts or types."""
    if type(a) is not type(b):
        return False
    if isinstance(a, float):
        return math.isfinite(a) and math.isfinite(b) and math.isclose(
            a, b, rel_tol=0.0, abs_tol=1e-12
        )
    if isinstance(a, dict):
        return a.keys() == b.keys() and all(_expected_equal(a[k], b[k]) for k in a)
    if isinstance(a, list):
        return len(a) == len(b) and all(_expected_equal(x, y) for x, y in zip(a, b))
    return a == b


def _compare_trees(fresh: Path, committed: Path) -> bool:
    def files(root: Path):
        found = set()
        for path in root.rglob("*"):
            if not path.is_file() or path.name == ".DS_Store":
                continue
            if "__pycache__" in path.parts:
                continue
            found.add(path.relative_to(root).as_posix())
        return found

    left = files(fresh)
    right = files(committed)
    ok = True
    if left != right:
        ok = False
        print("only regenerated: %s" % sorted(left - right), file=sys.stderr)
        print("only committed: %s" % sorted(right - left), file=sys.stderr)
    for rel in sorted(left & right):
        a = (fresh / rel).read_bytes()
        b = (committed / rel).read_bytes()
        equal = a == b
        if Path(rel).name == "EXPECTED.json":
            try:
                equal = _expected_equal(json.loads(a), json.loads(b))
            except (ValueError, UnicodeDecodeError):
                equal = False
        if not equal:
            ok = False
            print("differs: %s (%d vs %d bytes)" % (rel, len(a), len(b)), file=sys.stderr)
    if ok:
        print("fixtures match %s" % committed)
    return ok


def _recognized_fixture_names():
    return {spec["name"] for spec in SPECS}


def _unrelated_children(path: Path):
    """Names under ``path`` that regeneration must not touch.

    An absent directory has none. A present directory may contain the
    recognized fixture directories and ``.DS_Store``. Anything else,
    including a fixture name that is not a directory, is unrelated.
    """

    if not path.exists():
        return []
    if not path.is_dir():
        return [path.name]
    recognized = _recognized_fixture_names()
    unrelated = []
    for child in path.iterdir():
        if child.name == ".DS_Store":
            continue
        if child.name not in recognized or not child.is_dir():
            unrelated.append(child.name)
    return sorted(unrelated)


def _safe_dir(path: Path) -> Path:
    path = path.expanduser()
    if path.exists():
        resolved = path.resolve()
    else:
        resolved = path.absolute()
    package = Path(__file__).resolve().parent
    repo = package.parents[1]
    forbidden = {
        Path("/").resolve(),
        Path.home().resolve(),
        package,
        repo,
    }
    if resolved in forbidden:
        raise SystemExit("refusing to use %s as the fixture directory" % resolved)
    return path


if __name__ == "__main__":
    sys.exit(main())
