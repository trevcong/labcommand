"""Synthetic sessions for tests and the committed sample fixture.

These are not FAST-LIO outputs and they are not physical walks.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

from spatio.walkcheck.defaults import HISTORICAL_LABEL


def yaw_quat(deg: float) -> tuple[float, float, float, float]:
    half = math.radians(deg) * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


def write_tum(path: Path, rows, comments=()) -> None:
    lines = []
    for comment in comments:
        text = comment if str(comment).startswith("#") else f"# {comment}"
        lines.append(text)
    for row in rows:
        t, x, y, z, qx, qy, qz, qw = row
        lines.append(
            f"{t:.4f} {x:.6f} {y:.6f} {z:.6f} {qx:.8f} {qy:.8f} {qz:.8f} {qw:.8f}"
        )
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_pcd_ascii(path: Path, xyz: np.ndarray) -> None:
    points = np.asarray(xyz, dtype=np.float64)
    _write_pcd(path, points, binary=False)


def write_pcd_binary(path: Path, xyz: np.ndarray) -> None:
    points = np.asarray(xyz, dtype=np.float64)
    _write_pcd(path, points, binary=True)


def _write_pcd(path: Path, points: np.ndarray, binary: bool) -> None:
    count = int(len(points))
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z\n"
        "SIZE 4 4 4\n"
        "TYPE F F F\n"
        "COUNT 1 1 1\n"
        f"WIDTH {count}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {count}\n"
        f"DATA {'binary' if binary else 'ascii'}\n"
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    if binary:
        with path.open("wb") as handle:
            handle.write(header.encode("ascii"))
            handle.write(np.ascontiguousarray(points, dtype="<f4").tobytes())
        return
    body = "".join(f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n" for p in points)
    path.write_text(header + body, encoding="utf-8")


def hold_and_square(
    side: float = 3.0,
    offset=(0.05, 0.0, 0.0),
    duration: float = 40.0,
    rate: float = 10.0,
    hold: float = 6.0,
    end_yaw_deg: float = 0.0,
    start_yaw_deg: float = 0.0,
):
    """Square loop with a stationary hold at each end.

    The end hold sits at `offset`. Nothing is done to hide that offset.
    """
    count = int(round(duration * rate))
    times = np.arange(count + 1) / rate
    start_q = yaw_quat(start_yaw_deg)
    end_q = yaw_quat(end_yaw_deg)
    move_t0 = hold
    move_t1 = duration - hold
    corners = np.array(
        [
            [0.0, 0.0, 0.0],
            [side, 0.0, 0.0],
            [side, side, 0.0],
            [0.0, side, 0.0],
            [0.0, 0.0, 0.0],
        ],
        dtype=np.float64,
    )
    lengths = np.linalg.norm(np.diff(corners, axis=0), axis=1)
    cumulative = np.concatenate([[0.0], np.cumsum(lengths)])
    total = float(cumulative[-1])
    shift = np.asarray(offset, dtype=np.float64)
    rows = []
    for time in times:
        stamp = float(time)
        if stamp <= move_t0:
            xyz = np.zeros(3)
            quat = start_q
        elif stamp >= move_t1:
            xyz = shift
            quat = end_q
        else:
            fraction = (stamp - move_t0) / (move_t1 - move_t0)
            distance = fraction * total
            index = int(np.searchsorted(cumulative, distance, side="right") - 1)
            index = min(max(index, 0), len(lengths) - 1)
            span = float(lengths[index])
            local = 0.0 if span == 0.0 else (distance - float(cumulative[index])) / span
            xyz = corners[index] * (1.0 - local) + corners[index + 1] * local
            quat = start_q
        rows.append(
            (
                stamp,
                float(xyz[0]),
                float(xyz[1]),
                float(xyz[2]),
                quat[0],
                quat[1],
                quat[2],
                quat[3],
            )
        )
    return rows


def stationary_rows(duration: float = 30.0, rate: float = 10.0, xyz=(0.0, 0.0, 0.0)):
    count = int(round(duration * rate))
    times = np.arange(count + 1) / rate
    quat = yaw_quat(0.0)
    x, y, z = xyz
    return [(float(time), float(x), float(y), float(z), *quat) for time in times]


def insert_time_gap(rows, after_t: float, extra_s: float):
    shifted = []
    for row in rows:
        stamp = row[0] + extra_s if row[0] > after_t else row[0]
        shifted.append((stamp, *row[1:]))
    return shifted


def vertical_wall(const, axis, count, along0, along1, z0, z1, noise, rng):
    along = rng.uniform(along0, along1, count)
    height = rng.uniform(z0, z1, count)
    surface = np.full(count, const) + rng.normal(0.0, noise, count)
    if axis == "x":
        return np.stack([surface, along, height], axis=1)
    return np.stack([along, surface, height], axis=1)


def sharp_wall(count: int = 8000, noise: float = 0.004, seed: int = 1) -> np.ndarray:
    rng = np.random.default_rng(seed)
    return vertical_wall(1.0, "x", count, 0.0, 3.0, 0.0, 2.2, noise, rng)


def doubled_wall(
    separation_m: float = 0.10,
    count_per_sheet: int = 4000,
    noise: float = 0.003,
    seed: int = 1,
) -> np.ndarray:
    rng = np.random.default_rng(seed)
    first = vertical_wall(1.0, "x", count_per_sheet, 0.0, 3.0, 0.0, 2.2, noise, rng)
    second = vertical_wall(1.0 + separation_m, "x", count_per_sheet, 0.0, 3.0, 0.0, 2.2, noise, rng)
    return np.vstack([first, second])


def four_wall_room(count_per_wall: int = 4000, noise: float = 0.004, seed: int = 1) -> np.ndarray:
    rng = np.random.default_rng(seed)
    return np.vstack(
        [
            vertical_wall(0.0, "x", count_per_wall, 0.0, 4.0, 0.0, 2.4, noise, rng),
            vertical_wall(5.0, "x", count_per_wall, 0.0, 4.0, 0.0, 2.4, noise, rng),
            vertical_wall(0.0, "y", count_per_wall, 0.0, 5.0, 0.0, 2.4, noise, rng),
            vertical_wall(4.0, "y", count_per_wall, 0.0, 5.0, 0.0, 2.4, noise, rng),
        ]
    )


def write_session(dest: Path, rows, xyz=None, manifest=None, stops=None, comments=()) -> Path:
    dest.mkdir(parents=True, exist_ok=True)
    write_tum(dest / "trajectory.tum", rows, comments)
    if xyz is not None:
        write_pcd_ascii(dest / "map.pcd", xyz)
    if manifest is not None:
        (dest / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    if stops is not None:
        (dest / "stops.json").write_text(json.dumps(stops, indent=2) + "\n", encoding="utf-8")
    return dest


def sample_manifest() -> dict:
    return {
        "run_id": "fixture-sample-5cm",
        "source_recording": None,
        "sensor": "synthetic",
        "fastlio_revision": None,
        "config_file": None,
        "config_sha256": None,
        "image": None,
        "created_at": "2026-10-19T00:00:00+00:00",
        "notes": (
            "Synthetic sample session for the walk checker. Not a physical walk. "
            + HISTORICAL_LABEL
        ),
        "physical_accuracy_evaluated": False,
    }


def write_sample_fixture(dest: Path) -> Path:
    rows = hold_and_square(offset=(0.05, 0.0, 0.0))
    wall = sharp_wall(count=4000, noise=0.004, seed=0)
    return write_session(
        dest,
        rows,
        xyz=wall,
        manifest=sample_manifest(),
        comments=(
            "# SYNTHETIC trajectory. Not a physical walk and not a FAST-LIO output.",
        ),
    )
