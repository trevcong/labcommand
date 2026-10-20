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
    hold: float = 10.0,
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


def walking_loop(
    *,
    side: float = 3.0,
    speeds=(0.2, 0.5, 1.0, 1.5),
    offset=(0.0, 0.0, 0.0),
    holds=(10.0, 10.0),
    stops=(0.0, 0.0, 0.0),
    reposition=(0.0, 0.0, 0.0),
    rate: float = 10.0,
    origin: float = 0.0,
    start_yaw_deg: float = 0.0,
    end_yaw_deg: float = 0.0,
    drift=(0.0, 0.0, 0.0),
    drift_stretch=(0.35, 0.65),
):
    """Parameterized synthetic square; all distances are metres, times seconds.

    Four speeds set the actual undrifted leg speeds. The last leg ends at
    ``offset``. Three stops occur at the intermediate corners; holds are
    independent start/end durations (zero omits a hold). A nonzero reposition
    starts there and moves to the origin at the first leg's speed before
    holding. Segment boundaries are sampled exactly, at intervals <= 1/rate.

    Drift accumulates linearly over the supplied fractions of route distance
    and persists through the end hold, whose position is exactly offset+drift.
    This is a trajectory fault proxy for a featureless stretch, not a model
    of lidar geometry or FAST-LIO. Heading changes linearly over route distance.
    No smoothing, endpoint alignment, sensor settings, or scorer limits enter
    this generator.
    """
    speeds = np.asarray(speeds, dtype=float)
    holds = np.asarray(holds, dtype=float)
    stops = np.asarray(stops, dtype=float)
    offset, reposition, drift = (np.asarray(v, dtype=float) for v in (offset, reposition, drift))
    stretch = np.asarray(drift_stretch, dtype=float)
    for values, shape in ((speeds, (4,)), (holds, (2,)), (stops, (3,)),
                          (offset, (3,)), (reposition, (3,)), (drift, (3,)),
                          (stretch, (2,))):
        if values.shape != shape or not np.isfinite(values).all():
            raise ValueError("loop parameters must have the documented shape and finite values")
    if not np.isfinite([side, rate, origin, start_yaw_deg, end_yaw_deg]).all():
        raise ValueError("loop scalars must be finite")
    if side <= 0 or rate <= 0 or np.any(speeds <= 0):
        raise ValueError("side, rate and speeds must be positive")
    if np.any(holds < 0) or np.any(stops < 0) or not 0 <= stretch[0] < stretch[1] <= 1:
        raise ValueError("durations must be nonnegative and drift stretch within [0, 1]")

    corners = np.array([[0, 0, 0], [side, 0, 0], [side, side, 0],
                        [0, side, 0], offset], dtype=float)
    lengths = np.linalg.norm(np.diff(corners, axis=0), axis=1)
    fractions = np.r_[0.0, np.cumsum(lengths)] / lengths.sum()
    samples = [(0.0, reposition, 0.0)]

    def segment(duration, target, progress):
        t0, start, previous_progress = samples[-1]
        if duration == 0:
            return
        count = int(math.ceil(duration * rate))
        for i in range(1, count + 1):
            fraction = i / count
            samples.append((t0 + duration * fraction,
                            start + fraction * (target - start),
                            previous_progress + fraction * (progress - previous_progress)))

    segment(float(np.linalg.norm(reposition)) / speeds[0], corners[0], 0.0)
    segment(holds[0], corners[0], 0.0)
    for leg in range(4):
        segment(lengths[leg] / speeds[leg], corners[leg + 1], fractions[leg + 1])
        if leg < 3:
            segment(stops[leg], corners[leg + 1], fractions[leg + 1])
    segment(holds[1], offset, 1.0)
    rows = []
    for time, xyz, progress in samples:
        accumulated = np.clip((progress - stretch[0]) / (stretch[1] - stretch[0]), 0, 1)
        xyz = xyz + drift * accumulated
        yaw = start_yaw_deg + progress * (end_yaw_deg - start_yaw_deg)
        rows.append((float(origin + time), *map(float, xyz), *yaw_quat(yaw)))
    return rows


def jitter_timestamps(rows, *, amplitude_s: float, seed: int):
    """Add seeded bounded timestamp noise; never sort or repair bad clocks.

    End timestamps stay fixed. Large amplitudes can deliberately create
    non-increasing times; the scorer must reject those.
    """
    if not math.isfinite(amplitude_s) or amplitude_s < 0:
        raise ValueError("timestamp jitter amplitude must be finite and nonnegative")
    jitter = np.random.default_rng(seed).uniform(-amplitude_s, amplitude_s, len(rows))
    if len(jitter):
        jitter[0] = jitter[-1] = 0.0
    return [(float(row[0] + delta), *row[1:]) for row, delta in zip(rows, jitter)]


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
