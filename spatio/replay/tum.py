"""TUM trajectory helpers for the Spatio replay pipeline.

Stdlib only so it runs on the Mac (Python 3.9) and inside the ROS container.

A TUM line is ``t x y z qx qy qz qw``: t in seconds, position in metres in the
map frame, unit quaternion. Lines starting with ``#`` are comments.

Nothing here aligns, smooths, reorders or interpolates poses. Rows are written
in the order given; problems are reported, not repaired.
"""

from __future__ import annotations

import math
from typing import Iterable, List, NamedTuple, Optional, Sequence

QUAT_NORM_TOLERANCE = 1e-3


class Pose(NamedTuple):
    t: float
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float


class TumError(ValueError):
    pass


def stamp_to_seconds(sec: int, nanosec: int) -> float:
    """ROS builtin_interfaces/Time -> float seconds."""
    if nanosec < 0 or nanosec >= 1_000_000_000:
        raise TumError(f"nanosec out of range: {nanosec}")
    return sec + nanosec * 1e-9


def make_pose(t: float, position: Sequence[float], orientation: Sequence[float]) -> Pose:
    """Build a Pose, rejecting non-finite values and non-unit quaternions.

    ``orientation`` is (qx, qy, qz, qw), the ROS geometry_msgs order.
    """
    if len(position) != 3 or len(orientation) != 4:
        raise TumError("position needs 3 values and orientation needs 4")
    values = [t, *position, *orientation]
    if not all(math.isfinite(v) for v in values):
        raise TumError(f"non-finite value in pose at t={t!r}")
    norm = math.sqrt(sum(q * q for q in orientation))
    if abs(norm - 1.0) > QUAT_NORM_TOLERANCE:
        raise TumError(f"quaternion norm {norm:.6f} is not unit at t={t:.9f}")
    return Pose(float(t), *map(float, position), *map(float, orientation))


def format_line(p: Pose) -> str:
    # 9 decimals on t keeps nanosecond stamps; positions to 0.1 mm-ish.
    return (
        f"{p.t:.9f} {p.x:.6f} {p.y:.6f} {p.z:.6f} "
        f"{p.qx:.9f} {p.qy:.9f} {p.qz:.9f} {p.qw:.9f}"
    )


def write_tum(path: str, poses: Iterable[Pose], comments: Iterable[str] = ()) -> int:
    """Write poses to ``path``. Returns the number of pose lines written."""
    n = 0
    with open(path, "w", encoding="utf-8") as f:
        for c in comments:
            for line in str(c).splitlines() or [""]:
                f.write(f"# {line}\n")
        for p in poses:
            f.write(format_line(p) + "\n")
            n += 1
    return n


def parse_line(line: str, lineno: int = 0) -> Optional[Pose]:
    s = line.strip()
    if not s or s.startswith("#"):
        return None
    parts = s.split()
    if len(parts) != 8:
        raise TumError(f"line {lineno}: expected 8 fields, got {len(parts)}")
    try:
        v = [float(x) for x in parts]
    except ValueError as e:
        raise TumError(f"line {lineno}: {e}") from None
    try:
        return make_pose(v[0], v[1:4], v[4:8])
    except TumError as e:
        raise TumError(f"line {lineno}: {e}") from None


def read_tum(path: str) -> List[Pose]:
    poses = []
    with open(path, encoding="utf-8") as f:
        for i, line in enumerate(f, 1):
            p = parse_line(line, i)
            if p is not None:
                poses.append(p)
    return poses


def yaw_of(p: Pose) -> float:
    """Yaw (rotation about map z) in radians."""
    return math.atan2(
        2.0 * (p.qw * p.qz + p.qx * p.qy),
        1.0 - 2.0 * (p.qy * p.qy + p.qz * p.qz),
    )


def rotation_angle_between(a: Pose, b: Pose) -> float:
    """Total rotation angle (radians, 0..pi) between two orientations."""
    dot = abs(a.qx * b.qx + a.qy * b.qy + a.qz * b.qz + a.qw * b.qw)
    return 2.0 * math.acos(min(1.0, dot))


def summarize(poses: Sequence[Pose], gap_threshold_s: float = 0.5) -> dict:
    """Plain facts about a trajectory. No alignment of any kind.

    ``start_end_*`` compares the first and last pose exactly as written.
    """
    if not poses:
        raise TumError("trajectory is empty")
    first, last = poses[0], poses[-1]
    dx, dy, dz = last.x - first.x, last.y - first.y, last.z - first.z
    path_len = 0.0
    max_disp = 0.0
    backward = 0
    duplicate = 0
    gaps = []
    max_dt = 0.0
    for prev, cur in zip(poses, poses[1:]):
        path_len += math.dist((prev.x, prev.y, prev.z), (cur.x, cur.y, cur.z))
        dt = cur.t - prev.t
        if dt < 0:
            backward += 1
        elif dt == 0:
            duplicate += 1
        max_dt = max(max_dt, dt)
        if dt > gap_threshold_s:
            gaps.append({"after_t": prev.t, "dt": dt})
    for p in poses:
        max_disp = max(max_disp, math.dist((p.x, p.y, p.z), (first.x, first.y, first.z)))
    dyaw = yaw_of(last) - yaw_of(first)
    dyaw = (dyaw + math.pi) % (2.0 * math.pi) - math.pi
    return {
        "poses": len(poses),
        "t_first": first.t,
        "t_last": last.t,
        "duration_s": last.t - first.t,
        "start_end_3d_m": math.sqrt(dx * dx + dy * dy + dz * dz),
        "start_end_horizontal_m": math.hypot(dx, dy),
        "start_end_vertical_m": dz,
        "start_end_yaw_deg": math.degrees(dyaw),
        "start_end_rotation_deg": math.degrees(rotation_angle_between(first, last)),
        "path_length_m": path_len,
        "max_distance_from_start_m": max_disp,
        "backward_time_steps": backward,
        "duplicate_time_steps": duplicate,
        "max_dt_s": max_dt,
        "gaps_over_threshold": gaps,
        "gap_threshold_s": gap_threshold_s,
    }
