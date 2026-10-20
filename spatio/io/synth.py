"""Deterministic synthetic trajectories and room maps.

Times sit on one grid, ``t = t0 + i / rate_hz``. A hold of ``stationary_s``
seconds includes the sample on the boundary and excludes the next one.
Position noise is added only when ``noise_m`` is not zero, so a zero-noise
call does not depend on the seed.
"""

from __future__ import annotations

import math
from typing import Dict, Optional, Sequence

import numpy as np

from spatio.io.tum import Trajectory


def square_loop(
    side_m: float = 4.0,
    speed_mps: float = 0.5,
    rate_hz: float = 10.0,
    stationary_s: float = 10.0,
    end_offset_m: Sequence[float] = (0.0, 0.0, 0.0),
    end_yaw_deg: float = 0.0,
    t0: float = 1786223548.0,
    noise_m: float = 0.0,
    seed: int = 0,
) -> Trajectory:
    """A square driven at constant speed, with a hold at each end.

    The route is (0, 0, 0) → (side, 0, 0) → (side, side, 0) → (0, side, 0)
    → ``end_offset_m``. Yaw about +Z faces the current segment during the
    move, and equals ``end_yaw_deg`` during the final hold. The quaternion
    is xyzw.
    """

    side = _nonneg("side_m", side_m)
    speed = _positive("speed_mps", speed_mps)
    rate = _positive("rate_hz", rate_hz)
    noise = _nonneg("noise_m", noise_m)
    t0 = _finite("t0", t0)
    end_yaw = _finite("end_yaw_deg", end_yaw_deg)
    offset = _vec3("end_offset_m", end_offset_m)
    n_hold = _intervals(stationary_s, rate, "stationary_s")

    corners = np.array(
        [
            [0.0, 0.0, 0.0],
            [side, 0.0, 0.0],
            [side, side, 0.0],
            [0.0, side, 0.0],
            offset,
        ],
        dtype=np.float64,
    )
    seg = np.linalg.norm(np.diff(corners, axis=0), axis=1)
    total = float(seg.sum())
    steps = int(math.ceil((total / speed) * rate - 1e-9))
    if steps < 0:
        steps = 0
    i_arrive = n_hold + steps
    n = i_arrive + n_hold + 1

    t = t0 + np.arange(n, dtype=np.float64) / rate
    xyz = np.empty((n, 3), dtype=np.float64)
    quat = np.empty((n, 4), dtype=np.float64)
    q_start = _quat(0.0)
    q_end = _quat(math.radians(end_yaw))
    for i in range(n):
        if i <= n_hold:
            xyz[i] = 0.0
            quat[i] = q_start
        elif i >= i_arrive:
            xyz[i] = offset
            quat[i] = q_end
        else:
            arc = speed * (i - n_hold) / rate
            pos, yaw = _point_on_polyline(corners, seg, arc)
            xyz[i] = pos
            quat[i] = _quat(yaw)
    if noise != 0.0:
        rng = np.random.default_rng(int(seed))
        xyz = xyz + rng.normal(0.0, noise, size=xyz.shape)
    return Trajectory(t, xyz, quat, "synthetic://square_loop")


def stationary(
    duration_s: float,
    rate_hz: float,
    t0: float,
    noise_m: float = 0.0,
    seed: int = 0,
) -> Trajectory:
    """Hold the origin, yaw 0, for ``duration_s`` seconds inclusive."""

    rate = _positive("rate_hz", rate_hz)
    noise = _nonneg("noise_m", noise_m)
    t0 = _finite("t0", t0)
    n_hold = _intervals(duration_s, rate, "duration_s")
    n = n_hold + 1
    t = t0 + np.arange(n, dtype=np.float64) / rate
    xyz = np.zeros((n, 3), dtype=np.float64)
    quat = np.zeros((n, 4), dtype=np.float64)
    quat[:, 3] = 1.0
    if noise != 0.0:
        rng = np.random.default_rng(int(seed))
        xyz = xyz + rng.normal(0.0, noise, size=xyz.shape)
    return Trajectory(t, xyz, quat, "synthetic://stationary")


def with_reset(traj: Trajectory, at_index: int) -> Trajectory:
    """Restart the clock at ``at_index`` so time jumps backward.

    The suffix is shifted until that pose carries the original start time.
    If that is not strictly earlier than the previous pose, it is shifted
    again to land one second earlier. Positions and quaternions are copied
    unchanged.
    """

    at_index = int(at_index)
    n = len(traj)
    if at_index <= 0 or at_index >= n:
        raise ValueError("at_index must refer to a pose after the first")
    t = np.array(traj.t, copy=True)
    t[at_index:] -= t[at_index] - t[0]
    if not float(t[at_index]) < float(t[at_index - 1]):
        t[at_index:] -= t[at_index] - (t[at_index - 1] - 1.0)
    return Trajectory(t, traj.xyz, traj.quat_xyzw, traj.source)


def with_gap(traj: Trajectory, at_t: float, gap_s: float) -> Trajectory:
    """Add ``gap_s`` seconds to the first pose with ``t >= at_t`` and the suffix.

    The forward step at that pose grows by ``gap_s``. Positions are unchanged.
    """

    at_t = _finite("at_t", at_t)
    gap_s = float(gap_s)
    if not math.isfinite(gap_s) or gap_s < 0.0:
        raise ValueError("gap_s must be a finite number >= 0")
    idx = None
    for i, stamp in enumerate(traj.t):
        if float(stamp) >= at_t:
            idx = i
            break
    if idx is None:
        raise ValueError("at_t is after every pose")
    if idx == 0:
        raise ValueError("gap needs a pose before at_t")
    t = np.array(traj.t, copy=True)
    t[idx:] += gap_s
    return Trajectory(t, traj.xyz, traj.quat_xyzw, traj.source)


def room_map(
    size_m: Sequence[float] = (6.0, 5.0, 2.5),
    points_per_m2: float = 400.0,
    noise_m: float = 0.005,
    doubled_wall_offset_m: Optional[float] = None,
    seed: int = 0,
) -> Dict[str, np.ndarray]:
    """An axis-aligned room as float64 ``x``, ``y``, ``z`` arrays.

    Grid spacing is ``1/sqrt(points_per_m2)`` and both endpoints of each
    edge are included, so edges can appear on two faces. When
    ``doubled_wall_offset_m`` is set, the +X wall is copied and shifted by
    that many metres along +X.
    """

    sx, sy, sz = _vec3("size_m", size_m)
    if sx <= 0.0 or sy <= 0.0 or sz <= 0.0:
        raise ValueError("size_m components must be > 0")
    density = _positive("points_per_m2", points_per_m2)
    noise = _nonneg("noise_m", noise_m)
    if doubled_wall_offset_m is not None:
        doubled_wall_offset_m = _finite("doubled_wall_offset_m", doubled_wall_offset_m)
    spacing = 1.0 / math.sqrt(density)
    xs = _axis(sx, spacing)
    ys = _axis(sy, spacing)
    zs = _axis(sz, spacing)
    wall_x1 = _face_x(sx, ys, zs)
    parts = [
        _face_z(0.0, xs, ys),
        _face_z(sz, xs, ys),
        _face_x(0.0, ys, zs),
        wall_x1,
        _face_y(0.0, xs, zs),
        _face_y(sy, xs, zs),
    ]
    if doubled_wall_offset_m is not None:
        copied = np.array(wall_x1, copy=True)
        copied[:, 0] += doubled_wall_offset_m
        parts.append(copied)
    pts = np.concatenate(parts, axis=0)
    if noise != 0.0:
        rng = np.random.default_rng(int(seed))
        pts = pts + rng.normal(0.0, noise, size=pts.shape)
    return {
        "x": np.array(pts[:, 0], dtype=np.float64, copy=True),
        "y": np.array(pts[:, 1], dtype=np.float64, copy=True),
        "z": np.array(pts[:, 2], dtype=np.float64, copy=True),
    }


def _point_on_polyline(corners: np.ndarray, seg: np.ndarray, arc: float):
    if arc <= 0.0 or float(seg.sum()) == 0.0:
        return corners[0].copy(), _yaw_of(corners[1] - corners[0])
    left = float(arc)
    last = len(seg) - 1
    for i, length in enumerate(seg):
        length = float(length)
        if left < length or i == last:
            direction = corners[i + 1] - corners[i]
            if length <= 0.0:
                return corners[i + 1].copy(), _yaw_of(direction)
            u = min(max(left / length, 0.0), 1.0)
            pos = (1.0 - u) * corners[i] + u * corners[i + 1]
            return pos, _yaw_of(direction)
        left -= length
    return corners[-1].copy(), 0.0


def _face_x(x: float, ys: np.ndarray, zs: np.ndarray) -> np.ndarray:
    yy, zz = np.meshgrid(ys, zs, indexing="ij")
    return np.column_stack((np.full(yy.size, x, dtype=np.float64), yy.ravel(), zz.ravel()))


def _face_y(y: float, xs: np.ndarray, zs: np.ndarray) -> np.ndarray:
    xx, zz = np.meshgrid(xs, zs, indexing="ij")
    return np.column_stack((xx.ravel(), np.full(xx.size, y, dtype=np.float64), zz.ravel()))


def _face_z(z: float, xs: np.ndarray, ys: np.ndarray) -> np.ndarray:
    xx, yy = np.meshgrid(xs, ys, indexing="ij")
    return np.column_stack((xx.ravel(), yy.ravel(), np.full(xx.size, z, dtype=np.float64)))


def _axis(length: float, spacing: float) -> np.ndarray:
    n = int(math.floor(length / spacing + 1e-9)) + 1
    if n < 2:
        n = 2
    return np.linspace(0.0, length, n)


def _quat(yaw: float) -> np.ndarray:
    half = 0.5 * float(yaw)
    quat = np.array([0.0, 0.0, math.sin(half), math.cos(half)], dtype=np.float64)
    norm = float(np.linalg.norm(quat))
    if norm == 0.0:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
    return quat / norm


def _yaw_of(direction: np.ndarray) -> float:
    return math.atan2(float(direction[1]), float(direction[0]))


def _intervals(duration: float, rate: float, label: str) -> int:
    duration = _nonneg(label, duration)
    steps = duration * rate
    rounded = round(steps)
    if abs(steps - rounded) > 1e-6:
        raise ValueError("%s * rate_hz must land on a sample boundary" % label)
    return int(rounded)


def _vec3(label: str, value: Sequence[float]) -> np.ndarray:
    arr = np.asarray(value, dtype=np.float64)
    if arr.shape != (3,) or not np.isfinite(arr).all():
        raise ValueError("%s must be three finite numbers" % label)
    return arr


def _finite(label: str, value: float) -> float:
    value = float(value)
    if not math.isfinite(value):
        raise ValueError("%s must be finite" % label)
    return value


def _nonneg(label: str, value: float) -> float:
    value = _finite(label, value)
    if value < 0.0:
        raise ValueError("%s must be >= 0" % label)
    return value


def _positive(label: str, value: float) -> float:
    value = _finite(label, value)
    if value <= 0.0:
        raise ValueError("%s must be > 0" % label)
    return value
