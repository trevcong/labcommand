"""Conservative hold detection; all thresholds are provisional.

No start/end alignment or recording-specific timing is used. Slow drift that
exceeds the spread limit invalidates the whole candidate, rather than selecting
a convenient shorter slice.
"""

from pathlib import Path

import numpy as np

MIN_HOLD_S = 8.0
MAX_HOLD_SPEED_M_S = 0.02
MAX_ANGULAR_SPEED_DEG_S = 3.0
SIGNIFICANT_MOTION_M = 0.20
SIGNIFICANT_ROTATION_DEG = 10.0
MAX_IMU_GAP_S = 0.10
MAX_ACCEL_DEVIATION_M_S2 = 0.50
GRAVITY_M_S2 = 9.80665


def load_imu(path: Path) -> dict:
    """Read optional CSV: t,gx,gy,gz,ax,ay,az (seconds, rad/s, m/s²).

    Acceleration includes gravity. Timestamps must use the trajectory clock.
    An explicitly supplied unreadable file must never become "no IMU".
    """
    try:
        with Path(path).open(encoding="utf-8") as stream:
            if stream.readline().strip() != "t,gx,gy,gz,ax,ay,az":
                raise ValueError("expected header t,gx,gy,gz,ax,ay,az")
            data = np.loadtxt(stream, delimiter=",", ndmin=2)
        if data.ndim != 2 or data.shape[1] != 7 or len(data) < 2:
            raise ValueError("expected at least two rows of seven values")
        if not np.isfinite(data).all() or np.any(np.diff(data[:, 0]) <= 0):
            raise ValueError("IMU values must be finite with strictly increasing times")
        return {"ok": True, "data": data}
    except (OSError, ValueError) as exc:
        return {"ok": False, "detail": f"IMU: {exc}"}


def angular_distance_deg(quat, reference):
    # abs(dot) respects the q / -q equivalence.
    return np.degrees(2 * np.arccos(np.clip(np.abs(np.sum(quat * reference, axis=1)), 0, 1)))


def detect_holds(
    t, xyz, quat, *, min_hold_s, max_speed_m_s, max_spread_m,
    max_angular_speed_deg_s, significant_motion_m, significant_rotation_deg,
    min_samples, imu, max_imu_gap_s, max_accel_deviation_m_s2, gravity_m_s2,
):
    dt = np.diff(t)
    q = quat / np.linalg.norm(quat, axis=1)[:, None]
    angles = np.degrees(2 * np.arccos(np.clip(np.abs(np.sum(q[1:] * q[:-1], axis=1)), 0, 1)))
    stationary = (np.linalg.norm(np.diff(xyz, axis=0), axis=1) / dt <= max_speed_m_s)
    stationary &= angles / dt <= max_angular_speed_deg_s
    report = {
        "method": "trajectory speed, rotation and spread" + (" plus IMU" if imu is not None else ""),
        "minimum_duration_s": min_hold_s,
        "imu_used": imu is not None,
        "intervals": [],
        "start": None,
        "end": None,
        "flags": [],
    }
    if imu is not None:
        if not imu["ok"]:
            report["error"] = imu["detail"]
            return report
        data = imu["data"]
        it = data[:, 0]
        quiet = np.linalg.norm(data[:, 1:4], axis=1) <= np.radians(max_angular_speed_deg_s)
        quiet &= np.abs(np.linalg.norm(data[:, 4:7], axis=1) - gravity_m_s2) <= max_accel_deviation_m_s2
        # Each trajectory edge must be bracketed by continuous, quiet IMU.
        # Prefix sums include every IMU sample, even at a much higher rate.
        bad = np.r_[0, np.cumsum(~quiet)]
        # One timestamp ULP allows an exactly-on-limit gap represented on
        # an epoch clock; this is numerical precision, not missing coverage.
        precision = np.spacing(np.maximum(np.abs(it[1:]), np.abs(it[:-1])))
        gaps = np.r_[0, np.cumsum(np.diff(it) > max_imu_gap_s + precision)]
        left = np.searchsorted(it, t[:-1], side="right") - 1
        right = np.searchsorted(it, t[1:], side="left")
        covered = (left >= 0) & (right < len(it))
        lo = np.clip(left, 0, len(it) - 1)
        hi = np.clip(right, 0, len(it) - 1)
        stationary &= covered & (bad[hi + 1] == bad[lo]) & (gaps[hi] == gaps[lo])

    edges = np.diff(np.r_[False, stationary, False].astype(int))
    for first, last in zip(np.flatnonzero(edges == 1), np.flatnonzero(edges == -1)):
        # stationary edges [first, last) cover poses [first, last].
        duration = float(t[last] - t[first])
        if duration < min_hold_s or last - first + 1 < min_samples:
            continue
        points = xyz[first:last + 1]
        mean = points.mean(axis=0)
        spread = float(np.linalg.norm(points - mean, axis=1).max())
        rotation = float(angular_distance_deg(q[first:last + 1], q[first]).max())
        if spread > max_spread_m or rotation > significant_rotation_deg:
            continue
        report["intervals"].append({
            "t_start": float(t[first]), "t_end": float(t[last]),
            "duration_s": duration, "samples": int(last - first + 1),
            "mean_xyz_m": mean, "spread_m": spread,
            "first_index": int(first), "last_index": int(last),
        })
    holds = report["intervals"]
    if not holds:
        return report
    report["start"], report["end"] = holds[0], holds[-1]
    a, b = holds[0]["first_index"], holds[-1]["last_index"]

    def moved(points, rotations, position, rotation):
        return bool(
            np.any(np.linalg.norm(points - position, axis=1) > significant_motion_m)
            or np.any(angular_distance_deg(rotations, rotation) > significant_rotation_deg)
        )

    if moved(xyz[:a + 1], q[:a + 1], xyz[a], q[a]):
        report["flags"].append("motion_before_start_hold")
    if moved(xyz[b:], q[b:], xyz[b], q[b]):
        report["flags"].append("motion_after_end_hold")
    # An intermediate hold at a different pose could be repositioning or an
    # intentional route stop. Flag the ambiguity; never pick it as the start.
    for hold in holds[1:-1]:
        index = hold["first_index"]
        if moved(xyz[index:index + 1], q[index:index + 1], xyz[a], q[a]):
            report["flags"].append("possible_repositioning")
            break
    return report
