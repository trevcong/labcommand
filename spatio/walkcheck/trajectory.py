"""TUM trajectory parsing and the return-to-start score.

The end window is never aligned onto the start window. The error is the
distance between the two mean positions.
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np

from spatio.walkcheck.defaults import (
    MAX_GAP_S,
    MIN_EXCURSION_M,
    MIN_PATH_LENGTH_M,
    MIN_POSES,
    MIN_WINDOW_SAMPLES,
    RETURN_PASS_M,
    STATIONARY_WINDOW_S,
)


def load_tum(path: Path) -> dict:
    """Parse a TUM file. A bad data line fails the whole file."""
    try:
        text = Path(path).read_text(encoding="utf-8")
    except OSError as exc:
        return _empty_load("unreadable_trajectory", str(exc))
    return parse_tum(text)


def parse_tum(text: str) -> dict:
    comments: list[str] = []
    rows: list[list[float]] = []
    for lineno, line in enumerate(text.splitlines(), start=1):
        stripped = line.strip()
        if not stripped:
            continue
        if stripped.startswith("#"):
            comments.append(stripped)
            continue
        parts = stripped.split()
        if len(parts) != 8:
            return _empty_load(
                "malformed_trajectory",
                f"line {lineno} has {len(parts)} values, expected 8",
                comments,
            )
        try:
            values = [float(part) for part in parts]
        except ValueError:
            return _empty_load(
                "malformed_trajectory",
                f"line {lineno} is not eight numbers",
                comments,
            )
        rows.append(values)
    if not rows:
        return _empty_load("too_few_poses", "no pose lines", comments)
    data = np.asarray(rows, dtype=np.float64)
    return {
        "ok": True,
        "reason": None,
        "detail": None,
        "comments": comments,
        "t": data[:, 0],
        "xyz": data[:, 1:4],
        "q": data[:, 4:8],
    }


def score_trajectory(
    loaded: dict,
    *,
    window_s: float = STATIONARY_WINDOW_S,
    max_gap_s: float = MAX_GAP_S,
    min_path_length_m: float = MIN_PATH_LENGTH_M,
    min_excursion_m: float = MIN_EXCURSION_M,
) -> dict:
    """Score one parsed trajectory. Does not read the map."""
    if not loaded.get("ok"):
        return {
            "evaluated": False,
            "result_reasons": [loaded.get("reason") or "unreadable_trajectory"],
            "detail": loaded.get("detail"),
            "comments": loaded.get("comments") or [],
            "pose_count": 0,
            "return_to_start": None,
            "motion": None,
            "timestamps": None,
        }

    t = np.asarray(loaded["t"], dtype=np.float64)
    xyz = np.asarray(loaded["xyz"], dtype=np.float64)
    quat = np.asarray(loaded["q"], dtype=np.float64)
    comments = list(loaded.get("comments") or [])
    pose_count = int(len(t))
    reasons: list[str] = []

    finite = np.isfinite(t) & np.isfinite(xyz).all(axis=1) & np.isfinite(quat).all(axis=1)
    if not bool(finite.all()):
        reasons.append("non_finite_pose")

    if pose_count < MIN_POSES:
        reasons.append("too_few_poses")

    norms = np.linalg.norm(quat, axis=1) if pose_count else np.zeros(0)
    bad_quat = pose_count == 0 or bool(np.any((norms < 0.5) | (norms > 2.0) | ~np.isfinite(norms)))
    if bad_quat:
        reasons.append("bad_quaternion")

    timestamps = _timestamp_report(t, max_gap_s)
    if timestamps["backward_steps"]:
        reasons.append("non_increasing_timestamps")
    if timestamps["gap_steps"]:
        reasons.append("timestamp_gap")

    windows = None
    if not reasons:
        windows = _windows(t, xyz, quat, window_s)
        if windows["reason"]:
            reasons.append(windows["reason"])

    if reasons:
        return {
            "evaluated": False,
            "result_reasons": reasons,
            "detail": timestamps.get("fault") or (windows or {}).get("detail"),
            "comments": comments,
            "pose_count": pose_count,
            "return_to_start": None,
            "motion": None,
            "timestamps": timestamps,
        }

    assert windows is not None
    start_mean = windows["start_mean_xyz"]
    end_mean = windows["end_mean_xyz"]
    delta = end_mean - start_mean
    error_3d = float(np.linalg.norm(delta))
    horizontal = float(math.hypot(float(delta[0]), float(delta[1])))
    vertical = float(abs(float(delta[2])))
    heading = _wrap_deg(windows["end_yaw_deg"] - windows["start_yaw_deg"])
    attitude = _attitude_deg(windows["start_quat"], windows["end_quat"])
    # Nanometre resolution. The norm of an exact 0.10 m offset can land a
    # hair under 0.10 in binary floating point, and that must not pass.
    within = round(error_3d, 9) < RETURN_PASS_M

    path_length = float(np.linalg.norm(np.diff(xyz, axis=0), axis=1).sum())
    excursion = float(np.linalg.norm(xyz - start_mean, axis=1).max())
    enough_motion = path_length >= min_path_length_m and excursion >= min_excursion_m

    verdict_reasons: list[str] = []
    if not enough_motion:
        verdict_reasons.append("stationary_trajectory")
    if not within:
        verdict_reasons.append("return_error")

    return_block = {
        "evaluated": True,
        "start_window_s": windows["start_window_s"],
        "end_window_s": windows["end_window_s"],
        "start_samples": windows["start_samples"],
        "end_samples": windows["end_samples"],
        "start_mean_xyz_m": start_mean,
        "end_mean_xyz_m": end_mean,
        "delta_xyz_m": delta,
        "error_3d_m": error_3d,
        "error_3d_cm": error_3d * 100.0,
        "error_horizontal_cm": horizontal * 100.0,
        "error_vertical_cm": vertical * 100.0,
        "heading_error_deg": heading,
        "attitude_error_deg": attitude,
        "threshold_m": RETURN_PASS_M,
        "within_threshold": within,
        "alignment": "none",
    }
    motion = {
        "path_length_m": path_length,
        "excursion_m": excursion,
        "min_path_length_m": min_path_length_m,
        "min_excursion_m": min_excursion_m,
        "enough_motion": enough_motion,
        "return_error_pct_of_path": None
        if path_length <= 0
        else 100.0 * error_3d / path_length,
    }
    return {
        "evaluated": True,
        "result_reasons": verdict_reasons,
        "detail": None,
        "comments": comments,
        "pose_count": pose_count,
        "return_to_start": return_block,
        "motion": motion,
        "timestamps": timestamps,
    }


def score_stops(t: np.ndarray, xyz: np.ndarray, stops: list) -> list[dict]:
    """Distance between mean positions of repeated visits. Not a verdict."""
    reports = []
    for stop in stops:
        if not isinstance(stop, dict):
            reports.append(
                {
                    "name": "unnamed",
                    "visits": 0,
                    "visits_used": 0,
                    "max_mean_separation_cm": None,
                    "notes": ["stop entry is not an object"],
                }
            )
            continue
        name = str(stop.get("name") or "unnamed")
        visits = stop.get("visits") or []
        means = []
        notes: list[str] = []
        if not isinstance(visits, list):
            notes.append("visits is not a list")
            visits = []
        for visit in visits:
            if not isinstance(visit, dict) or "t0" not in visit or "t1" not in visit:
                notes.append("a visit is missing t0 or t1")
                continue
            try:
                t0 = float(visit["t0"])
                t1 = float(visit["t1"])
            except (TypeError, ValueError):
                notes.append("a visit has a non-numeric time")
                continue
            mask = (t >= t0) & (t <= t1)
            if int(mask.sum()) < MIN_WINDOW_SAMPLES:
                notes.append(f"visit {t0:g}–{t1:g} s has fewer than {MIN_WINDOW_SAMPLES} samples")
                continue
            means.append(xyz[mask].mean(axis=0))
        spread = None
        if len(means) >= 2:
            stack = np.stack(means, axis=0)
            farthest = 0.0
            for i in range(len(stack)):
                for j in range(i + 1, len(stack)):
                    farthest = max(farthest, float(np.linalg.norm(stack[i] - stack[j])))
            spread = farthest
        elif len(visits) >= 2 and len(means) < 2:
            notes.append("not enough usable visits to compare")
        reports.append(
            {
                "name": name,
                "visits": len(visits) if isinstance(visits, list) else 0,
                "visits_used": len(means),
                "max_mean_separation_cm": None if spread is None else spread * 100.0,
                "notes": notes,
            }
        )
    return reports


def _empty_load(reason: str, detail: str, comments: list[str] | None = None) -> dict:
    return {
        "ok": False,
        "reason": reason,
        "detail": detail,
        "comments": comments or [],
        "t": np.zeros(0),
        "xyz": np.zeros((0, 3)),
        "q": np.zeros((0, 4)),
    }


def _timestamp_report(t: np.ndarray, max_gap_s: float) -> dict:
    report = {
        "t_start": None if len(t) == 0 else float(t[0]),
        "t_end": None if len(t) == 0 else float(t[-1]),
        "duration_s": None if len(t) < 2 else float(t[-1] - t[0]),
        "median_dt_s": None,
        "max_dt_s": None,
        "backward_steps": 0,
        "gap_steps": 0,
        "first_backward": None,
        "first_gap": None,
        "fault": None,
    }
    if len(t) < 2:
        return report
    dt = np.diff(t)
    report["median_dt_s"] = float(np.median(dt))
    report["max_dt_s"] = float(np.max(dt))
    backward = np.flatnonzero(dt <= 0.0)
    gaps = np.flatnonzero(dt > max_gap_s)
    report["backward_steps"] = int(len(backward))
    report["gap_steps"] = int(len(gaps))
    if len(backward):
        index = int(backward[0])
        report["first_backward"] = {
            "after_pose": index,
            "t_before": float(t[index]),
            "t_after": float(t[index + 1]),
        }
        report["fault"] = (
            f"time goes from {t[index]:.6f} s to {t[index + 1]:.6f} s at pose {index + 1}"
        )
    if len(gaps):
        index = int(gaps[0])
        report["first_gap"] = {
            "after_pose": index,
            "dt_s": float(dt[index]),
            "t_before": float(t[index]),
            "t_after": float(t[index + 1]),
        }
        if report["fault"] is None:
            report["fault"] = (
                f"gap of {dt[index]:.3f} s between pose {index} and {index + 1}"
            )
    return report


def _windows(t: np.ndarray, xyz: np.ndarray, quat: np.ndarray, window_s: float) -> dict:
    t0 = float(t[0])
    t1 = float(t[-1])
    if t1 - t0 < 2.0 * window_s:
        return {
            "reason": "trajectory_too_short",
            "detail": (
                f"duration {t1 - t0:.3f} s is shorter than two {window_s:.3f} s windows"
            ),
        }
    start = t <= (t0 + window_s)
    end = (t >= (t1 - window_s)) & ~start
    if int(start.sum()) < MIN_WINDOW_SAMPLES or int(end.sum()) < MIN_WINDOW_SAMPLES:
        return {
            "reason": "window_too_few_samples",
            "detail": (
                f"start window has {int(start.sum())} poses, "
                f"end window has {int(end.sum())}"
            ),
        }
    start_q = _average_quaternion(quat[start])
    end_q = _average_quaternion(quat[end])
    if start_q is None or end_q is None:
        return {"reason": "bad_quaternion", "detail": "could not average the window quaternions"}
    return {
        "reason": None,
        "detail": None,
        "start_window_s": [t0, t0 + window_s],
        "end_window_s": [t1 - window_s, t1],
        "start_samples": int(start.sum()),
        "end_samples": int(end.sum()),
        "start_mean_xyz": xyz[start].mean(axis=0),
        "end_mean_xyz": xyz[end].mean(axis=0),
        "start_quat": start_q,
        "end_quat": end_q,
        "start_yaw_deg": _yaw_deg(start_q),
        "end_yaw_deg": _yaw_deg(end_q),
    }


def _average_quaternion(quat: np.ndarray) -> np.ndarray | None:
    """Markley average. None when a quaternion is not usable."""
    q = np.asarray(quat, dtype=np.float64)
    norms = np.linalg.norm(q, axis=1)
    if np.any(~np.isfinite(norms)) or np.any(norms < 0.5) or np.any(norms > 2.0):
        return None
    q = q / norms[:, None]
    flip = q @ q[0]
    q = q.copy()
    q[flip < 0.0] *= -1.0
    matrix = q.T @ q
    _evals, evecs = np.linalg.eigh(matrix)
    average = evecs[:, -1]
    if average[3] < 0.0:
        average = -average
    length = float(np.linalg.norm(average))
    if length < 1e-12:
        return None
    return average / length


def _yaw_deg(q: np.ndarray) -> float:
    qx, qy, qz, qw = (float(v) for v in q)
    sin_yaw = 2.0 * (qw * qz + qx * qy)
    cos_yaw = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.degrees(math.atan2(sin_yaw, cos_yaw))


def _attitude_deg(q_start: np.ndarray, q_end: np.ndarray) -> float:
    relative = _quat_mul(q_end, _quat_conj(q_start))
    w = min(1.0, max(0.0, abs(float(relative[3]))))
    return math.degrees(2.0 * math.acos(w))


def _quat_conj(q: np.ndarray) -> np.ndarray:
    return np.array([-q[0], -q[1], -q[2], q[3]], dtype=np.float64)


def _quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    ax, ay, az, aw = (float(v) for v in a)
    bx, by, bz, bw = (float(v) for v in b)
    return np.array(
        [
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz,
        ],
        dtype=np.float64,
    )


def _wrap_deg(delta: float) -> float:
    return (delta + 180.0) % 360.0 - 180.0
