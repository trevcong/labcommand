"""Compare two Spatio sessions.

Primary numbers are raw-frame position and rotation differences. An Umeyama
SE(3) fit (scale fixed at 1) is stored only under
``diagnostic only (after SE(3) alignment)``.

``time_offset`` is added to session B only when the caller passes it.
Nothing here searches for a clock shift.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import List, Optional, Sequence, Tuple, Union

import numpy as np
from scipy.spatial import cKDTree

from spatio.replaycompare.pcd import read_pcd, voxel_downsample
from spatio.replaycompare.tum import read_tum

PathLike = Union[str, Path]

SCHEMA = "spatio.replaycompare/1"
DIAGNOSTIC_LABEL = "diagnostic only (after SE(3) alignment)"
_NO_PAIRS_REASON = "no pose pairs within max_dt"
MANIFEST_KEYS = (
    "run_id",
    "source_recording",
    "sensor",
    "fastlio_revision",
    "config_file",
    "config_sha256",
    "image",
    "created_at",
    "notes",
    "physical_accuracy_evaluated",
)
_WITHIN_M = (0.05, 0.10)


def compare_sessions(
    session_a: PathLike,
    session_b: PathLike,
    max_dt: float = 0.05,
    voxel: float = 0.05,
    time_offset: Optional[float] = None,
) -> Tuple[dict, dict]:
    """Return ``(report, plot_series)``.

    ``time_offset`` None means the caller did not pass one. A float is added
    to every timestamp of session B before the overlap test.
    """
    if max_dt < 0:
        raise ValueError(f"max_dt must be >= 0, got {max_dt}")
    if voxel <= 0:
        raise ValueError(f"voxel must be > 0, got {voxel}")

    loaded_a = _load_session(session_a)
    loaded_b = _load_session(session_b)
    reasons: List[str] = []
    if time_offset is not None:
        reasons.append(
            "explicit time offset "
            f"{float(time_offset):.6g} s was added to session B timestamps"
        )
    for label, loaded in (("A", loaded_a), ("B", loaded_b)):
        if loaded["manifest_note"]:
            reasons.append(f"session {label} {loaded['manifest_note']}")

    report, series = _compare_loaded(
        loaded_a,
        loaded_b,
        max_dt=float(max_dt),
        voxel=float(voxel),
        time_offset=None if time_offset is None else float(time_offset),
        reasons=reasons,
    )
    return report, series


def _load_session(path: PathLike) -> dict:
    folder = Path(path).resolve()
    manifest, run_id, note = _load_manifest(folder)
    trajectory = read_tum(folder / "trajectory.tum")
    points = read_pcd(folder / "map.pcd")
    return {
        "path": folder,
        "run_id": run_id,
        "manifest": manifest,
        "manifest_note": note,
        "trajectory": trajectory,
        "points": points,
    }


def _load_manifest(folder: Path) -> Tuple[Optional[dict], str, Optional[str]]:
    manifest_path = folder / "manifest.json"
    if not manifest_path.is_file():
        return None, folder.name, "has no manifest.json"
    try:
        data = json.loads(manifest_path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise ValueError(f"{manifest_path}: {exc}") from exc
    if not isinstance(data, dict):
        raise ValueError(f"{manifest_path}: expected a JSON object")
    excerpt = {key: data[key] for key in MANIFEST_KEYS if key in data}
    run_id = excerpt.get("run_id") or folder.name
    excerpt["run_id"] = run_id
    return excerpt, str(run_id), None


def _compare_loaded(
    loaded_a: dict,
    loaded_b: dict,
    max_dt: float,
    voxel: float,
    time_offset: Optional[float],
    reasons: List[str],
) -> Tuple[dict, dict]:
    traj_a = loaded_a["trajectory"]
    traj_b = loaded_b["trajectory"]
    times_a = traj_a[:, 0] if len(traj_a) else np.zeros(0, dtype=np.float64)
    times_b = traj_b[:, 0] if len(traj_b) else np.zeros(0, dtype=np.float64)
    if time_offset is None:
        times_b_cmp = times_b
    else:
        times_b_cmp = times_b + time_offset

    for label, traj in (("A", traj_a), ("B", traj_b)):
        if len(traj) == 0:
            continue
        error = _quaternion_norm_error(traj[:, 4:8])
        if error > 1e-3:
            reasons.append(
                f"session {label} quaternions differ from unit length by up to {error:.3e}"
            )

    association, pairs, evaluable, gate_reasons = _associate_trajectories(
        times_a, times_b, times_b_cmp, max_dt
    )
    reasons.extend(gate_reasons)

    raw = _raw_trajectory(traj_a, traj_b, pairs, evaluable)
    diagnostic = _diagnostic(traj_a, traj_b, pairs, evaluable, gate_reasons)
    map_report, map_reasons = _compare_maps(loaded_a["points"], loaded_b["points"], voxel)
    reasons.extend(map_reasons)

    if evaluable and map_report is not None:
        status = "COMPARED"
    else:
        status = "NOT_EVALUABLE"

    report = {
        "schema": SCHEMA,
        "a": _session_excerpt(loaded_a),
        "b": _session_excerpt(loaded_b),
        "status": status,
        "reasons": reasons,
        "time_offset_s": time_offset,
        "time_offset_applied_to": None if time_offset is None else "b",
        "trajectory": {
            "association": association,
            "raw": raw,
            "diagnostic_aligned": diagnostic,
        },
        "map": map_report,
    }
    series = {
        "a": _series(loaded_a["run_id"], traj_a),
        "b": _series(loaded_b["run_id"], traj_b),
    }
    return report, series


def _session_excerpt(loaded: dict) -> dict:
    return {
        "run_id": loaded["run_id"],
        "path": str(loaded["path"]),
        "manifest": loaded["manifest"],
    }


def _series(run_id: str, trajectory: np.ndarray) -> dict:
    if len(trajectory) == 0:
        return {"run_id": run_id, "x": np.zeros(0), "y": np.zeros(0)}
    return {
        "run_id": run_id,
        "x": np.array(trajectory[:, 1], dtype=np.float64, copy=True),
        "y": np.array(trajectory[:, 2], dtype=np.float64, copy=True),
    }


def _associate_trajectories(
    times_a: np.ndarray,
    times_b: np.ndarray,
    times_b_cmp: np.ndarray,
    max_dt: float,
) -> Tuple[dict, List[Tuple[int, int]], bool, List[str]]:
    reasons: List[str] = []
    a_range = _range(times_a)
    b_range = _range(times_b)
    b_compared = _range(times_b_cmp)
    empty = len(times_a) == 0 or len(times_b) == 0
    if len(times_a) == 0:
        reasons.append("session A has no poses")
    if len(times_b) == 0:
        reasons.append("session B has no poses")

    if empty:
        gap = None
        overlap = 0.0
        evaluable = False
        pairs: List[Tuple[int, int]] = []
    else:
        gap = _range_gap(a_range[0], a_range[1], b_compared[0], b_compared[1])
        overlap = _overlap_duration(a_range[0], a_range[1], b_compared[0], b_compared[1])
        # A separation larger than max_dt cannot contain a legal pair.
        # Do not invent a clock offset to close that gap.
        if gap > max_dt:
            evaluable = False
            pairs = []
            reasons.append("trajectory time ranges do not overlap")
            reasons.append(
                f"session A time range seconds: [{a_range[0]:.6f}, {a_range[1]:.6f}]"
            )
            reasons.append(
                "session B time range seconds: "
                f"[{b_range[0]:.6f}, {b_range[1]:.6f}]"
            )
            reasons.append("no automatic time shift was applied")
        else:
            # Spans can overlap while every sample is still farther than max_dt.
            # Zero pairs is not a comparison.
            pairs = _pair_nearest(times_a, times_b_cmp, max_dt)
            evaluable = bool(pairs)
            if not pairs:
                reasons.append(_NO_PAIRS_REASON)

    association = {
        "max_dt_s": float(max_dt),
        "matched": len(pairs),
        "unmatched_a": int(len(times_a) - len(pairs)),
        "unmatched_b": int(len(times_b) - len(pairs)),
        "overlap_duration_s": float(overlap),
        "range_gap_s": None if gap is None else float(gap),
        "a_range_s": a_range,
        "b_range_s": b_range,
        "b_range_compared_s": b_compared,
        "a_pose_count": int(len(times_a)),
        "b_pose_count": int(len(times_b)),
        "a_duration_s": _duration(times_a),
        "b_duration_s": _duration(times_b),
        "a_rate_hz": _rate_hz(times_a),
        "b_rate_hz": _rate_hz(times_b),
    }
    return association, pairs, evaluable, reasons


def _pair_nearest(
    times_a: np.ndarray,
    times_b: np.ndarray,
    max_dt: float,
) -> List[Tuple[int, int]]:
    """One-to-one greedy match: smallest |dt|, then A index, then B index."""
    candidates = []
    start_b = 0
    count_b = len(times_b)
    for index_a, time_a in enumerate(times_a):
        while start_b < count_b and times_b[start_b] < time_a - max_dt:
            start_b += 1
        index_b = start_b
        while index_b < count_b and times_b[index_b] <= time_a + max_dt:
            delta = abs(float(times_b[index_b] - time_a))
            candidates.append((delta, index_a, index_b))
            index_b += 1
    candidates.sort()
    used_a = set()
    used_b = set()
    pairs = []
    for _delta, index_a, index_b in candidates:
        if index_a in used_a or index_b in used_b:
            continue
        used_a.add(index_a)
        used_b.add(index_b)
        pairs.append((index_a, index_b))
    pairs.sort(key=lambda pair: (float(times_a[pair[0]]), pair[0], pair[1]))
    return pairs


def _raw_trajectory(
    traj_a: np.ndarray,
    traj_b: np.ndarray,
    pairs: Sequence[Tuple[int, int]],
    evaluable: bool,
) -> dict:
    raw = {
        "position_m": None,
        "rotation_deg": None,
        "first_pose": {
            "a": _pose(traj_a, 0),
            "b": _pose(traj_b, 0),
        },
        "start_to_end_m": {
            "a": _start_to_end(traj_a),
            "b": _start_to_end(traj_b),
        },
        "path_length_m": {
            "a": _path_length(traj_a),
            "b": _path_length(traj_b),
        },
    }
    if not evaluable or not pairs:
        return raw
    pos_a = np.array([traj_a[i, 1:4] for i, _j in pairs], dtype=np.float64)
    pos_b = np.array([traj_b[j, 1:4] for _i, j in pairs], dtype=np.float64)
    quat_a = np.array([traj_a[i, 4:8] for i, _j in pairs], dtype=np.float64)
    quat_b = np.array([traj_b[j, 4:8] for _i, j in pairs], dtype=np.float64)
    position = np.linalg.norm(pos_a - pos_b, axis=1)
    rotation = _rotation_angles_deg(quat_a, quat_b)
    raw["position_m"] = {
        "mean": float(np.mean(position)),
        "median": float(np.median(position)),
        "p95": _percentile(position, 95),
        "max": float(np.max(position)),
        "final": float(position[-1]),
    }
    raw["rotation_deg"] = {
        "median": float(np.median(rotation)),
        "p95": _percentile(rotation, 95),
        "max": float(np.max(rotation)),
    }
    return raw


def _diagnostic(
    traj_a: np.ndarray,
    traj_b: np.ndarray,
    pairs: Sequence[Tuple[int, int]],
    evaluable: bool,
    gate_reasons: Sequence[str],
) -> dict:
    base = {"label": DIAGNOSTIC_LABEL, "computed": False}
    if not evaluable or len(pairs) == 0:
        if _NO_PAIRS_REASON in gate_reasons:
            base["reason"] = "no matched poses to align"
        else:
            base["reason"] = "not computed because trajectory time ranges do not overlap"
        return base
    src = np.array([traj_a[i, 1:4] for i, _j in pairs], dtype=np.float64)
    dst = np.array([traj_b[j, 1:4] for _i, j in pairs], dtype=np.float64)
    rotation, translation, rmse = _umeyama_se3(src, dst)
    quat = _rotmat_to_quat(rotation)
    return {
        "label": DIAGNOSTIC_LABEL,
        "computed": True,
        "method": "umeyama_se3_no_scale",
        "aligns": "a_onto_b",
        "ate_rmse_m": rmse,
        "translation_m": [float(value) for value in translation],
        "rotation_quat_xyzw": [float(value) for value in quat],
        "rotation_deg": _rotation_angle_deg_from_matrix(rotation),
    }


def _compare_maps(
    points_a: np.ndarray,
    points_b: np.ndarray,
    voxel: float,
) -> Tuple[Optional[dict], List[str]]:
    reasons: List[str] = []
    finite_a, dropped_a = _finite_points(points_a)
    finite_b, dropped_b = _finite_points(points_b)
    if dropped_a:
        reasons.append(f"session A map dropped {dropped_a} non-finite points")
    if dropped_b:
        reasons.append(f"session B map dropped {dropped_b} non-finite points")
    if len(finite_a) == 0 or len(finite_b) == 0:
        reasons.append("map has no finite points to compare")
        return None, reasons

    down_a = voxel_downsample(finite_a, voxel)
    down_b = voxel_downsample(finite_b, voxel)
    a_to_b = _nearest_stats(down_a, down_b)
    b_to_a = _nearest_stats(down_b, down_a)
    centroid_a = finite_a.mean(axis=0)
    centroid_b = finite_b.mean(axis=0)
    offset = centroid_b - centroid_a
    report = {
        "voxel_m": float(voxel),
        "downsample": "centroid_per_voxel",
        "a_points_before": int(len(points_a)),
        "a_points_after": int(len(down_a)),
        "b_points_before": int(len(points_b)),
        "b_points_after": int(len(down_b)),
        "a_nonfinite_dropped": dropped_a,
        "b_nonfinite_dropped": dropped_b,
        "a_to_b": a_to_b,
        "b_to_a": b_to_a,
        "bbox_a": _bbox(finite_a),
        "bbox_b": _bbox(finite_b),
        "centroid_a_m": [float(value) for value in centroid_a],
        "centroid_b_m": [float(value) for value in centroid_b],
        "centroid_offset_m": [float(value) for value in offset],
        "centroid_offset_norm_m": float(np.linalg.norm(offset)),
        "centroid_offset_definition": (
            "centroid_b minus centroid_a on finite points before voxel downsampling"
        ),
    }
    return report, reasons


def _nearest_stats(source: np.ndarray, target: np.ndarray) -> dict:
    tree = cKDTree(target)
    distances, _indexes = tree.query(source, k=1, workers=1)
    distances = np.asarray(distances, dtype=np.float64)
    del tree
    return {
        "p50_m": _percentile(distances, 50),
        "p95_m": _percentile(distances, 95),
        "p99_m": _percentile(distances, 99),
        "fraction_within_0_05m": float(np.mean(distances <= _WITHIN_M[0])),
        "fraction_within_0_10m": float(np.mean(distances <= _WITHIN_M[1])),
    }


def _finite_points(points: np.ndarray) -> Tuple[np.ndarray, int]:
    cloud = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    mask = np.isfinite(cloud).all(axis=1)
    return cloud[mask], int((~mask).sum())


def _bbox(points: np.ndarray) -> dict:
    return {
        "min": [float(value) for value in points.min(axis=0)],
        "max": [float(value) for value in points.max(axis=0)],
    }


def _umeyama_se3(src: np.ndarray, dst: np.ndarray) -> Tuple[np.ndarray, np.ndarray, float]:
    """Align ``src`` onto ``dst`` with a rigid transform. Scale stays 1."""
    count = int(src.shape[0])
    mu_src = src.mean(axis=0)
    mu_dst = dst.mean(axis=0)
    src_centered = src - mu_src
    dst_centered = dst - mu_dst
    if count == 1 or np.linalg.matrix_rank(src_centered, tol=1e-10) == 0:
        rotation = np.eye(3)
    else:
        covariance = dst_centered.T @ src_centered / count
        rotation = _kabsch(covariance)
    translation = mu_dst - rotation @ mu_src
    aligned = src @ rotation.T + translation
    residual = np.linalg.norm(aligned - dst, axis=1)
    rmse = float(np.sqrt(np.mean(residual ** 2)))
    return rotation, translation, rmse


def _kabsch(covariance: np.ndarray) -> np.ndarray:
    left, _singular, vh = np.linalg.svd(covariance)
    rotation = left @ vh
    if np.linalg.det(rotation) < 0:
        left = left.copy()
        left[:, -1] *= -1.0
        rotation = left @ vh
    return rotation


def _rotation_angles_deg(quat_a: np.ndarray, quat_b: np.ndarray) -> np.ndarray:
    unit_a = _unit_quaternions(quat_a)
    unit_b = _unit_quaternions(quat_b)
    dots = np.abs(np.sum(unit_a * unit_b, axis=1))
    np.clip(dots, 0.0, 1.0, out=dots)
    return np.degrees(2.0 * np.arccos(dots))


def _unit_quaternions(quaternions: np.ndarray) -> np.ndarray:
    norms = np.linalg.norm(quaternions, axis=1)
    if np.any(norms <= 1e-12):
        raise ValueError("trajectory contains a zero quaternion")
    return quaternions / norms[:, None]


def _quaternion_norm_error(quaternions: np.ndarray) -> float:
    if len(quaternions) == 0:
        return 0.0
    norms = np.linalg.norm(np.asarray(quaternions, dtype=np.float64), axis=1)
    return float(np.max(np.abs(norms - 1.0)))


def _rotmat_to_quat(rotation: np.ndarray) -> np.ndarray:
    """Hamilton quaternion xyzw. The sign is chosen so w >= 0."""
    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = np.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (rotation[2, 1] - rotation[1, 2]) / scale
        y = (rotation[0, 2] - rotation[2, 0]) / scale
        z = (rotation[1, 0] - rotation[0, 1]) / scale
    elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
        scale = np.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
        w = (rotation[2, 1] - rotation[1, 2]) / scale
        x = 0.25 * scale
        y = (rotation[0, 1] + rotation[1, 0]) / scale
        z = (rotation[0, 2] + rotation[2, 0]) / scale
    elif rotation[1, 1] > rotation[2, 2]:
        scale = np.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
        w = (rotation[0, 2] - rotation[2, 0]) / scale
        x = (rotation[0, 1] + rotation[1, 0]) / scale
        y = 0.25 * scale
        z = (rotation[1, 2] + rotation[2, 1]) / scale
    else:
        scale = np.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
        w = (rotation[1, 0] - rotation[0, 1]) / scale
        x = (rotation[0, 2] + rotation[2, 0]) / scale
        y = (rotation[1, 2] + rotation[2, 1]) / scale
        z = 0.25 * scale
    quat = np.array([x, y, z, w], dtype=np.float64)
    norm = np.linalg.norm(quat)
    if norm == 0.0:
        return np.array([0.0, 0.0, 0.0, 1.0])
    quat = quat / norm
    if quat[3] < 0.0:
        quat = -quat
    return quat


def _rotation_angle_deg_from_matrix(rotation: np.ndarray) -> float:
    cosine = (float(np.trace(rotation)) - 1.0) * 0.5
    cosine = float(np.clip(cosine, -1.0, 1.0))
    return float(np.degrees(np.arccos(cosine)))


def _pose(trajectory: np.ndarray, index: int) -> Optional[dict]:
    if len(trajectory) == 0:
        return None
    row = trajectory[index]
    return {
        "t": float(row[0]),
        "x": float(row[1]),
        "y": float(row[2]),
        "z": float(row[3]),
        "qx": float(row[4]),
        "qy": float(row[5]),
        "qz": float(row[6]),
        "qw": float(row[7]),
    }


def _start_to_end(trajectory: np.ndarray) -> Optional[float]:
    if len(trajectory) == 0:
        return None
    if len(trajectory) == 1:
        return 0.0
    return float(np.linalg.norm(trajectory[-1, 1:4] - trajectory[0, 1:4]))


def _path_length(trajectory: np.ndarray) -> Optional[float]:
    if len(trajectory) == 0:
        return None
    if len(trajectory) == 1:
        return 0.0
    steps = np.diff(trajectory[:, 1:4], axis=0)
    return float(np.linalg.norm(steps, axis=1).sum())


def _range(times: np.ndarray) -> Optional[List[float]]:
    if len(times) == 0:
        return None
    return [float(times[0]), float(times[-1])]


def _duration(times: np.ndarray) -> Optional[float]:
    if len(times) == 0:
        return None
    return float(times[-1] - times[0])


def _rate_hz(times: np.ndarray) -> Optional[float]:
    if len(times) < 2:
        return None
    duration = float(times[-1] - times[0])
    if duration <= 0.0:
        return None
    return float((len(times) - 1) / duration)


def _range_gap(a0: float, a1: float, b0: float, b1: float) -> float:
    if a1 < b0:
        return float(b0 - a1)
    if b1 < a0:
        return float(a0 - b1)
    return 0.0


def _overlap_duration(a0: float, a1: float, b0: float, b1: float) -> float:
    return float(max(0.0, min(a1, b1) - max(a0, b0)))


def _percentile(values: np.ndarray, quantile: float) -> float:
    return float(np.percentile(np.asarray(values, dtype=np.float64), quantile, method="linear"))
