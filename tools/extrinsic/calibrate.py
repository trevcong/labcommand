"""Offline rigid extrinsic from a JT128 cloud to a Mid-360 cloud.

Estimates ``T_jt128←mid360`` (``p_jt128 = R p_livox + t``). It does not level the
rig into the room frame, and it does not write the room state directory.

The room frame is a later, separate transform ``T_room←jt128`` (phase 3 P14).
``T_room←mid360 = T_room←jt128 × T_jt128←mid360``. ``RigTransforms.hesaiToWorld``
is the identity because the Mac draws the JT128 origin as its world, not because
that frame has been levelled.

Run:
    python3 tools/extrinsic/calibrate.py Tests/Fixtures/room/snapshots/a -o extrinsic.json
    python3 tools/extrinsic/calibrate.py --jt128 jt128.bin --mid360 mid360.bin -o extrinsic.json
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

SCHEMA = "labcommand.extrinsic.v1"
PARENT_FRAME = "hesai_lidar"
CHILD_FRAME = "livox_frame"
CONVENTION = "p_parent = R * p_child + t"
CHARACTERISTIC_LENGTH_M = 1.0
AUTHORITATIVE_PATH = "~/room-memory/<room_id>/T_jt128_mid360.json"
LEVEL_PATH = "~/room-memory/<room_id>/T_room_jt128.json"
RESIDUAL_TRIM_NOTE = "fixed radius only; solver trimming is not applied"

YAW_DEG = -86.5
PITCH_DEG = 0.0
ROLL_DEG = 180.0
NOMINAL_TRANSLATION_M = (0.0, 0.0, -0.110)

# 21 bits per axis around a ±2^20 cell bias covers about ±50 km at a 5 cm cell.
_BIAS = 1 << 20
_SHIFT = 21


class ExtrinsicError(Exception):
    """A bad path, cloud, or initialization. The CLI exits 1 and writes nothing."""


class InitError(ExtrinsicError):
    """``--init`` is not a usable ``labcommand.extrinsic.v1`` matrix."""


@dataclass(frozen=True)
class SolverSettings:
    """Knobs recorded in the JSON. Defaults are the real-rig calibration."""

    min_range_m: float = 1.5
    max_range_m: float = 8.0
    voxel_m: float = 0.05
    point_cap: int = 20000
    normal_radius_m: float = 0.20
    normal_min_points: int = 8
    planarity_ratio: float = 0.2
    gate_initial_m: float = 0.25
    gate_min_m: float = 0.05
    gate_rms_factor: float = 3.0
    search_cell_m: float = 0.25
    characteristic_length_m: float = CHARACTERISTIC_LENGTH_M
    residual_trim_floor_m: float = 0.05
    residual_trim_factor: float = 2.5
    huber_delta_m: float = 0.05
    max_iterations: int = 25
    step_translation_m: float = 1e-4
    step_rotation_deg: float = 0.01
    min_correspondences: int = 50
    min_points: int = 50
    condition_ratio_min: float = 1e-4
    accept_translation_m: float = 0.05
    accept_rotation_deg: float = 3.0
    accept_residual_rms_m: float = 0.05
    accept_inlier_fraction: float = 0.15
    inlier_radius_m: float = 0.10

    def __post_init__(self):
        query = max(self.normal_radius_m, self.gate_initial_m, self.inlier_radius_m)
        if self.search_cell_m + 1e-12 < query:
            raise ExtrinsicError(
                "search_cell_m must be at least the normal radius, the initial gate, and the "
                "fixed inlier radius; a 27-cell neighborhood does not cover a larger radius"
            )
        if self.characteristic_length_m <= 0:
            raise ExtrinsicError("characteristic_length_m must be positive")


def rotation_ypr(yaw, pitch, roll):
    """``R = Rz(yaw) Ry(pitch) Rx(roll)``, radians, the Swift ``RigidTransform`` order."""
    cy, sy = math.cos(yaw), math.sin(yaw)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cr, sr = math.cos(roll), math.sin(roll)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=np.float64,
    )


def nominal_mount():
    """The published Mid-360 mount in the JT128 frame. A seed, not a calibration."""
    rotation = rotation_ypr(math.radians(YAW_DEG), math.radians(PITCH_DEG), math.radians(ROLL_DEG))
    translation = np.array(NOMINAL_TRANSLATION_M, dtype=np.float64)
    return rotation, translation


def apply_rigid(points, rotation, translation):
    """``((r0 x + r1 y) + r2 z) + t``. No ``ndarray @`` on the cloud (numpy 2.0.2 warns)."""
    points = np.asarray(points, dtype=np.float64)
    rotation = np.asarray(rotation, dtype=np.float64)
    translation = np.asarray(translation, dtype=np.float64)
    if points.size == 0:
        return np.zeros((0, 3), dtype=np.float64)
    x = points[:, 0]
    y = points[:, 1]
    z = points[:, 2]
    out = np.empty((points.shape[0], 3), dtype=np.float64)
    with np.errstate(divide="ignore", over="ignore", invalid="ignore"):
        for row in range(3):
            out[:, row] = ((rotation[row, 0] * x + rotation[row, 1] * y) + rotation[row, 2] * z) + translation[row]
    if not np.isfinite(out).all():
        raise ExtrinsicError("transform produced a non-finite point")
    return out


def inverse_rigid(rotation, translation):
    rotation = np.asarray(rotation, dtype=np.float64)
    translation = np.asarray(translation, dtype=np.float64)
    transposed = rotation.T
    return transposed, apply_rigid(translation.reshape(1, 3) * -1.0, transposed, np.zeros(3)).reshape(3)


def _matvec(rotation, vector):
    vector = np.asarray(vector, dtype=np.float64)
    return np.array(
        [
            rotation[0, 0] * vector[0] + rotation[0, 1] * vector[1] + rotation[0, 2] * vector[2],
            rotation[1, 0] * vector[0] + rotation[1, 1] * vector[1] + rotation[1, 2] * vector[2],
            rotation[2, 0] * vector[0] + rotation[2, 1] * vector[1] + rotation[2, 2] * vector[2],
        ],
        dtype=np.float64,
    )


def _skew(vector):
    x, y, z = vector
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]], dtype=np.float64)


def exp_so3(vector):
    theta = float(np.linalg.norm(vector))
    if theta < 1e-12:
        return np.eye(3) + _skew(vector)
    axis = vector / theta
    hat = _skew(axis)
    return np.eye(3) + math.sin(theta) * hat + (1.0 - math.cos(theta)) * (hat @ hat)


def orthonormalize(rotation):
    u, _, vt = np.linalg.svd(rotation)
    cleaned = u @ vt
    if np.linalg.det(cleaned) < 0.0:
        u = u.copy()
        u[:, -1] *= -1.0
        cleaned = u @ vt
    return cleaned


def left_update(rotation, translation, delta_t, delta_w):
    """Left perturbation on the parent frame: ``R ← Exp(δω) R``, ``t ← Exp(δω) t + δt``."""
    delta = exp_so3(delta_w)
    return orthonormalize(delta @ rotation), _matvec(delta, translation) + delta_t


def pose_delta(rotation, translation, rotation_init, translation_init):
    delta_t = float(np.linalg.norm(np.asarray(translation) - np.asarray(translation_init)))
    relative = np.asarray(rotation) @ np.asarray(rotation_init).T
    cosine = float(np.clip((np.trace(relative) - 1.0) / 2.0, -1.0, 1.0))
    return delta_t, float(np.degrees(math.acos(cosine)))


def quaternion_xyzw(rotation):
    """Unit quaternion, ``w >= 0``. Shepperd's method."""
    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (rotation[2, 1] - rotation[1, 2]) / scale
        y = (rotation[0, 2] - rotation[2, 0]) / scale
        z = (rotation[1, 0] - rotation[0, 1]) / scale
    elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
        scale = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
        w = (rotation[2, 1] - rotation[1, 2]) / scale
        x = 0.25 * scale
        y = (rotation[0, 1] + rotation[1, 0]) / scale
        z = (rotation[0, 2] + rotation[2, 0]) / scale
    elif rotation[1, 1] > rotation[2, 2]:
        scale = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
        w = (rotation[0, 2] - rotation[2, 0]) / scale
        x = (rotation[0, 1] + rotation[1, 0]) / scale
        y = 0.25 * scale
        z = (rotation[1, 2] + rotation[2, 1]) / scale
    else:
        scale = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
        w = (rotation[1, 0] - rotation[0, 1]) / scale
        x = (rotation[0, 2] + rotation[2, 0]) / scale
        y = (rotation[1, 2] + rotation[2, 1]) / scale
        z = 0.25 * scale
    quat = np.array([x, y, z, w], dtype=np.float64)
    norm = float(np.linalg.norm(quat))
    if norm == 0.0 or not np.isfinite(quat).all():
        raise ExtrinsicError("rotation has no finite quaternion")
    quat /= norm
    if quat[3] < 0.0:
        quat = -quat
    return quat


def _pack_indices(indices):
    shifted = np.asarray(indices, dtype=np.int64) + _BIAS
    if shifted.size and (np.any(shifted < 0) or np.any(shifted >= (1 << _SHIFT))):
        raise ExtrinsicError("point is outside the spatial-hash bounds")
    return (shifted[:, 0] << (2 * _SHIFT)) | (shifted[:, 1] << _SHIFT) | shifted[:, 2]


def _pack_one(ix, iy, iz):
    return ((ix + _BIAS) << (2 * _SHIFT)) | ((iy + _BIAS) << _SHIFT) | (iz + _BIAS)


class SpatialHash:
    """27-cell hash. Complete for ``radius`` only when ``cell >= radius``."""

    def __init__(self, points, cell):
        self.points = np.ascontiguousarray(np.asarray(points, dtype=np.float64).reshape(-1, 3))
        self.cell = float(cell)
        if self.cell <= 0.0:
            raise ExtrinsicError("search cell must be positive")
        self.buckets = {}
        if len(self.points) == 0:
            self.cells = np.zeros((0, 3), dtype=np.int64)
            return
        self.cells = np.floor(self.points / self.cell).astype(np.int64)
        keys = _pack_indices(self.cells)
        order = np.argsort(keys, kind="mergesort")
        sorted_keys = keys[order]
        splits = np.flatnonzero(np.diff(sorted_keys)) + 1
        starts = np.concatenate(([0], splits))
        ends = np.concatenate((splits, [len(order)]))
        for start, end in zip(starts, ends):
            self.buckets[int(sorted_keys[start])] = order[start:end]

    def _check_radius(self, radius):
        if radius > self.cell + 1e-9:
            raise ValueError(
                f"search radius {radius} m exceeds hash cell {self.cell} m; "
                "a 27-cell neighborhood covers the radius only when the cell is at least that large"
            )

    def _candidate_ids(self, ix, iy, iz):
        parts = []
        for ox in (-1, 0, 1):
            for oy in (-1, 0, 1):
                for oz in (-1, 0, 1):
                    bucket = self.buckets.get(_pack_one(ix + ox, iy + oy, iz + oz))
                    if bucket is not None and len(bucket):
                        parts.append(bucket)
        if not parts:
            return np.zeros(0, dtype=np.int64)
        if len(parts) == 1:
            return parts[0]
        return np.concatenate(parts)

    def indices_within(self, query, radius):
        self._check_radius(radius)
        query = np.asarray(query, dtype=np.float64)
        cell = np.floor(query / self.cell).astype(np.int64)
        cand = self._candidate_ids(int(cell[0]), int(cell[1]), int(cell[2]))
        if len(cand) == 0:
            return np.zeros(0, dtype=np.int64)
        delta = self.points[cand] - query
        d2 = (delta[:, 0] * delta[:, 0] + delta[:, 1] * delta[:, 1]) + delta[:, 2] * delta[:, 2]
        limit = (radius + 1e-12) * (radius + 1e-12)
        return cand[d2 <= limit]

    def nearest_batch(self, queries, radius):
        """Nearest index and distance. Index is -1 when nothing lies inside ``radius``."""
        self._check_radius(radius)
        queries = np.asarray(queries, dtype=np.float64).reshape(-1, 3)
        out_index = np.full(len(queries), -1, dtype=np.int64)
        out_dist = np.full(len(queries), np.inf)
        if len(queries) == 0 or len(self.points) == 0:
            return out_index, out_dist
        q_cells = np.floor(queries / self.cell).astype(np.int64)
        q_keys = _pack_indices(q_cells)
        order = np.argsort(q_keys, kind="mergesort")
        sorted_keys = q_keys[order]
        limit = (radius + 1e-12) * (radius + 1e-12)
        start = 0
        while start < len(order):
            key = int(sorted_keys[start])
            end = start + 1
            while end < len(order) and int(sorted_keys[end]) == key:
                end += 1
            qids = order[start:end]
            cx, cy, cz = (int(value) for value in q_cells[qids[0]])
            cand = self._candidate_ids(cx, cy, cz)
            if len(cand):
                # Smallest index wins an exact tie, matching a brute-force argmin.
                cand_order = np.argsort(cand, kind="mergesort")
                cand = cand[cand_order]
                neigh = self.points[cand]
                block = queries[qids]
                chosen, dist2 = _nearest_in_block(block, neigh, cand, limit)
                out_index[qids] = chosen
                finite = np.isfinite(dist2)
                out_dist[qids[finite]] = np.sqrt(dist2[finite])
            start = end
        return out_index, out_dist


def _nearest_in_block(block, neigh, cand, limit):
    count = len(block)
    chosen = np.full(count, -1, dtype=np.int64)
    dist2 = np.full(count, np.inf)
    width = max(1, 2_000_000 // max(len(neigh), 1))
    for start in range(0, count, width):
        end = min(count, start + width)
        part = block[start:end]
        d2 = (part[:, None, 0] - neigh[None, :, 0]) ** 2
        d2 += (part[:, None, 1] - neigh[None, :, 1]) ** 2
        d2 += (part[:, None, 2] - neigh[None, :, 2]) ** 2
        local = np.argmin(d2, axis=1)
        rows = np.arange(end - start)
        mind = d2[rows, local]
        chosen[start:end] = cand[local]
        dist2[start:end] = mind
    miss = dist2 > limit
    chosen[miss] = -1
    dist2[miss] = np.inf
    return chosen, dist2


def range_gate(points, min_range, max_range):
    """Keep ``min_range <= range <= max_range``. A point inside the band is kept.

    The band is not a static-structure test. A person standing 2 m from the rig
    stays in the default 1.5–8 m window. Callers still have to record a stationary scene.
    """
    if len(points) == 0:
        return points
    radius2 = (points[:, 0] * points[:, 0] + points[:, 1] * points[:, 1]) + points[:, 2] * points[:, 2]
    keep = np.isfinite(points).all(axis=1)
    if min_range is not None and min_range > 0.0:
        keep &= radius2 >= min_range * min_range
    if max_range is not None:
        keep &= radius2 <= max_range * max_range
    return points[keep]


def voxel_downsample(points, voxel, cap):
    """First point of each ``voxel`` cell, in input order. ``voxel <= 0`` keeps every point."""
    if len(points) == 0:
        return points
    if voxel is not None and voxel > 0.0:
        cells = np.floor(points / voxel).astype(np.int64)
        _, first = np.unique(_pack_indices(cells), return_index=True)
        points = points[np.sort(first)]
    if cap is not None and len(points) > cap:
        take = (np.arange(cap) * (len(points) / cap)).astype(np.int64)
        points = points[take]
    return np.ascontiguousarray(points)


def prepare_cloud(points, settings):
    points = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    finite = np.isfinite(points).all(axis=1)
    points = points[finite]
    points = range_gate(points, settings.min_range_m, settings.max_range_m)
    return voxel_downsample(points, settings.voxel_m, settings.point_cap)


def estimate_normals(points, settings):
    """Unit normals from a 0.20 m PCA. Invalid rows are NaN.

    A point is planar when the smallest eigenvalue is at most ``planarity_ratio``
    times the middle one, with at least ``normal_min_points`` neighbors. The normal
    is flipped so it points toward the target sensor origin.
    """
    normals = np.full((len(points), 3), np.nan, dtype=np.float64)
    if len(points) == 0:
        return normals
    spatial = SpatialHash(points, settings.search_cell_m)
    radius2 = (settings.normal_radius_m + 1e-12) ** 2
    for ids in spatial.buckets.values():
        cx, cy, cz = (int(value) for value in spatial.cells[int(ids[0])])
        cand = spatial._candidate_ids(cx, cy, cz)
        if len(cand) < settings.normal_min_points:
            continue
        neigh = points[cand]
        block = points[ids]
        width = max(1, 2_000_000 // len(neigh))
        for start in range(0, len(ids), width):
            end = min(len(ids), start + width)
            part = block[start:end]
            d2 = (part[:, None, 0] - neigh[None, :, 0]) ** 2
            d2 += (part[:, None, 1] - neigh[None, :, 1]) ** 2
            d2 += (part[:, None, 2] - neigh[None, :, 2]) ** 2
            for local in range(end - start):
                sel = d2[local] <= radius2
                count = int(sel.sum())
                if count < settings.normal_min_points:
                    continue
                chosen = neigh[sel]
                center = chosen.mean(axis=0)
                delta = chosen - center
                cov = (delta.T @ delta) / count
                evals, evecs = np.linalg.eigh(cov)
                if evals[1] <= 1e-12 or evals[0] > settings.planarity_ratio * evals[1]:
                    continue
                normal = evecs[:, 0]
                point = points[int(ids[start + local])]
                if float(np.dot(normal, point)) > 0.0:
                    normal = -normal
                norm = float(np.linalg.norm(normal))
                if norm == 0.0:
                    continue
                normals[int(ids[start + local])] = normal / norm
    return normals


def condition_ratio(jacobian):
    """``λ_min / λ_max`` of ``JᵀJ``. Both eigenvalues scale the same way with row copies.

    ``J``'s rotational columns must already be divided by the characteristic length,
    so the two eigenvalues share units. Dividing only ``λ_min`` by the row count and
    comparing that with the raw ``λ_max`` makes the ratio depend on how many times
    each sample was copied.
    """
    jacobian = np.asarray(jacobian, dtype=np.float64)
    if jacobian.ndim != 2 or jacobian.shape[0] < jacobian.shape[1] or jacobian.shape[1] == 0:
        return 0.0, 0.0, 0.0
    gram = jacobian.T @ jacobian
    evals = np.linalg.eigvalsh(gram)
    lam_min = max(float(evals[0]), 0.0)
    lam_max = max(float(evals[-1]), 0.0)
    if lam_max <= 0.0:
        return 0.0, lam_min, lam_max
    return lam_min / lam_max, lam_min, lam_max


def fixed_metrics(source, target_spatial, planar_spatial, planar_normals, rotation, translation, radius):
    """Inlier fraction and plane RMS inside a fixed radius. Solver trimming is not applied."""
    moved = apply_rigid(source, rotation, translation)
    note = RESIDUAL_TRIM_NOTE
    if len(moved) == 0:
        return {
            "inlier_radius_m": radius,
            "inlier_fraction": 0.0,
            "residual_rms_m": None,
            "correspondences": 0,
            "residual_trim_note": note,
        }
    index, dist = target_spatial.nearest_batch(moved, radius)
    fraction = float(np.mean(dist <= radius + 1e-12))
    p_index, p_dist = planar_spatial.nearest_batch(moved, radius)
    use = p_dist <= radius + 1e-12
    count = int(use.sum())
    rms = None
    if count:
        residual = np.sum(planar_normals[p_index[use]] * (moved[use] - planar_spatial.points[p_index[use]]), axis=1)
        rms = float(np.sqrt(np.mean(residual * residual)))
    return {
        "inlier_radius_m": float(radius),
        "inlier_fraction": fraction,
        "residual_rms_m": rms,
        "correspondences": count,
        "residual_trim_note": note,
    }


def _plane_system(source_xyz, target_xyz, normals, length):
    crossed = np.cross(source_xyz, normals)
    jacobian = np.empty((len(source_xyz), 6), dtype=np.float64)
    jacobian[:, 0:3] = normals
    jacobian[:, 3:6] = crossed / length
    residual = np.sum(normals * (source_xyz - target_xyz), axis=1)
    return jacobian, residual


def _trim_mask(residual, floor_m, factor, gate):
    magnitude = np.abs(residual)
    if len(magnitude) == 0:
        return np.zeros(0, dtype=bool)
    median = float(np.median(magnitude))
    limit = min(gate, max(floor_m, factor * median))
    return magnitude <= limit


def _huber(residual, delta):
    magnitude = np.abs(residual)
    weights = np.ones(len(residual), dtype=np.float64)
    heavy = magnitude > delta
    weights[heavy] = delta / magnitude[heavy]
    return weights


def _empty_observability(threshold, length):
    return {
        "characteristic_length_m": float(length),
        "lambda_min": None,
        "lambda_max": None,
        "lambda_min_over_n": None,
        "lambda_max_over_n": None,
        "ratio": None,
        "threshold": float(threshold),
        "rows": 0,
        "observable": False,
    }


def _observability(jacobian, weights, threshold, length):
    scale = np.sqrt(weights)
    ratio, lam_min, lam_max = condition_ratio(jacobian * scale[:, None])
    rows = int(jacobian.shape[0])
    over_n_min = lam_min / rows if rows else None
    over_n_max = lam_max / rows if rows else None
    return {
        "characteristic_length_m": float(length),
        "lambda_min": lam_min,
        "lambda_max": lam_max,
        "lambda_min_over_n": over_n_min,
        "lambda_max_over_n": over_n_max,
        "ratio": ratio,
        "threshold": float(threshold),
        "rows": rows,
        "observable": bool(ratio >= threshold and lam_max > 0.0),
    }


def _gather(moved, spatial, normals, gate):
    index, dist = spatial.nearest_batch(moved, gate)
    keep = dist <= gate + 1e-12
    if not np.any(keep):
        return None
    return moved[keep], spatial.points[index[keep]], normals[index[keep]]


def calibrate_points(parent, child, init=None, init_source="nominal_mount", settings=None, sources=None):
    """Estimate ``T_parent←child``. A rejected result is never marked eligible."""
    settings = settings or SolverSettings()
    if init is None:
        rotation_init, translation_init = nominal_mount()
        init_source = "nominal_mount"
    else:
        rotation_init = np.asarray(init[0], dtype=np.float64).copy()
        translation_init = np.asarray(init[1], dtype=np.float64).copy()
    rotation_init = orthonormalize(rotation_init)
    translation_init = translation_init.astype(np.float64).reshape(3)
    parent_pts = prepare_cloud(parent, settings)
    child_pts = prepare_cloud(child, settings)
    rotation = rotation_init.copy()
    translation = translation_init.copy()
    reason = None
    converged = False
    iterations = 0
    observability = _empty_observability(settings.condition_ratio_min, settings.characteristic_length_m)
    metrics_before = None
    metrics_after = None

    if len(parent_pts) < settings.min_points or len(child_pts) < settings.min_points:
        reason = "insufficient_points"
    else:
        normals = estimate_normals(parent_pts, settings)
        valid = np.isfinite(normals).all(axis=1)
        if int(valid.sum()) < settings.min_correspondences:
            reason = "insufficient_normals"
        else:
            planar_pts = parent_pts[valid]
            planar_normals = normals[valid]
            target_hash = SpatialHash(parent_pts, settings.search_cell_m)
            planar_hash = SpatialHash(planar_pts, settings.search_cell_m)
            metrics_before = fixed_metrics(
                child_pts, target_hash, planar_hash, planar_normals, rotation, translation, settings.inlier_radius_m
            )
            if settings.max_iterations <= 0:
                reason = "not_refined"
                metrics_after = dict(metrics_before)
            else:
                gate = settings.gate_initial_m
                for iteration in range(settings.max_iterations):
                    iterations = iteration + 1
                    moved = apply_rigid(child_pts, rotation, translation)
                    gathered = _gather(moved, planar_hash, planar_normals, gate)
                    if gathered is None:
                        reason = "insufficient_correspondences"
                        break
                    src, tgt, nrm = gathered
                    jacobian, residual = _plane_system(src, tgt, nrm, settings.characteristic_length_m)
                    keep = _trim_mask(
                        residual, settings.residual_trim_floor_m, settings.residual_trim_factor, gate
                    )
                    if int(keep.sum()) < settings.min_correspondences:
                        reason = "insufficient_correspondences"
                        break
                    weights = _huber(residual[keep], settings.huber_delta_m)
                    observability = _observability(
                        jacobian[keep], weights, settings.condition_ratio_min, settings.characteristic_length_m
                    )
                    if not observability["observable"]:
                        reason = "unobservable"
                        break
                    scale = np.sqrt(weights)
                    weighted = jacobian[keep] * scale[:, None]
                    target = -residual[keep] * scale
                    update, _, _, _ = np.linalg.lstsq(weighted, target, rcond=None)
                    if not np.isfinite(update).all():
                        reason = "non_finite_step"
                        break
                    delta_t = update[:3]
                    delta_w = update[3:] / settings.characteristic_length_m
                    rotation, translation = left_update(rotation, translation, delta_t, delta_w)
                    if not (np.isfinite(rotation).all() and np.isfinite(translation).all()):
                        reason = "non_finite_step"
                        break
                    step_t = float(np.linalg.norm(delta_t))
                    step_deg = float(np.degrees(np.linalg.norm(delta_w)))
                    trimmed = residual[keep]
                    rms = float(np.sqrt(np.mean(trimmed * trimmed)))
                    gate = min(
                        settings.search_cell_m,
                        max(settings.gate_min_m, settings.gate_rms_factor * rms),
                    )
                    if step_t < settings.step_translation_m and step_deg < settings.step_rotation_deg:
                        converged = True
                        reason = None
                        break
                else:
                    reason = "iteration_exhaustion"
                if reason in (None, "iteration_exhaustion"):
                    moved = apply_rigid(child_pts, rotation, translation)
                    gathered = _gather(moved, planar_hash, planar_normals, gate)
                    if gathered is None:
                        if reason is None:
                            reason = "insufficient_correspondences"
                        converged = False
                    else:
                        src, tgt, nrm = gathered
                        jacobian, residual = _plane_system(src, tgt, nrm, settings.characteristic_length_m)
                        keep = _trim_mask(
                            residual, settings.residual_trim_floor_m, settings.residual_trim_factor, gate
                        )
                        if int(keep.sum()) < settings.min_correspondences:
                            if reason is None:
                                reason = "insufficient_correspondences"
                            converged = False
                        else:
                            weights = _huber(residual[keep], settings.huber_delta_m)
                            observability = _observability(
                                jacobian[keep], weights, settings.condition_ratio_min, settings.characteristic_length_m
                            )
                            if not observability["observable"]:
                                reason = "unobservable"
                                converged = False
                metrics_after = fixed_metrics(
                    child_pts, target_hash, planar_hash, planar_normals, rotation, translation, settings.inlier_radius_m
                )
                if reason is None:
                    if not converged:
                        reason = "iteration_exhaustion"
                    else:
                        delta_m, delta_deg = pose_delta(rotation, translation, rotation_init, translation_init)
                        rms = metrics_after["residual_rms_m"]
                        if (
                            delta_m > settings.accept_translation_m
                            or delta_deg > settings.accept_rotation_deg
                            or rms is None
                            or rms > settings.accept_residual_rms_m
                            or metrics_after["inlier_fraction"] < settings.accept_inlier_fraction
                            or metrics_after["correspondences"] < settings.min_correspondences
                        ):
                            reason = "outside_envelope"

    if metrics_before is None:
        metrics_before = {
            "inlier_radius_m": float(settings.inlier_radius_m),
            "inlier_fraction": 0.0,
            "residual_rms_m": None,
            "correspondences": 0,
            "residual_trim_note": RESIDUAL_TRIM_NOTE,
        }
        metrics_after = dict(metrics_before)
    accepted = reason is None
    return _payload(
        rotation, translation, rotation_init, translation_init, init_source, settings, sources,
        parent_pts, child_pts, metrics_before, metrics_after, observability, iterations, converged,
        accepted, reason,
    )


def _num(value):
    number = float(value)
    if not math.isfinite(number):
        raise ExtrinsicError("refusing to write a non-finite extrinsic")
    return number


def _matrix(rotation, translation):
    rows = []
    for row in range(3):
        rows.append([
            _num(rotation[row, 0]), _num(rotation[row, 1]), _num(rotation[row, 2]), _num(translation[row])
        ])
    rows.append([0.0, 0.0, 0.0, 1.0])
    return rows


def _metric_json(metrics):
    rms = metrics["residual_rms_m"]
    return {
        "inlier_radius_m": _num(metrics["inlier_radius_m"]),
        "inlier_fraction": _num(metrics["inlier_fraction"]),
        "residual_rms_m": None if rms is None else _num(rms),
        "correspondences": int(metrics["correspondences"]),
        "residual_trim_note": metrics["residual_trim_note"],
    }


def _settings_json(settings):
    return {
        "method": "point_to_plane",
        "scene_precondition": "stationary_rig_and_static_scene",
        "min_range_m": settings.min_range_m,
        "max_range_m": settings.max_range_m,
        "voxel_m": settings.voxel_m,
        "point_cap": settings.point_cap,
        "normal_radius_m": settings.normal_radius_m,
        "normal_min_points": settings.normal_min_points,
        "planarity_ratio": settings.planarity_ratio,
        "gate_initial_m": settings.gate_initial_m,
        "gate_min_m": settings.gate_min_m,
        "gate_rms_factor": settings.gate_rms_factor,
        "search_cell_m": settings.search_cell_m,
        "characteristic_length_m": settings.characteristic_length_m,
        "residual_trim_floor_m": settings.residual_trim_floor_m,
        "residual_trim_factor": settings.residual_trim_factor,
        "huber_delta_m": settings.huber_delta_m,
        "max_iterations": settings.max_iterations,
        "step_translation_m": settings.step_translation_m,
        "step_rotation_deg": settings.step_rotation_deg,
        "min_correspondences": settings.min_correspondences,
        "min_points": settings.min_points,
        "condition_ratio_min": settings.condition_ratio_min,
        "accept_translation_m": settings.accept_translation_m,
        "accept_rotation_deg": settings.accept_rotation_deg,
        "accept_residual_rms_m": settings.accept_residual_rms_m,
        "accept_inlier_fraction": settings.accept_inlier_fraction,
        "inlier_radius_m": settings.inlier_radius_m,
    }


def _payload(rotation, translation, rotation_init, translation_init, init_source, settings, sources,
             parent_pts, child_pts, metrics_before, metrics_after, observability, iterations, converged,
             accepted, reason):
    quat = quaternion_xyzw(rotation)
    delta_m, delta_deg = pose_delta(rotation, translation, rotation_init, translation_init)
    obs = {
        "characteristic_length_m": _num(observability["characteristic_length_m"]),
        "lambda_min": None if observability["lambda_min"] is None else _num(observability["lambda_min"]),
        "lambda_max": None if observability["lambda_max"] is None else _num(observability["lambda_max"]),
        "lambda_min_over_n": None if observability["lambda_min_over_n"] is None else _num(observability["lambda_min_over_n"]),
        "lambda_max_over_n": None if observability["lambda_max_over_n"] is None else _num(observability["lambda_max_over_n"]),
        "ratio": None if observability["ratio"] is None else _num(observability["ratio"]),
        "threshold": _num(observability["threshold"]),
        "rows": int(observability["rows"]),
        "observable": bool(observability["observable"]),
    }
    before = _metric_json(metrics_before)
    after = _metric_json(metrics_after)
    return {
        "schema": SCHEMA,
        "parent_frame": PARENT_FRAME,
        "child_frame": CHILD_FRAME,
        "convention": CONVENTION,
        "T_parent_child": _matrix(rotation, translation),
        "translation_m": [_num(translation[0]), _num(translation[1]), _num(translation[2])],
        "quaternion_xyzw": [_num(quat[0]), _num(quat[1]), _num(quat[2]), _num(quat[3])],
        "init_source": init_source,
        "nominal_mount": {
            "yaw_deg": YAW_DEG,
            "pitch_deg": PITCH_DEG,
            "roll_deg": ROLL_DEG,
            "translation_m": list(NOMINAL_TRANSLATION_M),
            "rotation": "R = Rz(yaw) Ry(pitch) Rx(roll)",
        },
        "T_init": _matrix(rotation_init, translation_init),
        "room_level": {
            "status": "deferred",
            "composition": "T_room_mid360 = T_room_jt128 * T_jt128_mid360",
            "T_room_jt128_path": LEVEL_PATH,
            "note": (
                "This file is T_jt128_mid360. hesai_lidar is not the room frame. "
                "T_room_jt128 levels the JT128 so z is up and is not estimated here."
            ),
        },
        "scene_precondition": "stationary_rig_and_static_scene",
        "solver": _settings_json(settings),
        "delta_from_init_m": _num(delta_m),
        "delta_from_init_deg": _num(delta_deg),
        "metrics_before": before,
        "metrics_after": after,
        "inlier_radius_m": after["inlier_radius_m"],
        "inlier_fraction": after["inlier_fraction"],
        "residual_rms_m": after["residual_rms_m"],
        "correspondences": after["correspondences"],
        "source_points": int(len(child_pts)),
        "target_points": int(len(parent_pts)),
        "observability": obs,
        "observable": bool(obs["observable"]),
        "iterations": int(iterations),
        "converged": bool(converged),
        "accepted": bool(accepted),
        "reject_reason": reason,
        "activation": {
            "eligible": bool(accepted),
            "authoritative_path": AUTHORITATIVE_PATH,
            "rule": (
                "A candidate with accepted false is not copied onto the authoritative path "
                "and cannot become the active calibration."
            ),
        },
        "sources": list(sources or []),
    }


def _as_matrix(value, what):
    if not isinstance(value, list) or len(value) != 4 or any(not isinstance(row, list) or len(row) != 4 for row in value):
        raise InitError(f"{what} must be a 4x4 matrix")
    try:
        matrix = np.array(value, dtype=np.float64)
    except (TypeError, ValueError) as exc:
        raise InitError(f"{what} is not numeric") from exc
    return matrix


def load_init(path):
    """Validate schema, frame direction, bottom row, finiteness, and a proper rotation."""
    try:
        payload = json.loads(Path(path).read_text())
    except (OSError, json.JSONDecodeError) as exc:
        raise InitError(f"cannot read init: {exc}") from exc
    if not isinstance(payload, dict):
        raise InitError("init schema must be a JSON object")
    if payload.get("schema") != SCHEMA:
        raise InitError(f"init schema must be {SCHEMA}")
    if payload.get("parent_frame") != PARENT_FRAME or payload.get("child_frame") != CHILD_FRAME:
        raise InitError(
            f"init frame direction must be parent {PARENT_FRAME} and child {CHILD_FRAME}"
        )
    if payload.get("convention") != CONVENTION:
        raise InitError(f"init convention must be {CONVENTION!r}")
    matrix = _as_matrix(payload.get("T_parent_child"), "T_parent_child")
    if not np.isfinite(matrix).all():
        raise InitError("init matrix must be finite")
    if not np.allclose(matrix[3], [0.0, 0.0, 0.0, 1.0], atol=1e-6):
        raise InitError("init bottom row must be 0 0 0 1")
    rotation = matrix[:3, :3]
    if abs(float(np.linalg.det(rotation)) - 1.0) > 1e-6 or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6):
        raise InitError("init rotation must be a proper rotation (orthonormal, det +1)")
    translation = matrix[:3, 3]
    if "translation_m" in payload:
        stated = np.array(payload["translation_m"], dtype=np.float64)
        if stated.shape != (3,) or not np.isfinite(stated).all() or not np.allclose(stated, translation, atol=1e-6):
            raise InitError("init translation_m does not match T_parent_child")
    if "quaternion_xyzw" in payload:
        stated = np.array(payload["quaternion_xyzw"], dtype=np.float64)
        expect = quaternion_xyzw(rotation)
        if stated.shape != (4,) or not np.isfinite(stated).all() or not np.allclose(stated, expect, atol=1e-5):
            raise InitError("init quaternion_xyzw does not match T_parent_child")
    return rotation, translation


def _decode_cloud(path, expected_frame):
    spark = Path(__file__).resolve().parents[2] / "spark" / "room_memory"
    if str(spark) not in sys.path:
        sys.path.insert(0, str(spark))
    from room_memory.core.pointcloud2 import PointCloudError, decode_pointcloud2

    try:
        data = Path(path).read_bytes()
    except OSError as exc:
        raise ExtrinsicError(f"cannot read {path}: {exc}") from exc
    try:
        cloud = decode_pointcloud2(data)
    except PointCloudError as exc:
        raise ExtrinsicError(str(exc)) from exc
    if cloud.frame_id != expected_frame:
        raise ExtrinsicError(f"{path}: frame_id {cloud.frame_id!r}, expected {expected_frame!r}")
    return np.asarray(cloud.positions, dtype=np.float64)


def _load_npy(path):
    try:
        points = np.load(path)
    except (OSError, ValueError) as exc:
        raise ExtrinsicError(f"cannot read {path}: {exc}") from exc
    points = np.asarray(points, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ExtrinsicError(f"{path}: expected an (n, 3) array")
    return points


def write_result(path, payload):
    text = json.dumps(payload, indent=2, allow_nan=False) + "\n"
    destination = Path(path)
    if destination.parent and not destination.parent.exists():
        raise ExtrinsicError(f"output directory does not exist: {destination.parent}")
    destination.write_text(text)


def _apply_setting_overrides(settings, min_range, max_range, max_iterations):
    values = {key: getattr(settings, key) for key in settings.__dataclass_fields__}
    if min_range is not None:
        values["min_range_m"] = min_range
    if max_range is not None:
        values["max_range_m"] = max_range
    if max_iterations is not None:
        values["max_iterations"] = max_iterations
    return SolverSettings(**values)


def main(argv=None):
    parser = argparse.ArgumentParser(description="Estimate T_jt128←mid360 from two static clouds.")
    parser.add_argument("pair", nargs="?", help="Directory containing jt128.bin and mid360.bin")
    parser.add_argument("--jt128", help="JT128 PointCloud2 CDR, frame_id hesai_lidar")
    parser.add_argument("--mid360", help="Mid-360 PointCloud2 CDR, frame_id livox_frame")
    parser.add_argument("--parent-npy", help="Parent cloud as an (n, 3) .npy (tests)")
    parser.add_argument("--child-npy", help="Child cloud as an (n, 3) .npy (tests)")
    parser.add_argument("--init", help="Seed JSON of schema labcommand.extrinsic.v1")
    parser.add_argument("--init-matrix", help="Same as --init")
    parser.add_argument("-o", "--output", required=True)
    parser.add_argument("--min-range", type=float)
    parser.add_argument("--max-range", type=float)
    parser.add_argument("--max-iterations", type=int)
    try:
        args = parser.parse_args(argv)
        init_path = args.init or args.init_matrix
        if args.init and args.init_matrix:
            raise ExtrinsicError("pass only one of --init and --init-matrix")
        modes = [args.pair is not None, bool(args.jt128 or args.mid360), bool(args.parent_npy or args.child_npy)]
        if sum(modes) != 1:
            raise ExtrinsicError("pass a snapshot directory, --jt128 and --mid360, or --parent-npy and --child-npy")
        if args.pair is not None:
            directory = Path(args.pair)
            jt_path = directory / "jt128.bin"
            mid_path = directory / "mid360.bin"
            if not jt_path.is_file() or not mid_path.is_file():
                raise ExtrinsicError(f"{directory} must contain jt128.bin and mid360.bin")
            parent = _decode_cloud(jt_path, PARENT_FRAME)
            child = _decode_cloud(mid_path, CHILD_FRAME)
            sources = [str(jt_path), str(mid_path)]
        elif args.jt128 or args.mid360:
            if not args.jt128 or not args.mid360:
                raise ExtrinsicError("pass both --jt128 and --mid360")
            parent = _decode_cloud(args.jt128, PARENT_FRAME)
            child = _decode_cloud(args.mid360, CHILD_FRAME)
            sources = [args.jt128, args.mid360]
        else:
            if not args.parent_npy or not args.child_npy:
                raise ExtrinsicError("pass both --parent-npy and --child-npy")
            parent = _load_npy(args.parent_npy)
            child = _load_npy(args.child_npy)
            sources = [args.parent_npy, args.child_npy]
        if init_path:
            rotation, translation = load_init(init_path)
            init = (rotation, translation)
            init_source = "file"
        else:
            init = None
            init_source = "nominal_mount"
        tuned = _apply_setting_overrides(SolverSettings(), args.min_range, args.max_range, args.max_iterations)
        payload = calibrate_points(parent, child, init=init, init_source=init_source, settings=tuned, sources=sources)
        write_result(args.output, payload)
    except (ExtrinsicError, InitError) as exc:
        print(f"extrinsic: {exc}", file=sys.stderr)
        return 1
    print(
        f"accepted={str(payload['accepted']).lower()} "
        f"reason={payload['reject_reason']} wrote {args.output}"
    )
    return 0 if payload["accepted"] else 2


if __name__ == "__main__":
    sys.exit(main())
