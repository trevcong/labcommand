"""Wall sharpness from a registered cloud.

Two half-cell-shifted voxel grids are fit with local PCA. The reported
residual is the worse grid, so a voxel boundary cannot hide a doubled wall.
Points are not downsampled before the residual.

A doubled sheet inside one cell fails the single-plane planarity test:
the gap is a large part of a 30 cm cell. Those cells are measured before
they would have been discarded. Dropping them left a thin sliver that
could pass the gate after a small shift.

A wall that runs along the voxel diagonal has the same failure for a
different reason. The points inside one cell are about as deep as they
are wide, so the cell's own normal is noise. The cell is dropped, or
called a floor, and pairing never sees the two sheets. The headline then
keeps a thin sliver of one sheet. Those cells are measured again with a
normal fit on a wider neighborhood. Cells that already have a reliable
normal are left as they were.

This is a geometric estimate. Two real parallel surfaces look the same as
drift. The number does not change the walk verdict.
"""

from __future__ import annotations

import math

import numpy as np

from spatio.walkcheck.defaults import (
    PROPOSED_WALL_LABEL,
    WALL_DOUBLE_MAX_M,
    WALL_DOUBLE_MIN_M,
    WALL_GATE_CM,
    WALL_LATERAL_MAX_M,
    WALL_LAYER_MIN_FRACTION,
    WALL_LAYER_P95_MAX_M,
    WALL_MAX_ABS_NZ,
    WALL_MIN_CELL_POINTS,
    WALL_NORMAL_SUPPORT_M,
    WALL_PARALLEL_DEG,
    WALL_PLANARITY_MIN,
    WALL_VOXEL_M,
)


def wall_sharpness(
    xyz: np.ndarray,
    *,
    voxel_m: float = WALL_VOXEL_M,
    planarity_min: float = WALL_PLANARITY_MIN,
    max_abs_nz: float = WALL_MAX_ABS_NZ,
    min_cell_points: int = WALL_MIN_CELL_POINTS,
    gate_cm: float = WALL_GATE_CM,
) -> dict:
    points = np.asarray(xyz, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("xyz must have shape (N, 3)")
    finite_mask = np.isfinite(points).all(axis=1)
    finite = points[finite_mask]
    base = {
        "point_count": int(len(points)),
        "finite_point_count": int(len(finite)),
        "bounds_min_m": None if len(finite) == 0 else finite.min(axis=0),
        "bounds_max_m": None if len(finite) == 0 else finite.max(axis=0),
        "voxel_m": voxel_m,
        "proposed_gate_cm": gate_cm,
        "proposed_gate_label": PROPOSED_WALL_LABEL,
        "method": (
            f"voxel PCA, {voxel_m:.2f} m cells, two grids half a cell apart, "
            "vertical patches only, no downsampling. "
            "A cell with two thin sheets is kept even when one plane does not fit. "
            "A cell whose own normal is noise is measured again on a wider neighborhood."
        ),
    }
    if len(finite) < min_cell_points:
        return _unevaluable(base, "too_few_points")

    phases = [
        _phase(finite, voxel_m, shift * voxel_m, planarity_min, max_abs_nz, min_cell_points)
        for shift in (0.0, 0.5)
    ]
    usable = [phase for phase in phases if phase["p95_m"] is not None]
    if not usable:
        return _unevaluable(base, "no_vertical_planar_patches", phases)

    headline = max(usable, key=lambda phase: phase["p95_m"])
    separations = [sep for phase in usable for sep in phase["separations_m"]]
    offending = [sep for sep in separations if sep > WALL_DOUBLE_MIN_M]
    doubled = len(offending) > 0
    p95_cm = headline["p95_m"] * 100.0
    residual_ok = p95_cm <= gate_cm
    proposed = "PASS" if residual_ok and not doubled else "FAIL"
    return {
        **base,
        "status": "OK",
        "reason": None,
        "p95_point_to_plane_cm": p95_cm,
        "median_point_to_plane_cm": headline["median_m"] * 100.0,
        "headline_phase_origin_m": headline["origin_m"],
        "planar_patch_count": headline["planar_patches"],
        "planar_point_count": headline["planar_points"],
        "phases": [_public_phase(phase) for phase in phases],
        "doubled_wall": doubled,
        "revisit_separation_cm": None if not offending else min(offending) * 100.0,
        "max_revisit_separation_cm": None if not offending else max(offending) * 100.0,
        "closest_parallel_cm": None if not separations else min(separations) * 100.0,
        "parallel_pair_count": len(offending),
        "proposed_gate_result": proposed,
    }


def _unevaluable(base: dict, reason: str, phases: list | None = None) -> dict:
    return {
        **base,
        "status": "NOT-EVALUABLE",
        "reason": reason,
        "p95_point_to_plane_cm": None,
        "median_point_to_plane_cm": None,
        "headline_phase_origin_m": None,
        "planar_patch_count": 0,
        "planar_point_count": 0,
        "phases": [] if phases is None else [_public_phase(phase) for phase in phases],
        "doubled_wall": None,
        "revisit_separation_cm": None,
        "max_revisit_separation_cm": None,
        "closest_parallel_cm": None,
        "parallel_pair_count": 0,
        "proposed_gate_result": "NOT-EVALUABLE",
    }


def _public_phase(phase: dict) -> dict:
    return {
        "origin_m": phase["origin_m"],
        "p95_cm": None if phase["p95_m"] is None else phase["p95_m"] * 100.0,
        "planar_patches": phase["planar_patches"],
        "planar_points": phase["planar_points"],
    }


def _phase(points, voxel, origin, planarity_min, max_abs_nz, min_cell_points) -> dict:
    ijk = np.floor((points - origin) / voxel).astype(np.int64)
    order = np.lexsort((ijk[:, 2], ijk[:, 1], ijk[:, 0]))
    ijk_sorted = ijk[order]
    sorted_points = points[order]
    change = np.empty(len(sorted_points), dtype=bool)
    change[0] = True
    if len(sorted_points) > 1:
        change[1:] = np.any(ijk_sorted[1:] != ijk_sorted[:-1], axis=1)
    starts = np.flatnonzero(change)
    ends = np.empty_like(starts)
    ends[:-1] = starts[1:]
    ends[-1] = len(sorted_points)

    all_blocks: dict[tuple[int, int, int], np.ndarray] = {}
    big_cells: list[tuple[int, int, int]] = []
    for start, end in zip(starts, ends):
        cell = tuple(int(v) for v in ijk_sorted[start])
        all_blocks[cell] = sorted_points[start:end]
        if end - start >= min_cell_points:
            big_cells.append(cell)
    support_offsets = _support_offsets(voxel, WALL_NORMAL_SUPPORT_M)

    residuals: list[np.ndarray] = []
    patches: list[tuple] = []
    separations: list[float] = []
    parallel_cos = math.cos(math.radians(WALL_PARALLEL_DEG))

    for cell in big_cells:
        block = all_blocks[cell]
        centroid = block.mean(axis=0)
        centered = block - centroid
        # np.dot, not @. On this Mac, Accelerate warns "divide by zero" for
        # some (N, 3) @ (3,) products even when every result is finite.
        cov = np.dot(centered.T, centered) / float(len(block))
        if not np.isfinite(cov).all():
            continue
        evals, evecs = np.linalg.eigh(cov)
        lam0, lam1, lam2 = (float(v) for v in evals)
        if not np.isfinite(lam2) or lam2 <= 1e-12:
            continue
        planarity = (lam1 - lam0) / lam2
        normal = evecs[:, 0].astype(np.float64, copy=True)
        if not np.isfinite(normal).all():
            continue
        signed = np.dot(centered, normal)
        if not np.isfinite(signed).all():
            continue
        # A confident floor stays a floor. A weak vertical "normal" is the
        # diagonal-wall case: the cell is a blob, and the smallest axis can
        # point up by chance. That one gets a second look below.
        vertical_normal = abs(float(normal[2])) <= max_abs_nz
        weak_normal = planarity < planarity_min
        if not vertical_normal and not weak_normal:
            continue
        # Two sheets make the cell look thick, so the single-plane test
        # fails and the cell used to be dropped. A later shift can then
        # leave only a thin sliver, which passes. Look first.
        layer = None
        residual = signed
        if vertical_normal:
            layer = _layer_separation(signed, min_cell_points)
        single_plane = vertical_normal and not weak_normal
        if not single_plane and layer is None and weak_normal:
            wider = _sheets_from_support(
                cell,
                block,
                all_blocks,
                support_offsets,
                min_cell_points,
                max_abs_nz,
                planarity_min,
            )
            if wider is not None:
                layer, residual = wider
        if layer is not None:
            separations.append(layer)
        if not single_plane and layer is None:
            continue
        residuals.append(np.abs(residual))
        if not single_plane:
            continue
        patches.append((cell, centroid.astype(np.float64, copy=True), normal))

    by_cell = {cell: (centroid, normal) for cell, centroid, normal in patches}
    for cell, centroid, normal in patches:
        i, j, k = cell
        for di in (-1, 0, 1):
            for dj in (-1, 0, 1):
                for dk in (-1, 0, 1):
                    if di == 0 and dj == 0 and dk == 0:
                        continue
                    other_key = (i + di, j + dj, k + dk)
                    if other_key <= cell:
                        continue
                    other = by_cell.get(other_key)
                    if other is None:
                        continue
                    other_centroid, other_normal = other
                    dot = float(np.dot(normal, other_normal))
                    if abs(dot) < parallel_cos:
                        continue
                    # Anti-parallel normals are the same wall direction.
                    # Distance is measured along this cell's normal either way.
                    delta = other_centroid - centroid
                    along = float(np.dot(delta, normal))
                    lateral = float(np.linalg.norm(delta - along * normal))
                    separation = abs(along)
                    # In-plane offsets past one cell are the same sheet, seen
                    # from a slightly tilted normal. Only stacked sheets count.
                    if (
                        lateral < WALL_LATERAL_MAX_M
                        and WALL_LAYER_P95_MAX_M < separation <= WALL_DOUBLE_MAX_M
                    ):
                        separations.append(separation)

    if not residuals:
        return {
            "origin_m": origin,
            "p95_m": None,
            "median_m": None,
            "planar_patches": 0,
            "planar_points": 0,
            "separations_m": [],
        }
    all_residuals = np.concatenate(residuals)
    return {
        "origin_m": float(origin),
        "p95_m": float(np.percentile(all_residuals, 95)),
        "median_m": float(np.median(all_residuals)),
        "planar_patches": len(patches),
        "planar_points": int(all_residuals.size),
        "separations_m": separations,
    }


def _support_offsets(voxel: float, support_m: float) -> list[tuple[int, int, int]]:
    """Cell offsets whose centers fall inside the wider normal fit."""
    if voxel <= 0.0 or support_m <= 0.0:
        return [(0, 0, 0)]
    radius = int(math.ceil(support_m / voxel))
    limit2 = support_m * support_m + 1e-12
    step2 = voxel * voxel
    offsets = []
    for di in range(-radius, radius + 1):
        for dj in range(-radius, radius + 1):
            for dk in range(-radius, radius + 1):
                if (di * di + dj * dj + dk * dk) * step2 <= limit2:
                    offsets.append((di, dj, dk))
    return offsets


def _sheets_from_support(
    cell: tuple[int, int, int],
    block: np.ndarray,
    all_blocks: dict[tuple[int, int, int], np.ndarray],
    offsets: list[tuple[int, int, int]],
    min_cell_points: int,
    max_abs_nz: float,
    planarity_min: float,
) -> tuple[float, np.ndarray] | None:
    """Two thin sheets around one cell, seen with a wider normal.

    The cell's own normal is noise. The fit uses every point within the
    support radius, including cells too small to score on their own, so a
    sheet split across a voxel corner is still visible. The residual is
    this cell's points against that plane, which sits between the sheets.
    """
    i, j, k = cell
    parts = []
    for di, dj, dk in offsets:
        other = all_blocks.get((i + di, j + dj, k + dk))
        if other is not None:
            parts.append(other)
    if not parts:
        return None
    gathered = parts[0] if len(parts) == 1 else np.concatenate(parts)
    if len(gathered) < min_cell_points:
        return None
    centroid = gathered.mean(axis=0)
    centered = gathered - centroid
    cov = np.dot(centered.T, centered) / float(len(gathered))
    if not np.isfinite(cov).all():
        return None
    evals, evecs = np.linalg.eigh(cov)
    lam0, lam1, lam2 = (float(v) for v in evals)
    if not np.isfinite(lam2) or lam2 <= 1e-12:
        return None
    planarity = (lam1 - lam0) / lam2
    normal = evecs[:, 0]
    if not np.isfinite(normal).all():
        return None
    if planarity < planarity_min or abs(float(normal[2])) > max_abs_nz:
        return None
    signed = np.dot(centered, normal)
    if not np.isfinite(signed).all():
        return None
    separation = _layer_separation(signed, min_cell_points)
    # Under the doubled-wall minimum this is thickness, not a second sheet.
    if separation is None or separation <= WALL_DOUBLE_MIN_M:
        return None
    cell_signed = np.dot(block - centroid, normal)
    if not np.isfinite(cell_signed).all():
        return None
    return separation, cell_signed


def _layer_separation(signed: np.ndarray, min_points: int) -> float | None:
    """Separation of two thin sheets along one normal, or None."""
    if signed.size < min_points:
        return None
    center_a = float(np.percentile(signed, 25))
    center_b = float(np.percentile(signed, 75))
    if center_b - center_a < WALL_LAYER_P95_MAX_M:
        return None
    mask = np.ones(signed.shape, dtype=bool)
    for _ in range(12):
        dist_a = np.abs(signed - center_a)
        dist_b = np.abs(signed - center_b)
        mask = dist_a <= dist_b
        if not bool(mask.any()) or bool(mask.all()):
            return None
        next_a = float(signed[mask].mean())
        next_b = float(signed[~mask].mean())
        if abs(next_a - center_a) < 1e-7 and abs(next_b - center_b) < 1e-7:
            center_a, center_b = next_a, next_b
            break
        center_a, center_b = next_a, next_b
    if center_a > center_b:
        center_a, center_b = center_b, center_a
        mask = ~mask
    separation = center_b - center_a
    if not (WALL_LAYER_P95_MAX_M < separation <= WALL_DOUBLE_MAX_M):
        return None
    fraction = float(mask.mean())
    if fraction < WALL_LAYER_MIN_FRACTION or fraction > 1.0 - WALL_LAYER_MIN_FRACTION:
        return None
    side_a = float(np.percentile(np.abs(signed[mask] - center_a), 95))
    side_b = float(np.percentile(np.abs(signed[~mask] - center_b), 95))
    if side_a > WALL_LAYER_P95_MAX_M or side_b > WALL_LAYER_P95_MAX_M:
        return None
    return separation
