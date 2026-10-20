"""Deterministic zone decisions at the configured voxel resolution."""

import numpy as np

from .model import number


def validate_min_free_rays(value):
    if type(value) is not int or value < 1:
        raise ValueError("min_free_rays must be a positive integer")
    return value


def classify(zone, grid, uncertainty_m, min_free_rays=2):
    min_free_rays = validate_min_free_rays(min_free_rays)
    uncertainty = number(uncertainty_m, "uncertainty_m")
    if uncertainty < 0:
        raise ValueError("uncertainty_m must be nonnegative")
    required = list(grid.volume_cells(zone))
    occupied, free, unknown, conflicts = [], [], [], []
    insufficient = []
    times = set()
    for index in required:
        cell = grid.cells.get(index)
        if cell:
            times.update(cell.free_times | cell.occupied_times)
        if cell and cell.occupied_times:
            occupied.append(index)
            if cell.free_times:
                conflicts.append(index)
        elif cell and cell.free_times:
            free.append(index)
        else:
            unknown.append(index)
        if not (cell and cell.occupied_times) and (
                cell is None or cell.free_ray_count < min_free_rays):
            insufficient.append(index)

    axis, direction, ref = zone.axis, zone.direction, zone.reference
    tangents = [a for a in range(3) if a != axis]
    # Measured clearance uses actual hits, not voxel centres.
    distances = []
    for cell in grid.cells.values():
        for timestamp, point in cell.hits:
            distance = float(direction * (point[axis] - ref))
            in_footprint = all(zone.low[a] <= point[a] < zone.high[a]
                               for a in tangents)
            in_band = uncertainty > 0 and all(
                zone.low[a] - uncertainty - 1e-12 <= point[a]
                <= zone.high[a] + uncertainty + 1e-12 for a in tangents)
            if distance > 0 and (in_footprint or in_band):
                distances.append(distance)
                times.add(timestamp)
    measured = min(distances) if distances else None
    lower = None
    if occupied:
        lower = min(max(0.0, float(direction * (
            (grid.bounds(i)[0] if direction == 1 else grid.bounds(i)[1])[axis] - ref)))
                    for i in occupied)
    # If a partially intersecting occupied voxel has no actual hit in the
    # volume, retain the conservative voxel obstruction rather than ignore it.
    cell_clearances = []
    for index in occupied:
        in_volume_distances = []
        for _, point in grid.cells[index].hits:
            if np.all(point >= zone.low) and np.all(point < zone.high):
                in_volume_distances.append(float(direction * (point[axis] - ref)))
        cell_low, cell_high = grid.bounds(index)
        conservative = max(0.0, float(direction * (
            (cell_low if direction == 1 else cell_high)[axis] - ref)))
        cell_clearances.append(min(in_volume_distances) if in_volume_distances else conservative)
    decision_clearance = min(cell_clearances) if cell_clearances else None
    if occupied:
        margin = decision_clearance - zone.depth
        near = uncertainty > 0 and abs(margin) <= uncertainty + 1e-12
        status = "INDETERMINATE" if near else "FAIL"
        reason = "occupied required cells; margin within uncertainty" if near else \
            "occupied required cells"
    elif insufficient:
        margin = None if measured is None else measured - zone.depth
        status, reason = "NOT_MEASURED", "required cells lack minimum free-ray evidence"
    else:
        margin = None if measured is None else measured - zone.depth
        # A sideways hit can lie deep inside the required depth while outside
        # its nominal footprint. It must also block PASS, not only end-face hits.
        near = margin is not None and uncertainty > 0 and margin <= uncertainty + 1e-12
        status = "INDETERMINATE" if near else "PASS"
        reason = "hit within expanded uncertainty volume" if near else \
            "every required cell has minimum free-ray evidence"
    return {
        "id": zone.id, "shape": zone.shape, "rule_id": zone.rule_id,
        "provision": zone.provision, "status": status, "reason": reason,
        "required_depth_m": zone.depth, "min_clearance_m": measured,
        "occupied_voxel_clearance_lower_bound_m": lower,
        "decision_margin_m": margin, "uncertainty_m": uncertainty,
        "min_free_rays": min_free_rays,
        "insufficient_evidence_cells": len(insufficient),
        "required_cells": len(required), "free_only_cells": len(free),
        "occupied_cells": len(occupied), "unknown_cells": len(unknown),
        "conflicting_cells": [list(i) for i in conflicts],
        "free_coverage": len(free) / len(required),
        "evidence_time_range": [min(times), max(times)] if times else None,
    }
