"""Deterministic zone decisions at the configured voxel resolution."""

import numpy as np

from .model import number


def classify(zone, grid, uncertainty_m):
    uncertainty = number(uncertainty_m, "uncertainty_m")
    if uncertainty < 0:
        raise ValueError("uncertainty_m must be nonnegative")
    required = list(grid.volume_cells(zone))
    occupied, free, unknown, conflicts = [], [], [], []
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

    axis, direction, ref = zone.axis, zone.direction, zone.reference
    tangents = [a for a in range(3) if a != axis]
    # Measured clearance uses actual hits, not voxel centres.
    distances = []
    for cell in grid.cells.values():
        for timestamp, point in cell.hits:
            distance = float(direction * (point[axis] - ref))
            if distance > 0 and all(zone.low[a] <= point[a] < zone.high[a]
                                     for a in tangents):
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
    elif unknown:
        margin = None if measured is None else measured - zone.depth
        status, reason = "NOT_MEASURED", "required cells lack free-space evidence"
    else:
        margin = None if measured is None else measured - zone.depth
        near = margin is not None and uncertainty > 0 and abs(margin) <= uncertainty + 1e-12
        status = "INDETERMINATE" if near else "PASS"
        reason = "margin within uncertainty" if near else "every required cell observed free"
    return {
        "id": zone.id, "shape": zone.shape, "rule_id": zone.rule_id,
        "provision": zone.provision, "status": status, "reason": reason,
        "required_depth_m": zone.depth, "min_clearance_m": measured,
        "occupied_voxel_clearance_lower_bound_m": lower,
        "decision_margin_m": margin, "uncertainty_m": uncertainty,
        "required_cells": len(required), "free_only_cells": len(free),
        "occupied_cells": len(occupied), "unknown_cells": len(unknown),
        "conflicting_cells": [list(i) for i in conflicts],
        "free_coverage": len(free) / len(required),
        "evidence_time_range": [min(times), max(times)] if times else None,
    }
