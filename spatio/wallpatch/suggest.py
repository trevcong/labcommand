"""Suggest vertical wall patches in one map frame.

Search path (suggestions only): drop the outer 1% on each axis, voxelize,
estimate a normal from neighboring voxel centroids, grow planar regions,
and put one fixed window on each wall plane. The window is then scored on
the full map, with no percentile crop.

The window half extents are [0.50, 0.25, 0.50] m (along, across, vertical),
the same shape as the schema example, so the patches are comparable.
Measure does not use this voxel grid.
"""

from __future__ import annotations

import os

import numpy as np

from spatio.wallpatch.errors import WallpatchError
from spatio.wallpatch.geometry import box_axes, crop_mask, gram3, project
from spatio.wallpatch.numbers import parse_suggest_n, parse_voxel_m
from spatio.wallpatch.pcd import iter_xyz, sha256_file
from spatio.wallpatch.util import write_json

PATCH_SCHEMA = "spatio.wall_patches/1"
DEFAULT_VOXEL_M = 0.08
HALF_EXTENT_M = (0.50, 0.25, 0.50)
_MIN_REGION_VOXELS = 40
_MIN_WINDOW_POINTS = 200
_MAX_ABS_NZ = 0.34
_NEIGHBOR_RADIUS = 2
_MIN_NEIGHBORS = 8
_VOXEL_EIGEN_RATIO = 0.08
_GROW_DEG = 18.0
_MERGE_DEG = 15.0
_MERGE_ACROSS_M = 0.10
_OVERLAP_ACROSS_M = 0.15
_OVERLAP_IN_PLANE_M = 0.80


def suggest_map(map_path, out_path, n=6, voxel_m=DEFAULT_VOXEL_M, verbose=False):
    """Write a patches.json of the n best vertical wall windows."""
    n = parse_suggest_n(n)
    voxel_m = parse_voxel_m(voxel_m)
    map_path = os.path.abspath(map_path)
    if not os.path.isfile(map_path):
        raise WallpatchError("map not found: %s" % map_path)
    sha = sha256_file(map_path)
    lo, hi, n_points = _robust_bounds(map_path)
    sums = _accumulate(map_path, lo, hi, voxel_m)
    candidates = []
    if len(sums) >= _MIN_REGION_VOXELS:
        keys, cents = _materialize(sums)
        normals, planar = _normals(keys, cents)
        regions = _grow(keys, planar, normals)
        planes = _planes(cents, regions)
        for plane in _merge_planes(planes, cents):
            window = _place_window(plane, cents)
            if window is not None:
                candidates.append(window)
    scored = _score(map_path, candidates) if candidates else []
    chosen = _suppress(scored)[:n]
    document = _document(map_path, sha, n_points, voxel_m, chosen)
    write_json(out_path, document)
    if verbose:
        print("map_sha256 %s" % sha)
        print("suggested %d patches from %d map points" % (len(chosen), n_points))
        for patch in document["patches"]:
            print(
                "%s n=%s rms=%s center=%s normal=%s"
                % (
                    patch["name"],
                    patch["n_points"],
                    patch["planarity_rms_m"],
                    patch["box_map_frame"]["center_m"],
                    patch["box_map_frame"]["normal_hint"],
                )
            )
        print("wrote %s" % os.path.abspath(out_path))
    return document


def _robust_bounds(path):
    """Per-axis 1st and 99th percentiles from a stride sample. Suggest only."""
    samples = []
    seen = 0
    for pts in iter_xyz(path):
        seen += int(pts.shape[0])
        finite = np.isfinite(pts).all(axis=1)
        good = pts[finite]
        if good.shape[0] == 0:
            continue
        samples.append(good[::16])
    if seen == 0 or not samples:
        raise WallpatchError("map has no points")
    sample = np.concatenate(samples, axis=0)
    if sample.shape[0] < 10:
        raise WallpatchError("map has too few finite points to suggest patches")
    lo = np.percentile(sample, 1.0, axis=0)
    hi = np.percentile(sample, 99.0, axis=0)
    hi = np.maximum(hi, lo + 1e-3)
    return lo, hi, seen


def _accumulate(path, lo, hi, voxel):
    sums = {}
    for pts in iter_xyz(path):
        finite = np.isfinite(pts).all(axis=1)
        good = pts[finite]
        if good.shape[0] == 0:
            continue
        inside = np.all((good >= lo) & (good <= hi), axis=1)
        good = good[inside]
        if good.shape[0] == 0:
            continue
        _merge_chunk(sums, good, voxel)
    return sums


def _merge_chunk(sums, pts, voxel):
    quantized = np.floor(pts / voxel).astype(np.int64)
    uniq, inverse, counts = np.unique(
        quantized, axis=0, return_inverse=True, return_counts=True
    )
    acc = np.zeros((len(uniq), 3), dtype=np.float64)
    np.add.at(acc, inverse, pts)
    for i in range(len(uniq)):
        key = (int(uniq[i, 0]), int(uniq[i, 1]), int(uniq[i, 2]))
        prev = sums.get(key)
        if prev is None:
            sums[key] = [
                float(acc[i, 0]),
                float(acc[i, 1]),
                float(acc[i, 2]),
                int(counts[i]),
            ]
        else:
            prev[0] += float(acc[i, 0])
            prev[1] += float(acc[i, 1])
            prev[2] += float(acc[i, 2])
            prev[3] += int(counts[i])


def _materialize(sums):
    keys = list(sums.keys())
    cents = np.empty((len(keys), 3), dtype=np.float64)
    for i, key in enumerate(keys):
        sx, sy, sz, count = sums[key]
        cents[i, 0] = sx / float(count)
        cents[i, 1] = sy / float(count)
        cents[i, 2] = sz / float(count)
    return keys, cents


def _normals(keys, cents):
    index = {key: i for i, key in enumerate(keys)}
    radius = _NEIGHBOR_RADIUS
    offsets = [
        (dx, dy, dz)
        for dx in range(-radius, radius + 1)
        for dy in range(-radius, radius + 1)
        for dz in range(-radius, radius + 1)
    ]
    normals = np.zeros((len(keys), 3), dtype=np.float64)
    planar = np.zeros(len(keys), dtype=bool)
    for i, key in enumerate(keys):
        gathered = []
        for dx, dy, dz in offsets:
            other = index.get((key[0] + dx, key[1] + dy, key[2] + dz))
            if other is not None:
                gathered.append(cents[other])
        if len(gathered) < _MIN_NEIGHBORS:
            continue
        cloud = np.vstack(gathered)
        centered = cloud - cloud.mean(axis=0)
        eigvals, eigvecs = np.linalg.eigh(gram3(centered))
        if float(eigvals[1]) <= 1e-12:
            continue
        if float(eigvals[0] / eigvals[1]) > _VOXEL_EIGEN_RATIO:
            continue
        normal = _orient_wall(np.asarray(eigvecs[:, 0], dtype=np.float64))
        if normal is None:
            continue
        normals[i] = normal
        planar[i] = True
    return normals, planar


def _orient_wall(normal):
    """Unit normal with its largest component positive, or None if it is not a wall."""
    norm = float(np.linalg.norm(normal))
    if norm < 1e-12 or not np.isfinite(norm):
        return None
    normal = np.asarray(normal, dtype=np.float64) / norm
    if abs(float(normal[2])) > _MAX_ABS_NZ:
        return None
    axis = int(np.argmax(np.abs(normal)))
    if normal[axis] < 0.0:
        normal = -normal
    return normal


def _grow(keys, planar, normals):
    index = {key: i for i, key in enumerate(keys)}
    offsets = tuple(
        (dx, dy, dz)
        for dx in (-1, 0, 1)
        for dy in (-1, 0, 1)
        for dz in (-1, 0, 1)
        if not (dx == 0 and dy == 0 and dz == 0)
    )
    cos_tol = float(np.cos(np.deg2rad(_GROW_DEG)))
    visited = np.zeros(len(keys), dtype=bool)
    regions = []
    for seed in range(len(keys)):
        if visited[seed] or not planar[seed]:
            continue
        visited[seed] = True
        stack = [seed]
        group = []
        seed_normal = normals[seed]
        while stack:
            current = stack.pop()
            group.append(current)
            key = keys[current]
            for dx, dy, dz in offsets:
                other = index.get((key[0] + dx, key[1] + dy, key[2] + dz))
                if other is None or visited[other] or not planar[other]:
                    continue
                if abs(float(np.dot(normals[other], seed_normal))) < cos_tol:
                    continue
                visited[other] = True
                stack.append(other)
        if len(group) >= _MIN_REGION_VOXELS:
            regions.append(np.asarray(group, dtype=np.int64))
    return regions


def _planes(cents, regions):
    planes = []
    for indices in regions:
        fitted = _fit_cloud(cents[indices])
        if fitted is None:
            continue
        centroid, normal = fitted
        planes.append({"indices": indices, "centroid": centroid, "normal": normal})
    return planes


def _fit_cloud(points):
    centroid = points.mean(axis=0)
    centered = points - centroid
    eigvals, eigvecs = np.linalg.eigh(gram3(centered))
    if float(eigvals[1]) < 1e-10:
        return None
    if float(eigvals[0] / eigvals[1]) > 0.5:
        return None
    normal = _orient_wall(np.asarray(eigvecs[:, 0], dtype=np.float64))
    if normal is None:
        return None
    return centroid, normal


def _merge_planes(planes, cents):
    """Join pieces of the same wall, including pieces a gap split apart.

    Opposite walls stay apart: their centroids are metres apart along the normal.
    """
    n_planes = len(planes)
    if n_planes == 0:
        return []
    parent = list(range(n_planes))

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    cos_tol = float(np.cos(np.deg2rad(_MERGE_DEG)))
    for i in range(n_planes):
        for j in range(i + 1, n_planes):
            if abs(float(np.dot(planes[i]["normal"], planes[j]["normal"]))) < cos_tol:
                continue
            delta = planes[j]["centroid"] - planes[i]["centroid"]
            across = abs(float(np.dot(delta, planes[i]["normal"])))
            if across > _MERGE_ACROSS_M:
                continue
            ri, rj = find(i), find(j)
            if ri != rj:
                parent[rj] = ri
    groups = {}
    for i in range(n_planes):
        groups.setdefault(find(i), []).append(i)
    merged = []
    for ids in groups.values():
        indices = np.concatenate([planes[i]["indices"] for i in ids])
        fitted = _fit_cloud(cents[indices])
        if fitted is None:
            continue
        centroid, normal = fitted
        merged.append({"indices": indices, "centroid": centroid, "normal": normal})
    return merged


def _place_window(plane, cents):
    """One 1 m by 1 m window on the densest part of the wall."""
    pts = cents[plane["indices"]]
    normal = plane["normal"]
    centroid = pts.mean(axis=0)
    along_axis, _across_axis, vertical_axis = box_axes(normal)
    delta = pts - centroid
    along = project(delta, along_axis)
    vertical = project(delta, vertical_axis)
    across = project(delta, normal)
    placed = _densest_window(along, vertical)
    if placed is None:
        return None
    along_c, vert_c = placed
    selected = (np.abs(along - along_c) <= HALF_EXTENT_M[0]) & (
        np.abs(vertical - vert_c) <= HALF_EXTENT_M[2]
    )
    if int(np.count_nonzero(selected)) < 10:
        return None
    across_c = float(np.median(across[selected]))
    center = (
        centroid
        + along_c * along_axis
        + vert_c * vertical_axis
        + across_c * normal
    )
    return {
        "center": np.asarray(center, dtype=np.float64),
        "normal": np.asarray(normal, dtype=np.float64),
        "half": np.asarray(HALF_EXTENT_M, dtype=np.float64),
    }


def _densest_window(along, vertical, step=0.10):
    """Centre of the fullest 1.0 m by 1.0 m bin block, or None if the wall is small."""
    a0 = float(along.min())
    a1 = float(along.max())
    v0 = float(vertical.min())
    v1 = float(vertical.max())
    if (a1 - a0) < 0.90 or (v1 - v0) < 0.90:
        return None
    a_bins = int(np.floor((a1 - a0) / step)) + 1
    v_bins = int(np.floor((v1 - v0) / step)) + 1
    a_index = np.clip(np.floor((along - a0) / step).astype(np.int32), 0, a_bins - 1)
    v_index = np.clip(np.floor((vertical - v0) / step).astype(np.int32), 0, v_bins - 1)
    grid = np.zeros((v_bins, a_bins), dtype=np.int32)
    np.add.at(grid, (v_index, a_index), 1)
    width_a = max(1, int(round((2.0 * HALF_EXTENT_M[0]) / step)))
    width_v = max(1, int(round((2.0 * HALF_EXTENT_M[2]) / step)))
    if a_bins < width_a or v_bins < width_v:
        return 0.5 * (a0 + a1), 0.5 * (v0 + v1)
    integral = np.pad(grid, ((1, 0), (1, 0))).cumsum(axis=0).cumsum(axis=1)
    best = -1
    best_rc = (0, 0)
    for row in range(0, v_bins - width_v + 1):
        for col in range(0, a_bins - width_a + 1):
            total = int(
                integral[row + width_v, col + width_a]
                - integral[row, col + width_a]
                - integral[row + width_v, col]
                + integral[row, col]
            )
            if total > best:
                best = total
                best_rc = (row, col)
    row, col = best_rc
    along_c = a0 + (col + 0.5 * width_a) * step
    vert_c = v0 + (row + 0.5 * width_v) * step
    return along_c, vert_c


def _score(map_path, candidates):
    """Count raw map points inside each window and the rms to its plane."""
    n_cand = len(candidates)
    counts = np.zeros(n_cand, dtype=np.int64)
    sum_sq = np.zeros(n_cand, dtype=np.float64)
    for pts in iter_xyz(map_path):
        finite = np.isfinite(pts).all(axis=1)
        good = pts[finite]
        if good.shape[0] == 0:
            continue
        for i, cand in enumerate(candidates):
            mask = crop_mask(good, cand["center"], cand["normal"], cand["half"])
            if not np.any(mask):
                continue
            hit = good[mask]
            counts[i] += int(hit.shape[0])
            signed = project(hit - cand["center"], cand["normal"])
            sum_sq[i] += float(np.dot(signed, signed))
    scored = []
    for i, cand in enumerate(candidates):
        count = int(counts[i])
        if count < _MIN_WINDOW_POINTS:
            continue
        rms = float(np.sqrt(sum_sq[i] / float(count)))
        scored.append(
            {
                "center": cand["center"],
                "normal": cand["normal"],
                "half": cand["half"],
                "n_points": count,
                "rms": rms,
                "rank_score": count / (1.0 + rms / 0.01),
            }
        )
    return scored


def _suppress(scored):
    """Drop a lower-ranked window that sits on top of one already kept."""
    ordered = sorted(scored, key=lambda item: (-item["rank_score"], -item["n_points"], item["rms"]))
    kept = []
    cos_tol = float(np.cos(np.deg2rad(_MERGE_DEG)))
    for cand in ordered:
        if any(_overlaps(cand, other, cos_tol) for other in kept):
            continue
        kept.append(cand)
    return kept


def _overlaps(cand, other, cos_tol):
    if abs(float(np.dot(cand["normal"], other["normal"]))) < cos_tol:
        return False
    delta = cand["center"] - other["center"]
    across = abs(float(np.dot(delta, other["normal"])))
    in_plane = delta - float(np.dot(delta, other["normal"])) * other["normal"]
    return across < _OVERLAP_ACROSS_M and float(np.linalg.norm(in_plane)) < _OVERLAP_IN_PLANE_M


def _document(map_path, sha, n_points, voxel_m, chosen):
    patches = []
    for rank, cand in enumerate(chosen, start=1):
        patches.append(
            {
                "name": "suggest-%d" % rank,
                "origin": "auto_suggested",
                "box_map_frame": {
                    "center_m": [float(v) for v in cand["center"]],
                    "normal_hint": [float(v) for v in cand["normal"]],
                    "half_extent_m": [float(v) for v in cand["half"]],
                },
                "rank": rank,
                "n_points": int(cand["n_points"]),
                "planarity_rms_m": float(cand["rms"]),
                "rank_score": float(cand["rank_score"]),
            }
        )
    return {
        "schema": PATCH_SCHEMA,
        "map_sha256": sha,
        "map_path": map_path,
        "notes": (
            "Auto-suggested patches, not Trevor's preselected patches. "
            "A box is valid only in this map frame. The search ignores points "
            "outside the per-axis 1st to 99th percentile; each box is scored "
            "on the full map."
        ),
        "suggest": {
            "voxel_m": float(voxel_m),
            "half_extent_m": [float(v) for v in HALF_EXTENT_M],
            "ranked_by": "n_points / (1 + planarity_rms_m/0.01), after dropping overlapping boxes",
            "n_points_in_map": int(n_points),
        },
        "patches": patches,
    }
