"""Occupancy diff between two clouds, the Python port of ``ChangeDetection.swift`` (v2).

``docs/CHANGE-DETECTION.md`` is the contract. Every value it calls binary32 is a
``numpy.float32`` here, so ``to_json`` reproduces the Swift bytes on the same points and
``Config``. Occupied sets are sorted unique ``int64`` key arrays, so no hash order reaches
the output.
"""

from collections import deque
from dataclasses import dataclass, field
from typing import Optional, Tuple

import numpy as np

F32 = np.float32
LIMIT = 1 << 20
_LANE = (1 << 21) - 1
_LIMIT_F32 = F32(LIMIT)
_ONE_F32 = F32(1)
_INF_F32 = F32(np.inf)
# A cast_rays exit cell no index reaches: an axis that does not step never leaves the box.
_NO_EXIT = np.iinfo(np.int64).min
_SIGN = np.uint32(0x80000000)

KIND_RANK = {"moved": 0, "appeared": 1, "disappeared": 2, "unobserved": 3}


class ChangeDetectionError(ValueError):
    """Base class for rejected inputs."""


class InvalidConfig(ChangeDetectionError):
    """A config field is out of range, or evidence is on without ray endpoints."""


class VoxelSizeMismatch(ChangeDetectionError):
    """The two maps, or a map and ``config.voxel_size``, differ in edge length (meters)."""


@dataclass(frozen=True)
class Config:
    """Knobs for occupancy, dilation, clustering, move pairing, and free-space evidence.

    Distances are meters and are rounded to binary32 on use. ``tolerance``,
    ``min_voxels``, and ``min_extent`` are voxel counts. Defaults match the Swift ``Config()``.
    """

    voxel_size: float = 0.05
    max_range: Optional[float] = None
    min_points: int = 1
    tolerance: int = 1
    min_voxels: int = 6
    min_extent: int = 2
    pair_moves: bool = True
    max_move: float = 1.5
    move_count_ratio: float = 2.0
    evidence_of_absence: bool = False
    origin: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    ray_stride: int = 1
    report_unobserved: bool = False

    @property
    def voxel_size_f32(self):
        """Voxel edge as binary32, in meters."""
        return F32(self.voxel_size)

    @property
    def inverse(self):
        """``f32(1) / voxel_size``: one binary32 division, in voxels per meter."""
        return _ONE_F32 / self.voxel_size_f32

    @property
    def origin_f32(self):
        """Sensor origin of the after frame as three binary32 values, in meters."""
        return np.array(self.origin, dtype=np.float32)


@dataclass
class OccupancyMap:
    """Occupied voxels of one frame. The original cloud is not kept.

    ``occupied`` is a sorted unique ``int64`` array of packed keys. ``endpoints`` is
    ``float32`` ``(len(occupied), 3)``: the representative return of each occupied voxel,
    present only when the map was built with evidence on.
    """

    voxel_size: np.float32
    occupied: np.ndarray
    accepted_points: int
    dropped_points: int
    endpoints: Optional[np.ndarray] = None


@dataclass
class Region:
    """One changed object after clustering and optional move pairing.

    Geometry fields are binary32 meters: centroid is the mean cell center, ``bounds_min``
    the inclusive low corner, ``bounds_max`` the exclusive high corner, ``volume`` the
    occupied-cell volume in cubic meters. For ``moved`` they describe the appeared side and
    the ``from_*`` fields the disappeared side. ``keys`` (and ``from_keys``) hold the
    component's packed voxel keys, sorted; they are not part of the JSON record.
    """

    kind: str
    voxel_count: int
    centroid: np.ndarray
    bounds_min: np.ndarray
    bounds_max: np.ndarray
    volume: np.float32
    min_key: int
    from_voxel_count: Optional[int] = None
    from_min_key: Optional[int] = None
    from_centroid: Optional[np.ndarray] = None
    from_bounds_min: Optional[np.ndarray] = None
    from_bounds_max: Optional[np.ndarray] = None
    keys: np.ndarray = field(default=None, repr=False, compare=False)
    from_keys: Optional[np.ndarray] = field(default=None, repr=False, compare=False)


@dataclass
class Report:
    """Diff of two occupancy maps. Voxel counts are after dilation and before the size cuts."""

    regions: list
    appeared_voxels: int
    disappeared_voxels: int
    unknown_voxels: int
    before_voxels: int
    after_voxels: int


# MARK: Keys


def pack(i, j, k):
    """Packs index arrays (or ints) into keys. Indices must lie in ``-LIMIT .. LIMIT-1``."""
    i = np.asarray(i, dtype=np.int64)
    j = np.asarray(j, dtype=np.int64)
    k = np.asarray(k, dtype=np.int64)
    return ((i + LIMIT) << 42) | ((j + LIMIT) << 21) | (k + LIMIT)


def unpack(keys):
    """Unpacks keys into three ``int64`` index arrays."""
    keys = np.asarray(keys, dtype=np.int64)
    return (keys >> 42) - LIMIT, ((keys >> 21) & _LANE) - LIMIT, (keys & _LANE) - LIMIT


def in_range(*indices):
    """True where every index lies in ``-LIMIT .. LIMIT-1``, so the triple packs."""
    ok = None
    for index in indices:
        inside = (index >= -LIMIT) & (index < LIMIT)
        ok = inside if ok is None else ok & inside
    return ok


def floor_index(q):
    """Spec quantizer on binary32 ``q`` already inside the ``|q| < 2^20`` guard."""
    truncated = np.trunc(q)
    index = truncated.astype(np.int64)
    return index - ((q < 0) & (truncated != q))


def contains(sorted_keys, keys):
    """Boolean membership of ``keys`` in the sorted unique array ``sorted_keys``."""
    if len(sorted_keys) == 0:
        return np.zeros(np.shape(keys), dtype=bool)
    slot = np.searchsorted(sorted_keys, keys)
    np.minimum(slot, len(sorted_keys) - 1, out=slot)
    return sorted_keys[slot] == keys


def splitmix64(keys):
    """SplitMix64 finalizer on packed keys, mod 2^64."""
    with np.errstate(over="ignore"):
        z = np.asarray(keys, dtype=np.int64).view(np.uint64) + np.uint64(0x9E3779B97F4A7C15)
        z = (z ^ (z >> np.uint64(30))) * np.uint64(0xBF58476D1CE4E5B9)
        z = (z ^ (z >> np.uint64(27))) * np.uint64(0x94D049BB133111EB)
        return z ^ (z >> np.uint64(31))


def _order_key(values):
    bits = np.ascontiguousarray(values, dtype=np.float32).view(np.uint32)
    return np.where(bits & _SIGN != 0, ~bits, bits | _SIGN)


def _from_order_key(keys):
    return np.where(keys & _SIGN != 0, keys & ~_SIGN, ~keys).astype(np.uint32).view(np.float32)


def _representatives(keys, accepted):
    """Unique keys, their point counts, and the minimum point of each voxel under the order key.

    The minimum is lexicographic on (x, y, z): the smallest x order key, then the smallest y
    among the points that share it, then the smallest z. Order keys invert exactly, so the
    returned coordinates are the original binary32 values.
    """
    order = np.argsort(keys)
    sorted_keys = keys[order]
    first = np.ones(len(sorted_keys), dtype=bool)
    first[1:] = sorted_keys[1:] != sorted_keys[:-1]
    starts = np.flatnonzero(first)
    group = np.cumsum(first) - 1
    counts = np.diff(np.append(starts, len(sorted_keys)))
    best = []
    tied = np.ones(len(sorted_keys), dtype=bool)
    for axis in range(3):
        values = _order_key(accepted[order, axis])
        smallest = np.minimum.reduceat(np.where(tied, values, np.uint32(0xFFFFFFFF)), starts)
        tied &= values == smallest[group]
        best.append(_from_order_key(smallest))
    return sorted_keys[starts], counts, np.stack(best, axis=1)


# MARK: Occupancy


def validate(config):
    """Raises ``InvalidConfig`` when ``config`` is out of range (same checks as Swift)."""
    voxel = config.voxel_size_f32
    if not np.isfinite(voxel) or voxel <= 0:
        raise InvalidConfig("voxelSize must be finite and > 0 m")
    if config.max_range is not None:
        max_range = F32(config.max_range)
        if not np.isfinite(max_range) or max_range <= 0:
            raise InvalidConfig("maxRange must be finite and > 0 m")
    if config.min_points < 1:
        raise InvalidConfig("minPoints must be >= 1")
    if config.min_voxels < 1:
        raise InvalidConfig("minVoxels must be >= 1")
    if config.min_extent < 1:
        raise InvalidConfig("minExtent must be >= 1 voxel")
    if not 0 <= config.tolerance <= 4:
        raise InvalidConfig("tolerance must be in 0...4 voxels")
    max_move = F32(config.max_move)
    if not np.isfinite(max_move) or max_move < 0:
        raise InvalidConfig("maxMove must be finite and >= 0 m")
    ratio = F32(config.move_count_ratio)
    if not np.isfinite(ratio) or ratio < 1:
        raise InvalidConfig("moveCountRatio must be finite and >= 1")
    if config.ray_stride not in (1, 2, 4, 8, 16, 32, 64):
        raise InvalidConfig("rayStride must be a power of two from 1 through 64")
    origin = config.origin_f32
    if not np.isfinite(origin).all():
        raise InvalidConfig("origin must be finite")
    if config.evidence_of_absence:
        q = origin * config.inverse
        if not np.isfinite(q).all() or (np.abs(q) >= _LIMIT_F32).any():
            raise InvalidConfig("origin is outside the voxel grid")


def occupancy_map(positions, config):
    """Builds an occupancy map from ``(n, 3)`` positions in meters.

    A point is dropped when a coordinate is non-finite, when its binary32 squared range
    ``(x*x + y*y) + z*z`` exceeds ``maxRange²``, or when its quantized index leaves the key
    range. With evidence on the map keeps one representative return per occupied voxel.

    Raises:
        InvalidConfig: When ``config`` is out of range.
    """
    validate(config)
    return _occupy(np.asarray(positions, dtype=np.float32).reshape(-1, 3), config, config.inverse)


def _occupy(points, config, inverse):
    count = len(points)
    with np.errstate(over="ignore", invalid="ignore"):
        keep = np.isfinite(points).all(axis=1)
        if config.max_range is not None:
            max_range = F32(config.max_range)
            x, y, z = points[:, 0], points[:, 1], points[:, 2]
            keep &= ~(((x * x + y * y) + z * z) > max_range * max_range)
        q = points * inverse
        keep &= (np.abs(q) < _LIMIT_F32).all(axis=1)
    accepted = points[keep]
    q = q[keep]
    keys = pack(floor_index(q[:, 0]), floor_index(q[:, 1]), floor_index(q[:, 2]))
    endpoints = None
    if config.evidence_of_absence:
        unique, counts, representative = _representatives(keys, accepted)
        dense = counts >= config.min_points
        occupied = unique[dense]
        endpoints = representative[dense]
    elif config.min_points == 1:
        occupied = np.unique(keys)
    else:
        unique, counts = np.unique(keys, return_counts=True)
        occupied = unique[counts >= config.min_points]
    return OccupancyMap(config.voxel_size_f32, occupied, int(len(accepted)), int(count - len(accepted)), endpoints)


# MARK: Diff


def compare(before, after, config):
    """Builds both maps with ``config``, then diffs them. Positions are ``(n, 3)`` meters."""
    validate(config)
    inverse = config.inverse
    before_map = _occupy(np.asarray(before, dtype=np.float32).reshape(-1, 3), config, inverse)
    after_map = _occupy(np.asarray(after, dtype=np.float32).reshape(-1, 3), config, inverse)
    return before_map, after_map, diff(before_map, after_map, config)


def diff(before, after, config, before_known=None):
    """Diffs two maps built with the same voxel size.

    A voxel only in one map is unchanged when the other occupies a voxel within Chebyshev
    ``tolerance``. With evidence on, a before-only voxel is a disappeared candidate only
    when an after-ray traverses it, and is counted unknown otherwise. Components under
    ``min_voxels`` voxels, or under ``min_extent`` cells on any axis of their box, drop. Mutually nearest appeared/disappeared clusters of similar size
    pair as ``moved``; equal binary32 distances do not pair.

    ``before_known`` is an extension for the room-memory engine, outside the v2 contract:
    the sorted keys the before side observed (occupied or free). When given, an after-only
    voxel is an appeared candidate only if the before side observed it, so new coverage of
    never-seen space is not a change. ``None`` is the v2 behavior the Swift goldens pin.

    Raises:
        InvalidConfig, VoxelSizeMismatch
    """
    validate(config)
    if before.voxel_size != after.voxel_size or before.voxel_size != config.voxel_size_f32:
        other = config.voxel_size_f32 if before.voxel_size == after.voxel_size else after.voxel_size
        raise VoxelSizeMismatch(f"voxel size mismatch {before.voxel_size} m vs {other} m")
    if config.evidence_of_absence and (after.endpoints is None or len(after.endpoints) != len(after.occupied)):
        raise InvalidConfig("evidence requires the after map to be built with evidenceOfAbsence enabled")

    only_after = np.setdiff1d(after.occupied, before.occupied, assume_unique=True)
    if before_known is not None:
        only_after = only_after[contains(before_known, only_after)]
    only_before = np.setdiff1d(before.occupied, after.occupied, assume_unique=True)
    appeared = _surviving(only_after, before.occupied, config.tolerance)
    if not config.evidence_of_absence:
        free = only_before
        unknown = only_before[:0]
    elif len(only_before) == 0:
        free = only_before
        unknown = only_before
    else:
        free = cast_rays(after, config, keep=only_before)
        unknown = np.setdiff1d(only_before, free, assume_unique=True)
    disappeared = _surviving(free, after.occupied, config.tolerance)

    appeared_clusters = clusters(appeared, config)
    disappeared_clusters = clusters(disappeared, config)
    unobserved = clusters(unknown, config) if config.evidence_of_absence and config.report_unobserved else []
    regions = _assemble(appeared_clusters, disappeared_clusters, unobserved, config)
    return Report(
        regions=regions,
        appeared_voxels=int(len(appeared)),
        disappeared_voxels=int(len(disappeared)),
        unknown_voxels=int(len(unknown)),
        before_voxels=int(len(before.occupied)),
        after_voxels=int(len(after.occupied)),
    )


def _offsets(radius):
    span = range(-radius, radius + 1)
    return [(di, dj, dk) for di in span for dj in span for dk in span if (di, dj, dk) != (0, 0, 0)]


def _surviving(candidates, other, tolerance):
    if tolerance == 0 or len(candidates) == 0 or len(other) == 0:
        return candidates
    i, j, k = unpack(candidates)
    suppressed = np.zeros(len(candidates), dtype=bool)
    member = _Membership(other)
    for di, dj, dk in _offsets(tolerance):
        suppressed |= member.contains((i + di, j + dj, k + dk))
    return candidates[~suppressed]


# MARK: Evidence


def ray_endpoints(after, config):
    """Kept rays: ``(keys, points)`` of occupied after voxels passing the stride filter."""
    keys = after.occupied
    points = after.endpoints
    if config.ray_stride > 1:
        kept = (splitmix64(keys) & np.uint64(config.ray_stride - 1)) == 0
        keys = keys[kept]
        points = points[kept]
    return keys, points


class _Membership:
    """Membership of index triples in a sorted key set, restricted to the set's index box.

    A dense boolean grid over the box when it holds at most ``DENSE_CELLS`` cells, else a
    binary search of the packed keys. Both answer the same question exactly.
    """

    DENSE_CELLS = 1 << 25

    def __init__(self, keys):
        self.keys = keys
        indices = unpack(keys)
        self.box = [(int(axis.min()), int(axis.max())) for axis in indices]
        self.shape = [high - low + 1 for low, high in self.box]
        self.grid = None
        if self.shape[0] * self.shape[1] * self.shape[2] <= self.DENSE_CELLS:
            self.grid = np.zeros(self.shape[0] * self.shape[1] * self.shape[2], dtype=bool)
            self.grid[self._linear(indices)] = True

    def _linear(self, indices):
        (i0, _), (j0, _), (k0, _) = self.box
        return ((indices[0] - i0) * self.shape[1] + (indices[1] - j0)) * self.shape[2] + (indices[2] - k0)

    def contains(self, current):
        """True where the triple in ``current`` belongs to the set (a triple outside the set's box,
        one that does not pack among them, never does)."""
        inside = np.ones(len(current[0]), dtype=bool)
        for (low, high), position in zip(self.box, current):
            inside &= (position >= low) & (position <= high)
        indices = [position[inside] for position in current]
        found = np.zeros(len(inside), dtype=bool)
        if self.grid is not None:
            found[inside] = self.grid[self._linear(indices)]
        else:
            found[inside] = contains(self.keys, pack(*indices))
        return found

    def marked(self, current):
        """Packed keys of the triples in ``current`` that belong to the set."""
        inside = np.ones(len(current[0]), dtype=bool)
        for (low, high), position in zip(self.box, current):
            inside &= (position >= low) & (position <= high)
        indices = [position[inside] for position in current]
        if self.grid is not None:
            hit = self.grid[self._linear(indices)]
            return pack(*(axis[hit] for axis in indices))
        visited = pack(*indices)
        return visited[contains(self.keys, visited)]


def cast_rays(after, config, keep=None):
    """Walks the spec's 3D DDA from ``config.origin`` to every kept ray endpoint.

    Every cell a ray marks is free evidence; the endpoint voxel is never marked by its own
    ray. On a ``tMax`` tie every tied axis steps, except an axis already on its endpoint
    index: that axis is clamped (``tMax`` becomes +inf) so a tie cannot drag it past the end
    cell. Each axis therefore stays between its start and end index, inside the origin–end
    box where every triple packs, which is why the walk needs neither the spec's box skip
    nor its pack-failure exit. With ``keep`` (sorted keys) only marks inside it are
    returned, otherwise all marks. Occupied after voxels are removed in both cases:
    occupied wins over free.

    With ``keep``, a ray stops being walked once it has left the index box of ``keep`` on
    some axis in its step direction (or sits outside it on an axis that does not step).
    Every axis moves monotonically toward the end voxel, so such a ray can mark no key of
    ``keep`` again, and dropping it leaves the result unchanged.

    Returns:
        Sorted unique ``int64`` keys.
    """
    if keep is not None and len(keep) == 0:
        return keep
    end_keys, points = ray_endpoints(after, config)
    inverse = config.inverse
    q_origin = config.origin_f32 * inverse
    start = floor_index(q_origin)
    end_i, end_j, end_k = unpack(end_keys)
    with np.errstate(divide="ignore", invalid="ignore", over="ignore"):
        direction = points * inverse - q_origin
        lanes = []
        for axis, end in enumerate((end_i, end_j, end_k)):
            begin = int(start[axis])
            step = np.sign(end - begin).astype(np.int64)
            d = direction[:, axis]
            inf = np.full(len(d), np.inf, dtype=np.float32)
            t_delta = np.where(step == 0, inf, _ONE_F32 / np.abs(d))
            plane = np.where(step > 0, F32(begin + 1), F32(begin)) - q_origin[axis]
            t_max = np.where(step == 0, inf, plane / d)
            t_max = np.where(t_max < 0, F32(0), t_max)
            lanes.append((end, step, t_delta, t_max))
    ends = [lane[0] for lane in lanes]
    cap = sum(np.abs(end - int(start[axis])) for axis, end in enumerate(ends))
    alive = cap > 0
    for _, _, t_delta, t_max in lanes:
        alive &= ~np.isnan(t_delta) & ~np.isnan(t_max)

    rows = np.nonzero(alive)[0]
    cap = cap[rows]
    ends = [end[rows] for end in ends]
    steps = [lane[1][rows] for lane in lanes]
    deltas = [lane[2][rows] for lane in lanes]
    t_maxes = [lane[3][rows].copy() for lane in lanes]
    current = [np.full(len(rows), int(start[axis]), dtype=np.int64) for axis in range(3)]

    if keep is not None:
        member = _Membership(keep)
        box = member.box
    # Without keep the marks are every cell walked, often millions: when the start-to-ends box (where
    # every walked cell lies) is small enough they go into a dense grid over it, whose cell order is
    # key order, instead of being sorted for np.unique.
    seen = None
    if keep is None and len(rows):
        corner = [min(int(start[axis]), int(end.min())) for axis, end in enumerate(ends)]
        shape = [max(int(start[axis]), int(end.max())) - corner[axis] + 1 for axis, end in enumerate(ends)]
        if shape[0] * shape[1] * shape[2] <= _Membership.DENSE_CELLS:
            seen = np.zeros(shape, dtype=bool)
    if keep is not None:
        # Past the box of keep on an axis in its step direction (or outside it on an axis that does
        # not step) a ray is done. Moving one cell at a time, a stepping axis gets past the box on its
        # exit cell, one beyond the box; a ray that starts past it is dropped before it walks.
        outside = np.zeros(len(cap), dtype=bool)
        exits = []
        for (low, high), position, step in zip(box, current, steps):
            outside |= ((step >= 0) & (position > high)) | ((step <= 0) & (position < low))
            exits.append(np.where(step > 0, high + 1, np.where(step < 0, low - 1, _NO_EXIT)))
        if outside.any():
            cap, ends, steps, deltas, t_maxes, current, exits = (
                cap[~outside], [end[~outside] for end in ends], [step[~outside] for step in steps],
                [delta[~outside] for delta in deltas], [t_max[~outside] for t_max in t_maxes],
                [position[~outside] for position in current], [edge[~outside] for edge in exits])
    marks = []
    iteration = 0
    while len(cap):
        # Every axis stays between its start and end index, so it is short of its end exactly where it
        # differs from it (an axis that does not step starts on its end).
        short = [position != end for position, end in zip(current, ends)]
        live = (iteration < cap) & (short[0] | short[1] | short[2])
        if keep is not None:
            live &= (current[0] != exits[0]) & (current[1] != exits[1]) & (current[2] != exits[2])
        if not live.all():
            cap = cap[live]
            ends = [end[live] for end in ends]
            steps = [step[live] for step in steps]
            deltas = [delta[live] for delta in deltas]
            t_maxes = [t_max[live] for t_max in t_maxes]
            current = [position[live] for position in current]
            short = [ahead[live] for ahead in short]
            if keep is not None:
                exits = [edge[live] for edge in exits]
            if not len(cap):
                break
        if seen is not None:
            seen[current[0] - corner[0], current[1] - corner[1], current[2] - corner[2]] = True
        elif keep is None:
            marks.append(pack(*current))
        else:
            marks.append(member.marked(current))
        for axis in range(3):
            t_maxes[axis] = np.where(short[axis], t_maxes[axis], _INF_F32)
        t_next = t_maxes[0]
        t_next = np.where(t_maxes[1] < t_next, t_maxes[1], t_next)
        t_next = np.where(t_maxes[2] < t_next, t_maxes[2], t_next)
        for axis in range(3):
            moves = (t_maxes[axis] == t_next) & short[axis]
            current[axis] = current[axis] + np.where(moves, steps[axis], 0)
            t_maxes[axis] = np.where(moves, t_maxes[axis] + deltas[axis], t_maxes[axis])
        iteration += 1

    if seen is not None:
        i, j, k = np.nonzero(seen)
        marked = pack(i + corner[0], j + corner[1], k + corner[2])
    elif not marks:
        return np.zeros(0, dtype=np.int64)
    else:
        marked = np.unique(np.concatenate(marks))
    return np.setdiff1d(marked, after.occupied, assume_unique=True)


# MARK: Clusters


@dataclass
class _Cluster:
    voxel_count: int
    centroid: np.ndarray
    bounds_min: np.ndarray
    bounds_max: np.ndarray
    volume: np.float32
    min_key: int
    keys: np.ndarray


_NEIGHBOR_DELTAS = [(di << 42) + (dj << 21) + dk for di, dj, dk in _offsets(1)]
_NEIGHBOR_STEPS = _offsets(1)


def _interior(key):
    return all(0 < lane < _LANE for lane in (key >> 42, (key >> 21) & _LANE, key & _LANE))


def _neighbors(key):
    if _interior(key):
        return [key + delta for delta in _NEIGHBOR_DELTAS]
    i, j, k = (key >> 42) - LIMIT, ((key >> 21) & _LANE) - LIMIT, (key & _LANE) - LIMIT
    found = []
    for di, dj, dk in _NEIGHBOR_STEPS:
        ni, nj, nk = i + di, j + dj, k + dk
        if -LIMIT <= ni < LIMIT and -LIMIT <= nj < LIMIT and -LIMIT <= nk < LIMIT:
            found.append(((ni + LIMIT) << 42) | ((nj + LIMIT) << 21) | (nk + LIMIT))
    return found


def clusters(keys, config):
    """26-connected components of ``keys`` with at least ``min_voxels`` voxels whose index box
    spans at least ``min_extent`` cells on every axis.

    Geometry comes from exact index sums, so the result does not depend on visit order.
    Components are returned in ascending min-key order.
    """
    if len(keys) == 0:
        return []
    remaining = set(keys.tolist())
    voxel = config.voxel_size_f32
    size = float(voxel)
    found = []
    for seed in keys.tolist():
        if seed not in remaining:
            continue
        remaining.discard(seed)
        members = [seed]
        queue = deque([seed])
        while queue:
            for neighbor in _neighbors(queue.popleft()):
                if neighbor in remaining:
                    remaining.discard(neighbor)
                    members.append(neighbor)
                    queue.append(neighbor)
        if len(members) < config.min_voxels:
            continue
        member_keys = np.sort(np.array(members, dtype=np.int64))
        i, j, k = unpack(member_keys)
        low = np.array([i.min(), j.min(), k.min()])
        high = np.array([i.max(), j.max(), k.max()]) + 1
        if (high - low < config.min_extent).any():
            continue
        n = float(len(members))
        centroid = np.array([((float(int(axis.sum())) / n) + 0.5) * size for axis in (i, j, k)]).astype(np.float32)
        found.append(_Cluster(
            voxel_count=len(members),
            centroid=centroid,
            bounds_min=low.astype(np.float32) * voxel,
            bounds_max=high.astype(np.float32) * voxel,
            volume=((F32(len(members)) * voxel) * voxel) * voxel,
            min_key=int(member_keys[0]),
            keys=member_keys,
        ))
    return found


# MARK: Pairing and order


def _region(cluster, kind):
    return Region(
        kind=kind, voxel_count=cluster.voxel_count, centroid=cluster.centroid, bounds_min=cluster.bounds_min,
        bounds_max=cluster.bounds_max, volume=cluster.volume, min_key=cluster.min_key, keys=cluster.keys,
    )


def _moved(appeared, disappeared):
    region = _region(appeared, "moved")
    region.from_voxel_count = disappeared.voxel_count
    region.from_min_key = disappeared.min_key
    region.from_centroid = disappeared.centroid
    region.from_bounds_min = disappeared.bounds_min
    region.from_bounds_max = disappeared.bounds_max
    region.from_keys = disappeared.keys
    return region


def _distance(a, b):
    d = a - b
    return np.sqrt((d[0] * d[0] + d[1] * d[1]) + d[2] * d[2])


def _gated(appeared, disappeared, config):
    distance = _distance(appeared.centroid, disappeared.centroid)
    if not distance <= F32(config.max_move):
        return None
    hi = F32(max(appeared.voxel_count, disappeared.voxel_count))
    lo = F32(min(appeared.voxel_count, disappeared.voxel_count))
    if not hi / lo <= F32(config.move_count_ratio):
        return None
    return distance


def _unique_nearest(candidates):
    if not candidates:
        return None
    best = candidates[0]
    unique = True
    for candidate in candidates[1:]:
        if candidate[1] < best[1]:
            best = candidate
            unique = True
        elif candidate[1] == best[1]:
            unique = False
    return best if unique else None


def _ambiguous(candidates, partner, distance):
    limit = distance * F32(1.5)
    return any(index != partner and d < limit for index, d in candidates)


def _mutual_pairs(appeared, disappeared, config):
    by_appeared = [[] for _ in appeared]
    by_disappeared = [[] for _ in disappeared]
    for a, a_cluster in enumerate(appeared):
        for d, d_cluster in enumerate(disappeared):
            distance = _gated(a_cluster, d_cluster, config)
            if distance is not None:
                by_appeared[a].append((d, distance))
                by_disappeared[d].append((a, distance))
    pairs = []
    for a in range(len(appeared)):
        nearest = _unique_nearest(by_appeared[a])
        if nearest is None:
            continue
        back = _unique_nearest(by_disappeared[nearest[0]])
        if back is None or back[0] != a:
            continue
        if _ambiguous(by_appeared[a], nearest[0], nearest[1]):
            continue
        if _ambiguous(by_disappeared[nearest[0]], a, nearest[1]):
            continue
        pairs.append((a, nearest[0], nearest[1]))
    pairs.sort(key=lambda pair: (pair[2], appeared[pair[0]].min_key, disappeared[pair[1]].min_key))
    return pairs


def _assemble(appeared, disappeared, unobserved, config):
    regions = []
    used_appeared = set()
    used_disappeared = set()
    if config.pair_moves and appeared and disappeared:
        for a, d, _ in _mutual_pairs(appeared, disappeared, config):
            if a in used_appeared or d in used_disappeared:
                continue
            used_appeared.add(a)
            used_disappeared.add(d)
            regions.append(_moved(appeared[a], disappeared[d]))
    regions.extend(_region(c, "appeared") for index, c in enumerate(appeared) if index not in used_appeared)
    regions.extend(_region(c, "disappeared") for index, c in enumerate(disappeared) if index not in used_disappeared)
    regions.extend(_region(c, "unobserved") for c in unobserved)
    regions.sort(key=lambda r: (
        KIND_RANK[r.kind], -r.voxel_count, r.min_key, -1 if r.from_min_key is None else r.from_min_key,
    ))
    return regions


# MARK: JSON


def _bits(value):
    if value is None:
        return None
    return f"{int(np.asarray(value, dtype=np.float32).view(np.uint32)):08x}"


def _bits3(values):
    if values is None:
        return None
    return tuple(_bits(value) for value in np.asarray(values, dtype=np.float32))


def _json_point(values):
    return "[" + ",".join(f'"{bits}"' for bits in _bits3(values)) + "]"


def _json_map(occupancy):
    points = occupancy.accepted_points + occupancy.dropped_points
    return (
        f'{{"points":{points},"accepted":{occupancy.accepted_points},'
        f'"dropped":{occupancy.dropped_points},"voxels":{len(occupancy.occupied)}}}'
    )


def _json_region(region):
    text = (
        f'{{"kind":"{region.kind}","voxel_count":{region.voxel_count},"min_key":"{region.min_key}"'
        f',"centroid_bits":{_json_point(region.centroid)}'
        f',"bounds_min_bits":{_json_point(region.bounds_min)}'
        f',"bounds_max_bits":{_json_point(region.bounds_max)}'
        f',"volume_bits":"{_bits(region.volume)}"'
    )
    if region.kind == "moved":
        text += (
            f',"from_voxel_count":{region.from_voxel_count},"from_min_key":"{region.from_min_key}"'
            f',"from_centroid_bits":{_json_point(region.from_centroid)}'
            f',"from_bounds_min_bits":{_json_point(region.from_bounds_min)}'
            f',"from_bounds_max_bits":{_json_point(region.from_bounds_max)}'
        )
    return text + "}"


def _json_bool(value):
    return "true" if value else "false"


def to_json(report, before, after, config):
    """Canonical v2 record (schema ``labcommand.change-detection.v2``), no trailing newline.

    Hand-written so key order and number formatting match the Swift encoder byte for byte.
    """
    max_range = "null" if config.max_range is None else f'"{_bits(F32(config.max_range))}"'
    parts = [
        '{"schema":"labcommand.change-detection.v2"',
        f',"evidence":{_json_bool(config.evidence_of_absence)}',
        f',"report_unobserved":{_json_bool(config.report_unobserved)}',
        f',"origin_bits":{_json_point(config.origin_f32)}',
        f',"ray_stride":{config.ray_stride}',
        ',"config":{',
        f'"voxel_size_bits":"{_bits(config.voxel_size_f32)}"',
        f',"max_range_bits":{max_range}',
        f',"min_points":{config.min_points}',
        f',"tolerance":{config.tolerance}',
        f',"min_voxels":{config.min_voxels}',
        f',"min_extent":{config.min_extent}',
        f',"pair_moves":{_json_bool(config.pair_moves)}',
        f',"max_move_bits":"{_bits(F32(config.max_move))}"',
        f',"move_count_ratio_bits":"{_bits(F32(config.move_count_ratio))}"',
        "}",
        f',"before":{_json_map(before)}',
        f',"after":{_json_map(after)}',
        f',"appeared_voxels":{report.appeared_voxels}',
        f',"disappeared_voxels":{report.disappeared_voxels}',
        f',"unknown_voxels":{report.unknown_voxels}',
        ',"regions":[',
        ",".join(_json_region(region) for region in report.regions),
        "]}",
    ]
    return "".join(parts)
