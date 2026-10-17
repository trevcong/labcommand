"""The rig-pose guard's geometry: poses, the per-window point transform, the registration cloud,
and ICP against it: KISS-ICP-style point-to-point stages (Vizzo et al., RA-L 2023: voxel-downsampled
source, adaptive threshold, Geman-McClure kernel; weighted Horn steps) for the basin, then
point-to-plane stages against the cloud's normals for the final pose. No ROS imports.

Point-to-point alone is biased between two different samplings of the same surfaces: on the
synthetic room it settles 2-8 mm and 0.15-0.2 degrees off at zero noise, against 0.03 mm and 0.0003
degrees for the point-to-plane finish (tests/test_guard.py).

Conventions (both engines; PLAN-RIGGUARD.md is the design):

- A channel's pose ``T = (R, t)`` maps a point in the sensor's *current* frame into its reference
  frame: ``p_ref = R p + t``. Composition is ``(A ∘ B)(p) = A(B(p))``.
- A physical move of the rig is an increment ``D`` in the pre-move sensor frame, so the new pose is
  ``T ∘ D`` and the increment an estimate implies is ``T⁻¹ ∘ estimate``.
- Every estimate, seed and stored pose is absolute; increments are only derived from them.

The C++ twin is ``room_memory_cpp/src/guard.cpp``. It runs the same algorithms in the same order
(the same grids, tie rules, Jacobi sweeps and iteration limits), so the two engines reach the same
decisions; sums are not order-free, so their floats agree to rounding, not to the bit
(PLAN-RIGGUARD.md, Design 7: the v1 relaxation).
"""

import math
from dataclasses import dataclass, field
from typing import Optional, Tuple

import numpy as np

from .core.change_detection import contains, pack, unpack

F32 = np.float32

# Target grids: one per cell size, each cell keeping at most GRID_CAP points (every k-th of the cell,
# in target order). A correspondence search with threshold tau uses the finest grid whose cell is at
# least tau and looks at the 27 cells around the query.
GRID_LEVELS = (0.04, 0.08, 0.16, 0.32, 0.64)
GRID_CAP = 20
# The overlap metric: the share of source points with a target point within this distance.
INLIER_RADIUS_M = 0.10
MAX_STAGES = 5
MAX_ITERATIONS = 10
FINE_ITERATIONS = 10
# A point-to-plane step leaves out the directions whose eigenvalue is below this share of the
# largest one, so a degenerate view (a corridor) does not slide.
STEP_EIG_FLOOR = 1e-4
STEP_STOP_M = 1e-4
STEP_STOP_DEG = 1e-3
TAU_MIN_M = 0.04
TAU_MAX_M = 0.5
TAU_FIRST_MIN_M = 0.05
# Normals for the point-to-plane stages and the observability matrix: PCA over the cloud's points
# within NORMAL_RADIUS_M (each cell of the search grid sampled to at most GRID_CAP points), at least
# NORMAL_MIN_POINTS of them, planar when the smallest eigenvalue is at most NORMAL_PLANARITY of the
# middle one. 0.15 m spans two or more scan rows of a spinning lidar at room range; 2.5 registration
# voxels (5 cm) holds one row, a line, and leaves walls without normals.
NORMAL_RADIUS_M = 0.15
NORMAL_MIN_POINTS = 5
NORMAL_PLANARITY = 0.2
# The yaw sweep without a rotation prior: every whole degree in [-align_max_deg, align_max_deg].
SWEEP_STEP_DEG = 1.0
JACOBI_SWEEPS = 50
_OFFSETS = tuple((di, dj, dk) for di in (-1, 0, 1) for dj in (-1, 0, 1) for dk in (-1, 0, 1))


# MARK: Poses


def _matmul(a, b):
    """3 x 3 row-major tuples, each entry ((a0 b0 + a1 b1) + a2 b2)."""
    return tuple((a[3 * i] * b[j] + a[3 * i + 1] * b[3 + j]) + a[3 * i + 2] * b[6 + j]
                 for i in range(3) for j in range(3))


def _matvec(a, v):
    return tuple((a[3 * i] * v[0] + a[3 * i + 1] * v[1]) + a[3 * i + 2] * v[2] for i in range(3))


def _transpose(a):
    return (a[0], a[3], a[6], a[1], a[4], a[7], a[2], a[5], a[8])


_IDENTITY = (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)


@dataclass(frozen=True)
class Pose:
    """``p_ref = R p + t``: ``R`` row-major (9 floats), ``t`` (3 floats), binary64."""

    R: Tuple[float, ...] = _IDENTITY
    t: Tuple[float, float, float] = (0.0, 0.0, 0.0)

    @property
    def is_identity(self):
        """Exactly the identity (the transform pass is skipped, so bytes match an unguarded run)."""
        return self.R == _IDENTITY and self.t == (0.0, 0.0, 0.0)

    def compose(self, other):
        """``self ∘ other``."""
        Rt = _matvec(self.R, other.t)
        return Pose(_matmul(self.R, other.R), tuple(Rt[n] + self.t[n] for n in range(3)))

    def inverse(self):
        Rt = _transpose(self.R)
        t = _matvec(Rt, self.t)
        return Pose(Rt, (-t[0], -t[1], -t[2]))

    def increment(self, estimate):
        """``self⁻¹ ∘ estimate``: the move ``estimate`` implies from this pose, in this pose's frame."""
        return self.inverse().compose(estimate)

    @property
    def cos_angle(self):
        """``cos`` of the rotation angle, from the trace, clamped to [-1, 1]."""
        return max(-1.0, min(1.0, ((self.R[0] + self.R[4]) + self.R[8] - 1.0) / 2.0))

    @property
    def angle_deg(self):
        return math.degrees(math.acos(self.cos_angle))

    @property
    def distance_m(self):
        return math.sqrt((self.t[0] * self.t[0] + self.t[1] * self.t[1]) + self.t[2] * self.t[2])

    def apply(self, points):
        """Binary64 ``(n, 3)`` points through the pose, each coordinate ((r0 x + r1 y) + r2 z) + t."""
        x, y, z = points[:, 0], points[:, 1], points[:, 2]
        R, t = self.R, self.t
        return np.stack([((R[3 * i] * x + R[3 * i + 1] * y) + R[3 * i + 2] * z) + t[i] for i in range(3)], axis=1)

    def quaternion(self):
        """``(x, y, z, w)``, ``w >= 0``."""
        return rotation_quaternion(self.R)

    def rig_pose(self):
        """The ``RigPose`` pose field: ``tx ty tz qx qy qz qw``."""
        return (*self.t, *self.quaternion())

    def text(self):
        """Twelve ``%.17g`` numbers (R row-major, then t): what the rig state file stores."""
        return " ".join(f"{value:.17g}" for value in (*self.R, *self.t))

    @classmethod
    def from_values(cls, values):
        values = [float(value) for value in values]
        if len(values) != 12 or not all(math.isfinite(value) for value in values):
            raise ValueError("a pose is 12 finite numbers")
        return cls(tuple(values[:9]), tuple(values[9:]))


def rotation_about(axis, degrees):
    """Rotation by ``degrees`` about the unit ``axis`` (Rodrigues; scalar libm cos and sin)."""
    angle = math.radians(degrees)
    c, s = math.cos(angle), math.sin(angle)
    x, y, z = axis
    v = 1.0 - c
    return (c + x * x * v, x * y * v - z * s, x * z * v + y * s,
            y * x * v + z * s, c + y * y * v, y * z * v - x * s,
            z * x * v - y * s, z * y * v + x * s, c + z * z * v)


def rotation_vector(rx, ry, rz):
    """The rotation of the rotation vector ``(rx, ry, rz)`` (radians)."""
    angle = math.sqrt((rx * rx + ry * ry) + rz * rz)
    if angle == 0.0:
        return _IDENTITY
    return rotation_about((rx / angle, ry / angle, rz / angle), math.degrees(angle))


def quaternion_rotation(w, x, y, z):
    """The rotation of the unit quaternion ``(w, x, y, z)``."""
    return (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y),
            2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x),
            2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y))


def rotation_quaternion(R):
    """``(x, y, z, w)`` of a rotation matrix (Shepperd's method), ``w >= 0``, unit length."""
    trace = (R[0] + R[4]) + R[8]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w, x, y, z = 0.25 * s, (R[7] - R[5]) / s, (R[2] - R[6]) / s, (R[3] - R[1]) / s
    elif R[0] > R[4] and R[0] > R[8]:
        s = math.sqrt(1.0 + R[0] - R[4] - R[8]) * 2.0
        w, x, y, z = (R[7] - R[5]) / s, 0.25 * s, (R[1] + R[3]) / s, (R[2] + R[6]) / s
    elif R[4] > R[8]:
        s = math.sqrt(1.0 + R[4] - R[0] - R[8]) * 2.0
        w, x, y, z = (R[2] - R[6]) / s, (R[1] + R[3]) / s, 0.25 * s, (R[5] + R[7]) / s
    else:
        s = math.sqrt(1.0 + R[8] - R[0] - R[4]) * 2.0
        w, x, y, z = (R[3] - R[1]) / s, (R[2] + R[6]) / s, (R[5] + R[7]) / s, 0.25 * s
    if w < 0.0:
        w, x, y, z = -w, -x, -y, -z
    norm = math.sqrt(((w * w + x * x) + y * y) + z * z)
    return (x / norm, y / norm, z / norm, w / norm)


def angle_between_deg(a, b):
    """Angle between two nonzero vectors, degrees."""
    dot = (a[0] * b[0] + a[1] * b[1]) + a[2] * b[2]
    na = math.sqrt((a[0] * a[0] + a[1] * a[1]) + a[2] * a[2])
    nb = math.sqrt((b[0] * b[0] + b[1] * b[1]) + b[2] * b[2])
    return math.degrees(math.acos(max(-1.0, min(1.0, dot / (na * nb)))))


def rotation_between(a, b):
    """The smallest rotation taking direction ``a`` onto direction ``b``."""
    na = math.sqrt((a[0] * a[0] + a[1] * a[1]) + a[2] * a[2])
    nb = math.sqrt((b[0] * b[0] + b[1] * b[1]) + b[2] * b[2])
    u = (a[0] / na, a[1] / na, a[2] / na)
    v = (b[0] / nb, b[1] / nb, b[2] / nb)
    axis = (u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0])
    s = math.sqrt((axis[0] * axis[0] + axis[1] * axis[1]) + axis[2] * axis[2])
    if s == 0.0:
        return _IDENTITY
    c = max(-1.0, min(1.0, (u[0] * v[0] + u[1] * v[1]) + u[2] * v[2]))
    return rotation_about((axis[0] / s, axis[1] / s, axis[2] / s), math.degrees(math.atan2(s, c)))


# MARK: Window points


def range_keep(points, max_range):
    """Finite points within ``max_range`` by the quantizer's binary32 rule ``(x*x + y*y) + z*z``."""
    with np.errstate(over="ignore", invalid="ignore"):
        keep = np.isfinite(points).all(axis=1)
        if max_range is not None:
            limit = F32(max_range)
            x, y, z = points[:, 0], points[:, 1], points[:, 2]
            keep &= ~(((x * x + y * y) + z * z) > limit * limit)
    return keep


def transform_points(points, pose):
    """Binary32 ``(n, 3)`` points through ``pose`` in binary32, ((r0 x + r1 y) + r2 z) + t, no FMA."""
    R = [F32(value) for value in pose.R]
    t = [F32(value) for value in pose.t]
    x, y, z = points[:, 0], points[:, 1], points[:, 2]
    return np.stack([((R[3 * i] * x + R[3 * i + 1] * y) + R[3 * i + 2] * z) + t[i] for i in range(3)], axis=1)


def corrected(points, pose, max_range):
    """A window's points for the change engine: unchanged at the identity; otherwise range-clipped in
    the sensor frame, then moved into the reference frame (the quantizer then has no range clip)."""
    if pose.is_identity:
        return points
    return transform_points(points[range_keep(points, max_range)], pose)


def _cells(points, size):
    """Packed cell keys of binary64 points: floor(p / size) per axis."""
    index = np.floor(points / size).astype(np.int64)
    return pack(index[:, 0], index[:, 1], index[:, 2])


def first_per_cell(points, size):
    """Indices of the first point of each ``size`` cell (in input order), ordered by cell key."""
    keys = _cells(points.astype(np.float64), size)
    _, first = np.unique(keys, return_index=True)
    return first


def decimate(count, limit):
    """Every k-th of ``count`` items, k = ceil(count / limit), so at most ``limit`` remain."""
    if count <= limit:
        return np.arange(count)
    step = -(-count // limit)
    return np.arange(0, count, step)


def source_cloud(points, max_range, voxel_m, limit):
    """The ICP source: finite points within ``max_range`` (sensor frame), the first per ``voxel_m``
    cell in arrival order, ordered by cell key, then decimated to at most ``limit``. Binary64."""
    points = points[range_keep(points, max_range)]
    picked = points[first_per_cell(points, voxel_m)]
    return picked[decimate(len(picked), limit)].astype(np.float64)


def voxel_keys(points, voxel_size):
    """The change engine's voxel keys of binary64 points: binary32 q = p * (1 / voxel), floored."""
    inverse = F32(1.0) / F32(voxel_size)
    q = points.astype(np.float32) * inverse
    index = np.floor(q).astype(np.int64)
    return pack(index[:, 0], index[:, 1], index[:, 2])


# MARK: Small symmetric eigenproblems


def jacobi_eigh(matrix):
    """Eigenvalues (ascending) and unit eigenvectors (columns) of a small symmetric matrix by cyclic
    Jacobi rotations in a fixed order; each eigenvector's largest component is made positive."""
    n = len(matrix)
    a = [list(map(float, row)) for row in matrix]
    v = [[1.0 if i == j else 0.0 for j in range(n)] for i in range(n)]
    for _ in range(JACOBI_SWEEPS):
        off = 0.0
        for p in range(n - 1):
            for q in range(p + 1, n):
                off += a[p][q] * a[p][q]
        if off == 0.0:
            break
        for p in range(n - 1):
            for q in range(p + 1, n):
                if a[p][q] == 0.0:
                    continue
                theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q])
                if abs(theta) > 1e150:
                    tan = 0.5 / theta
                else:
                    tan = (1.0 if theta >= 0.0 else -1.0) / (abs(theta) + math.sqrt(theta * theta + 1.0))
                cos = 1.0 / math.sqrt(tan * tan + 1.0)
                sin = tan * cos
                for k in range(n):
                    akp, akq = a[k][p], a[k][q]
                    a[k][p] = cos * akp - sin * akq
                    a[k][q] = sin * akp + cos * akq
                for k in range(n):
                    apk, aqk = a[p][k], a[q][k]
                    a[p][k] = cos * apk - sin * aqk
                    a[q][k] = sin * apk + cos * aqk
                for k in range(n):
                    vkp, vkq = v[k][p], v[k][q]
                    v[k][p] = cos * vkp - sin * vkq
                    v[k][q] = sin * vkp + cos * vkq
    order = sorted(range(n), key=lambda i: (a[i][i], i))
    values = [a[i][i] for i in order]
    vectors = []
    for i in order:
        column = [v[k][i] for k in range(n)]
        lead = max(range(n), key=lambda k: (abs(column[k]), -k))
        if column[lead] < 0.0:
            column = [-value for value in column]
        vectors.append(column)
    return values, vectors


# MARK: Target grids


class TargetGrid:
    """A cell table over the target points: per cell (sorted keys) the start and count of its kept
    indices (every k-th of the cell's points in target order, at most ``cap``)."""

    def __init__(self, points, cell, cap):
        self.cell = cell
        keys = _cells(points, cell)
        order = np.argsort(keys, kind="stable")
        sorted_keys = keys[order]
        self.keys, first, counts = np.unique(sorted_keys, return_index=True, return_counts=True)
        step = np.where(counts > cap, -(-counts // cap), 1) if cap else np.ones_like(counts)
        kept = -(-counts // step)
        self.count = kept
        self.start = np.concatenate([[0], np.cumsum(kept)[:-1]]).astype(np.int64)
        offsets = np.arange(int(kept.sum())) - np.repeat(self.start, kept)
        self.indices = order[np.repeat(first, kept) + offsets * np.repeat(step, kept)]

    def nearest(self, queries, points, radius):
        """For each binary64 query, the nearest kept target within ``radius`` in the 27 cells around
        it (ties to the lower target index): (index or -1, squared distance or inf)."""
        best = np.full(len(queries), np.inf)
        best_index = np.full(len(queries), -1, dtype=np.int64)
        if not len(queries) or not len(self.keys):
            return best_index, best
        base = np.floor(queries / self.cell).astype(np.int64)
        for di, dj, dk in _OFFSETS:
            keys = pack(base[:, 0] + di, base[:, 1] + dj, base[:, 2] + dk)
            at = np.minimum(np.searchsorted(self.keys, keys), len(self.keys) - 1)
            found = self.keys[at] == keys
            starts = np.where(found, self.start[at], 0)
            counts = np.where(found, self.count[at], 0)
            for slot in range(int(counts.max(initial=0))):
                valid = slot < counts
                rows = np.nonzero(valid)[0]
                target = self.indices[starts[rows] + slot]
                delta = queries[rows] - points[target]
                d2 = (delta[:, 0] * delta[:, 0] + delta[:, 1] * delta[:, 1]) + delta[:, 2] * delta[:, 2]
                better = (d2 < best[rows]) | ((d2 == best[rows]) & (target < best_index[rows]))
                best[rows[better]] = d2[better]
                best_index[rows[better]] = target[better]
        outside = best > radius * radius
        best_index[outside] = -1
        best[outside] = np.inf
        return best_index, best


# MARK: Registration cloud


class RegistrationBuilder:
    """Collects a sensor's real points during BUILDING: the first point (arrival order) in each
    ``voxel_m`` cell of the sensor frame, from every window that counts toward the reference."""

    def __init__(self, voxel_m):
        self.voxel_m = voxel_m
        self.keys = np.zeros(0, dtype=np.int64)
        self.points = np.zeros((0, 3), dtype=np.float32)

    def add(self, points, max_range):
        points = points[range_keep(points, max_range)]
        if not len(points):
            return
        keys = _cells(points.astype(np.float64), self.voxel_m)
        merged = np.concatenate([self.keys, keys])
        self.keys, first = np.unique(merged, return_index=True)
        self.points = np.concatenate([self.points, points])[first]

    def finish(self, reference_occupied, voxel_size, limit):
        """The cloud: points whose change-engine voxel is reference-occupied, decimated to ``limit``."""
        keep = contains(reference_occupied, voxel_keys(self.points, voxel_size))
        points = self.points[keep]
        return RegistrationCloud(points[decimate(len(points), limit)], self.voxel_m)


class RegistrationCloud:
    """A sensor's ICP target: real points (binary32, cell-key order), their normals and the
    correspondence grids, all computed when the cloud is built or loaded, so no check pays for them.
    ``normals`` is ``(n, 3)`` unit normals, NaN where a point has fewer than NORMAL_MIN_POINTS sampled
    neighbours within NORMAL_RADIUS_M (itself included) or its neighbourhood is not planar."""

    def __init__(self, points, voxel_m):
        self.points = np.ascontiguousarray(points, dtype=np.float32).reshape(-1, 3)
        self.voxel_m = voxel_m
        self.points64 = self.points.astype(np.float64)
        self._grids = {cell: TargetGrid(self.points64, cell, GRID_CAP) for cell in GRID_LEVELS}
        self.normals = _normals(self.points64, NORMAL_RADIUS_M)

    def __len__(self):
        return len(self.points)

    def grid(self, radius):
        """The finest grid whose cell is at least ``radius``."""
        return self._grids[next((level for level in GRID_LEVELS if level >= radius), GRID_LEVELS[-1])]


def _normals(points, radius):
    n = len(points)
    normals = np.full((n, 3), np.nan)
    if not n:
        return normals
    grid = TargetGrid(points, radius, GRID_CAP)
    count = np.zeros(n)
    s = np.zeros((n, 3))
    ss = np.zeros((n, 6))  # xx xy xz yy yz zz
    base = np.floor(points / radius).astype(np.int64)
    r2 = radius * radius
    for di, dj, dk in _OFFSETS:
        keys = pack(base[:, 0] + di, base[:, 1] + dj, base[:, 2] + dk)
        at = np.minimum(np.searchsorted(grid.keys, keys), len(grid.keys) - 1)
        found = grid.keys[at] == keys
        starts = np.where(found, grid.start[at], 0)
        counts = np.where(found, grid.count[at], 0)
        for slot in range(int(counts.max(initial=0))):
            rows = np.nonzero(slot < counts)[0]
            other = points[grid.indices[starts[rows] + slot]]
            delta = points[rows] - other
            near = ((delta[:, 0] * delta[:, 0] + delta[:, 1] * delta[:, 1]) + delta[:, 2] * delta[:, 2]) <= r2
            rows, other = rows[near], other[near]
            count[rows] += 1.0
            s[rows] += other
            x, y, z = other[:, 0], other[:, 1], other[:, 2]
            ss[rows] += np.stack([x * x, x * y, x * z, y * y, y * z, z * z], axis=1)
    rows = np.nonzero(count >= NORMAL_MIN_POINTS)[0]
    if not len(rows):
        return normals
    k = count[rows]
    m = s[rows] / k[:, None]
    c = ss[rows] / k[:, None]
    cov = np.empty((len(rows), 3, 3))
    pairs = ((0, 0, 0), (0, 1, 1), (0, 2, 2), (1, 1, 3), (1, 2, 4), (2, 2, 5))
    for i, j, slot in pairs:
        cov[:, i, j] = cov[:, j, i] = c[:, slot] - m[:, i] * m[:, j]
    # Batched here; the C++ engine runs jacobi_eigh per point. The normal's sign does not matter.
    values, vectors = np.linalg.eigh(cov)
    planar = values[:, 0] <= NORMAL_PLANARITY * values[:, 1]
    normals[rows[planar]] = vectors[planar, :, 0]
    return normals


# MARK: Alignment


def horn(source, target, weights):
    """The weighted least-squares rigid motion taking ``source`` onto ``target`` (Horn 1987):
    a ``Pose`` with ``target ≈ R source + t``. None without weight."""
    total = float(weights.sum())
    if not total > 0.0:
        return None
    ps = (weights[:, None] * source).sum(axis=0) / total
    qs = (weights[:, None] * target).sum(axis=0) / total
    a = source - ps
    b = target - qs
    S = [[float((weights * a[:, i] * b[:, j]).sum()) for j in range(3)] for i in range(3)]
    (xx, xy, xz), (yx, yy, yz), (zx, zy, zz) = S
    N = [[xx + yy + zz, yz - zy, zx - xz, xy - yx],
         [yz - zy, xx - yy - zz, xy + yx, zx + xz],
         [zx - xz, xy + yx, -xx + yy - zz, yz + zy],
         [xy - yx, zx + xz, yz + zy, -xx - yy + zz]]
    _, vectors = jacobi_eigh(N)
    w, x, y, z = vectors[-1]
    norm = math.sqrt(((w * w + x * x) + y * y) + z * z)
    R = quaternion_rotation(w / norm, x / norm, y / norm, z / norm)
    Rp = _matvec(R, tuple(float(value) for value in ps))
    return Pose(R, tuple(float(qs[n]) - Rp[n] for n in range(3)))


@dataclass
class Estimate:
    """One ICP result: the absolute pose and its metrics. ``rms_m`` is the RMS point-to-plane residual
    of the final correspondences; ``observe`` is λ_min(H) / Σw of their point-to-plane information
    matrix, ``weakest`` its eigenvector (translation x y z, then rotation as meters at the median
    source range)."""

    pose: Pose
    inlier_frac: float
    rms_m: float
    observe: float
    correspondences: int
    tau_m: float
    weakest: Tuple[float, ...] = field(default=(0.0,) * 6)


def median_range(source):
    """The median distance of the source points from the sensor (a sort, not a selection)."""
    ranges = np.sort(np.sqrt((source[:, 0] * source[:, 0] + source[:, 1] * source[:, 1]) + source[:, 2] * source[:, 2]))
    return float(ranges[len(ranges) // 2]) if len(ranges) else 0.0


def _plane_system(moved, pose, cloud, match, tau, scale):
    """The point-to-plane normal equations at ``pose`` over the correspondences whose target has a
    normal: H = Σ w J Jᵀ and g = Σ w r J with J = [n, ((p - t) × n) / scale], r = n · (p - q), and
    Geman-McClure weights w = κ² / (κ² + r²)², κ = τ / 3. Also Σw, Σ w r² ... (count, Σ r²)."""
    used = match >= 0
    rows = np.nonzero(used)[0]
    normals = cloud.normals[match[rows]]
    has = np.isfinite(normals).all(axis=1)
    rows, n = rows[has], normals[has]
    p = moved[rows]
    q = cloud.points64[match[rows]]
    d = p - q
    r = (d[:, 0] * n[:, 0] + d[:, 1] * n[:, 1]) + d[:, 2] * n[:, 2]
    c = p - np.array(pose.t)
    cross = np.stack([c[:, 1] * n[:, 2] - c[:, 2] * n[:, 1], c[:, 2] * n[:, 0] - c[:, 0] * n[:, 2],
                      c[:, 0] * n[:, 1] - c[:, 1] * n[:, 0]], axis=1) / scale
    J = np.concatenate([n, cross], axis=1)
    kappa2 = (tau / 3.0) * (tau / 3.0)
    w = kappa2 / ((kappa2 + r * r) * (kappa2 + r * r))
    H = [[float((w * J[:, i] * J[:, j]).sum()) for j in range(6)] for i in range(6)]
    g = [float((w * r * J[:, i]).sum()) for i in range(6)]
    return H, g, float(w.sum()), len(rows), float((r * r).sum())


def _plane_step(H, g, pose, scale):
    """The Gauss-Newton step of the point-to-plane system, solved through its eigenvectors with the
    directions below STEP_EIG_FLOOR of the largest eigenvalue left out, as a pose in the reference
    frame that rotates about the sensor's current position. Also (λ_min, its eigenvector)."""
    values, vectors = jacobi_eigh(H)
    floor = STEP_EIG_FLOOR * values[-1]
    x = [0.0] * 6
    for value, vector in zip(values, vectors):
        if not value > floor:
            continue
        along = -sum(vector[k] * g[k] for k in range(6)) / value
        x = [x[k] + along * vector[k] for k in range(6)]
    R = rotation_vector(x[3] / scale, x[4] / scale, x[5] / scale)
    Rt = _matvec(R, pose.t)
    step = Pose(R, tuple(pose.t[n] - Rt[n] + x[n] for n in range(3)))
    return step, values[0], tuple(vectors[0])


def align(source, cloud, seed, sigma0):
    """ICP of the binary64 ``source`` (sensor frame) onto ``cloud`` from the absolute ``seed``.

    Point-to-point stages first (KISS-ICP): the threshold starts at clamp(3 sigma0, 0.05, 0.5) m;
    each stage iterates weighted Horn steps (Geman-McClure weights, kappa = tau / 3) until the step is
    below 0.1 mm and 0.001 degrees or MAX_ITERATIONS, then tau becomes max(0.04, 3 x inlier RMS);
    they stop when tau reaches 0.04 m, shrinks by less than 5 %, or after MAX_STAGES. Then up to
    FINE_ITERATIONS point-to-plane steps at that tau. None when nothing corresponds."""
    if not len(source) or not len(cloud):
        return None
    targets = cloud.points64
    pose = seed
    tau = min(TAU_MAX_M, max(TAU_FIRST_MIN_M, 3.0 * sigma0))
    stop_cos = math.cos(math.radians(STEP_STOP_DEG))
    for _ in range(MAX_STAGES):
        grid = cloud.grid(tau)
        kappa2 = (tau / 3.0) * (tau / 3.0)
        for _ in range(MAX_ITERATIONS):
            moved = pose.apply(source)
            match, d2 = grid.nearest(moved, targets, tau)
            used = match >= 0
            if used.sum() < 3:
                return None
            weights = kappa2 / ((kappa2 + d2[used]) * (kappa2 + d2[used]))
            step = horn(moved[used], targets[match[used]], weights)
            if step is None:
                return None
            pose = step.compose(pose)
            if step.distance_m < STEP_STOP_M and step.cos_angle >= stop_cos:
                break
        match, d2 = grid.nearest(pose.apply(source), targets, tau)
        used = match >= 0
        if used.sum() < 3:
            return None
        shrunk = max(TAU_MIN_M, 3.0 * math.sqrt(float(d2[used].mean())))
        if shrunk >= 0.95 * tau:
            break
        tau = shrunk
        if tau <= TAU_MIN_M:
            break
    scale = median_range(source)
    if not scale > 0.0:
        return None
    grid = cloud.grid(tau)
    for _ in range(FINE_ITERATIONS):
        moved = pose.apply(source)
        match, _ = grid.nearest(moved, targets, tau)
        H, g, total, count, _ = _plane_system(moved, pose, cloud, match, tau, scale)
        if count < 6:
            return None
        step, _, _ = _plane_step(H, g, pose, scale)
        pose = step.compose(pose)
        if step.distance_m < STEP_STOP_M and step.cos_angle >= stop_cos:
            break
    moved = pose.apply(source)
    match, _ = grid.nearest(moved, targets, tau)
    H, g, total, count, squares = _plane_system(moved, pose, cloud, match, tau, scale)
    if count < 6:
        return None
    values, vectors = jacobi_eigh(H)
    near, _ = cloud.grid(INLIER_RADIUS_M).nearest(moved, targets, INLIER_RADIUS_M)
    return Estimate(pose, float((near >= 0).sum()) / len(source), math.sqrt(squares / count),
                    max(0.0, values[0]) / total, count, tau, tuple(vectors[0]))


def overlap(source, pose, reference_index_keys, voxel_size):
    """How many source points land, through ``pose``, in a reference-occupied voxel."""
    return int(contains(reference_index_keys, voxel_keys(pose.apply(source), voxel_size)).sum())


def sweep_seed(source, base, shift, reference_occupied, voxel_size, max_deg, axis=(0.0, 0.0, 1.0)):
    """The yaw sweep without a rotation prior: candidates ``(I, v) ∘ base ∘ (R_axis(ψ), 0)`` for ψ in
    whole degrees within ±max_deg and v in (shift, 0), scored by reference-occupied overlap; ties to
    the smaller |ψ|, the negative one, then v = shift. Returns (seed, ψ)."""
    best = None
    steps = int(math.floor(max_deg / SWEEP_STEP_DEG))
    for magnitude in range(steps + 1):
        for psi in ((0.0,) if magnitude == 0 else (-magnitude * SWEEP_STEP_DEG, magnitude * SWEEP_STEP_DEG)):
            rotated = base.compose(Pose(rotation_about(axis, psi)))
            for v in (shift, (0.0, 0.0, 0.0)):
                candidate = Pose(rotated.R, tuple(rotated.t[n] + v[n] for n in range(3)))
                score = overlap(source, candidate, reference_occupied, voxel_size)
                if best is None or score > best[0]:
                    best = (score, candidate, psi)
    return best[1], best[2]
