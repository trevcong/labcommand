"""Deterministic test scenes, bit for bit the binary32 builders of ``ChangeDetectionTests.swift``.

Every arithmetic step is a ``numpy.float32`` operation in the Swift order, so a scene written
with ``encode_pointcloud2`` and read by ``swift run change-detect`` holds the same values the
Python tests compare.
"""

import struct

import numpy as np

F32 = np.float32
MASK64 = (1 << 64) - 1
VOXEL = F32(0.05)
GRID_STEP = F32(0.02)
GRID_PHASE = F32(0.007)


class LCG:
    """The test runner's 64-bit LCG (``SplitMix`` in the Swift tests)."""

    def __init__(self, state):
        self.state = state & MASK64

    def next(self):
        self.state = (self.state * 6364136223846793005 + 1442695040888963407) & MASK64
        return self.state

    def unit(self):
        return F32(self.next() >> 40) / F32(1 << 24)


def shuffled(points, seed):
    """Fisher-Yates permutation driven by the LCG, as ``shuffledPoints`` in Swift."""
    order = list(range(len(points)))
    rng = LCG(seed)
    index = len(order)
    while index > 1:
        index -= 1
        swap = rng.next() % (index + 1)
        order[index], order[swap] = order[swap], order[index]
    return np.asarray(points, dtype=np.float32)[order]


def samples(low, high):
    values = []
    i = 0
    while True:
        value = (F32(low) + GRID_PHASE) + F32(i) * GRID_STEP
        if value >= F32(high):
            return values
        values.append(value)
        i += 1


def room_shell():
    """Floor and four walls, about 4 x 4 x 2 m, on the 0.02 m lattice."""
    x0, x1, y0, y1, z0, z1 = F32(-1.5), F32(2.5), F32(-1.5), F32(2.5), F32(-0.7), F32(1.3)
    xs, ys, zs = samples(x0, x1), samples(y0, y1), samples(z0, z1)
    points = []
    floor_z = z0 + GRID_PHASE
    points += [(x, y, floor_z) for x in xs for y in ys]
    x_lo, x_hi = x0 + GRID_PHASE, x1 - GRID_PHASE
    for y in ys:
        for z in zs:
            points += [(x_lo, y, z), (x_hi, y, z)]
    y_lo, y_hi = y0 + GRID_PHASE, y1 - GRID_PHASE
    for x in xs:
        for z in zs:
            points += [(x, y_lo, z), (x, y_hi, z)]
    return np.array(points, dtype=np.float32)


def box_shell(center, side=0.40):
    """Hollow box: six faces sampled just inside the geometric faces."""
    half = F32(side) * F32(0.5)
    cx, cy, cz = (F32(value) for value in center)
    x0, x1, y0, y1, z0, z1 = cx - half, cx + half, cy - half, cy + half, cz - half, cz + half
    xs, ys, zs = samples(x0, x1), samples(y0, y1), samples(z0, z1)
    points = []
    for y in ys:
        for z in zs:
            points += [(x0 + GRID_PHASE, y, z), (x1 - GRID_PHASE, y, z)]
    for x in xs:
        for z in zs:
            points += [(x, y0 + GRID_PHASE, z), (x, y1 - GRID_PHASE, z)]
    for x in xs:
        for y in ys:
            points += [(x, y, z0 + GRID_PHASE), (x, y, z1 - GRID_PHASE)]
    return np.array(points, dtype=np.float32)


def shifted(points, dx):
    out = np.array(points, dtype=np.float32, copy=True)
    out[:, 0] = out[:, 0] + F32(dx)
    return out


def jittered(points, seed):
    """Drops a seeded 20 % and offsets the rest by up to 1 cm per axis (noise-only group)."""
    rng = LCG(seed)
    out = []
    for x, y, z in np.asarray(points, dtype=np.float32):
        draw = rng.unit()
        dx = (rng.unit() * F32(2) - F32(1)) * F32(0.01)
        dy = (rng.unit() * F32(2) - F32(1)) * F32(0.01)
        dz = (rng.unit() * F32(2) - F32(1)) * F32(0.01)
        if draw < F32(0.20):
            continue
        out.append((x + dx, y + dy, z + dz))
    return np.array(out, dtype=np.float32)


def axis_center(index):
    return (F32(index) + F32(0.5)) * VOXEL


def cell_point(i, j, k):
    return (axis_center(i), axis_center(j), axis_center(k))


def column(indices):
    return np.array([cell_point(i, 0, 0) for i in indices], dtype=np.float32).reshape(-1, 3)


def indoor(move_box_a):
    """The spec's 8 x 8 x 3 m room with three hollow boxes: 89,320 points per cloud."""
    i, j, k = np.meshgrid(np.arange(-80, 80), np.arange(-80, 80), np.arange(-30, 30), indexing="ij")
    shell = (i == -80) | (i == 79) | (j == -80) | (j == 79) | (k == -30) | (k == 29)
    cells = [np.stack([i[shell], j[shell], k[shell]], axis=1)]

    def box(i0, i1, j0, j1, k0, k1, shift):
        bi, bj, bk = np.meshgrid(np.arange(i0, i1 + 1), np.arange(j0, j1 + 1), np.arange(k0, k1 + 1), indexing="ij")
        face = (bi == i0) | (bi == i1) | (bj == j0) | (bj == j1) | (bk == k0) | (bk == k1)
        cells.append(np.stack([bi[face] + shift, bj[face], bk[face]], axis=1))

    box(20, 27, -10, -3, -8, -1, 20 if move_box_a else 0)
    box(-30, -23, 16, 23, -4, 3, 0)
    box(-6, 5, -28, -17, 4, 13, 0)
    indices = np.concatenate(cells).astype(np.float32)
    return (indices + F32(0.5)) * VOXEL


def long_ray():
    """The spec's long-ray benchmark pair: 80,000 distinct in-ball cells, seed 0x15A6."""
    state = 0x15A6
    seen = set()
    points = []
    while len(points) < 80_000:
        state = (state * 6364136223846793005 + 1442695040888963407) & MASK64
        i = state % 599 - 299
        j = (state >> 21) % 599 - 299
        k = (state >> 42) % 599 - 299
        if i * i + j * j + k * k > 299 * 299 or (i, j, k) in seen:
            continue
        seen.add((i, j, k))
        points.append((i, j, k))
    before = (np.array(points, dtype=np.float32) + F32(0.5)) * VOXEL
    after = before.copy()
    limit = F32(15) * F32(15)
    for index in range(8_000):
        x = after[index, 0] + F32(1)
        y, z = after[index, 1], after[index, 2]
        if (x * x + y * y) + z * z <= limit:
            after[index, 0] = x
    return before, after


def board_scene():
    """Box removed with its back face occluded.

    Before: room shell plus a 0.4 m box centered at (1.0, 0.5, 0.3). After: the box is
    gone, and a 0.6 m board stands at x = 1.003, inside the box's old volume. The after
    cloud is ray-consistent: room points whose sight line from the origin crosses the
    board are removed, so nothing is seen through it. The box's front face lies in front
    of the board (traversed, free) and its back face in the board's shadow (unknown).
    """
    room = room_shell()
    box = box_shell((1.0, 0.5, 0.30))
    board_x = F32(1.003)
    ys = samples(0.2, 0.8)
    zs = samples(0.0, 0.6)
    board = np.array([(board_x, y, z) for y in ys for z in zs], dtype=np.float32)
    y_lo, y_hi = float(ys[0]), float(ys[-1])
    z_lo, z_hi = float(zs[0]), float(zs[-1])
    x = room[:, 0].astype(np.float64)
    with np.errstate(divide="ignore", invalid="ignore"):
        t = float(board_x) / x
    y_hit = room[:, 1] * t
    z_hit = room[:, 2] * t
    shadowed = (x > float(board_x)) & (y_hit >= y_lo) & (y_hit <= y_hi) & (z_hit >= z_lo) & (z_hit <= z_hi)
    before = np.concatenate([room, box])
    after = np.concatenate([room[~shadowed], board])
    back_face = box[box[:, 0] > F32(1.15)]
    front_face = box[box[:, 0] < F32(0.85)]
    return before, after, front_face, back_face


def encode_pointcloud2(points, frame_id="test"):
    """CDR little-endian PointCloud2 with x, y, z, intensity float32 (point_step 16)."""
    points = np.asarray(points, dtype=np.float32).reshape(-1, 3)
    out = bytearray(b"\x00\x01\x00\x00")

    def align(size):
        while (len(out) - 4) % size:
            out.append(0)

    def u32(value):
        align(4)
        out.extend(struct.pack("<I", value))

    def string(text):
        data = text.encode() + b"\x00"
        u32(len(data))
        out.extend(data)

    u32(0)
    u32(0)
    string(frame_id)
    u32(1)
    u32(len(points))
    u32(4)
    for offset, name in enumerate(("x", "y", "z", "intensity")):
        string(name)
        u32(offset * 4)
        out.append(7)
        u32(1)
    out.append(0)
    u32(16)
    u32(16 * len(points))
    records = np.zeros((len(points), 4), dtype="<f4")
    records[:, :3] = points
    u32(records.nbytes)
    out.extend(records.tobytes())
    out.append(1)
    return bytes(out)
