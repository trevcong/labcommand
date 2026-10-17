"""The rig guard's geometry (room_memory/guard.py): pose conventions, the binary32 transform, the
registration cloud, the solver pieces, and ICP on ray-cast rooms."""

import copy
import math
import os
import pathlib
import shutil
import warnings
from dataclasses import dataclass, replace

import numpy as np
import pytest

from room_memory import guard as g
from room_memory.core import Config, decode_pointcloud2, occupancy_map
from room_memory.core.change_detection import LIMIT
from room_memory import state as rig_files
from room_memory.memory import MemoryConfig, RoomMemory, load_epoch, load_reference, pose_shift
from room_memory.messages import (CAUSE_IMU_SHOCK, CAUSE_IMU_TILT, CAUSE_PERIODIC, CAUSE_POSE_OFFSET, CAUSE_STARTUP,
                                  SENSOR_JT128, SENSOR_MID360, EventKind, EventState, ImuSample, MapBricks, MapKind,
                                  MemoryState, RigChannel, RigOutcome, Time)

import rooms
from test_memory import GENERATION

SYNTHETIC = pathlib.Path(__file__).resolve().parents[3] / "Tests" / "Fixtures" / "room" / "synthetic"
VOXEL = 0.05

warnings.filterwarnings("ignore", message=".*encountered in matmul")


def reference_of(scans):
    """(registration cloud, reference-occupied keys) of the change engine's quantizer over ``scans``."""
    config = Config(voxel_size=VOXEL, max_range=15.0, evidence_of_absence=True)
    builder = g.RegistrationBuilder(0.02)
    occupied = set()
    for points in scans:
        builder.add(points, 15.0)
        occupied |= set(occupancy_map(points, config).occupied.tolist())
    keys = np.array(sorted(occupied), dtype=np.int64)
    return builder.finish(keys, VOXEL, 300_000), keys


@pytest.fixture(scope="module")
def furnished():
    return reference_of([rooms.scan(seed=seed) for seed in range(10)])


def solve(source, cloud, keys, shift=(0.0, 0.0, 0.0)):
    seed, _ = g.sweep_seed(source, g.Pose(), shift, keys, VOXEL, 10.0)
    return g.align(source, cloud, seed, 0.5 * math.radians(1.0) * g.median_range(source) + g.Pose(t=shift).distance_m)


# MARK: Poses and small matrices


def test_pose_conventions():
    D = g.Pose(g.rotation_about((0.0, 0.0, 1.0), 30.0), (1.0, 2.0, 3.0))
    T = g.Pose(g.rotation_about((1.0, 0.0, 0.0), 10.0), (0.5, 0.0, -1.0))
    p = np.array([[0.3, -0.2, 1.5]])
    # (A ∘ B)(p) = A(B(p)), and the increment from T to T ∘ D is D.
    assert np.allclose(T.compose(D).apply(p), T.apply(D.apply(p)), atol=1e-12)
    increment = T.increment(T.compose(D))
    assert increment.distance_m == pytest.approx(D.distance_m, abs=1e-12)
    assert increment.angle_deg == pytest.approx(30.0, abs=1e-9)
    assert np.allclose(T.inverse().apply(T.apply(p)), p, atol=1e-12)
    # Quaternion round trip and the stored text round trip.
    x, y, z, w = T.quaternion()
    assert np.allclose(g.quaternion_rotation(w, x, y, z), T.R, atol=1e-12)
    assert g.Pose.from_values(T.text().split()) == T
    assert g.Pose().is_identity and not g.Pose(t=(0.0, 0.0, 1e-300)).is_identity


def test_jacobi_matches_closed_form_eigenpairs():
    rng = np.random.default_rng(3)
    for n in (3, 4, 6):
        for _ in range(5):
            a = rng.normal(size=(n, n))
            a = a + a.T
            values, vectors = g.jacobi_eigh(a.tolist())
            assert np.allclose(values, np.linalg.eigvalsh(a), atol=1e-10)
            for value, vector in zip(values, vectors):
                assert np.allclose(a @ vector, value * np.array(vector), atol=1e-10)
                assert max(vector, key=abs) > 0
    assert g.jacobi_eigh([[2.0, 0.0], [0.0, 1.0]]) == ([1.0, 2.0], [[0.0, 1.0], [1.0, 0.0]])
    assert g.jacobi_eigh([[4.0, 1.0], [1.0, 4.0]])[0] == pytest.approx([3.0, 5.0], abs=1e-15)


def test_horn_recovers_a_known_motion_and_ignores_zero_weights():
    rng = np.random.default_rng(1)
    truth = g.Pose(g.rotation_about((0.0, 0.6, 0.8), 7.0), (0.1, -0.2, 0.05))
    source = rng.normal(size=(60, 3))
    target = truth.apply(source)
    target[50:] += rng.normal(size=(10, 3))  # outliers with weight 0
    weights = np.r_[np.ones(50), np.zeros(10)]
    estimate = g.horn(source, target, weights)
    assert np.allclose(estimate.R, truth.R, atol=1e-12)
    assert np.allclose(estimate.t, truth.t, atol=1e-12)
    assert g.horn(source, target, np.zeros(60)) is None


def test_transform_is_binary32_in_a_fixed_order():
    rng = np.random.default_rng(2)
    points = (rng.normal(size=(500, 3)) * 5).astype(np.float32)
    pose = g.Pose(g.rotation_about((0.0, 0.0, 1.0), 3.3), (0.123, -0.456, 0.789))
    out = g.transform_points(points, pose)
    assert out.dtype == np.float32
    R = [np.float32(value) for value in pose.R]
    t = [np.float32(value) for value in pose.t]
    for row, p in zip(out[:50], points[:50]):
        for i in range(3):
            expected = np.float32(np.float32(np.float32(R[3 * i] * p[0]) + np.float32(R[3 * i + 1] * p[1]))
                                  + np.float32(R[3 * i + 2] * p[2]))
            assert row[i].tobytes() == np.float32(expected + t[i]).tobytes()


def test_corrected_skips_the_identity_and_clips_range_before_the_transform():
    points = np.array([[1.0, 0.0, 0.0], [14.9, 0.0, 0.0], [0.0, 15.1, 0.0], [np.nan, 0.0, 0.0]], dtype=np.float32)
    assert g.corrected(points, g.Pose(), 15.0) is points
    # Moved 0.5 m toward +x: 14.9 m in the sensor frame stays, although it lands past 15 m.
    out = g.corrected(points, g.Pose(t=(0.5, 0.0, 0.0)), 15.0)
    assert out.tolist() == [[1.5, 0.0, 0.0], [np.float32(15.4), 0.0, 0.0]]


# MARK: Registration cloud


def test_registration_cloud_keeps_reference_occupied_points_first_per_cell():
    builder = g.RegistrationBuilder(0.02)
    builder.add(np.array([[1.001, 0.0, 0.0], [1.005, 0.001, 0.001], [3.0, 0.0, 0.0], [20.0, 0.0, 0.0]], np.float32), 15.0)
    builder.add(np.array([[1.002, 0.0, 0.0], [2.0, 0.0, 0.0]], np.float32), 15.0)
    # First point per 2 cm cell, in arrival order across windows; 20 m is out of range.
    assert sorted(map(tuple, builder.points.tolist())) == sorted(
        map(tuple, np.array([[1.001, 0.0, 0.0], [2.0, 0.0, 0.0], [3.0, 0.0, 0.0]], np.float32).tolist()))
    occupied = np.unique(g.voxel_keys(np.array([[1.0, 0.0, 0.0], [3.0, 0.0, 0.0]]), VOXEL))
    cloud = builder.finish(occupied, VOXEL, 300_000)
    assert sorted(cloud.points[:, 0].tolist()) == pytest.approx([1.001, 3.0])
    assert len(builder.finish(occupied, VOXEL, 1)) == 1
    assert g.decimate(10, 3).tolist() == [0, 4, 8]


def test_grid_finds_the_nearest_point_with_ties_to_the_lower_index():
    targets = np.array([[0.0, 0.0, 0.0], [0.1, 0.0, 0.0], [-0.1, 0.0, 0.0], [0.5, 0.5, 0.5]])
    grid = g.TargetGrid(targets, 0.16, 20)
    index, d2 = grid.nearest(np.array([[0.05, 0.0, 0.0], [0.0, 0.0, 0.09], [0.3, 0.3, 0.3]]), targets, 0.1)
    assert index.tolist() == [0, 0, -1]
    assert d2[0] == pytest.approx(0.0025)


def reference_nearest(grid, queries, points, radius):
    """TargetGrid.nearest as #19 wrote it (and guard.cpp runs it): every kept slot of the 27 cells in
    offset order, a smaller d2 or an equal d2 and a lower index replacing the best."""
    best = np.full(len(queries), np.inf)
    best_index = np.full(len(queries), -1, dtype=np.int64)
    if not len(queries) or not len(grid.keys):
        return best_index, best
    base = np.floor(queries / grid.cell).astype(np.int64)
    for di, dj, dk in g._OFFSETS:
        keys = g.pack(base[:, 0] + di, base[:, 1] + dj, base[:, 2] + dk)
        at = np.minimum(np.searchsorted(grid.keys, keys), len(grid.keys) - 1)
        found = grid.keys[at] == keys
        starts = np.where(found, grid.start[at], 0)
        counts = np.where(found, grid.count[at], 0)
        for slot in range(int(counts.max(initial=0))):
            rows = np.nonzero(slot < counts)[0]
            target = grid.indices[starts[rows] + slot]
            delta = queries[rows] - points[target]
            d2 = (delta[:, 0] * delta[:, 0] + delta[:, 1] * delta[:, 1]) + delta[:, 2] * delta[:, 2]
            better = (d2 < best[rows]) | ((d2 == best[rows]) & (target < best_index[rows]))
            best[rows[better]] = d2[better]
            best_index[rows[better]] = target[better]
    outside = best > radius * radius
    best_index[outside] = -1
    best[outside] = np.inf
    return best_index, best


@pytest.mark.filterwarnings("ignore:invalid value encountered in cast", "ignore:overflow encountered in multiply")
def test_grid_nearest_is_bit_identical_to_the_slot_scan():
    """The vectorized search (one pass over the candidate pairs, neighbour cells pruned by box
    distance) returns the slot scan's index and binary64 d2 exactly, for points whose cell indices
    pack: random clouds, lattices full of exact ties, queries on cell faces with radii equal to
    lattice distances, non-finite queries, d2 that overflows, binary32 targets (cast to binary64),
    a points array other than the grid's, and a ray-cast room."""
    rng = np.random.default_rng(7)
    cases = []
    for trial in range(160):
        n, m = int(rng.integers(0, 600)), int(rng.integers(0, 400))
        cell = float(rng.choice([0.05, 0.16, 0.5]))
        radius = float(rng.choice([0.0, 0.03, 0.1, 1.0, math.inf]))
        kind = trial % 5
        if kind == 0:
            points, queries = rng.normal(size=(n, 3)), rng.normal(size=(m, 3))
        elif kind in (1, 2):  # ties: many targets at one lattice distance, queries on or between sites
            points = rng.integers(-4, 4, size=(n, 3)) * 0.05
            queries = rng.integers(-4, 4, size=(m, 3)) * 0.05 + (0.025 if kind == 2 else 0.0)
        elif kind == 3:  # exact binary lattices: queries on cell faces, d2 == radius**2 exactly
            step = float(rng.choice([0.125, 0.25]))
            points = rng.integers(-5, 5, size=(n, 3)) * step
            queries = rng.integers(-5, 5, size=(m, 3)) * step
            cell = float(rng.choice([0.125, 0.25, 0.5]))
            radius = float(rng.choice([step, 2 * step, cell]))
        else:
            points, queries = rng.integers(-4, 4, size=(n, 3)) * 0.05, rng.normal(size=(m, 3)) * 0.2
            queries[rng.random(m) < 0.1] = np.nan
            queries[rng.random(m) < 0.05] = np.inf
        if trial % 9 == 0:
            points = points.astype(np.float32)
        cases.append((points, queries, cell, int(rng.choice([0, 1, 3, 20])), radius, trial % 7 == 0))
    room = rooms.scan(seed=1).astype(np.float64)
    moved = rooms.scan(rooms.rig(0.02, yaw_deg=1.0), seed=2).astype(np.float64)
    cases += [(room, moved, 0.08, g.GRID_CAP, 0.05, False), (room, moved[::4], 0.16, g.GRID_CAP, 0.16, False)]
    # Neighbours whose d2 overflows to inf: never a match, not even within an infinite radius.
    # The cell is 1e160, so floor(p / cell) stays inside the 21-bit key range.
    huge = rng.integers(-3, 3, size=(200, 3)) * 1e160
    cases.append((huge, huge[:100] + 5e159, 1e160, 3, math.inf, False))
    for points, queries, cell, cap, radius, foreign in cases:
        grid = g.TargetGrid(points, cell, cap)
        coordinates = points.copy() if foreign else points
        index, d2 = grid.nearest(queries, coordinates, radius)
        expected_index, expected_d2 = reference_nearest(grid, queries, coordinates, radius)
        assert index.tobytes() == expected_index.tobytes() and d2.tobytes() == expected_d2.tobytes()


def test_grid_casts_float32_queries_and_targets_to_float64():
    """Binary32 floor(p / cell) can pick a different cell than binary64. The grid casts first, so a
    float32 cloud matches the float64 grid of the same values, and an already-float64 array keeps
    its identity (the kept-column path)."""
    cell = 0.1
    xs = np.linspace(-2, 2, 40001, dtype=np.float32)
    narrow = np.floor(xs / cell).astype(np.int64)
    promoted = np.floor(xs.astype(np.float64) / cell).astype(np.int64)
    mismatch = np.flatnonzero(narrow != promoted)
    assert len(mismatch)
    points32 = np.array([[xs[mismatch[0]], 0.0, 0.0], [0.0, 0.0, 0.0], [0.05, 0.0, 0.0]], np.float32)
    grid = g.TargetGrid(points32, cell, 20)
    assert grid._points.dtype == np.float64 and grid._points is not points32
    wide = g.TargetGrid(points32.astype(np.float64), cell, 20)
    assert grid.keys.tobytes() == wide.keys.tobytes()
    queries32 = np.array([[xs[mismatch[0]], 0.01, 0.0], [0.02, 0.0, 0.0]], np.float32)
    index, d2 = grid.nearest(queries32, points32, 0.5)
    expected = reference_nearest(grid, queries32.astype(np.float64), grid._points, 0.5)
    assert index.tobytes() == expected[0].tobytes() and d2.tobytes() == expected[1].tobytes()
    same_index, same_d2 = wide.nearest(queries32.astype(np.float64), wide._points, 0.5)
    assert index.tobytes() == same_index.tobytes() and d2.tobytes() == same_d2.tobytes()
    points64 = points32.astype(np.float64)
    assert g.TargetGrid(points64, cell, 20)._points is points64


def test_grid_rejects_a_point_outside_the_21_bit_cell_range():
    """floor(p / cell) +/- 1 must pack. The last index that does still matches the slot scan; one
    step further, on either side, is an error from the grid, from nearest, and from the normals."""
    cell = 0.5  # a power of two, so (LIMIT - 2) * cell / cell is that index exactly
    edge = np.array([[(LIMIT - 2) * cell, 0.0, 0.0], [0.0, (-LIMIT + 1) * cell, 0.0]])
    grid = g.TargetGrid(edge, cell, 20)
    index, d2 = grid.nearest(edge, grid._points, 1.0)
    expected = reference_nearest(grid, edge, grid._points, 1.0)
    assert index.tobytes() == expected[0].tobytes() and d2.tobytes() == expected[1].tobytes()
    beyond = np.array([[(LIMIT - 1) * cell, 0.0, 0.0]])
    with pytest.raises(ValueError, match="21-bit"):
        g.TargetGrid(beyond, cell, 20)
    with pytest.raises(ValueError, match="21-bit"):
        grid.nearest(beyond, grid._points, 1.0)
    with pytest.raises(ValueError, match="21-bit"):
        grid.nearest(np.array([[-LIMIT * cell, 0.0, 0.0]]), grid._points, 1.0)
    with pytest.raises(ValueError, match="21-bit"):
        grid.nearest(np.array([[-1e30, 0.0, 0.0]]), grid._points, 1.0)
    with pytest.raises(ValueError, match="21-bit"):
        g.TargetGrid(np.array([[np.nan, 0.0, 0.0]]), cell, 20)
    outside = np.vstack([np.zeros((4, 3)), [(LIMIT - 1) * g.NORMAL_RADIUS_M, 0.0, 0.0]])
    with pytest.raises(ValueError, match="21-bit"):
        g._normals(outside, g.NORMAL_RADIUS_M)


def reference_normals(points, radius):
    """guard._normals as #19 wrote it: the sums run pass by pass, offset-major and slot-minor."""
    normals = np.full((len(points), 3), np.nan)
    if not len(points):
        return normals
    grid = g.TargetGrid(points, radius, g.GRID_CAP)
    count, s, ss = np.zeros(len(points)), np.zeros((len(points), 3)), np.zeros((len(points), 6))
    base = np.floor(points / radius).astype(np.int64)
    for di, dj, dk in g._OFFSETS:
        keys = g.pack(base[:, 0] + di, base[:, 1] + dj, base[:, 2] + dk)
        at = np.minimum(np.searchsorted(grid.keys, keys), len(grid.keys) - 1)
        found = grid.keys[at] == keys
        starts = np.where(found, grid.start[at], 0)
        counts = np.where(found, grid.count[at], 0)
        for slot in range(int(counts.max(initial=0))):
            rows = np.nonzero(slot < counts)[0]
            other = points[grid.indices[starts[rows] + slot]]
            delta = points[rows] - other
            d2 = (delta[:, 0] * delta[:, 0] + delta[:, 1] * delta[:, 1]) + delta[:, 2] * delta[:, 2]
            near = d2 <= radius * radius
            rows, other = rows[near], other[near]
            count[rows] += 1.0
            s[rows] += other
            x, y, z = other[:, 0], other[:, 1], other[:, 2]
            ss[rows] += np.stack([x * x, x * y, x * z, y * y, y * z, z * z], axis=1)
    rows = np.nonzero(count >= g.NORMAL_MIN_POINTS)[0]
    m, c = s[rows] / count[rows, None], ss[rows] / count[rows, None]
    cov = np.empty((len(rows), 3, 3))
    for i, j, slot in ((0, 0, 0), (0, 1, 1), (0, 2, 2), (1, 1, 3), (1, 2, 4), (2, 2, 5)):
        cov[:, i, j] = cov[:, j, i] = c[:, slot] - m[:, i] * m[:, j]
    values, vectors = np.linalg.eigh(cov)
    planar = values[:, 0] <= g.NORMAL_PLANARITY * values[:, 1]
    normals[rows[planar]] = vectors[planar, :, 0]
    return normals


def test_normals_are_bit_identical_to_the_pass_by_pass_sums():
    """The normals' sums (pruned cells, all candidates of a chunk at once, then one pass per
    neighbour rank) add the same terms in the same order as the pass-by-pass scan: the same bytes
    on a ray-cast room, a corridor, and lattices full of points on cell faces."""
    rng = np.random.default_rng(11)
    builder = g.RegistrationBuilder(0.02)
    builder.add(rooms.scan(seed=3), 15.0)
    corridor = g.RegistrationBuilder(0.02)
    corridor.add(rooms.scan(room=rooms.CORRIDOR, boxes=(), seed=4)[::3], 15.0)
    clouds = [builder.points.astype(np.float64), corridor.points.astype(np.float64),
              rng.integers(-6, 6, size=(3000, 3)) * 0.05, rng.integers(-4, 4, size=(2000, 3)) * 0.15,
              rng.normal(size=(2000, 3)) * 0.2, np.zeros((0, 3))]
    for points in clouds:
        expected = reference_normals(points, g.NORMAL_RADIUS_M)
        assert g._normals(points, g.NORMAL_RADIUS_M).tobytes() == expected.tobytes()


# MARK: Alignment


@pytest.mark.parametrize("truth", [
    rooms.rig(), rooms.rig(0.02), rooms.rig(0.0, 0.05), rooms.rig(0.1, 0.1), rooms.rig(dz=0.03),
    rooms.rig(yaw_deg=0.5), rooms.rig(yaw_deg=2.0), rooms.rig(yaw_deg=-5.0), rooms.rig(0.05, yaw_deg=2.0),
    rooms.rig(roll_deg=0.5), rooms.rig(roll_deg=3.0), rooms.rig(dz=0.03, pitch_deg=1.0),
], ids=lambda pose: f"t{np.round(pose.t, 2).tolist()}r{pose.angle_deg:.1f}")
def test_convergence_grid(furnished, truth):
    cloud, keys = furnished
    source = g.source_cloud(rooms.scan(truth, seed=99), 15.0, 0.10, 50_000)
    estimate = solve(source, cloud, keys)
    error_m, error_deg = rooms.pose_error(estimate.pose, truth)
    assert error_m <= 0.005 and error_deg <= 0.05
    assert estimate.inlier_frac > 0.95 and estimate.observe > 0.05 and estimate.rms_m < 0.015


def test_envelope_corner_converges_from_a_saturated_translation_seed(furnished):
    # pose_offset saturates at 4 voxels (0.2 m) per axis: that is the seed a 0.3 m push gets.
    cloud, keys = furnished
    truth = rooms.rig(0.3, 0.0, yaw_deg=10.0)
    source = g.source_cloud(rooms.scan(truth, seed=99), 15.0, 0.10, 50_000)
    estimate = solve(source, cloud, keys, shift=(0.2, 0.0, 0.0))
    error_m, error_deg = rooms.pose_error(estimate.pose, truth)
    assert error_m <= 0.005 and error_deg <= 0.05


def test_robust_to_a_person_and_a_moved_box(furnished):
    cloud, keys = furnished
    truth = rooms.rig(0.03, yaw_deg=1.0)
    clutter = rooms.FURNITURE[1:] + (((1.0, -0.5, -1.2), (1.5, -0.2, 0.5)), ((-0.5, 1.5, -1.2), (0.3, 2.3, -0.4)))
    source = g.source_cloud(rooms.scan(truth, boxes=clutter, seed=99), 15.0, 0.10, 50_000)
    estimate = solve(source, cloud, keys)
    error_m, error_deg = rooms.pose_error(estimate.pose, truth)
    assert error_m <= 0.005 and error_deg <= 0.05
    assert estimate.inlier_frac < 0.99


def test_point_to_plane_finish_removes_the_point_to_point_sampling_bias():
    """Documents why the last stages are point-to-plane: two samplings of the same noise-free room,
    no move. Point-to-point alone settles millimetres and a tenth of a degree off."""
    cloud, keys = reference_of([rooms.scan(seed=seed, noise_m=0.0) for seed in range(10)])
    source = g.source_cloud(rooms.scan(seed=99, noise_m=0.0), 15.0, 0.10, 50_000)
    pose = g.Pose()
    grid = cloud.grid(0.05)
    for _ in range(60):
        moved = pose.apply(source)
        match, d2 = grid.nearest(moved, cloud.points64, 0.05)
        used = match >= 0
        kappa2 = (0.05 / 3) ** 2
        pose = g.horn(moved[used], cloud.points64[match[used]], kappa2 / (kappa2 + d2[used]) ** 2).compose(pose)
    assert pose.distance_m > 0.001 or pose.angle_deg > 0.05
    estimate = g.align(source, cloud, g.Pose(), 0.0)
    assert estimate.pose.distance_m < 1e-4 and estimate.pose.angle_deg < 0.01


@pytest.mark.parametrize("room, boxes, ok", [
    (rooms.ROOM, rooms.FURNITURE, True), (rooms.ROOM, (), True),
    (rooms.CORRIDOR, (), False), (rooms.FLOOR_ONLY, (), False),
], ids=["furnished", "box room", "corridor", "floor"])
def test_observability_separates_degenerate_views(room, boxes, ok):
    cloud, _ = reference_of([rooms.scan(room=room, boxes=boxes, seed=seed) for seed in range(6)])
    source = g.source_cloud(rooms.scan(rooms.rig(0.03), room=room, boxes=boxes, seed=99), 15.0, 0.10, 50_000)
    estimate = g.align(source, cloud, g.Pose(), 0.0)
    if ok:
        assert estimate.observe > 0.05
    else:
        assert estimate.observe < 0.005
    if room is rooms.CORRIDOR:
        # The corridor slides along x: the weakest direction names it, and the step did not slide.
        assert abs(estimate.weakest[0]) > 0.95
        assert abs(estimate.pose.t[0]) < 0.01


def _fixture_points(name):
    data = (SYNTHETIC / name).read_bytes()
    return decode_pointcloud2(data).positions


@pytest.mark.skipif(not (SYNTHETIC / "rig_shifted.bin").exists(), reason=f"no synthetic fixtures at {SYNTHETIC}")
def test_rig_shifted_golden_is_a_5_cm_move():
    cloud, keys = reference_of([_fixture_points("base_s1.bin")])
    window = _fixture_points("rig_shifted.bin")
    config = Config(voxel_size=VOXEL, max_range=15.0, evidence_of_absence=True)
    # The engine's translation seed: pose_shift of the window's voxels against the reference's. It reads
    # about 2 mm here: both scans use one ray pattern, so the floor voxels overlap best unshifted.
    shift = pose_shift(occupancy_map(window, config).occupied, keys, VOXEL)
    estimate = solve(g.source_cloud(window, 15.0, 0.10, 50_000), cloud, keys, shift)
    assert estimate.pose.t[0] == pytest.approx(0.05, abs=0.005)
    assert abs(estimate.pose.t[1]) < 0.005 and abs(estimate.pose.t[2]) < 0.005
    assert estimate.pose.angle_deg < 0.05


# MARK: The engine with the guard on


REFERENCE_WINDOWS = 10
# Azimuths per scan for the tests of saving, loading and failing writes: what they check does not
# depend on the scan's density, and half the rays is half the work.
SPARSE = 360
# relearn_after_s for the tests that relearn: the wait itself is all a longer one would add.
RELEARN_AFTER_S = 5
CUBE = ((2.0, -0.4, -1.2), (2.4, 0.0, -0.8))


def guarded(state_dir=None, **changes):
    config = MemoryConfig(sensors=("mid360",), state_dir=None if state_dir is None else str(state_dir),
                          reference_windows=REFERENCE_WINDOWS, rig_guard=True)
    return RoomMemory(replace(config, **changes), Time(0, 0), GENERATION)


@dataclass
class Window:
    second: int
    events: list
    state: MemoryState
    rig: object  # RigPose or None


def drive(memory, poses, first, boxes=rooms.FURNITURE, imu=None, frames=1, room=rooms.ROOM, columns=720):
    """One window per pose, at seconds ``first``, ``first + 1``, ..., of ``frames`` scans each of
    ``columns`` azimuths; ``imu(second)`` gives the samples received during the second before that
    window closes."""
    trace = []
    for second, pose in enumerate(poses, first):
        for sample in (imu(second) if imu else ()):
            memory.add_imu(sample.to_cdr(), sample.stamp)
        for frame in range(frames):
            memory.add_points("mid360", rooms.scan(pose, room=room, boxes=boxes, seed=second * frames + frame,
                                                   columns=columns))
        stamp = Time(second, 0)
        events = memory.close_window(stamp)
        rig = memory.rig_pose(stamp)
        assert rig is None or rig.publisher_generation == memory.publisher_generation  # protocol §1.4
        trace.append(Window(second, events, memory.status(stamp, 1.0).state, rig))
    return trace


def outcomes(trace):
    """The RigPose outcomes in order, repeats collapsed."""
    out = []
    for window in trace:
        if window.rig is not None and (not out or out[-1] != window.rig.outcome):
            out.append(window.rig.outcome)
    return out


def candidates(trace):
    return [event for window in trace for event in window.events if event.state == EventState.CANDIDATE]


@pytest.fixture(scope="module")
def built_run():
    """A guarded engine without a state dir after REFERENCE_WINDOWS + 2 windows of the rig in place
    (built, then tracking), and their trace: the start several tests share. Read it through built()."""
    memory = guarded()
    return memory, drive(memory, [rooms.Pose()] * (REFERENCE_WINDOWS + 2), 1)


def built(built_run):
    """A copy of built_run's engine and trace, the same as building them anew."""
    memory, trace = built_run
    return copy.deepcopy(memory), list(trace)


@pytest.fixture(scope="module")
def moved_run(built_run):
    """Astra's repro: the reference, then a 10 cm + 2 degree push, then a cube placed in the room."""
    memory, trace = built(built_run)
    truth = rooms.rig(0.10, yaw_deg=2.0)
    trace += drive(memory, [truth] * 20, len(trace) + 1)
    pose_after_move = memory.channels["mid360"].pose
    # A few frames per window, as a 10 Hz lidar gives: one sparse fan makes the cube's voxels flicker
    # past the shape gate.
    trace += drive(memory, [truth] * 50, len(trace) + 1, boxes=rooms.FURNITURE + (CUBE,), frames=4)
    return memory, truth, trace, pose_after_move


def test_a_pushed_rig_realigns_without_false_events(moved_run):
    memory, truth, trace, pose = moved_run
    before_cube = trace[:REFERENCE_WINDOWS + 22]
    assert not [event for window in before_cube for event in window.events]
    assert MemoryState.RIG_MOVED in [window.state for window in before_cube]
    assert outcomes(before_cube)[-3:] == [RigOutcome.IN_PLACE, RigOutcome.MOVED, RigOutcome.REALIGNED]
    moved = next(window.rig for window in before_cube if window.rig and window.rig.outcome == RigOutcome.MOVED)
    assert moved.causes & CAUSE_POSE_OFFSET and moved.channel[0] == RigChannel.MOVED
    error_m, error_deg = rooms.pose_error(pose, truth)
    assert error_m <= 0.005 and error_deg <= 0.05
    assert memory.rig_outcome in (RigOutcome.REALIGNED, RigOutcome.IN_PLACE, RigOutcome.CHECKING)
    assert memory.map_epoch == 1  # re-alignment sends no map


def test_a_cube_placed_after_the_realignment_is_found(moved_run):
    _, _, trace, _ = moved_run
    [found] = candidates(trace)
    assert found.kind == EventKind.APPEARED
    center = [(low + high) / 2 for low, high in zip(*CUBE)]
    assert math.dist(found.centroid, center) <= 0.25


def test_without_the_guard_the_push_floods_candidates():
    memory = guarded(rig_guard=False)
    drive(memory, [rooms.Pose()] * (REFERENCE_WINDOWS + 2), 1)
    trace = drive(memory, [rooms.rig(0.10, yaw_deg=2.0)] * 45, REFERENCE_WINDOWS + 3)
    assert candidates(trace)
    assert all(window.rig is None for window in trace)


def test_a_yaw_alone_realigns_before_the_candidate_gate(built_run):
    memory, _ = built(built_run)
    truth = rooms.rig(yaw_deg=3.0)
    trace = drive(memory, [truth] * 45, REFERENCE_WINDOWS + 3)
    assert not candidates(trace)
    assert RigOutcome.REALIGNED in outcomes(trace)
    error_m, error_deg = rooms.pose_error(memory.channels["mid360"].pose, truth)
    assert error_m <= 0.005 and error_deg <= 0.05


def test_a_static_rig_checks_periodically_and_stays_in_place(built_run):
    memory, trace = built(built_run)
    trace += drive(memory, [rooms.Pose()] * 68, REFERENCE_WINDOWS + 3)  # REFERENCE_WINDOWS + 70 windows in all
    assert not [event for window in trace for event in window.events]
    assert {window.state for window in trace} == {MemoryState.BUILDING, MemoryState.TRACKING}
    assert set(outcomes(trace)) == {RigOutcome.IN_PLACE, RigOutcome.CHECKING}
    # Ten synthetic reference windows read a few centimetres of pose_offset, so that trigger fires too.
    checks = [w.rig for w in trace if w.rig and w.rig.outcome == RigOutcome.CHECKING]
    assert any(rig.causes & CAUSE_PERIODIC for rig in checks)
    assert memory.channels["mid360"].pose.is_identity


def imu_stream(shock_at=None, yaw_deg=0.0, rate=50, roll_deg=0.0):
    """Quiet samples at ``rate`` Hz, and with ``shock_at`` a 0.2 s jolt that turns the sensor by
    ``yaw_deg`` about its z axis, or rolls it by ``roll_deg`` about x so gravity reads tilted after it."""
    yaw_rate = math.radians(yaw_deg) / 0.2
    roll_rate = math.radians(roll_deg) / 0.2
    rolled = (0.0, -math.sin(math.radians(roll_deg)), -math.cos(math.radians(roll_deg)))

    def samples(second):
        out = []
        for n in range(rate):
            t = (second - 1) + n / rate
            a, w = (0.0, 0.0, -1.0), (0.0, 0.0, 0.0)
            if shock_at is not None and shock_at <= t < shock_at + 0.2:
                a, w = (0.3, 0.0, -1.2), (roll_rate, 0.0, yaw_rate)
            elif shock_at is not None and t >= shock_at + 0.2:
                a = rolled
            ns = int(round(t * 1e9))
            out.append(ImuSample(Time(ns // 1_000_000_000, ns % 1_000_000_000), "livox_frame", w, a))
        return out
    return samples


def test_an_imu_shock_with_a_move_realigns_from_the_gyro_prior():
    memory = guarded()
    truth = rooms.rig(0.05, yaw_deg=2.0)
    shock = REFERENCE_WINDOWS + 2.5
    imu = imu_stream(shock, 2.0)
    poses = [rooms.Pose()] * (REFERENCE_WINDOWS + 3) + [truth] * 20
    trace = drive(memory, poses, 1, imu=imu)
    assert not [event for window in trace for event in window.events]
    assert outcomes(trace)[-3:] == [RigOutcome.CHECKING, RigOutcome.MOVED, RigOutcome.REALIGNED]
    checking = next(w.rig for w in trace if w.rig and w.rig.outcome == RigOutcome.CHECKING)
    assert checking.causes & CAUSE_IMU_SHOCK and checking.imu_ok
    error_m, error_deg = rooms.pose_error(memory.channels["mid360"].pose, truth)
    assert error_m <= 0.005 and error_deg <= 0.05


def test_an_imu_shock_alone_checks_and_stays_in_place():
    memory = guarded()
    trace = drive(memory, [rooms.Pose()] * (REFERENCE_WINDOWS + 15), 1, imu=imu_stream(REFERENCE_WINDOWS + 2.5))
    assert outcomes(trace)[-2:] == [RigOutcome.CHECKING, RigOutcome.IN_PLACE]
    assert MemoryState.RIG_MOVED not in [window.state for window in trace]
    assert memory.channels["mid360"].pose.is_identity


def test_the_gravity_baseline_is_set_while_building(tmp_path):
    """The IMU streams through BUILDING, so the new frame starts with a gravity baseline."""
    memory = guarded(tmp_path)
    drive(memory, [rooms.Pose()] * REFERENCE_WINDOWS, 1, imu=imu_stream(), columns=SPARSE)
    assert memory.state == MemoryState.TRACKING
    state = rig_files.parse_rig_state((tmp_path / "lab" / "rig_state").read_bytes(), "rig_state")
    assert state.g_base is not None and math.isclose(state.g_base[2], -1.0)


def test_an_imu_tilt_is_move_evidence_where_icp_cannot_register():
    """Codex's repro: a planar reference with the IMU streaming throughout, then a 3 degree roll with
    a gyro shock. ICP rejects the floor, so the tilt alone must give RIG_MOVED, not INCONCLUSIVE."""
    memory = guarded()
    shock = REFERENCE_WINDOWS + 2.5
    tilted = rooms.rig(roll_deg=3.0)
    poses = [rooms.Pose()] * (REFERENCE_WINDOWS + 3) + [tilted] * 20
    trace = drive(memory, poses, 1, boxes=(), room=rooms.FLOOR_ONLY, imu=imu_stream(shock, roll_deg=3.0))
    assert MemoryState.RIG_MOVED in [window.state for window in trace]
    assert RigOutcome.MOVED in outcomes(trace) and RigOutcome.INCONCLUSIVE not in outcomes(trace)
    assert any(w.rig.causes & CAUSE_IMU_TILT for w in trace if w.rig and w.rig.outcome == RigOutcome.MOVED)


def test_an_imu_that_comes_up_after_the_build_starts_the_gravity_baseline(tmp_path):
    """The IMU is silent through BUILDING, so the frame starts without a baseline: quiet gravity in
    place starts it (and saves it), and a tilt on a floor ICP cannot register still gives RIG_MOVED."""
    memory = guarded(tmp_path)
    stream = imu_stream(REFERENCE_WINDOWS + 6.5, roll_deg=3.0)
    late = lambda second: stream(second) if second > REFERENCE_WINDOWS else ()
    drive(memory, [rooms.Pose()] * REFERENCE_WINDOWS, 1, boxes=(), room=rooms.FLOOR_ONLY, imu=late)
    assert memory.state == MemoryState.TRACKING and memory._g_base is None
    trace = drive(memory, [rooms.Pose()] * 6, REFERENCE_WINDOWS + 1, boxes=(), room=rooms.FLOOR_ONLY, imu=late)
    state = rig_files.parse_rig_state((tmp_path / "lab" / "rig_state").read_bytes(), "rig_state")
    assert state.g_base is not None and math.isclose(state.g_base[2], -1.0)
    trace += drive(memory, [rooms.rig(roll_deg=3.0)] * 20, REFERENCE_WINDOWS + 7, boxes=(), room=rooms.FLOOR_ONLY,
                   imu=late)
    assert MemoryState.RIG_MOVED in [window.state for window in trace]
    assert RigOutcome.MOVED in outcomes(trace) and RigOutcome.INCONCLUSIVE not in outcomes(trace)
    assert any(w.rig.causes & CAUSE_IMU_TILT for w in trace if w.rig and w.rig.outcome == RigOutcome.MOVED)


def test_a_shock_while_building_does_not_start_a_check():
    memory = guarded()
    trace = drive(memory, [rooms.Pose()] * (REFERENCE_WINDOWS + 5), 1, imu=imu_stream(REFERENCE_WINDOWS / 2))
    assert not any(window.rig.causes & CAUSE_IMU_SHOCK for window in trace if window.rig)


def test_a_sensor_lost_after_a_move_does_not_block_the_relearn(tmp_path):
    """Codex's repro: both sensors see a move outside the envelope, then the JT128 drops. The
    Mid-360 relearns on its own at relearn_after_s and tracks again; the JT128 builds its reference
    when it returns, and only then is the new frame committed. The move rolls the rig 3 degrees under a
    streaming IMU, so the tilt confirms it whatever ICP makes of a pose it cannot reach. The wait is
    shortened to RELEARN_AFTER_S; the default 60 s is the test below."""
    memory = guarded(tmp_path, sensors=("mid360", "jt128"), relearn_after_s=float(RELEARN_AFTER_S))
    far = rooms.rig(0.5, yaw_deg=20.0, roll_deg=3.0)
    imu = imu_stream()
    trace = []

    def step(second, pose, sensors, boxes=rooms.FURNITURE, frames=1):
        for sample in imu(second):
            memory.add_imu(sample.to_cdr(), sample.stamp)
        for name in sensors:
            for frame in range(frames):
                memory.add_points(name, rooms.scan(pose, boxes=boxes, seed=second * frames + frame))
        stamp = Time(second, 0)
        events = memory.close_window(stamp)
        memory.map_update(stamp)
        trace.append(Window(second, events, memory.status(stamp, 1.0).state, memory.rig_pose(stamp)))

    both, mid = ("mid360", "jt128"), ("mid360",)
    second = 1
    # Until an accepted check gives both an inlier baseline: without one, gross mismatch is no evidence.
    while any(c.baseline is None for c in memory.channels.values()):
        step(second, rooms.Pose(), both)
        second += 1
        assert second < REFERENCE_WINDOWS + 60
    moving = second
    imu = imu_stream(moving - 0.5, roll_deg=3.0)
    old_epoch, old_frame = memory.map_epoch, memory._frame_epoch
    while not all(c.mode == "moved" for c in memory.channels.values()):
        step(second, far, both)
        second += 1
        assert second < moving + 20
    moved_at = second
    while memory.rig_outcome != RigOutcome.RELEARNING:
        step(second, far, mid)
        second += 1
        assert second <= moved_at + RELEARN_AFTER_S + 2, "the silent JT128 blocked the relearn"
    # The room rebuilds on the Mid-360 alone and tracks again; the JT128 waits unbuilt.
    relearned_at = second
    while memory.state != MemoryState.TRACKING:
        assert memory.rig_outcome == RigOutcome.RELEARNING
        step(second, far, mid)
        second += 1
        assert second <= relearned_at + REFERENCE_WINDOWS + 1
    jt128 = memory.channels["jt128"]
    assert memory.channels["mid360"].cloud is not None and jt128.cloud is None
    assert memory.map_epoch > old_epoch
    # Nothing commits a frame without the JT128: the files on disk are still the old frame's.
    state = rig_files.parse_rig_state((tmp_path / "lab" / "rig_state").read_bytes(), "rig_state")
    assert state.frame_epoch == old_frame
    # A restart now loads that old frame whole and checks it.
    shutil.copytree(tmp_path / "lab", tmp_path / "copy" / "lab")
    restarted = guarded(tmp_path / "copy", sensors=("mid360", "jt128"))
    assert restarted.state == MemoryState.TRACKING and restarted.rig_outcome == RigOutcome.CHECKING
    assert restarted._frame_epoch == old_frame and restarted.channels["jt128"].cloud is not None
    start = len(trace)
    for _ in range(40):
        step(second, far, mid, boxes=rooms.FURNITURE + (CUBE,), frames=4)
        second += 1
    after = trace[start:]
    assert {window.state for window in after} == {MemoryState.DEGRADED}
    [found] = [event for window in after for event in window.events if event.state == EventState.CANDIDATE]
    cube = [(low + high) / 2 for low, high in zip(*CUBE)]
    assert found.sensors == SENSOR_MID360
    assert math.dist(found.centroid, far.inverse().apply(np.array([cube]))[0]) <= 0.25
    assert not any(window.rig.sensors & SENSOR_JT128 for window in after if window.rig)
    # The JT128 returns: it builds its reference in the new frame, which is then committed.
    for _ in range(REFERENCE_WINDOWS):
        assert jt128.cloud is None
        step(second, far, both, boxes=rooms.FURNITURE + (CUBE,))
        second += 1
    assert jt128.cloud is not None
    step(second, far, both, boxes=rooms.FURNITURE + (CUBE,))
    assert trace[-1].state == MemoryState.TRACKING
    assert jt128.available and jt128.mode == "normal" and jt128.pose.is_identity
    state = rig_files.parse_rig_state((tmp_path / "lab" / "rig_state").read_bytes(), "rig_state")
    assert state.frame_epoch == memory._frame_epoch > old_frame
    restarted = guarded(tmp_path, sensors=("mid360", "jt128"))
    assert restarted.state == MemoryState.TRACKING
    assert len(restarted.channels["jt128"].cloud) == len(jt128.cloud)


def test_a_sensor_lost_after_a_move_relearns_at_the_default_60s():
    """The default relearn_after_s is 60 s. Both sensors are moved out of reach on sparse scans,
    then the JT128 sends nothing: a silent sensor must not hold the relearn, and a shortened
    override must not be what makes this pass. The wait itself is a 16-ray fan. The clock does not
    need a fit (fewer than MIN_SOURCE_POINTS never reaches ICP), and one return keeps the Mid-360
    from going silent too."""
    memory = guarded(sensors=("mid360", "jt128"))
    assert memory.config.relearn_after_s == 60.0
    far = rooms.rig(0.5, yaw_deg=20.0, roll_deg=3.0)
    imu = imu_stream()
    second = 1
    # Fixed fan: no per-frame azimuth jitter, and small enough that the wait does not run ICP.
    wait_fan = rooms.fan(rows=2, columns=8)

    def step(stamp, pose, sensors, columns=SPARSE, directions=None):
        for sample in imu(stamp):
            memory.add_imu(sample.to_cdr(), sample.stamp)
        for name in sensors:
            memory.add_points(name, rooms.scan(pose, seed=stamp, columns=columns, directions=directions))
        memory.close_window(Time(stamp, 0))

    both, mid = ("mid360", "jt128"), ("mid360",)
    while any(channel.baseline is None for channel in memory.channels.values()):
        step(second, rooms.Pose(), both)
        second += 1
        assert second < REFERENCE_WINDOWS + 60
    moving = second
    imu = imu_stream(moving - 0.5, roll_deg=3.0)
    while not all(channel.mode == "moved" for channel in memory.channels.values()):
        step(second, far, both)
        second += 1
        assert second < moving + 20
    moved_at = second
    step(second, far, mid, directions=wait_fan)
    second += 1
    assert memory.channels["jt128"].mode == "stale" and memory.channels["mid360"].mode == "moved"
    assert memory.rig_outcome != RigOutcome.RELEARNING
    while memory.rig_outcome != RigOutcome.RELEARNING:
        # Still moved on the way in. The window that relearns resets every channel to normal.
        assert memory.channels["jt128"].mode == "stale" and memory.channels["mid360"].mode == "moved"
        step(second, far, mid, directions=wait_fan)
        second += 1
        assert second <= moved_at + 62, "the silent JT128 blocked the relearn"
    assert second - moved_at >= 60
    assert memory.state == MemoryState.BUILDING
    assert memory.channels["mid360"].builder is not None and memory.channels["jt128"].cloud is None


def test_a_sensor_silent_through_a_realignment_realigns_when_it_returns():
    """Astra's repro: the JT128 is silent while the Mid-360 realigns, then returns to a moved rig."""
    memory = guarded(sensors=("mid360", "jt128"))
    truth = rooms.rig(0.10, yaw_deg=2.0)
    states = []

    def step(second, pose, sensors):
        for name in sensors:
            memory.add_points(name, rooms.scan(pose, seed=second))
        events = memory.close_window(Time(second, 0))
        memory.rig_pose(Time(second, 0))
        states.append(memory.status(Time(second, 0), 1.0).state)
        return events

    events = [step(s, rooms.Pose(), ("mid360", "jt128")) for s in range(1, REFERENCE_WINDOWS + 3)]
    events += [step(s, truth, ("mid360",)) for s in range(REFERENCE_WINDOWS + 3, REFERENCE_WINDOWS + 23)]
    assert memory.channels["mid360"].mode == "normal" and memory.channels["jt128"].mode == "stale"
    events += [step(s, truth, ("mid360", "jt128")) for s in range(REFERENCE_WINDOWS + 23, REFERENCE_WINDOWS + 43)]
    assert not [event for window in events for event in window]
    assert MemoryState.RIG_MOVED in states and states[-1] == MemoryState.TRACKING
    for name in ("mid360", "jt128"):
        assert memory.channels[name].mode == "normal"
        error_m, error_deg = rooms.pose_error(memory.channels[name].pose, truth)
        assert error_m <= 0.005 and error_deg <= 0.05


def test_a_sensor_still_building_when_the_rig_moves_starts_its_build_over():
    """The JT128 is silent through the build and returns to build its reference while the room
    tracks. The rig moves two windows into that build: the JT128 starts over in the new pose, so
    it joins with a reference of one pose and reports nothing on the unchanged room."""
    memory = guarded(sensors=("mid360", "jt128"))
    truth = rooms.rig(0.10, yaw_deg=2.0)
    jt128 = memory.channels["jt128"]
    trace, built = [], []

    def step(second, pose, sensors):
        for name in sensors:
            memory.add_points(name, rooms.scan(pose, seed=second))
        stamp = Time(second, 0)
        events = memory.close_window(stamp)
        trace.append(Window(second, events, memory.status(stamp, 1.0).state, memory.rig_pose(stamp)))
        built.append(jt128.reference_windows)

    both = ("mid360", "jt128")
    second = 1
    # Until an accepted check gives the Mid-360 an inlier baseline, so a fit can say MOVED.
    while memory.state != MemoryState.TRACKING or memory.channels["mid360"].baseline is None:
        step(second, rooms.Pose(), ("mid360",))
        second += 1
        assert second < REFERENCE_WINDOWS + 60
    assert jt128.builder is not None and jt128.reference_windows == 0
    for _ in range(2):
        step(second, rooms.Pose(), both)
        second += 1
    assert jt128.reference_windows == 2
    moving = len(trace)
    while jt128.cloud is None:
        step(second, truth, both)
        second += 1
        assert second < REFERENCE_WINDOWS + 100
    states = [window.state for window in trace[moving:]]
    assert MemoryState.RIG_MOVED in states
    # The move restarted the build: the count fell back instead of running on from 2.
    assert min(built[moving:]) < 2
    error_m, error_deg = rooms.pose_error(memory.channels["mid360"].pose, truth)
    assert error_m <= 0.005 and error_deg <= 0.05
    joined = len(trace)
    for _ in range(45):
        step(second, truth, both)
        second += 1
    assert {window.state for window in trace[joined:]} == {MemoryState.TRACKING}
    assert jt128.mode == "normal" and jt128.pose.is_identity
    assert not [event for window in trace[moving:] for event in window.events]


def test_close_window_times_the_guard_icp():
    """window_estimates and window_estimate_ms cover the guard's ICP in the last window only: none
    while building or between checks, and time in each window that checks."""
    memory = guarded(guard_check_period_s=5.0)
    counts = []
    for second in range(1, REFERENCE_WINDOWS + 13):
        drive(memory, [rooms.Pose()], second, columns=SPARSE)
        counts.append((memory.window_estimates, memory.window_estimate_ms))
    assert counts[:REFERENCE_WINDOWS - 1] == [(0, 0.0)] * (REFERENCE_WINDOWS - 1)
    checked = [(n, ms) for n, ms in counts if n]
    assert checked and all(ms > 0 for _, ms in checked)
    assert (0, 0.0) in counts[REFERENCE_WINDOWS:]  # a window between checks


def test_too_few_points_to_register_is_never_move_evidence(built_run):
    """Astra's repro: sparse windows, rejected for fewer than MIN_SOURCE_POINTS, after a baseline and
    with no IMU, end checks INCONCLUSIVE; they never enter RIG_MOVED or relearn."""
    memory, _ = built(built_run)
    drive(memory, [rooms.Pose()] * 8, REFERENCE_WINDOWS + 3)  # REFERENCE_WINDOWS + 10 windows in all
    assert memory.channels["mid360"].baseline is not None
    sparse = rooms.fan(rows=2, columns=8)
    trace = []
    for second in range(REFERENCE_WINDOWS + 11, REFERENCE_WINDOWS + 111):
        memory.add_points("mid360", rooms.scan(directions=sparse, seed=second))
        stamp = Time(second, 0)
        events = memory.close_window(stamp)
        trace.append(Window(second, events, memory.status(stamp, 1.0).state, memory.rig_pose(stamp)))
    assert {window.state for window in trace} == {MemoryState.TRACKING}
    assert RigOutcome.INCONCLUSIVE in outcomes(trace)
    assert RigOutcome.MOVED not in outcomes(trace) and RigOutcome.RELEARNING not in outcomes(trace)
    assert memory.channels["mid360"].pose.is_identity and memory.map_epoch == 1


def test_the_rig_state_round_trips_and_a_restart_checks_the_pose(tmp_path):
    memory = guarded(tmp_path)
    truth = rooms.rig(0.10, yaw_deg=2.0)
    drive(memory, [rooms.Pose()] * (REFERENCE_WINDOWS + 2), 1)
    drive(memory, [truth] * 10, REFERENCE_WINDOWS + 3)
    pose = memory.channels["mid360"].pose
    assert not pose.is_identity
    room = tmp_path / "lab"
    assert (room / "rig_state").exists() and (room / "registration-mid360.bin").exists()
    restarted = guarded(tmp_path)
    channel = restarted.channels["mid360"]
    assert channel.pose == pose and len(channel.cloud) == len(memory.channels["mid360"].cloud)
    assert restarted.rig_outcome == RigOutcome.CHECKING
    trace = drive(restarted, [truth] * 5, 1)
    assert outcomes(trace)[-1] == RigOutcome.IN_PLACE and not candidates(trace)
    assert trace[0].rig.causes == CAUSE_STARTUP


def test_a_frame_add_imu_cannot_decode_is_counted_never_raised(monkeypatch):
    """add_imu rejects what decode_imu cannot read, whatever it raises (the C++ engine catches every
    std::exception), and counts it; with the guard off it reads nothing."""
    memory = guarded()
    good = ImuSample(Time(1, 0), "livox_frame", (0.0, 0.0, 0.0), (0.0, 0.0, -1.0)).to_cdr()
    assert memory.add_imu(good, Time(1, 0)) and memory.imu_failed == 0
    assert not memory.add_imu(good[:40], Time(1, 0)) and memory.imu_failed == 1  # truncated: a CDRError

    def broken(data):
        raise OverflowError("not a CDRError")

    monkeypatch.setattr("room_memory.memory.decode_imu", broken)
    assert not memory.add_imu(good, Time(1, 0)) and memory.imu_failed == 2
    off = RoomMemory(MemoryConfig(sensors=("mid360",), state_dir=None), Time(0, 0), GENERATION)
    assert not off.add_imu(good, Time(1, 0)) and off.imu_failed == 0


def test_a_failed_rig_save_never_leaves_close_window_and_retries(tmp_path):
    """A rig_state write that fails (a directory where its temp file goes, as a full disk or a
    permission error would) is caught: the guard runs on from memory, the failure is counted, every
    window retries, and once a save goes through the state loads back."""
    memory = guarded(tmp_path)
    room = tmp_path / "lab"
    blocker = room / "rig_state.tmp"
    blocker.mkdir(parents=True)
    drive(memory, [rooms.Pose()] * (REFERENCE_WINDOWS + 3), 1, columns=SPARSE)
    assert memory.state == MemoryState.TRACKING and memory.rig_save_pending
    # The build's save, then a retry per window.
    assert memory.rig_save_failed >= 4 and "rig_state.tmp" in memory.rig_save_error
    assert (room / "registration-mid360.bin").exists() and not (room / "rig_state").exists()
    failed = memory.rig_save_failed
    blocker.rmdir()
    drive(memory, [rooms.Pose()], REFERENCE_WINDOWS + 4, columns=SPARSE)
    assert not memory.rig_save_pending and memory.rig_save_failed == failed
    restarted = guarded(tmp_path)
    assert len(restarted.channels["mid360"].cloud) == len(memory.channels["mid360"].cloud)
    assert restarted.rig_outcome == RigOutcome.CHECKING


def relearned(tmp_path, blocked):
    """A guarded engine with an inlier baseline, moved out of reach until it relearns; ``blocked``
    names files placed in the room just before the move. Returns the engine, a step function, the
    old frame and the old frame's files on disk at the relearn (map_epoch aside: it only moves on)."""
    memory = guarded(tmp_path, relearn_after_s=float(RELEARN_AFTER_S))
    far = rooms.rig(0.5, yaw_deg=20.0, roll_deg=3.0)
    imu = imu_stream()
    second = 1

    def step(pose=far):
        nonlocal second
        drive(memory, [pose], second, imu=imu, columns=SPARSE)
        second += 1

    # Until an accepted check gives the Mid-360 an inlier baseline.
    while memory.channels["mid360"].baseline is None:
        step(rooms.Pose())
        assert second < REFERENCE_WINDOWS + 60
    room = tmp_path / "lab"
    for name in blocked:
        (room / name).write_bytes(b"")
    old_frame = memory._frame_epoch
    imu = imu_stream(second - 0.5, roll_deg=3.0)  # the roll confirms the move whatever ICP makes of it
    while memory.rig_outcome != RigOutcome.RELEARNING:
        step()
        assert second < REFERENCE_WINDOWS + 100
    names = ["rig_state", "reference-mid360.cdr", "registration-mid360.bin"]
    return memory, step, old_frame, {name: (room / name).read_bytes() for name in names}


def rebuild(memory, step):
    for _ in range(3 * REFERENCE_WINDOWS):
        if memory.state == MemoryState.TRACKING:
            return
        step()
    assert memory.state == MemoryState.TRACKING


def test_a_failed_relearn_archive_holds_the_old_frame_until_it_goes_through(tmp_path):
    """The relearn archives the old frame under prev/ (D41); when that fails (here prev is a file)
    the relearn goes on, but the old frame's files stay as they are: the new frame commits only once
    the archive, retried every window, goes through."""
    memory, step, old_frame, old_files = relearned(tmp_path, ["prev"])
    room = tmp_path / "lab"
    assert memory.rig_save_pending and "prev" in memory.rig_save_error
    rebuild(memory, step)
    failed = memory.rig_save_failed
    step()
    assert memory.rig_save_pending and memory.rig_save_failed > failed
    assert memory._frame_epoch > old_frame
    assert {name: (room / name).read_bytes() for name in old_files} == old_files
    (room / "prev").unlink()
    step()
    assert not memory.rig_save_pending
    [archive] = (room / "prev").iterdir()
    assert archive.name.endswith(f"-frame-{old_frame}")
    assert {name: (archive / name).read_bytes() for name in old_files} == old_files
    assert rig_files.parse_rig_state((room / "rig_state").read_bytes(), "rig_state").frame_epoch == memory._frame_epoch


def test_a_failed_epoch_write_holds_the_rebuild_until_it_goes_through(tmp_path):
    """The rebuild's new epoch goes into the epoch file before anything carries it: when that write
    fails (here a directory where its temp file goes, as a full disk would) the build's end is held,
    so every window stays BUILDING under an epoch the file holds, nothing of the new frame reaches
    disk, and the next window retries. A start that cannot record its epoch stops: nothing carries
    that epoch yet."""
    memory, step, old_frame, old_files = relearned(tmp_path, [])
    room = tmp_path / "lab"
    assert not memory.rig_save_pending  # the archive went through
    (room / "map_epoch.tmp").mkdir()
    for _ in range(3 * REFERENCE_WINDOWS):
        step()
        assert memory.state == MemoryState.BUILDING and memory.map_update(Time(1000, 0)) is None
        assert memory.map_epoch <= load_epoch(room / "map_epoch")
    assert memory.epoch_held and not memory.rig_save_pending  # no save is pending: the node reports it from epoch_held
    assert memory.rig_save_failed >= 2 * REFERENCE_WINDOWS and "map_epoch.tmp" in memory.rig_save_error
    assert {name: (room / name).read_bytes() for name in old_files} == old_files
    with pytest.raises(RuntimeError, match="map_epoch.tmp"):
        guarded(tmp_path)
    (room / "map_epoch.tmp").rmdir()
    step()
    assert memory.state == MemoryState.TRACKING and not memory.rig_save_pending and not memory.epoch_held
    assert load_epoch(room / "map_epoch") == memory.map_epoch
    assert rig_files.parse_rig_state((room / "rig_state").read_bytes(), "rig_state").frame_epoch == memory._frame_epoch
    assert guarded(tmp_path).map_epoch == memory.map_epoch + 1


def test_a_failed_epoch_write_holds_a_settle_until_it_goes_through(tmp_path):
    """While tracking, voxels that settle take the next map_epoch. When the epoch file cannot record it
    the settle is held: the counts, the reference and map_epoch stay, no delta goes out, and the next
    window retries; once the write goes through the delta carries the epoch the file holds."""
    memory = guarded(tmp_path, learn_windows=30, delta_period_s=2.0, keyframe_period_s=1000.0,
                     keyframe_delta_ratio=100.0)
    room = tmp_path / "lab"

    def window(second, max_range_m):
        memory.add_points("mid360", rooms.scan(seed=second, max_range_m=max_range_m))
        memory.close_window(Time(second, 0))
        data = memory.map_update(Time(second, 0))
        return None if data is None else MapBricks.from_cdr(data)

    # The reference sees only what is within 4 m; the far walls are new coverage once tracking.
    second, opening = 1, None
    while opening is None:
        opening = window(second, 4.0)
        second += 1
        assert second < 3 * REFERENCE_WINDOWS
    assert opening.kind == MapKind.SNAPSHOT and opening.header.map_epoch == memory.map_epoch
    epoch, bricks = memory.map_epoch, memory.channels["mid360"].reference.to_bricks()
    (room / "map_epoch.tmp").mkdir()
    for second in range(second, second + 7):
        assert window(second, 30.0) is None
        assert memory.map_epoch == epoch == load_epoch(room / "map_epoch")
        assert memory.channels["mid360"].reference.to_bricks() == bricks
    assert len(memory.channels["mid360"].stats.keys) and "map_epoch.tmp" in memory.rig_save_error
    assert memory.epoch_held and not memory.rig_save_pending  # no save is pending: the node reports it from epoch_held
    failed = memory.rig_save_failed
    assert failed >= 2
    (room / "map_epoch.tmp").rmdir()
    delta = window(second + 1, 30.0)
    assert delta.kind == MapKind.DELTA and delta.header.map_epoch == epoch + 1 == load_epoch(room / "map_epoch")
    assert memory.rig_save_failed == failed and not memory.rig_save_pending and not memory.epoch_held
    assert len(memory.channels["mid360"].reference.to_bricks()) > len(bricks)


def test_a_moved_sensor_gone_silent_keeps_the_move(moved_window):
    """Codex's repro: a window without points right after a detected 10 cm + 2 degree push takes the
    sensor off line (stale), never the move: RIG_MOVED and MOVED until it realigns, and no check
    (here a periodic one falls due) starts over the unresolved move."""
    memory, truth, second = moved_window
    states, rigs = [], []
    for second in range(second, second + 3):
        stamp = Time(second, 0)
        memory.close_window(stamp)
        states.append(memory.status(stamp, 1.0).state)
        rigs.append(memory.rig_pose(stamp))
    assert memory.channels["mid360"].mode == "stale"
    assert states == [MemoryState.RIG_MOVED] * 3 and memory.rig_outcome == RigOutcome.MOVED
    assert rigs[0] is not None and rigs[0].outcome == RigOutcome.MOVED and memory._check is None
    trace = drive(memory, [truth] * 20, second + 1)
    assert RigOutcome.REALIGNED in outcomes(trace) and trace[-1].state == MemoryState.TRACKING
    error_m, error_deg = rooms.pose_error(memory.channels["mid360"].pose, truth)
    assert error_m <= 0.005 and error_deg <= 0.05


@pytest.fixture
def moved_window():
    """A single-sensor rig at the window its 10 cm + 2 degree push is detected (a periodic check
    every second), and the next second."""
    memory = guarded(guard_check_period_s=1.0)
    truth = rooms.rig(0.10, yaw_deg=2.0)
    drive(memory, [rooms.Pose()] * (REFERENCE_WINDOWS + 2), 1)
    second = REFERENCE_WINDOWS + 3
    while memory.channels["mid360"].mode != "moved":
        drive(memory, [truth], second)
        second += 1
        assert second < REFERENCE_WINDOWS + 20
    return memory, truth, second


@pytest.mark.skipif(
    os.geteuid() == 0,
    reason=(
        "Docker builds on the Spark run as root, permission-based fault injection cannot fail for root, "
        "so the test cannot pass there"
    ),
)
def test_a_failed_archive_leaves_no_partial_archive(tmp_path):
    """A copy that fails part way (an unreadable file, as a full disk would) removes the archive it
    started, so a retry takes the same number and pruning never keeps a partial copy over a whole one."""
    room = tmp_path / "lab"
    room.mkdir()
    (room / "rig_state").write_bytes(b"state")
    cloud = room / "registration-mid360.bin"
    cloud.write_bytes(b"cloud")
    files = ["rig_state", "registration-mid360.bin"]
    cloud.chmod(0)
    try:
        with pytest.raises(OSError):
            rig_files.archive(room, "frame-1", files)
    finally:
        cloud.chmod(0o644)
    assert list((room / "prev").iterdir()) == []
    target = rig_files.archive(room, "frame-1", files)
    assert target.name == "0001-frame-1" and (target / "registration-mid360.bin").read_bytes() == b"cloud"


def test_a_new_frame_cut_short_never_loads_with_the_old_frames_poses(tmp_path):
    """Codex's repro: the rebuilt frame's commit fails part way. With its registration cloud
    unwritable a restart loads the old frame whole; with rig_state unwritable it refuses the set.
    Never the new references with the old frame's cloud and pose. Once the writes go through, the
    new frame loads."""
    memory, step, old_frame, old_files = relearned(tmp_path, [])
    room = tmp_path / "lab"
    blocker = room / "registration-mid360.bin.tmp"
    blocker.mkdir()
    rebuild(memory, step)
    assert memory.rig_save_pending and "registration-mid360.bin.tmp" in memory.rig_save_error
    assert {name: (room / name).read_bytes() for name in old_files} == old_files
    old_reference = load_reference(room / "reference-mid360.cdr")[1].occupied
    assert not np.array_equal(old_reference, memory.channels["mid360"].reference.occupied)
    old_pose = rig_files.parse_rig_state(old_files["rig_state"], "rig_state").sensors["mid360"].pose
    restarted = guarded(tmp_path)
    assert restarted._frame_epoch == old_frame and restarted.channels["mid360"].pose == old_pose
    assert np.array_equal(restarted.channels["mid360"].reference.occupied, old_reference)
    blocker.rmdir()
    (room / "rig_state.tmp").mkdir()
    step()
    assert memory.rig_save_pending and "rig_state.tmp" in memory.rig_save_error
    with pytest.raises(ValueError, match="does not match"):
        guarded(tmp_path)
    (room / "rig_state.tmp").rmdir()
    step()
    assert not memory.rig_save_pending
    restarted = guarded(tmp_path)
    assert restarted._frame_epoch == memory._frame_epoch > old_frame
    assert len(restarted.channels["mid360"].cloud) == len(memory.channels["mid360"].cloud)


def test_a_rig_state_that_does_not_match_its_files_is_refused(tmp_path):
    memory = guarded(tmp_path)
    drive(memory, [rooms.Pose()] * REFERENCE_WINDOWS, 1, columns=SPARSE)
    cloud = tmp_path / "lab" / "registration-mid360.bin"
    data = bytearray(cloud.read_bytes())
    data[-5] ^= 1
    cloud.write_bytes(bytes(data))
    with pytest.raises(ValueError, match="does not match"):
        guarded(tmp_path)
    cloud.unlink()
    with pytest.raises(ValueError, match="does not match"):
        guarded(tmp_path)
    with pytest.raises(ValueError, match="rig_guard"):
        guarded(tmp_path, rig_guard=False)


def test_legacy_state_is_archived_and_rebuilt(tmp_path):
    drive(guarded(tmp_path, rig_guard=False), [rooms.Pose()] * REFERENCE_WINDOWS, 1, columns=SPARSE)
    memory = guarded(tmp_path)
    assert memory.state == MemoryState.BUILDING and memory.rig_outcome == RigOutcome.RELEARNING
    [archive] = (tmp_path / "lab" / "prev").iterdir()
    assert sorted(path.name for path in archive.iterdir()) == ["map_epoch", "reference-mid360.cdr"]
    assert not (tmp_path / "lab" / "reference-mid360.cdr").exists()
    drive(memory, [rooms.Pose()] * REFERENCE_WINDOWS, 1, columns=SPARSE)
    assert memory.state == MemoryState.TRACKING and (tmp_path / "lab" / "rig_state").exists()
    assert memory.map_epoch > 1


@pytest.mark.parametrize("changes", [
    dict(align_inlier_ratio=1.5), dict(align_stable_windows=0), dict(align_max_deg=91.0), dict(check_max_s=0.0),
    dict(guard_check_period_s=-1.0), dict(guard_check_period_s=34.0), dict(imu_quiet_s=math.nan),
])
def test_guard_config_rejections(changes):
    with pytest.raises(ValueError):
        guarded(**changes)
    guarded(rig_guard=False, **changes)  # only checked with the guard on
