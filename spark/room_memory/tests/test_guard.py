"""The rig guard's geometry (room_memory/guard.py): pose conventions, the binary32 transform, the
registration cloud, the solver pieces, and ICP on ray-cast rooms."""

import math
import pathlib
import warnings
from dataclasses import dataclass, replace

import numpy as np
import pytest

from room_memory import guard as g
from room_memory.core import Config, decode_pointcloud2, occupancy_map
from room_memory.memory import MemoryConfig, RoomMemory, pose_shift
from room_memory.messages import (CAUSE_IMU_SHOCK, CAUSE_PERIODIC, CAUSE_POSE_OFFSET, CAUSE_STARTUP, EventKind,
                                  EventState, ImuSample, MemoryState, RigChannel, RigOutcome, Time)

import rooms

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
CUBE = ((2.0, -0.4, -1.2), (2.4, 0.0, -0.8))


def guarded(state_dir=None, **changes):
    config = MemoryConfig(sensors=("mid360",), state_dir=None if state_dir is None else str(state_dir),
                          reference_windows=REFERENCE_WINDOWS, rig_guard=True)
    return RoomMemory(replace(config, **changes), Time(0, 0))


@dataclass
class Window:
    second: int
    events: list
    state: MemoryState
    rig: object  # RigPose or None


def drive(memory, poses, first, boxes=rooms.FURNITURE, imu=None, frames=1):
    """One window per pose, at seconds ``first``, ``first + 1``, ..., of ``frames`` scans each;
    ``imu(second)`` gives the samples received during the second before that window closes."""
    trace = []
    for second, pose in enumerate(poses, first):
        for sample in (imu(second) if imu else ()):
            memory.add_imu(sample.to_cdr(), sample.stamp)
        for frame in range(frames):
            memory.add_points("mid360", rooms.scan(pose, boxes=boxes, seed=second * frames + frame))
        stamp = Time(second, 0)
        events = memory.close_window(stamp)
        rig = memory.rig_pose(stamp)
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
def moved_run():
    """Astra's repro: the reference, then a 10 cm + 2 degree push, then a cube placed in the room."""
    memory = guarded()
    truth = rooms.rig(0.10, yaw_deg=2.0)
    trace = drive(memory, [rooms.Pose()] * (REFERENCE_WINDOWS + 2), 1)
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


def test_a_yaw_alone_realigns_before_the_candidate_gate():
    memory = guarded()
    truth = rooms.rig(yaw_deg=3.0)
    drive(memory, [rooms.Pose()] * (REFERENCE_WINDOWS + 2), 1)
    trace = drive(memory, [truth] * 45, REFERENCE_WINDOWS + 3)
    assert not candidates(trace)
    assert RigOutcome.REALIGNED in outcomes(trace)
    error_m, error_deg = rooms.pose_error(memory.channels["mid360"].pose, truth)
    assert error_m <= 0.005 and error_deg <= 0.05


def test_a_static_rig_checks_periodically_and_stays_in_place():
    memory = guarded()
    trace = drive(memory, [rooms.Pose()] * (REFERENCE_WINDOWS + 70), 1)
    assert not [event for window in trace for event in window.events]
    assert {window.state for window in trace} == {MemoryState.BUILDING, MemoryState.TRACKING}
    assert set(outcomes(trace)) == {RigOutcome.IN_PLACE, RigOutcome.CHECKING}
    # Ten synthetic reference windows read a few centimetres of pose_offset, so that trigger fires too.
    checks = [w.rig for w in trace if w.rig and w.rig.outcome == RigOutcome.CHECKING]
    assert any(rig.causes & CAUSE_PERIODIC for rig in checks)
    assert memory.channels["mid360"].pose.is_identity


def imu_stream(shock_at=None, yaw_deg=0.0, rate=50):
    """Quiet samples at ``rate`` Hz, and with ``shock_at`` a 0.2 s jolt that turns the sensor by
    ``yaw_deg`` about its z axis."""
    yaw_rate = math.radians(yaw_deg) / 0.2

    def samples(second):
        out = []
        for n in range(rate):
            t = (second - 1) + n / rate
            a, w = (0.0, 0.0, -1.0), (0.0, 0.0, 0.0)
            if shock_at is not None and shock_at <= t < shock_at + 0.2:
                a, w = (0.3, 0.0, -1.2), (0.0, 0.0, yaw_rate)
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


def test_a_rig_state_that_does_not_match_its_files_is_refused(tmp_path):
    memory = guarded(tmp_path)
    drive(memory, [rooms.Pose()] * REFERENCE_WINDOWS, 1)
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
    drive(guarded(tmp_path, rig_guard=False), [rooms.Pose()] * REFERENCE_WINDOWS, 1)
    memory = guarded(tmp_path)
    assert memory.state == MemoryState.BUILDING and memory.rig_outcome == RigOutcome.RELEARNING
    [archive] = (tmp_path / "lab" / "prev").iterdir()
    assert sorted(path.name for path in archive.iterdir()) == ["map_epoch", "reference-mid360.cdr"]
    assert not (tmp_path / "lab" / "reference-mid360.cdr").exists()
    drive(memory, [rooms.Pose()] * REFERENCE_WINDOWS, 1)
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
