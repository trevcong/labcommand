"""Core parity and behavior: Swift goldens byte for byte, spec fixtures, determinism, timing."""

import os
import pathlib
import time

import numpy as np
import pytest

import goldens
import scenes
from room_memory.core import (
    Config,
    InvalidConfig,
    VoxelSizeMismatch,
    compare,
    decode_pointcloud2,
    diff,
    occupancy_map,
    to_json,
)
from room_memory.core import change_detection
from room_memory.core.change_detection import LIMIT, contains, floor_index, pack, ray_endpoints, unpack
from room_memory.offline import parse_args

CD_FIXTURES = pathlib.Path(
    os.environ.get("ROOM_MEMORY_CD_FIXTURES", goldens.HERE.parents[2] / "Tests" / "Fixtures" / "change-detection")
)
SNAPSHOTS = pathlib.Path(
    os.environ.get("ROOM_MEMORY_SNAPSHOTS", goldens.HERE.parents[2] / "Tests" / "Fixtures" / "room" / "snapshots")
)

CLEAR_LINE_GOLDEN = (
    '{"schema":"labcommand.change-detection.v2","evidence":true,"report_unobserved":false,'
    '"origin_bits":["00000000","00000000","00000000"],"ray_stride":1,"config":{"voxel_size_bits":"3d4ccccd",'
    '"max_range_bits":null,"min_points":1,"tolerance":0,"min_voxels":6,"min_extent":1,"pair_moves":true,'
    '"max_move_bits":"3fc00000","move_count_ratio_bits":"40000000"},"before":{"points":9,"accepted":9,'
    '"dropped":0,"voxels":9},"after":{"points":1,"accepted":1,"dropped":0,"voxels":1},"appeared_voxels":0,'
    '"disappeared_voxels":8,"unknown_voxels":0,"regions":[{"kind":"disappeared","voxel_count":8,'
    '"min_key":"4611776178381914112","centroid_bits":["3f99999a","3ccccccd","3ccccccd"],'
    '"bounds_min_bits":["3f800000","00000000","00000000"],"bounds_max_bits":["3fb33333","3d4ccccd","3d4ccccd"],'
    '"volume_bits":"3a83126f"}]}'
)


def evidence(**changes):
    """The spec's evidence-test config: column fixtures are one cell thick, so ``min_extent`` 1."""
    values = dict(evidence_of_absence=True, tolerance=0, min_extent=1)
    values.update(changes)
    return Config(**values)


def record(before, after, config):
    before_map, after_map, report = compare(before, after, config)
    return report, to_json(report, before_map, after_map, config)


def report_record(report):
    """Every JSON-visible field of a report, binary32 values as bit patterns, so NaN-free equality is exact."""
    def bits(values):
        return None if values is None else np.asarray(values, dtype=np.float32).view(np.uint32).tolist()
    regions = tuple(
        (r.kind, r.voxel_count, r.min_key, bits(r.centroid), bits(r.bounds_min), bits(r.bounds_max), bits(r.volume),
         r.from_voxel_count, r.from_min_key, bits(r.from_centroid), bits(r.from_bounds_min), bits(r.from_bounds_max))
        for r in report.regions
    )
    return (regions, report.appeared_voxels, report.disappeared_voxels, report.unknown_voxels,
            report.before_voxels, report.after_voxels)


def config_for(flags):
    return parse_args(flags + ["before.bin", "after.bin"])[0]


def keys_of(points, config=Config()):
    return occupancy_map(points, config).occupied


# MARK: Goldens


@pytest.mark.parametrize("sensor", ["jt128", "mid360"])
def test_snapshot_golden_bytes(sensor):
    before = decode_pointcloud2((SNAPSHOTS / "a" / f"{sensor}.bin").read_bytes()).positions
    after = decode_pointcloud2((SNAPSHOTS / "b" / f"{sensor}.bin").read_bytes()).positions
    _, text = record(before, after, Config(evidence_of_absence=True))
    assert (text + "\n").encode() == (CD_FIXTURES / f"{sensor}.json").read_bytes()


@pytest.mark.parametrize("name", sorted(goldens.SCENES))
def test_swift_scene_golden_bytes(name):
    build, flags = goldens.SCENES[name]
    before, after = build()
    _, text = record(before, after, config_for(flags))
    assert (text + "\n").encode() == (goldens.SWIFT_DIR / f"{name}.json").read_bytes()


PHASE0 = goldens.phase0_pairs() if goldens.PHASE0_DIR.is_dir() else []


@pytest.mark.skipif(not PHASE0, reason=f"Phase 0 fixtures not found at {goldens.PHASE0_DIR}")
@pytest.mark.parametrize("name,before_file,after_file", PHASE0)
def test_phase0_golden_bytes(name, before_file, after_file):
    before = decode_pointcloud2((goldens.PHASE0_DIR / before_file).read_bytes()).positions
    after = decode_pointcloud2((goldens.PHASE0_DIR / after_file).read_bytes()).positions
    _, text = record(before, after, config_for(goldens.EVIDENCE))
    assert (text + "\n").encode() == (goldens.SWIFT_DIR / f"{name}.json").read_bytes()


def test_every_stored_swift_golden_has_a_case():
    stored = {path.stem for path in goldens.SWIFT_DIR.glob("*.json")}
    assert {name for name in stored if not name.startswith("phase0-")} == set(goldens.SCENES)
    assert {name for name, _, _ in PHASE0} <= stored


@pytest.mark.parametrize("name", ["indoor-moved", "board-occluded", "jitter", "boundary-endpoint", "phase0-box_moved"])
def test_sparse_membership_matches_the_dense_grid(name, monkeypatch):
    monkeypatch.setattr(change_detection._Membership, "DENSE_CELLS", 0)
    if name.startswith("phase0-"):
        if not PHASE0:
            pytest.skip(f"Phase 0 fixtures not found at {goldens.PHASE0_DIR}")
        _, before_file, after_file = next(pair for pair in PHASE0 if pair[0] == name)
        before = decode_pointcloud2((goldens.PHASE0_DIR / before_file).read_bytes()).positions
        after = decode_pointcloud2((goldens.PHASE0_DIR / after_file).read_bytes()).positions
        flags = goldens.EVIDENCE
    else:
        build, flags = goldens.SCENES[name]
        before, after = build()
    _, text = record(before, after, config_for(flags))
    assert (text + "\n").encode() == (goldens.SWIFT_DIR / f"{name}.json").read_bytes()


def test_clear_line_is_the_spec_literal():
    report, text = record(scenes.column(list(range(20, 28)) + [40]), scenes.column([40]), evidence())
    assert text == CLEAR_LINE_GOLDEN
    assert report.regions[0].min_key == 4611776178381914112


def test_pointcloud2_round_trip_and_rejections():
    points = np.array([[0.5, -1.25, 2.0], [np.nan, 0, 0], [1e3, 2e3, -3e3]], dtype=np.float32)
    cloud = decode_pointcloud2(scenes.encode_pointcloud2(points, frame_id="lidar"))
    assert cloud.frame_id == "lidar"
    assert np.array_equal(cloud.positions, points[[0, 2]])
    data = scenes.encode_pointcloud2(points)
    for length in range(len(data) - 1):
        with pytest.raises(ValueError):
            decode_pointcloud2(data[:length])
    with pytest.raises(ValueError, match="unsupported encapsulation"):
        decode_pointcloud2(b"\x00\x07" + data[2:])


# MARK: Keys and occupancy


def test_key_packing():
    samples = [(0, 0, 0), (1, -2, 3), (LIMIT - 1, 0, -LIMIT), (-LIMIT, LIMIT - 1, 4), (-LIMIT, -LIMIT, -LIMIT)]
    i, j, k = (np.array(axis) for axis in zip(*samples))
    keys = pack(i, j, k)
    assert (keys >= 0).all()
    assert [tuple(map(int, triple)) for triple in zip(*unpack(keys))] == samples
    assert np.array_equal(np.argsort(keys), np.lexsort((k, j, i)))
    assert int(pack(20, 0, 0)) == 4611776178381914112


def test_floor_quantizer():
    inverse = np.float32(1) / np.float32(0.05)
    q = np.array([-0.01, 0.06, -0.0, -0.05, 0.05], dtype=np.float32) * inverse
    assert floor_index(q).tolist() == [-1, 1, 0, -1, 1]
    assert floor_index(np.array([0.06], dtype=np.float32) * (np.float32(1) / np.float32(0.1))).tolist() == [0]


def test_drops_non_finite_far_and_out_of_limit():
    cloud = np.array([[np.nan, 0, 0], [0.06, -0.01, 0], [0, np.inf, 1], [60000, 0, 0]], dtype=np.float32)
    occupancy = occupancy_map(cloud, Config())
    assert (occupancy.accepted_points, occupancy.dropped_points) == (1, 3)
    assert occupancy.occupied.tolist() == [int(pack(1, -1, 0))]
    near = np.array([[14.999, 0, 0], [15.001, 0, 0]], dtype=np.float32)
    clipped = occupancy_map(near, Config(max_range=15))
    assert (clipped.accepted_points, clipped.dropped_points, len(clipped.occupied)) == (1, 1, 1)
    assert len(occupancy_map(near, Config()).occupied) == 2


def test_representative_return_is_the_binary32_order_minimum():
    points = np.array([
        [0.02, 0.0, 0.0], [0.01, 0.03, 0.04], [0.01, 0.02, 0.045], [0.01, 0.02, 0.041],
        [0.0, 0.26, 0.26], [-0.0, 0.26, 0.26], [-0.01, 0.26, 0.26],
    ], dtype=np.float32)
    for order in ([0, 1, 2, 3, 4, 5, 6], [5, 3, 6, 1, 4, 0, 2]):
        occupancy = occupancy_map(points[order], Config(evidence_of_absence=True))
        assert occupancy.occupied.tolist() == [int(pack(-1, 5, 5)), int(pack(0, 0, 0)), int(pack(0, 5, 5))]
        bits = occupancy.endpoints.view(np.uint32).tolist()
        assert bits == points[[6, 3, 5]].view(np.uint32).tolist()


def test_min_points():
    piled = np.array([[0.01, 0.01, 0.01], [0.02, 0.01, 0.01], [0.01, 0.02, 0.01], [0.01, 0.01, 0.02], [0.06, 0.01, 0.01]],
                     dtype=np.float32)
    kept = occupancy_map(piled, Config(min_points=2))
    assert len(kept.occupied) == 1 and kept.accepted_points == 5


@pytest.mark.parametrize("changes", [
    dict(voxel_size=0), dict(max_range=-1), dict(min_points=0), dict(min_voxels=0), dict(min_extent=0),
    dict(tolerance=5),
    dict(max_move=float("nan")), dict(move_count_ratio=0.5), dict(ray_stride=3), dict(origin=(np.nan, 0, 0)),
    dict(evidence_of_absence=True, origin=(80000, 0, 0)),
])
def test_invalid_config(changes):
    with pytest.raises(InvalidConfig):
        occupancy_map(np.zeros((0, 3), dtype=np.float32), Config(**changes))


def test_voxel_size_mismatch_and_missing_endpoints():
    room = scenes.room_shell()
    fine = occupancy_map(room, Config())
    coarse = occupancy_map(room, Config(voxel_size=0.10))
    with pytest.raises(VoxelSizeMismatch):
        diff(fine, coarse, Config())
    with pytest.raises(InvalidConfig, match="evidence requires"):
        diff(fine, fine, Config(evidence_of_absence=True))


# MARK: Synthetic scenes


def expect_box(region, kind, center):
    assert region.kind == kind and region.voxel_count > 6
    assert np.all(np.abs(region.centroid - np.array(center, dtype=np.float32)) <= 0.025)
    assert region.volume == np.float32(region.voxel_count) * np.float32(0.05) * np.float32(0.05) * np.float32(0.05)
    span = region.bounds_max - region.bounds_min
    assert np.all(span >= 0.399) and np.all(span < 0.70)


@pytest.mark.parametrize("config", [Config(), Config(evidence_of_absence=True)], ids=["v1", "evidence"])
def test_box_added_removed_moved(config):
    room = scenes.room_shell()
    box = scenes.box_shell((1.0, 0.5, 0.30))
    added = compare(room, np.concatenate([room, box]), config)[2]
    assert len(added.regions) == 1 and added.disappeared_voxels == 0
    expect_box(added.regions[0], "appeared", (1.0, 0.5, 0.30))

    removed = compare(np.concatenate([room, box]), room, config)[2]
    assert len(removed.regions) == 1 and removed.appeared_voxels == 0
    expect_box(removed.regions[0], "disappeared", (1.0, 0.5, 0.30))

    start = scenes.box_shell((0.0, 0.5, 0.30))
    moved = compare(np.concatenate([room, start]), np.concatenate([room, scenes.shifted(start, 1)]), config)[2]
    assert [region.kind for region in moved.regions] == ["moved"]
    region = moved.regions[0]
    expect_box(region, "moved", (1.0, 0.5, 0.30))
    if not config.evidence_of_absence:
        assert region.from_voxel_count == region.voxel_count
        assert np.all(np.abs(region.from_centroid - np.array([0.0, 0.5, 0.3], dtype=np.float32)) <= 0.025)


def test_box_removed_without_line_of_sight_is_unknown():
    before, after, front, back = scenes.board_scene()
    config = Config(evidence_of_absence=True, report_unobserved=True)
    report = compare(before, after, config)[2]
    back_keys = keys_of(back)
    front_keys = keys_of(front)
    disappeared = np.sort(np.concatenate([r.keys for r in report.regions if r.kind == "disappeared"]))
    unobserved = np.sort(np.concatenate([r.keys for r in report.regions if r.kind == "unobserved"]))
    assert not contains(disappeared, back_keys).any()
    assert contains(unobserved, back_keys).all()
    assert report.unknown_voxels >= len(back_keys)
    assert contains(disappeared, front_keys).mean() > 0.95
    assert not any(r.kind == "moved" for r in report.regions)


def test_occluded_column():
    before = scenes.column(list(range(8, 17)) + list(range(20, 28)))
    report = compare(before, scenes.column([16]), evidence())[2]
    assert (report.appeared_voxels, report.disappeared_voxels, report.unknown_voxels) == (0, 8, 8)
    assert [(r.kind, r.voxel_count) for r in report.regions] == [("disappeared", 8)]
    assert report.regions[0].bounds_min[0] == np.float32(8) * np.float32(0.05)
    full = compare(before, scenes.column([16]), evidence(report_unobserved=True))[2]
    assert [(r.kind, r.voxel_count) for r in full.regions] == [("disappeared", 8), ("unobserved", 8)]
    assert full.regions[1].bounds_min[0] == np.float32(20) * np.float32(0.05)


def test_diagonal_tie_steps_all_axes():
    cells = [(0, 0, 0), (1, 1, 1), (2, 2, 2), (1, 0, 0), (3, 3, 3)]
    before = np.array([scenes.cell_point(*cell) for cell in cells], dtype=np.float32)
    config = evidence(min_voxels=1, report_unobserved=True, origin=tuple(float(v) for v in scenes.cell_point(0, 0, 0)))
    report = compare(before, before[-1:], config)[2]
    assert (report.disappeared_voxels, report.unknown_voxels) == (3, 1)
    assert [(r.kind, r.voxel_count) for r in report.regions] == [("disappeared", 3), ("unobserved", 1)]
    assert report.regions[1].min_key == int(pack(1, 0, 0))


def test_slides():
    toward = compare(scenes.column([20]), scenes.column([19]), evidence(tolerance=1, min_voxels=1))[2]
    assert (toward.unknown_voxels, toward.disappeared_voxels, toward.appeared_voxels) == (1, 0, 0)
    away = compare(scenes.column([20]), scenes.column([21]), evidence(tolerance=1, min_voxels=1))[2]
    assert (away.unknown_voxels, away.disappeared_voxels, away.appeared_voxels) == (0, 0, 0)


def test_order_and_tie_rules():
    after = [scenes.cell_point(i, 0, 0) for i in range(6)] + [scenes.cell_point(1, j, 0) for j in range(20, 26)]
    report = compare(np.zeros((0, 3), dtype=np.float32), np.array(after, dtype=np.float32),
                     Config(tolerance=0, min_voxels=1, min_extent=1, pair_moves=False))[2]
    assert report.regions[0].min_key < report.regions[1].min_key
    assert report.regions[0].centroid[0] > report.regions[1].centroid[0]
    tied = compare(np.array([scenes.cell_point(-20, 0, 0), scenes.cell_point(20, 0, 0)], dtype=np.float32),
                   np.array([scenes.cell_point(0, 0, 0)], dtype=np.float32),
                   Config(tolerance=0, min_voxels=1, min_extent=1))[2]
    assert sorted(r.kind for r in tied.regions) == ["appeared", "disappeared", "disappeared"]


def test_threshold_counts_the_dropped_speck():
    before, after = goldens._speck_pair()
    tight = compare(before, after, Config())[2]
    loose = compare(before, after, Config(min_voxels=1, min_extent=1))[2]
    thin = compare(before, after, Config(min_voxels=1))[2]
    assert [r.kind for r in tight.regions] == ["appeared"]
    assert [r.voxel_count for r in loose.regions][1] == 2
    assert [r.voxel_count for r in thin.regions] == [tight.regions[0].voxel_count]
    assert tight.appeared_voxels == loose.appeared_voxels == thin.appeared_voxels == tight.regions[0].voxel_count + 2


def test_extent_gate():
    empty, sheet = goldens._sheet_pair()
    streak = compare(empty, sheet, Config())[2]
    assert streak.appeared_voxels == 7 and streak.regions == []
    kept = compare(empty, sheet, Config(min_extent=1))[2]
    assert [(r.kind, r.voxel_count) for r in kept.regions] == [("appeared", 7)]

    for build, kind in ((lambda: goldens._small_box_pair(None, (1.0, 0.5, 0.30)), "appeared"),
                        (lambda: goldens._small_box_pair((1.0, 0.5, 0.30), None), "disappeared"),
                        (goldens._small_moved_pair, "moved")):
        before, after = build()
        report = compare(before, after, Config())[2]
        assert [(r.kind, r.voxel_count) for r in report.regions] == [(kind, 8)]
        span = report.regions[0].bounds_max - report.regions[0].bounds_min
        assert np.all(np.abs(span - np.float32(0.10)) <= 0.001)
        if kind == "moved":
            assert report.regions[0].from_voxel_count == 8


def test_long_ray_stride_table():
    before, _ = scenes.long_ray()
    occupancy = occupancy_map(before, Config(evidence_of_absence=True, max_range=15))
    expected = {16: (5014, 1.68e6), 32: (2586, 8.74e5), 64: (1275, 4.32e5)}
    for stride, (rays, manhattan) in expected.items():
        keys, _ = ray_endpoints(occupancy, Config(evidence_of_absence=True, max_range=15, ray_stride=stride))
        i, j, k = unpack(keys)
        assert len(keys) == rays
        assert abs(int((np.abs(i) + np.abs(j) + np.abs(k)).sum()) - manhattan) <= 0.005 * manhattan


def test_boundary_endpoint_stops_on_the_end_cell():
    for far in (False, True):
        report = compare(*goldens.boundary_pair(far), config_for(goldens.UNIT))[2]
        assert (report.appeared_voxels, report.disappeared_voxels, report.unknown_voxels) == (0, 16, 4)
        assert [(r.kind, r.voxel_count) for r in report.regions if r.kind != "unobserved"] == [("disappeared", 16)]
        region = report.regions[0]
        assert region.min_key == int(pack(0, 0, 0))
        assert region.bounds_min.tolist() == [0, -15, 0] and region.bounds_max.tolist() == [16, 1, 1]


def test_boundary_octants_stay_inside_the_origin_end_box():
    config = config_for(goldens.UNIT)
    rng = scenes.LCG(0xB0A1DDA5)
    samples = 0
    for sx in (-1, 1):
        for sy in (-1, 1):
            for sz in (-1, 1):
                for mask in range(1, 8):
                    end = [sign * (int(rng.next() % 8) + 1) for sign in (sx, sy, sz)]
                    point = [np.float32(e) if mask & (1 << axis) else np.float32(e) + np.float32(0.5)
                             for axis, e in enumerate(end)]
                    assert floor_index(np.array(point, dtype=np.float32)).tolist() == end
                    low = [min(0, e) for e in end]
                    high = [max(0, e) for e in end]
                    grid = np.stack(np.meshgrid(*(np.arange(lo - 1, hi + 2) for lo, hi in zip(low, high)),
                                                indexing="ij"), axis=-1).reshape(-1, 3)
                    before = grid.astype(np.float32) + np.float32(0.5)
                    report = compare(before, np.array([point], dtype=np.float32), config)[2]
                    cells = np.prod([hi - lo + 1 for lo, hi in zip(low, high)])
                    assert max(abs(e) for e in end) <= report.disappeared_voxels <= cells - 1
                    disappeared = [r for r in report.regions if r.kind == "disappeared"]
                    assert disappeared
                    nearest = None
                    for region in disappeared:
                        lo = [int(v) for v in region.bounds_min]
                        hi = [int(v) - 1 for v in region.bounds_max]
                        assert all(low[a] <= lo[a] and hi[a] <= high[a] for a in range(3))
                        gap = max(max(lo[a] - end[a], end[a] - hi[a], 0) for a in range(3))
                        nearest = gap if nearest is None else min(nearest, gap)
                    assert nearest <= 1
                    samples += 1
    assert samples == 56


# MARK: Determinism


def test_shuffled_and_duplicated_input_keeps_the_bytes():
    before = scenes.column(list(range(20, 28)) + [40])
    after = scenes.column([40])
    before, after = np.concatenate([before, before]), np.concatenate([after, after])
    original = record(before, after, evidence())
    shuffled = record(scenes.shuffled(before, 0xC0FFEE), scenes.shuffled(after, 0xC0FFEE), evidence())
    assert report_record(shuffled[0]) == report_record(original[0]) and shuffled[1] == original[1]
    doubled = record(before, after, evidence(min_points=2))
    assert doubled[1] == original[1].replace('"min_points":1', '"min_points":2')

    room = scenes.room_shell()
    box = scenes.box_shell((0.0, 0.5, 0.30))
    moved_before = np.concatenate([room, box])
    moved_after = np.concatenate([room, scenes.shifted(box, 1)])
    config = Config(evidence_of_absence=True)
    moved = record(moved_before, moved_after, config)
    permuted = record(scenes.shuffled(moved_before, 0xC0FFEE), scenes.shuffled(moved_after, 0xC0FFEE), config)
    assert report_record(permuted[0]) == report_record(moved[0]) and permuted[1] == moved[1]
    assert [r.kind for r in moved[0].regions] == ["moved"]


def test_shuffled_snapshot_keeps_the_golden():
    before = decode_pointcloud2((SNAPSHOTS / "a" / "mid360.bin").read_bytes()).positions
    after = decode_pointcloud2((SNAPSHOTS / "b" / "mid360.bin").read_bytes()).positions
    rng = np.random.default_rng(7)
    _, text = record(before[rng.permutation(len(before))], after[rng.permutation(len(after))],
                     Config(evidence_of_absence=True))
    assert (text + "\n").encode() == (CD_FIXTURES / "mid360.json").read_bytes()


# MARK: Timing


def timed(before, after, config):
    compare(before, after, config)
    start = time.perf_counter()
    report = compare(before, after, config)[2]
    return report, (time.perf_counter() - start) * 1000


def test_indoor_timing(record_property):
    before, after = scenes.indoor(False), scenes.indoor(True)
    assert len(before) == len(after) == 89_320
    on, on_ms = timed(before, after, Config(evidence_of_absence=True))
    off, off_ms = timed(before, after, Config())
    assert [r.kind for r in on.regions] == ["moved"] and [r.kind for r in off.regions] == ["moved"]
    record_property("indoor_evidence_ms", round(on_ms, 1))
    record_property("indoor_v1_ms", round(off_ms, 1))
    print(f"\nindoor 89,320 points: evidence stride 1 {on_ms:.1f} ms, evidence off {off_ms:.1f} ms")
    assert on_ms < 3000 and off_ms < 3000
