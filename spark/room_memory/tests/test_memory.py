"""Voxel layers, bricks, reference persistence, and the window engine (PR 2.1 / 2.2)."""

import numpy as np
import pytest

import scenes
from room_memory.core import Config, occupancy_map
from room_memory.core.change_detection import contains, pack
from room_memory.memory import MemoryConfig, RoomMemory, VoxelLayer, load_reference, reference_path, save_reference
from room_memory.messages import (
    ChangeEvent,
    EventKind,
    EventState,
    Header,
    MemoryState,
    MemoryStatus,
    SENSOR_MID360,
    Time,
)
from test_messages import DELTA_BRICKS

START = Time(1760400000, 0)


def layer(occupied, known):
    keys = lambda cells: np.unique(pack(*np.array(cells, dtype=np.int64).reshape(-1, 3).T))
    return VoxelLayer(keys(occupied), keys(known))


def test_bricks_follow_the_protocol_bit_layout():
    occupied = [(-8, 16, -24), (-1, 23, -17)]
    known = occupied + [(-7, 16, -24)]
    assert layer(occupied, known).to_bricks() == DELTA_BRICKS


def test_bricks_keep_all_free_and_omit_unknown():
    free_brick = [(x, y, z) for x in range(8) for y in range(8) for z in range(8, 16)]
    bricks = layer([(0, 0, 0)], [(0, 0, 0)] + free_brick).to_bricks()
    assert [brick.coordinate for brick in bricks] == [(0, 0, 0), (0, 0, 1)]
    assert bricks[1].occupied == bytes(64) and bricks[1].known == b"\xff" * 64


def test_bricks_round_trip_random_layers():
    rng = np.random.default_rng(3)
    cells = rng.integers(-300, 300, size=(5000, 3))
    known = np.unique(pack(*cells.T))
    occupied = np.unique(known[rng.random(len(known)) < 0.3])
    original = VoxelLayer(occupied, known)
    bricks = original.to_bricks()
    assert [brick.coordinate for brick in bricks] == sorted(brick.coordinate for brick in bricks)
    restored = VoxelLayer.from_bricks(bricks)
    assert np.array_equal(restored.occupied, occupied) and np.array_equal(restored.known, known)
    assert original.brick_count == len(bricks)


def test_bricks_reject_out_of_range():
    with pytest.raises(ValueError, match="int16"):
        layer([(8 * 40000, 0, 0)], [(8 * 40000, 0, 0)]).to_bricks()


def test_observe_marks_the_ray_and_keeps_the_endpoint_occupied():
    config = Config(evidence_of_absence=True)
    window = VoxelLayer.observe(occupancy_map(scenes.column([40]), config), config)
    assert window.occupied.tolist() == [int(pack(40, 0, 0))]
    assert np.array_equal(window.known, np.unique(pack(np.arange(41), 0, 0)))


def test_save_and_load_reference(tmp_path):
    rng = np.random.default_rng(5)
    known = np.unique(pack(*rng.integers(-100, 100, size=(2000, 3)).T))
    original = VoxelLayer(known[::3], known)
    path = reference_path(tmp_path, "lab", "mid360")
    save_reference(path, original, Header("lab", 7, 0, START), 0.05)
    message, restored = load_reference(path)
    assert message.header.map_epoch == 7 and message.header.room_id == "lab"
    assert message.bricks == original.to_bricks()
    assert np.array_equal(restored.known, original.known) and np.array_equal(restored.occupied, original.occupied)
    assert not path.with_suffix(".cdr.tmp").exists()
    with pytest.raises(ValueError):
        reference_path(tmp_path, "../etc", "mid360")


def mid360_memory(tmp_path, **changes):
    values = dict(sensors=("mid360",), max_range=None, reference_windows=3, state_dir=str(tmp_path))
    values.update(changes)
    return RoomMemory(MemoryConfig(**values), START)


def stamp(second):
    return Time(START.sec + second, 0)


def build_reference(memory, cloud, windows=3):
    for second in range(windows):
        memory.add_points("mid360", cloud)
        assert memory.close_window(stamp(second)) == []
    return windows


def test_reference_build_then_tracking_with_unknown_back_face(tmp_path):
    before, after, front, back = scenes.board_scene()
    # The board's appeared faces are one cell thick; the default extent gate would drop them.
    memory = mid360_memory(tmp_path, min_extent=1)
    status = memory.status(stamp(0), 1.0)
    assert status.state == MemoryState.DEGRADED and status.header.map_epoch == 0 and status.mid360_hz == 0
    assert status.jt128_hz is None and status.pose_residual_m is None and status.coverage_known_frac is None
    memory.add_points("mid360", before)
    assert memory.status(stamp(0), 1.0).state == MemoryState.BUILDING
    second = build_reference(memory, before)
    memory.status(stamp(second), 1.0)
    assert memory.state == MemoryState.TRACKING and memory.map_epoch == 1
    assert reference_path(tmp_path, "lab", "mid360").exists()

    memory.add_points("mid360", after)
    events = memory.close_window(stamp(second))
    assert [(e.kind, e.state) for e in events] == [
        (EventKind.APPEARED, EventState.CANDIDATE), (EventKind.APPEARED, EventState.CANDIDATE),
        (EventKind.DISAPPEARED, EventState.CANDIDATE),
    ]
    back_keys = occupancy_map(back, Config()).occupied
    front_keys = occupancy_map(front, Config()).occupied
    disappeared = next(e for e in events if e.kind == EventKind.DISAPPEARED)
    mask = VoxelLayer.from_bricks(disappeared.mask.bricks)
    assert not contains(mask.occupied, back_keys).any()
    assert contains(mask.occupied, front_keys).mean() > 0.95
    assert disappeared.evidence == 1.0 and disappeared.alpha is None and disappeared.sensors == SENSOR_MID360
    assert disappeared.header.map_epoch == 1 and disappeared.first_seen == disappeared.last_seen == stamp(second)
    for event in events:
        assert ChangeEvent.from_cdr(event.to_cdr()) == event
    assert len({e.event_id for e in events}) == 3 and [e.header.seq for e in events] == [0, 1, 2]

    memory.add_points("mid360", after)
    assert memory.close_window(stamp(second + 1)) == []
    status = memory.status(stamp(second + 1), 2.0)
    assert status.state == MemoryState.TRACKING and status.n_open_events == 3 and status.mid360_hz == 1
    assert status.n_bricks == len(memory.channels["mid360"].reference.to_bricks())
    assert MemoryStatus.from_cdr(status.to_cdr()) == status

    memory.add_points("mid360", before)
    expired = memory.close_window(stamp(second + 2))
    assert [e.state for e in expired] == [EventState.EXPIRED] * 3
    assert sorted(e.event_id for e in expired) == sorted(e.event_id for e in events)
    assert all(e.last_seen == stamp(second + 1) for e in expired)
    assert memory.status(stamp(second + 2), 1.0).n_open_events == 0


def test_new_coverage_is_not_an_appearance(tmp_path):
    room = scenes.room_shell()
    missing_wall = room[room[:, 0] < 2.4]
    memory = mid360_memory(tmp_path, state_dir=None)
    second = build_reference(memory, missing_wall)
    wall_keys = occupancy_map(room[room[:, 0] >= 2.4], Config()).occupied
    assert not contains(memory.channels["mid360"].reference.known, wall_keys).all()
    memory.add_points("mid360", room)
    assert memory.close_window(stamp(second)) == []

    box = scenes.box_shell((0.3, 1.2, 0.3))
    memory.add_points("mid360", np.concatenate([room, box]))
    events = memory.close_window(stamp(second + 1))
    assert [(e.kind, e.state) for e in events] == [(EventKind.APPEARED, EventState.CANDIDATE)]
    assert np.all(np.abs(np.array(events[0].centroid) - [0.3, 1.2, 0.3]) <= 0.025)
    assert events[0].evidence == 1.0


def test_moved_box_is_one_moved_candidate(tmp_path):
    room = scenes.room_shell()
    box = scenes.box_shell((0.0, 0.5, 0.30))
    memory = mid360_memory(tmp_path, state_dir=None)
    second = build_reference(memory, np.concatenate([room, box]))
    memory.add_points("mid360", np.concatenate([room, scenes.shifted(box, 1)]))
    events = memory.close_window(stamp(second))
    assert [(e.kind, e.state) for e in events] == [(EventKind.MOVED, EventState.CANDIDATE)]
    assert events[0].from_centroid is not None and ChangeEvent.from_cdr(events[0].to_cdr()) == events[0]


def test_reload_restores_epoch_and_bricks(tmp_path):
    memory = mid360_memory(tmp_path)
    build_reference(memory, np.concatenate([scenes.room_shell(), scenes.box_shell((1.0, 0.5, 0.3))]))
    bricks = memory.channels["mid360"].reference.to_bricks()
    restarted = mid360_memory(tmp_path)
    assert restarted.state == MemoryState.TRACKING and restarted.map_epoch == memory.map_epoch == 1
    assert restarted.channels["mid360"].reference.to_bricks() == bricks
    assert restarted.status(stamp(9), 1.0).n_bricks == len(bricks)


def test_inconsistent_saved_state_is_refused(tmp_path):
    memory = mid360_memory(tmp_path)
    build_reference(memory, scenes.room_shell())
    with pytest.raises(ValueError, match="incomplete"):
        RoomMemory(MemoryConfig(sensors=("mid360", "jt128"), state_dir=str(tmp_path)), START)
    with pytest.raises(ValueError, match="voxel_size"):
        mid360_memory(tmp_path, voxel_size=0.1)


def test_degraded_and_dropped_windows(tmp_path):
    memory = RoomMemory(MemoryConfig(state_dir=None, reference_windows=1), START)
    memory.add_points("mid360", scenes.room_shell())
    assert memory.add_cdr("jt128", b"\x00\x01\x00\x00") is False
    memory.close_window(stamp(0))
    status = memory.status(stamp(1), 1.0)
    assert memory.state == MemoryState.BUILDING and memory.dropped_windows == 1
    assert status.state == MemoryState.DEGRADED and status.dropped_windows == 1
    assert (status.mid360_hz, status.jt128_hz) == (1, 1)
    memory.add_cdr("mid360", scenes.encode_pointcloud2(scenes.room_shell()))
    memory.add_cdr("jt128", scenes.encode_pointcloud2(scenes.room_shell()))
    memory.close_window(stamp(1))
    status = memory.status(stamp(2), 1.0)
    assert memory.state == MemoryState.TRACKING and status.state == MemoryState.TRACKING and memory.map_epoch == 1
    status = memory.status(stamp(3), 1.0)
    assert status.state == MemoryState.DEGRADED and (status.mid360_hz, status.jt128_hz) == (0, 0)


def test_windows_without_usable_points_do_not_build_the_reference(tmp_path):
    room = scenes.room_shell()
    memory = mid360_memory(tmp_path, max_range=15.0)
    unusable = [np.zeros((0, 3), dtype=np.float32), np.full((50, 3), np.nan, dtype=np.float32), room + 50]
    for second, cloud in enumerate(unusable):
        memory.add_points("mid360", cloud)
        assert memory.close_window(stamp(second)) == []
    assert memory.state == MemoryState.BUILDING and memory.map_epoch == 0
    assert memory.channels["mid360"].reference_windows == 0
    assert not reference_path(tmp_path, "lab", "mid360").exists()

    for second in range(3, 6):
        memory.add_points("mid360", room)
        memory.close_window(stamp(second))
    assert memory.state == MemoryState.TRACKING and memory.map_epoch == 1
    memory.add_points("mid360", np.concatenate([room, scenes.box_shell((0.3, 1.2, 0.3))]))
    events = memory.close_window(stamp(6))
    assert [(e.kind, e.state) for e in events] == [(EventKind.APPEARED, EventState.CANDIDATE)]


def test_each_sensor_reference_freezes_at_its_own_window_count():
    room = scenes.room_shell()
    box = scenes.box_shell((0.3, 1.2, 0.3))
    memory = RoomMemory(MemoryConfig(state_dir=None, max_range=None, reference_windows=1), START)
    memory.add_points("mid360", room)
    memory.close_window(stamp(0))
    assert memory.state == MemoryState.BUILDING
    # The JT128 arrives late; the box placed meanwhile is not part of the Mid-360 reference.
    memory.add_points("mid360", np.concatenate([room, box]))
    memory.add_points("jt128", room)
    assert memory.close_window(stamp(1)) == []
    assert memory.state == MemoryState.TRACKING
    assert memory.channels["mid360"].reference_windows == 1
    memory.add_points("mid360", np.concatenate([room, box]))
    events = memory.close_window(stamp(2))
    assert [(e.kind, e.state, e.sensors) for e in events] == [
        (EventKind.APPEARED, EventState.CANDIDATE, SENSOR_MID360),
    ]


def test_recent_observations_age_through_missing_windows(tmp_path):
    room = scenes.room_shell()
    memory = mid360_memory(tmp_path, state_dir=None, recent_windows=3)
    second = build_reference(memory, room)
    memory.add_points("mid360", np.concatenate([room, scenes.box_shell((0.3, 1.2, 0.3))]))
    appeared = memory.close_window(stamp(second))
    assert [(e.kind, e.state) for e in appeared] == [(EventKind.APPEARED, EventState.CANDIDATE)]
    # Outage: one window with no frames, one whose only frame is rejected.
    assert memory.close_window(stamp(second + 1)) == []
    assert memory.add_cdr("mid360", b"\x00\x01\x00\x00") is False
    assert memory.close_window(stamp(second + 2)) == []
    memory.add_points("mid360", room)
    expired = memory.close_window(stamp(second + 3))
    assert [(e.event_id, e.state) for e in expired] == [(appeared[0].event_id, EventState.EXPIRED)]
    assert expired[0].last_seen == stamp(second)


def test_config_rejections():
    with pytest.raises(ValueError):
        RoomMemory(MemoryConfig(sensors=("velodyne",)), START)
    with pytest.raises(ValueError):
        RoomMemory(MemoryConfig(sensors=()), START)
    with pytest.raises(ValueError):
        RoomMemory(MemoryConfig(window_s=0, state_dir=None), START)
