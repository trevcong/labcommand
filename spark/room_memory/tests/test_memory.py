"""Voxel layers, bricks, reference persistence, and the window engine (PR 2.1 / 2.2)."""

from dataclasses import replace

import numpy as np
import pytest

import scenes
from room_memory.core import Config, occupancy_map
from room_memory.core.change_detection import contains, pack
from room_memory.memory import (
    EVENT_LOG_DEPTH,
    CandidateTracker,
    MemoryConfig,
    RoomMemory,
    SettleRule,
    VoxelLayer,
    VoxelStats,
    epoch_path,
    load_reference,
    map_delta,
    pose_offset,
    reference_path,
    region_fresh,
    save_reference,
)
from room_memory.messages import (
    Activity,
    Brick,
    ChangeEvent,
    EventKind,
    EventState,
    Header,
    MapBricks,
    MapKind,
    MemoryState,
    MemoryStatus,
    RequestSync,
    SENSOR_MID360,
    SyncEnd,
    SyncTopic,
    Time,
    ZERO_TOKEN,
)
from test_messages import DELTA_BRICKS, TOKEN_A

START = Time(1760400000, 0)
GENERATION = 1760400000000
NONE = (1 << 64) - 1


def sync_request(since, topic, expected=GENERATION, room_id="lab", token=TOKEN_A):
    """A v2 request; ``expected=None`` makes a legacy one."""
    epoch = NONE if since is None else since[0]
    return RequestSync(Header(room_id, epoch, 0, START), since, topic, expected, None if expected is None else token)


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


# Single-window mechanics: every voxel a window sees settles, the recent layer is the last window,
# one supported voxel makes a region fresh, no activity holds a region back, and every region is
# sent and expired at once. The node's defaults are measured on the static recordings
# (room_memory_cpp/tests/recording).
SINGLE_WINDOW = dict(recent_windows=1, occupied_min_hits=1, occupied_min_ratio=0.0, free_min_seen=1,
                     candidate_windows=1, expire_windows=1, fresh_min_voxels=1, fresh_min_fraction=0.0,
                     activity_radius_m=0.0)


def mid360_memory(tmp_path, **changes):
    values = dict(sensors=("mid360",), max_range=None, reference_windows=3, state_dir=str(tmp_path), **SINGLE_WINDOW)
    values.update(changes)
    return RoomMemory(MemoryConfig(**values), START, GENERATION)


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
    assert status.events_seq is None
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
    assert status.events_seq == events[-1].header.seq == 2
    assert status.n_bricks == len(memory.channels["mid360"].reference.to_bricks())
    assert MemoryStatus.from_cdr(status.to_cdr()) == status

    memory.add_points("mid360", before)
    expired = memory.close_window(stamp(second + 2))
    assert [e.state for e in expired] == [EventState.EXPIRED] * 3
    assert sorted(e.event_id for e in expired) == sorted(e.event_id for e in events)
    assert all(e.last_seen == stamp(second + 1) for e in expired)
    status = memory.status(stamp(second + 2), 1.0)
    assert status.n_open_events == 0 and status.events_seq == expired[-1].header.seq == 5


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
    assert restarted.state == MemoryState.TRACKING and memory.map_epoch == 1 and restarted.map_epoch == 2
    assert restarted.channels["mid360"].reference.to_bricks() == bricks
    assert restarted.status(stamp(9), 1.0).n_bricks == len(bricks)


def test_sync_cursor_from_before_a_restart_names_nothing_after_it(tmp_path):
    """Codex P2: seq restarts at 0, so a restart that kept map_epoch reissued (map_epoch, seq)
    cursors, and an old client's cursor matched a new, unrelated event: an empty answer."""
    room, box = scenes.room_shell(), scenes.box_shell((0.3, 1.2, 0.3))
    memory = mid360_memory(tmp_path)
    second = build_reference(memory, room)
    memory.add_points("mid360", np.concatenate([room, box]))
    (old,) = memory.close_window(stamp(second))
    assert (old.header.map_epoch, old.header.seq) == (1, 0)
    config = memory.config
    restarted = RoomMemory(config, stamp(100), GENERATION + 1)
    assert restarted.map_epoch == 2 and RoomMemory(config, stamp(200), GENERATION + 2).map_epoch == 3
    restarted = RoomMemory(config, stamp(300), GENERATION + 3)
    restarted.add_points("mid360", np.concatenate([room, box]))
    (new,) = restarted.close_window(stamp(301))
    assert new.event_id != old.event_id and (new.header.map_epoch, new.header.seq) == (4, 0)
    # The old generation's cursor (and a legacy request) is answered as SINCE_NONE in the new one.
    since = (old.header.map_epoch, old.header.seq)
    for request in (sync_request(since, SyncTopic.EVENTS), sync_request(since, SyncTopic.EVENTS, expected=None)):
        messages, end = restarted.sync(request, stamp(302))
        assert messages == [new.to_cdr()]
        assert end.publisher_generation == GENERATION + 3 and end.delivered == ((0, 0),) and end.unavailable == ()


def test_a_sensor_subset_run_keeps_the_room_loadable(tmp_path):
    """Codex P2: a restart with sensors=['mid360'] saved only reference-mid360.cdr at the next
    epoch, so the saved references disagreed and a restart with both sensors refused the room."""
    room, partial = partial_room()
    # New coverage joins the references while tracking (learn_windows).
    both = MemoryConfig(sensors=("mid360", "jt128"), max_range=None, reference_windows=3, state_dir=str(tmp_path),
                        learn_windows=30, **SINGLE_WINDOW)
    memory = RoomMemory(both, START, GENERATION)
    for second in range(3):
        memory.add_points("mid360", partial)
        memory.add_points("jt128", partial)
        memory.close_window(stamp(second))
    assert memory.state == MemoryState.TRACKING and memory.map_epoch == 1
    subset = mid360_memory(tmp_path, delta_period_s=1.0, learn_windows=30)
    assert subset.state == MemoryState.TRACKING and subset.map_epoch == 2
    assert MapBricks.from_cdr(subset.map_update(stamp(10))).header.map_epoch == 2
    # The subset run learns the far wall, so reference-mid360.cdr moves ahead of reference-jt128.cdr.
    subset.add_points("mid360", room)
    subset.close_window(stamp(11))
    assert MapBricks.from_cdr(subset.map_update(stamp(11))).header.map_epoch == subset.map_epoch == 3
    epochs = {name: load_reference(reference_path(tmp_path, "lab", name))[0].header.map_epoch
              for name in ("mid360", "jt128")}
    assert epochs == {"mid360": 3, "jt128": 1}
    restarted = RoomMemory(both, stamp(20), GENERATION)
    assert restarted.state == MemoryState.TRACKING and restarted.map_epoch == 4
    assert restarted.channels["mid360"].reference.to_bricks() == subset.channels["mid360"].reference.to_bricks()
    assert RoomMemory(both, stamp(30), GENERATION).map_epoch == 5 and epoch_path(tmp_path, "lab").read_bytes() == b"5\n"


def test_a_sensor_subset_build_continues_the_room_epoch(tmp_path):
    """A sensor the room has no reference for builds one from the room's epoch, not from 0."""
    jt128 = mid360_memory(tmp_path, sensors=("jt128",))
    for second in range(3):
        jt128.add_points("jt128", scenes.room_shell())
        jt128.close_window(stamp(second))
    assert jt128.state == MemoryState.TRACKING and jt128.map_epoch == 1
    assert RoomMemory(jt128.config, stamp(10), GENERATION).map_epoch == 2
    mid360 = mid360_memory(tmp_path)
    assert mid360.state == MemoryState.BUILDING and mid360.map_epoch == 2
    build_reference(mid360, scenes.room_shell())
    assert mid360.state == MemoryState.TRACKING and mid360.map_epoch == 3
    assert RoomMemory(replace(mid360.config, sensors=("mid360", "jt128")), stamp(20), GENERATION).map_epoch == 4


def test_inconsistent_saved_state_is_refused(tmp_path):
    memory = mid360_memory(tmp_path)
    build_reference(memory, scenes.room_shell())
    with pytest.raises(ValueError, match="incomplete"):
        RoomMemory(MemoryConfig(sensors=("mid360", "jt128"), state_dir=str(tmp_path)), START, GENERATION)
    with pytest.raises(ValueError, match="voxel_size"):
        mid360_memory(tmp_path, voxel_size=0.1)
    for data in (b"", b"3", b"-3\n", b" 3\n", b"18446744073709551616\n"):
        epoch_path(tmp_path, "lab").write_bytes(data)
        with pytest.raises(ValueError, match="map_epoch"):
            mid360_memory(tmp_path)


def test_degraded_and_dropped_windows(tmp_path):
    # One window settles occupancy, so each sensor's reference is ready after its first window.
    memory = RoomMemory(MemoryConfig(state_dir=None, reference_windows=1, occupied_min_hits=1), START, GENERATION)
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
    memory = RoomMemory(MemoryConfig(state_dir=None, max_range=None, reference_windows=1, **SINGLE_WINDOW), START, GENERATION)
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


def test_tracking_waits_for_settled_reference_occupancy():
    """Codex P2: reference_windows=1 under occupied_min_hits=2 settled no occupancy in its one window,
    so TRACKING began on an empty reference that pose_offset could never register on, and that no
    window could ever teach. The reference keeps building until it has occupancy."""
    room = scenes.room_shell()
    memory = RoomMemory(MemoryConfig(sensors=("mid360",), state_dir=None, max_range=None, reference_windows=1,
                                     candidate_windows=3), START, GENERATION)
    memory.add_points("mid360", room)
    memory.close_window(stamp(0))
    assert memory.state == MemoryState.BUILDING and memory.map_epoch == 0
    assert memory.channels["mid360"].reference_windows == 1 and not len(memory.channels["mid360"].reference.occupied)
    memory.add_points("mid360", room)
    memory.close_window(stamp(1))
    assert memory.state == MemoryState.TRACKING and memory.map_epoch == 1
    assert memory.status(stamp(1), 1.0).n_bricks > 0
    boxed = np.concatenate([room, scenes.box_shell((0.3, 1.2, 0.3))])
    events = []
    for second in range(2, 5):
        memory.add_points("mid360", boxed)
        events += memory.close_window(stamp(second))
    assert [(e.kind, e.state, e.first_seen) for e in events] == [(EventKind.APPEARED, EventState.CANDIDATE, stamp(2))]


def test_a_return_kept_in_the_recent_layer_is_not_fresh_evidence():
    """Codex P2: with recent_windows=3 a box seen in one window stayed in three consecutive reports
    and passed the 3-window persistence gate alone. Only windows that observe it count now."""
    room = scenes.room_shell()
    boxed = np.concatenate([room, scenes.box_shell((0.3, 1.2, 0.3))])
    gate = dict(recent_windows=3, candidate_windows=3, expire_windows=3)
    memory = mid360_memory(None, state_dir=None, **gate)
    second = build_reference(memory, room)
    memory.add_points("mid360", boxed)
    assert memory.close_window(stamp(second)) == []
    for later in range(second + 1, second + 6):
        memory.add_points("mid360", room)
        assert memory.close_window(stamp(later)) == []
    # The same box seen in three windows in a row is a candidate on the third.
    memory = mid360_memory(None, state_dir=None, **gate)
    second = build_reference(memory, room)
    events = []
    for n in range(3):
        memory.add_points("mid360", boxed)
        events.append(memory.close_window(stamp(second + n)))
    assert events[:2] == [[], []]
    assert [(e.kind, e.state, e.first_seen) for e in events[2]] == [(EventKind.APPEARED, EventState.CANDIDATE, stamp(second))]


def test_free_space_kept_in_the_recent_layer_is_not_fresh_evidence():
    """Codex P2: with recent_windows=3 one window's rays through a removed box kept it disappeared
    in three consecutive reports, each counted fresh, so it passed the 3-window gate alone. Only
    windows whose own rays traverse it count now."""
    room = scenes.room_shell()
    boxed = np.concatenate([room, scenes.box_shell((0.3, 1.2, 0.3))])
    # The other half of the room: no ray from the origin to it comes near the box.
    other_half = room[room[:, 1] < 0]
    gate = dict(recent_windows=3, candidate_windows=3, expire_windows=3)
    memory = mid360_memory(None, state_dir=None, **gate)
    second = build_reference(memory, boxed)
    for points in (room, other_half, other_half, other_half):
        memory.add_points("mid360", points)
        assert memory.close_window(stamp(second)) == []
        second += 1
    # The box missing from three windows in a row is a candidate on the third.
    memory = mid360_memory(None, state_dir=None, **gate)
    second = build_reference(memory, boxed)
    events = []
    for n in range(3):
        memory.add_points("mid360", room)
        events.append(memory.close_window(stamp(second + n)))
    assert events[:2] == [[], []]
    assert [(e.kind, e.state, e.first_seen) for e in events[2]] == [
        (EventKind.DISAPPEARED, EventState.CANDIDATE, stamp(second))]


def test_a_stale_window_resets_the_run_and_starts_nothing():
    """Astra: a pending region kept its fresh-window count across stale windows, so fresh, stale,
    fresh, stale, fresh promoted under a 3-window gate. A stale window keeps the match but sets the
    run to 0, and a stale region alone starts nothing."""
    tracker = CandidateTracker(3, 1)
    ids = iter(range(100, 200))
    box = type("Region", (), {"kind": "appeared", "keys": keys_of([[0, 0, 0], [1, 0, 0]]), "from_keys": None})()
    next_id = lambda: next(ids)
    assert tracker.update([box], stamp(0), next_id, [False]) == [] and tracker.pending == []
    for second, flag in enumerate([True, False, True, False, True], start=1):
        assert tracker.update([box], stamp(second), next_id, [flag]) == []
        assert len(tracker.pending) == 1 and tracker.pending[0].windows == (1 if flag else 0)
    assert tracker.update([box], stamp(6), next_id, [True]) == []
    (started,) = tracker.update([box], stamp(7), next_id, [True])
    assert started[0] == EventState.CANDIDATE and started[1].first_seen == stamp(1)


def test_a_move_reported_as_its_halves_stays_one_open_move():
    """With 1 cm injection noise the diff's move-count gate flickered on the static recording: the
    open MOVED expired and reopened as an APPEARED plus a DISAPPEARED every few seconds."""
    tracker = CandidateTracker(1, 1)
    ids = iter(range(100, 200))
    next_id = lambda: next(ids)
    region = lambda kind, keys, from_keys=None: type("Region", (), {"kind": kind, "keys": keys, "from_keys": from_keys})()
    move = region("moved", keys_of([[10, 0, 0], [11, 0, 0]]), keys_of([[0, 0, 0], [1, 0, 0]]))
    appeared, disappeared = region("appeared", keys_of([[11, 0, 0], [12, 0, 0]])), region("disappeared", keys_of([0, 0, 0]))
    speck = region("appeared", keys_of([30, 0, 0]))
    ((state, opened),) = tracker.update([move], stamp(0), next_id)
    assert state == EventState.CANDIDATE and opened.kind == EventKind.MOVED
    halves = tracker.update([appeared, disappeared, speck], stamp(1), next_id)
    assert [(s, c.event_id) for s, c in halves] == [(EventState.CANDIDATE, 101)] and tracker.pending == []
    assert [(c.event_id, c.kind, c.last_seen, c.missed) for c in tracker.open] == [
        (100, EventKind.MOVED, stamp(1), 0), (101, EventKind.APPEARED, stamp(1), 0)]
    gone = tracker.update([], stamp(2), next_id)
    assert [(s, c.event_id, c.last_seen) for s, c in gone] == [(EventState.EXPIRED, 100, stamp(1)),
                                                               (EventState.EXPIRED, 101, stamp(1))]


def test_a_lone_half_keeps_nothing_alive():
    """Codex P2: an object moved from A to B and then removed from B left only the DISAPPEARED at
    A, which kept resetting the open MOVED: the move never expired and the removal never started
    a candidate. Only both halves together keep a move alive now; a lone one starts nothing until
    the move has expired."""
    tracker = CandidateTracker(3, 3)
    ids = iter(range(100, 200))
    next_id = lambda: next(ids)
    region = lambda kind, keys, from_keys=None: type("Region", (), {"kind": kind, "keys": keys, "from_keys": from_keys})()
    move = region("moved", keys_of([[10, 0, 0], [11, 0, 0]]), keys_of([[0, 0, 0], [1, 0, 0]]))
    removed = region("disappeared", keys_of([[0, 0, 0], [1, 0, 0]]))
    windows = [tracker.update([move], stamp(n), next_id) for n in range(3)]
    windows += [tracker.update([removed], stamp(n), next_id) for n in range(3, 13)]
    assert [n for n, w in enumerate(windows) if w] == [2, 5, 8]
    assert [(s, c.event_id, c.kind) for s, c in windows[2]] == [(EventState.CANDIDATE, 100, EventKind.MOVED)]
    assert [(s, c.event_id, c.last_seen) for s, c in windows[5]] == [(EventState.EXPIRED, 100, stamp(2))]
    assert [(s, c.event_id, c.kind, c.first_seen) for s, c in windows[8]] == [
        (EventState.CANDIDATE, 101, EventKind.DISAPPEARED, stamp(6))]
    # The other half alone, without persistence: the move expires that window, the half starts the next.
    tracker = CandidateTracker(1, 1)
    tracker.update([move], stamp(0), next_id)
    (gone,) = tracker.update([region("appeared", keys_of([[11, 0, 0], [12, 0, 0]]))], stamp(1), next_id)
    assert gone[0] == EventState.EXPIRED and gone[1].kind == EventKind.MOVED and tracker.open == []
    (opened,) = tracker.update([region("appeared", keys_of([[11, 0, 0], [12, 0, 0]]))], stamp(2), next_id)
    assert opened[0] == EventState.CANDIDATE and opened[1].kind == EventKind.APPEARED


def test_a_moved_box_removed_from_its_new_pose_expires_and_disappears():
    """Codex P2: the DISAPPEARED left at the old pose kept the MOVED open forever on the node
    defaults; nothing was sent in ten windows without the box."""
    room = scenes.room_shell()
    box = scenes.box_shell((0.0, 0.5, 0.30))
    memory = mid360_memory(None, state_dir=None, recent_windows=3, candidate_windows=3, expire_windows=3)
    second = build_reference(memory, np.concatenate([room, box]))
    events = []
    for n in range(16):
        memory.add_points("mid360", np.concatenate([room, scenes.shifted(box, 1)]) if n < 3 else room)
        events.append(memory.close_window(stamp(second + n)))
    assert [n for n, window in enumerate(events) if window] == [2, 7, 10]
    ((moved,), (expired,), (removed,)) = events[2], events[7], events[10]
    assert (moved.kind, moved.state) == (EventKind.MOVED, EventState.CANDIDATE)
    # The recent layer keeps the new pose for its 3 windows: both halves until the window after the removal.
    assert (expired.event_id, expired.state, expired.last_seen) == (moved.event_id, EventState.EXPIRED, stamp(second + 4))
    assert (removed.kind, removed.state, removed.first_seen) == (EventKind.DISAPPEARED, EventState.CANDIDATE, stamp(second + 8))


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
        RoomMemory(MemoryConfig(sensors=("velodyne",)), START, GENERATION)
    with pytest.raises(ValueError):
        RoomMemory(MemoryConfig(sensors=()), START, GENERATION)
    with pytest.raises(ValueError):
        RoomMemory(MemoryConfig(window_s=0, state_dir=None), START, GENERATION)


def test_window_s_must_be_finite_and_positive():
    for bad in (0.0, -1.0, float("inf"), float("nan")):
        with pytest.raises(ValueError, match="window_s"):
            RoomMemory(MemoryConfig(window_s=bad, state_dir=None), Time(0, 0), GENERATION)


def test_any_decode_failure_is_a_rejected_frame(monkeypatch):
    import room_memory.memory as memory_module

    def explode(data):
        raise MemoryError("decoder ran out of memory")

    monkeypatch.setattr(memory_module, "decode_pointcloud2", explode)
    memory = RoomMemory(MemoryConfig(state_dir=None), Time(0, 0), GENERATION)
    assert memory.add_cdr("mid360", b"\x00\x01\x00\x00") is False
    channel = memory.channels["mid360"]
    assert (channel.received, channel.failed) == (1, 1)


def keys_of(cells):
    return np.unique(pack(*np.array(cells, dtype=np.int64).reshape(-1, 3).T))


def test_voxel_stats_count_windows_and_settle_by_rule():
    stats = VoxelStats()
    stats.add(keys_of([0, 0, 0]), keys_of([[0, 0, 0], [1, 0, 0], [2, 0, 0]]))
    stats.add(keys_of([[0, 0, 0], [2, 0, 0]]), keys_of([[0, 0, 0], [2, 0, 0], [3, 0, 0]]))
    stats.add(keys_of([]), keys_of([[0, 0, 0], [1, 0, 0]]))
    assert stats.hits.tolist() == [2, 0, 1, 0] and stats.seen.tolist() == [3, 2, 2, 1]
    union = stats.settled(SettleRule())
    assert union.occupied.tolist() == keys_of([[0, 0, 0], [2, 0, 0]]).tolist() and len(union.known) == 4
    strict = stats.settled(SettleRule(2, 0.6, 2, 0.0))
    assert strict.occupied.tolist() == keys_of([0, 0, 0]).tolist()
    assert strict.known.tolist() == keys_of([[0, 0, 0], [1, 0, 0]]).tolist()
    assert len(stats.settled(SettleRule(2, 0.6, 2, 0.5)).known) == 3
    stats.erase(strict.known)
    assert stats.keys.tolist() == keys_of([[2, 0, 0], [3, 0, 0]]).tolist() and stats.hits.tolist() == [1, 0]


def test_map_delta_sends_changed_bricks_and_removes_absent_ones():
    def brick(i, bits):
        return Brick((i, 0, 0), bytes(64), bytes([bits]) + bytes(63))
    before = (brick(0, 1), brick(1, 1), brick(2, 1), brick(4, 1))
    after = (brick(0, 1), brick(2, 3), brick(3, 1), brick(4, 1))
    assert map_delta(before, after) == ((after[1], after[2]), (1,))
    assert map_delta(after, after) == ((), ())


def test_candidate_gate_waits_and_expiry_has_hysteresis():
    tracker = CandidateTracker(3, 2)
    ids = iter(range(100, 200))
    box = type("Region", (), {"kind": "appeared", "keys": keys_of([[0, 0, 0], [1, 0, 0]]), "from_keys": None})()
    speck = type("Region", (), {"kind": "appeared", "keys": keys_of([9, 9, 9]), "from_keys": None})()
    next_id = lambda: next(ids)
    assert tracker.update([box, speck], stamp(0), next_id) == []
    assert tracker.update([box], stamp(1), next_id) == [] and len(tracker.pending) == 1
    started = tracker.update([box, speck], stamp(2), next_id)
    assert [(s, c.event_id, c.first_seen) for s, c in started] == [(EventState.CANDIDATE, 100, stamp(0))]
    assert tracker.update([], stamp(3), next_id) == [] and tracker.update([box], stamp(4), next_id) == []
    assert tracker.update([], stamp(5), next_id) == []
    gone = tracker.update([], stamp(6), next_id)
    assert [(s, c.last_seen) for s, c in gone] == [(EventState.EXPIRED, stamp(4))] and tracker.open == []


def partial_room():
    room = scenes.room_shell()
    return room, room[room[:, 0] < 1.5]


def test_map_stream_snapshot_then_deltas_and_keyframes():
    room, partial = partial_room()
    memory = mid360_memory(None, state_dir=None, delta_period_s=2.0, keyframe_period_s=6.0, keyframe_delta_ratio=100.0,
                           learn_windows=30)
    assert memory.map_update(stamp(0)) is None
    second = build_reference(memory, partial)
    sent = []
    for s in range(second, second + 9):
        memory.add_points("mid360", room)
        memory.close_window(stamp(s))
        data = memory.map_update(stamp(s))
        if data is not None:
            sent.append((s - second, MapBricks.from_cdr(data)))
    assert [(at, m.kind, m.header.map_epoch, m.base_epoch, m.header.seq) for at, m in sent] == [
        (0, MapKind.SNAPSHOT, 1, 0, 0), (2, MapKind.DELTA, 2, 1, 1), (6, MapKind.SNAPSHOT, 2, 0, 2)]
    mirror = {b.coordinate: b for b in sent[0][1].bricks}
    mirror.update({b.coordinate: b for b in sent[1][1].bricks})
    assert tuple(mirror[c] for c in sorted(mirror)) == sent[2][1].bricks == memory.channels["mid360"].reference.to_bricks()


def test_sync_replays_or_snapshots_maps_and_replays_events():
    room = scenes.room_shell()
    boxed = np.concatenate([room, scenes.box_shell((0.3, 1.2, 0.3))])
    memory = mid360_memory(None, state_dir=None, delta_period_s=1.0, keyframe_period_s=1.0)

    # BUILDING: no map yet, an empty answer that says so (bound SEQ_NONE).
    messages, end = memory.sync(sync_request(None, SyncTopic.MAP_BRICKS), stamp(0))
    assert messages == [] and end.bound is None and end.delivered == () and end.token == TOKEN_A
    second = build_reference(memory, room)
    live = []
    for n in range(3):
        memory.add_points("mid360", boxed if n == 1 else room)
        memory.close_window(stamp(second + n))
        live.append(memory.map_update(stamp(second + n)))
    now = stamp(second + 3)
    epoch = MapBricks.from_cdr(live[0]).header.map_epoch
    newest = MapBricks.from_cdr(live[2]).header
    assert newest.seq == 2
    # since in the log, older than the bound: the later logged messages, unchanged.
    messages, end = memory.sync(sync_request((epoch, 0), SyncTopic.MAP_BRICKS), now)
    assert messages == live[1:] and end.bound == 2 and end.delivered == ((1, 2),)
    # since is the newest message: nothing missed, nothing sent (no snapshot).
    messages, end = memory.sync(sync_request((newest.map_epoch, 2), SyncTopic.MAP_BRICKS), now)
    assert messages == [] and end.bound == 2 and end.delivered == ()
    # since not in the log, of another generation, or a legacy request: one fresh snapshot.
    for request in (sync_request(None, SyncTopic.MAP_BRICKS), sync_request((newest.map_epoch + 1, 2), SyncTopic.MAP_BRICKS),
                    sync_request((epoch + 7, 0), SyncTopic.MAP_BRICKS), sync_request((epoch, 40), SyncTopic.MAP_BRICKS),
                    sync_request((epoch, 0), SyncTopic.MAP_BRICKS, expected=GENERATION - 1),
                    sync_request((epoch, 0), SyncTopic.MAP_BRICKS, expected=0),
                    sync_request((newest.map_epoch, 2), SyncTopic.MAP_BRICKS, expected=None)):
        (answer,), end = memory.sync(request, now)
        snapshot = MapBricks.from_cdr(answer)
        assert snapshot.kind == MapKind.SNAPSHOT and snapshot.header.seq == 2
        assert snapshot.publisher_generation == GENERATION
        assert snapshot.bricks == MapBricks.from_cdr(live[2]).bricks
        assert end.delivered == ((2, 2),) and end.token == (ZERO_TOKEN if request.token is None else TOKEN_A)
    assert memory.sync(sync_request(None, SyncTopic.MAP_BRICKS, room_id="other"), now) == ([], None)
    (opened, closed), end = memory.sync(sync_request(None, SyncTopic.EVENTS), now)
    assert end.bound == 1 and end.delivered == ((0, 1),) and end.unavailable == ()
    first = ChangeEvent.from_cdr(opened)
    assert first.state == EventState.CANDIDATE and ChangeEvent.from_cdr(closed).state == EventState.EXPIRED
    assert memory.sync(sync_request((first.header.map_epoch, first.header.seq), SyncTopic.EVENTS), now)[0] == [closed]
    messages, end = memory.sync(sync_request((first.header.map_epoch, 1), SyncTopic.EVENTS), now)
    assert messages == [] and end.bound == 1 and end.delivered == ()
    # One SyncEnd per answer, numbered from 0 in this generation; each one encodes.
    assert end.header.seq == memory.sync_end_seq - 1 and SyncEnd.from_cdr(end.to_cdr()) == end


def test_events_answer_with_no_events_is_known_empty():
    memory = mid360_memory(None, state_dir=None)
    build_reference(memory, scenes.room_shell())
    messages, end = memory.sync(sync_request(None, SyncTopic.EVENTS), stamp(10))
    assert messages == [] and end.bound is None and end.delivered == end.unavailable == ()
    assert memory.status(stamp(10), 1.0).events_seq is None


def test_event_fallback_resends_open_events_older_than_the_log():
    room = scenes.room_shell()
    box, speck = scenes.box_shell((0.3, 1.2, 0.3)), scenes.box_shell((-0.8, -0.8, 0.3))
    memory = mid360_memory(None, state_dir=None)
    second = build_reference(memory, room)
    memory.add_points("mid360", np.concatenate([room, box]))
    (opened,) = memory.close_window(stamp(second))
    transitions = 0
    unavailable = []
    while transitions < EVENT_LOG_DEPTH + 20:
        second += 1
        memory.add_points("mid360", np.concatenate([room, box] if transitions % 2 else [room, box, speck]))
        transitions += len(memory.close_window(stamp(second)))
        unavailable.append(memory.sync(sync_request(None, SyncTopic.EVENTS), stamp(second))[1].unavailable)
    legacy = RequestSync(Header("lab", NONE, 0, START), None, SyncTopic.EVENTS)
    messages, end = memory.sync(legacy, stamp(second))
    assert len(messages) == EVENT_LOG_DEPTH + 1 and messages[0] == opened.to_cdr()
    bound = memory.event_seq - 1
    oldest = bound - EVENT_LOG_DEPTH + 1
    assert end.token == ZERO_TOKEN and end.bound == bound
    assert end.delivered == ((0, 0), (oldest, bound)) and end.unavailable == ((1, oldest - 1),)
    # Within one generation a seq once unavailable stays unavailable.
    seen = set()
    for ranges in unavailable:
        now = {seq for first, last in ranges for seq in range(first, last + 1)}
        assert seen <= now
        seen = now
    # A since the log no longer holds limits the requested range to what follows it.
    epoch = ChangeEvent.from_cdr(messages[1]).header.map_epoch
    _, end = memory.sync(sync_request((epoch, 5), SyncTopic.EVENTS), stamp(second))
    assert end.unavailable == ((6, oldest - 1),) and end.delivered == ((0, 0), (oldest, bound))


def test_messages_carry_the_generation_and_references_stay_legacy(tmp_path):
    room, box = scenes.room_shell(), scenes.box_shell((0.3, 1.2, 0.3))
    memory = mid360_memory(tmp_path)
    second = build_reference(memory, room)
    memory.add_points("mid360", np.concatenate([room, box]))
    (event,) = memory.close_window(stamp(second))
    decoded = ChangeEvent.from_cdr(event.to_cdr())
    assert decoded.publisher_generation == decoded.mask.publisher_generation == GENERATION
    assert MapBricks.from_cdr(memory.map_update(stamp(second))).publisher_generation == GENERATION
    assert MemoryStatus.from_cdr(memory.status(stamp(second), 1.0).to_cdr()).publisher_generation == GENERATION
    assert Activity.from_cdr(memory.activity(stamp(second)).to_cdr()).header.room_id == "lab"  # unchanged message
    saved, _ = load_reference(reference_path(tmp_path, "lab", "mid360"))
    assert saved.publisher_generation is None
    data = reference_path(tmp_path, "lab", "mid360").read_bytes()
    assert MapBricks.from_cdr(data).to_cdr() == data  # the legacy layout, byte for byte
    # A v2 reference (written by hand) still loads.
    path = reference_path(tmp_path, "lab", "mid360")
    path.write_bytes(replace(saved, publisher_generation=7).to_cdr())
    assert mid360_memory(tmp_path, ).channels["mid360"].reference.to_bricks() == saved.bricks
    for bad in (0, -1, 1 << 64, None, 1.5):
        with pytest.raises(ValueError, match="publisher_generation"):
            RoomMemory(MemoryConfig(state_dir=None), START, bad)


def test_coverage_and_pose_residual():
    room = scenes.room_shell()
    memory = mid360_memory(None, state_dir=None)
    second = build_reference(memory, room)
    building = memory.status(stamp(second), 1.0)
    assert 0.2 < building.coverage_known_frac <= 1 and building.pose_residual_m is None
    memory.add_points("mid360", room)
    memory.close_window(stamp(second))
    assert memory.status(stamp(second), 1.0).pose_residual_m == 0.0
    # The rig shifted by (0.1, 0.05, 0.1) m: each wall registers on its own axis.
    memory.add_points("mid360", (room + np.float32([0.1, 0.05, 0.1])).astype(np.float32))
    memory.close_window(stamp(second + 1))
    assert abs(memory.status(stamp(second + 1), 1.0).pose_residual_m - 0.15) <= 0.01
    assert len(memory.channels["mid360"].stats.keys) == 0  # a shifted window teaches the reference nothing


def budget_memory(**changes):
    """One reference window, then one tracking window of budget. The due tick is 3 s later, so the
    window that spends the budget is not itself a map tick."""
    values = dict(reference_windows=1, learn_windows=1, delta_period_s=3.0, keyframe_period_s=4.0,
                  keyframe_delta_ratio=100.0)
    values.update(changes)
    return mid360_memory(None, state_dir=None, **values)


def feed(memory, cloud, second):
    if cloud is not None:
        memory.add_points("mid360", cloud)
    memory.close_window(stamp(second))
    data = memory.map_update(stamp(second))
    return None if data is None else MapBricks.from_cdr(data)


def test_learning_budget_flushes_on_the_next_due_tick_then_freezes():
    room, partial = partial_room()
    memory = budget_memory()
    opening = feed(memory, partial, 0)
    assert opening.kind == MapKind.SNAPSHOT and opening.header.map_epoch == 1
    assert len(memory.channels["mid360"].stats.keys) == 0
    # The learning window is not due, so the new wall stays in the counts.
    assert feed(memory, room, 1) is None and len(memory.channels["mid360"].stats.keys) > 0
    assert feed(memory, room, 2) is None and len(memory.channels["mid360"].stats.keys) > 0
    flushed = feed(memory, room, 3)
    assert flushed.kind == MapKind.DELTA and flushed.header.map_epoch == 2 and flushed.base_epoch == 1
    learned = memory.channels["mid360"].reference.to_bricks()
    assert len(learned) > len(opening.bricks) and len(memory.channels["mid360"].stats.keys) == 0
    key = feed(memory, room, 4)
    assert key.kind == MapKind.SNAPSHOT and key.header.map_epoch == 2 and key.bricks == learned
    assert feed(memory, room, 5) is None
    again = feed(memory, room, 8)
    assert again.kind == MapKind.SNAPSHOT and again.header.map_epoch == 2 and again.bricks == learned
    box = scenes.box_shell((0.3, 1.2, 0.3))
    memory.add_points("mid360", np.concatenate([room, box]))
    events = memory.close_window(stamp(9))
    assert any(e.kind == EventKind.APPEARED and np.all(np.abs(np.array(e.centroid) - [0.3, 1.2, 0.3]) <= 0.05)
               for e in events)
    assert memory.map_update(stamp(9)) is None
    far = scenes.box_shell((8.0, 8.0, 0.5))
    memory.add_points("mid360", np.concatenate([room, far]))
    # The open box expires. The outside cluster is not a candidate and is not learned.
    assert all(e.state != EventState.CANDIDATE for e in memory.close_window(stamp(10)))
    frozen = feed(memory, room, 12)
    assert frozen.kind == MapKind.SNAPSHOT and frozen.header.map_epoch == 2 and frozen.bricks == learned


def test_an_empty_tracking_window_consumes_the_learn_budget():
    room, partial = partial_room()
    memory = budget_memory()
    opening = feed(memory, partial, 0)
    assert feed(memory, None, 1) is None and len(memory.channels["mid360"].stats.keys) == 0
    memory.add_points("mid360", room)
    assert memory.close_window(stamp(2)) == [] and len(memory.channels["mid360"].stats.keys) == 0
    assert memory.map_update(stamp(3)) is None and memory.map_epoch == 1
    assert memory.channels["mid360"].reference.to_bricks() == opening.bricks
    key = feed(memory, room, 4)
    assert key.kind == MapKind.SNAPSHOT and key.header.map_epoch == 1 and key.bricks == opening.bricks


def test_a_pose_rejected_window_consumes_the_learn_budget():
    room, partial = partial_room()
    memory = budget_memory()
    opening = feed(memory, partial, 0)
    shifted = (partial + np.float32([0.1, 0.05, 0.1])).astype(np.float32)
    assert feed(memory, shifted, 1) is None
    assert memory.channels["mid360"].residual > memory.config.learn_max_offset_m
    assert len(memory.channels["mid360"].stats.keys) == 0
    memory.add_points("mid360", room)
    # The shifted window's candidate expires. The aligned room teaches nothing.
    assert all(e.state != EventState.CANDIDATE for e in memory.close_window(stamp(2)))
    assert len(memory.channels["mid360"].stats.keys) == 0
    assert memory.map_update(stamp(3)) is None and memory.map_epoch == 1
    key = feed(memory, room, 4)
    assert key.kind == MapKind.SNAPSHOT and key.header.map_epoch == 1 and key.bricks == opening.bricks


def test_learn_windows_zero_drops_leftovers_and_keeps_known_space_events():
    room, partial = partial_room()
    memory = budget_memory(learn_windows=0)
    opening = feed(memory, partial, 0)
    assert len(memory.channels["mid360"].stats.keys) == 0
    for second in range(1, 4):
        memory.add_points("mid360", room)
        assert memory.close_window(stamp(second)) == []
        assert memory.map_update(stamp(second)) is None and memory.map_epoch == 1
    assert memory.channels["mid360"].reference.to_bricks() == opening.bricks
    box = scenes.box_shell((0.3, 1.2, 0.3))
    memory.add_points("mid360", np.concatenate([room, box]))
    events = memory.close_window(stamp(4))
    assert any(e.kind == EventKind.APPEARED and np.all(np.abs(np.array(e.centroid) - [0.3, 1.2, 0.3]) <= 0.05)
               for e in events)
    sent = memory.map_update(stamp(4))
    assert MapBricks.from_cdr(sent).kind == MapKind.SNAPSHOT
    assert MapBricks.from_cdr(sent).header.map_epoch == 1 and MapBricks.from_cdr(sent).bricks == opening.bricks
    far = scenes.box_shell((8.0, 8.0, 0.5))
    memory.add_points("mid360", np.concatenate([room, far]))
    assert all(e.state != EventState.CANDIDATE for e in memory.close_window(stamp(5)))
    assert memory.channels["mid360"].reference.to_bricks() == opening.bricks


def test_a_reload_with_learn_windows_zero_does_not_learn(tmp_path):
    room, partial = partial_room()
    memory = mid360_memory(tmp_path, reference_windows=1, learn_windows=0, delta_period_s=1.0, keyframe_period_s=100.0)
    memory.add_points("mid360", partial)
    assert memory.close_window(stamp(0)) == []
    assert memory.state == MemoryState.TRACKING and memory.map_epoch == 1
    bricks = memory.channels["mid360"].reference.to_bricks()
    restarted = RoomMemory(memory.config, stamp(10), GENERATION)
    opening = MapBricks.from_cdr(restarted.map_update(stamp(10)))
    assert restarted.map_epoch == 2 and opening.header.map_epoch == 2 and opening.bricks == bricks
    restarted.add_points("mid360", room)
    assert restarted.close_window(stamp(11)) == []
    assert restarted.map_update(stamp(11)) is None and restarted.map_epoch == 2
    assert restarted.channels["mid360"].reference.to_bricks() == bricks


def test_pose_offset_is_binary64():
    """Overlap counts 9, 10, 1 along x refine to -0.4 voxels: 0.020000000000000004 m at 5 cm, just
    above learn_max_offset_m (0.02), where float rounding would land just below it. The C++ port
    pins the same value."""
    z = 10 * np.arange(10)  # 10 voxels apart, beyond the +-4 search, so y and z refine to 0
    recent = np.sort(pack(0, 0, z))
    reference = np.sort(np.concatenate([recent, pack(-1, 0, z[:9]), pack([1], [0], [0])]))
    offset = pose_offset(recent, reference, 0.05)
    assert offset == 0.020000000000000004 and offset > MemoryConfig().learn_max_offset_m


@pytest.mark.parametrize("changes", [
    dict(occupied_min_hits=0), dict(free_min_seen=0), dict(occupied_min_ratio=1.5), dict(free_max_ratio=-0.1),
    dict(candidate_windows=0), dict(expire_windows=0), dict(delta_period_s=0), dict(keyframe_period_s=float("nan")),
    dict(keyframe_delta_ratio=-1), dict(map_sensor="jt128"), dict(learn_max_offset_m=-0.1),
    dict(learn_windows=-1),
])
def test_stream_config_rejections(changes):
    with pytest.raises(ValueError):
        mid360_memory(None, state_dir=None, **changes)


@pytest.mark.parametrize("launch", ["room_memory/launch/room_memory.launch.py",
                                    "room_memory_cpp/launch/room_memory_cpp.launch.py"])
def test_launch_files_carry_the_memory_config_defaults(launch):
    import ast
    import dataclasses
    import pathlib

    tree = ast.parse((pathlib.Path(__file__).resolve().parents[2] / launch).read_text())
    table = next(node.value for node in tree.body if isinstance(node, ast.Assign) and node.targets[0].id == "ARGUMENTS")
    # name -> default text, the first element of each (default, type) tuple
    arguments = {ast.literal_eval(k): ast.literal_eval(v.elts[0]) for k, v in zip(table.keys, table.values)}
    assert set(arguments) == {field.name for field in dataclasses.fields(MemoryConfig)}
    defaults = MemoryConfig()
    for field in dataclasses.fields(MemoryConfig):
        text = arguments[field.name]
        value = getattr(defaults, field.name)
        if field.name == "sensors":
            assert ast.literal_eval(text) == list(value)
        elif isinstance(value, str):
            assert text == value, field.name
        else:
            assert type(value)(text) == value, field.name


# MARK: Activity and the settle gate


def region(kind, cells, from_cells=None, voxel_count=None):
    """A diff region as the tracker sees it: kind, sorted keys, and the old pose of a move."""
    keys = keys_of(cells)
    return type("Region", (), {"kind": kind, "keys": keys, "from_keys": None if from_cells is None else keys_of(from_cells),
                               "voxel_count": len(keys) if voxel_count is None else voxel_count})()


def block(x, y=0, z=0, side=3):
    return [[x + i, y + j, z + k] for i in range(side) for j in range(side) for k in range(side)]


def run_tracker(tracker, windows, fresh=None):
    """Feeds one region list per window; returns the window indices with a CANDIDATE."""
    ids = iter(range(100, 1000))
    out = []
    for n, regions in enumerate(windows):
        if regions is None:
            tracker.interrupt()
            continue
        flags = None if fresh is None else fresh[n]
        if any(state == EventState.CANDIDATE for state, _ in tracker.update(regions, stamp(n), lambda: next(ids), flags)):
            out.append(n)
    return out


def test_a_gap_breaks_the_run():
    """Astra: a window without usable points must not be bridged. 4 settled windows, a gap, 1: no
    CANDIDATE; the run starts again after the gap."""
    box = region("appeared", block(0))
    tracker = CandidateTracker(5, 1, 1, 20, 0.9)
    assert run_tracker(tracker, [[box]] * 4 + [None] + [[box]] * 5) == [9]


def test_a_gap_window_in_the_engine_interrupts_pending_regions():
    room = scenes.room_shell()
    boxed = np.concatenate([room, scenes.box_shell((0.3, 1.2, 0.3))])
    memory = mid360_memory(None, state_dir=None, recent_windows=1, candidate_windows=3, expire_windows=3)
    second = build_reference(memory, room)
    events = []
    for n, cloud in enumerate([boxed, boxed, None, boxed, boxed, boxed]):
        if cloud is not None:
            memory.add_points("mid360", cloud)
        events.append(memory.close_window(stamp(second + n)))
    assert [n for n, window in enumerate(events) if window] == [5]


def test_a_reshaping_region_is_activity_and_never_promotes():
    tracker = CandidateTracker(3, 1, 1, 20, 0.9)
    shapes = [region("appeared", block(0) + block(3 + 3 * (n % 2), 0, 0)) for n in range(8)]
    assert run_tracker(tracker, [[shape] for shape in shapes]) == []
    assert tracker.activity == [0] and tracker.pending[0].windows == 1


def test_a_change_next_to_motion_waits_until_the_motion_settles():
    """A stable box 4 voxels from a region that keeps reshaping counts only once that stops."""
    tracker = CandidateTracker(3, 1, 1, 5, 0.9)
    box = region("appeared", block(0))
    moving = [region("appeared", block(7, 0, 3 * (n % 2))) for n in range(6)]
    windows = [[box, moving[n]] for n in range(6)] + [[box]] * 4
    assert run_tracker(tracker, windows) == [8]


def test_motion_beyond_the_radius_does_not_hold_a_change():
    tracker = CandidateTracker(3, 1, 1, 5, 0.9)
    box = region("appeared", block(0))
    windows = [[box, region("appeared", block(20, 0, 3 * (n % 2)))] for n in range(6)]
    assert run_tracker(tracker, windows) == [2]


def test_a_still_residue_beyond_the_gate_is_reported():
    """The documented limit: a region that stays still, fresh and alone for candidate_windows is a
    change, whatever made it."""
    tracker = CandidateTracker(40, 1, 1, 20, 0.9)
    assert run_tracker(tracker, [[region("disappeared", block(0))]] * 41) == [39]


def test_a_run_restarted_after_a_stale_window_takes_a_new_anchor():
    """Codex P2 (round 2): a fresh window after a stale one counted on from the last run's anchor,
    so blocks alternating at x=0 and x=2 (each within 1 voxel of the old x=1 block, not of each
    other) promoted after 40 windows without being activity. The restart now anchors on its own window."""
    tracker = CandidateTracker(40, 1, 1, 20, 0.9)
    windows = [[region("appeared", block(1))]] * 2 + [[region("appeared", block(2 * (n % 2)))] for n in range(50)]
    fresh = [[True], [False]] + [[True]] * 50
    assert run_tracker(tracker, windows, fresh) == []
    assert tracker.activity == [0] and tracker.pending[0].windows == 1


def test_an_edge_that_flickers_keeps_the_shape():
    """Sparse returns at range: a voxel or two of the boundary comes and goes (at least 90 %
    within tolerance), and a small fragment off its edge joins it instead of being motion."""
    tracker = CandidateTracker(4, 1, 1, 20, 0.9)
    base = block(0, side=5)
    windows = [[region("appeared", base)], [region("appeared", base + [[6, 0, 0]])],
               [region("appeared", base), region("appeared", [[5, 3, 3], [6, 3, 3], [5, 4, 3]])],
               [region("appeared", base[:-2])]]
    assert run_tracker(tracker, windows) == [3]
    assert tracker.activity == []


def test_fresh_needs_a_count_and_a_share_of_the_voxels():
    keys = block(0)  # 27 voxels
    appeared = region("appeared", keys)
    seen = lambda n: keys_of(keys[:n])
    assert not region_fresh(appeared, seen(2), keys_of([]), 3, 0.5)
    assert not region_fresh(appeared, seen(13), keys_of([]), 3, 0.5)
    assert region_fresh(appeared, seen(14), keys_of([]), 3, 0.5)
    moved = region("moved", keys, block(10))
    assert not region_fresh(moved, seen(27), keys_of([]), 3, 0.5)  # the old pose needs its own rays
    assert region_fresh(moved, seen(27), keys_of(block(10)[:14]), 3, 0.5)
    assert region_fresh(region("disappeared", keys), keys_of([]), seen(14), 3, 0.5)


def test_a_move_seen_as_its_halves_keeps_its_run():
    """The diff reports one move whole in some windows and as its two halves in others; the
    pending move keeps counting through both, in either region order."""
    old, new = block(0), block(10)
    whole = region("moved", new, old)
    halves = [region("appeared", new), region("disappeared", old)]
    for order in (halves, halves[::-1]):
        tracker = CandidateTracker(4, 1, 1, 20, 0.9)
        assert run_tracker(tracker, [[whole], order, [whole], order]) == [3]
        assert len(tracker.open) == 1 and tracker.open[0].kind == EventKind.MOVED
    # A stale half sets the run to 0.
    tracker = CandidateTracker(4, 1, 1, 20, 0.9)
    assert run_tracker(tracker, [[whole], halves, halves, halves], [None, [True, False], None, None]) == []
    assert tracker.pending[0].windows == 2


def test_a_move_promoted_through_its_halves_reports_their_geometry():
    """Codex P2: a pending move matched by its halves kept the last whole move's geometry, so a
    half that reshaped and then settled was reported at its old place next to its new mask."""
    tracker = CandidateTracker(3, 1, 1, None, 0.9, 0.05)
    whole = region("moved", block(10), block(0))
    halves = [region("appeared", block(12)), region("disappeared", block(0))]
    ids = iter(range(100, 200))
    transitions = [tracker.update(regions, stamp(n), lambda: next(ids)) for n, regions in enumerate([[whole], halves, halves, halves])]
    ((state, moved),) = transitions[3]
    assert state == EventState.CANDIDATE and moved.kind == EventKind.MOVED
    assert moved.region.centroid.tolist() == pytest.approx([0.675, 0.075, 0.075]) and moved.region.voxel_count == 27
    assert moved.region.from_centroid.tolist() == pytest.approx([0.075, 0.075, 0.075])
    assert moved.region.bounds_min.tolist() == pytest.approx([0.6, 0.0, 0.0]) and np.array_equal(moved.region.keys, keys_of(block(12)))


def test_a_move_takes_over_its_pending_halves():
    """Halves first (2 and 1 settled windows), then the whole move: one pending move whose run is
    the shorter of theirs plus this window, both halves consumed. (No activity check: the new
    disappeared half would hold the appeared one back.)"""
    old, new = block(0), block(10)
    tracker = CandidateTracker(4, 1, 1, None, 0.9)
    appeared, disappeared, whole = region("appeared", new), region("disappeared", old), region("moved", new, old)
    run_tracker(tracker, [[appeared], [appeared, disappeared]])
    assert [(c.kind, c.windows) for c in tracker.pending] == [(EventKind.APPEARED, 2), (EventKind.DISAPPEARED, 1)]
    ids = iter(range(100, 200))
    assert tracker.update([whole], stamp(2), lambda: next(ids)) == []
    assert [(c.kind, c.windows, c.first_seen) for c in tracker.pending] == [(EventKind.MOVED, 2, stamp(0))]
    (started,) = tracker.update([whole], stamp(3), lambda: next(ids)) + tracker.update([whole], stamp(4), lambda: next(ids))
    assert started[0] == EventState.CANDIDATE and started[1].kind == EventKind.MOVED


def test_activity_reports_the_newest_window_with_motion():
    room = scenes.room_shell()
    memory = mid360_memory(None, state_dir=None, recent_windows=1, candidate_windows=5, activity_radius_m=1.0)
    idle = memory.activity(stamp(0))
    assert (idle.active, idle.since_s, idle.header.seq) == (False, None, 0)
    # Like every stateful message, Activity carries the process's generation (seq restarts at 0 per generation).
    assert idle.publisher_generation == memory.publisher_generation == GENERATION
    second = build_reference(memory, room)
    for n in range(4):
        box = scenes.box_shell((0.3 + 0.3 * (n % 2), 1.2, 0.3))
        memory.add_points("mid360", np.concatenate([room, box]))
        assert memory.close_window(stamp(second + n)) == []
    moving = memory.activity(stamp(second + 4))
    assert moving.active and moving.sensors == moving.largest_sensor == SENSOR_MID360 and moving.n_regions >= 1
    assert moving.voxel_count > 0 and moving.since_s == 1.0 and moving.header.seq == 1
    assert Activity.from_cdr(moving.to_cdr()) == moving
    for n in range(4, 6):
        memory.add_points("mid360", room)
        memory.close_window(stamp(second + n))
    later = memory.activity(stamp(second + 9))
    assert not later.active and later.n_regions == 0 and later.centroid is None and later.since_s == 6.0


@pytest.mark.parametrize("changes", [
    dict(fresh_min_voxels=0), dict(fresh_min_fraction=1.5), dict(stable_min_fraction=-0.1),
    dict(stable_min_fraction=float("nan")), dict(activity_radius_m=-1.0), dict(activity_radius_m=float("inf")),
])
def test_gate_config_rejections(changes):
    with pytest.raises(ValueError):
        RoomMemory(MemoryConfig(state_dir=None, **changes), START, GENERATION)
