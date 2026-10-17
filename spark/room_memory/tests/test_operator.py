"""Operator actions (protocol §3.6, D46) in the Python oracle: the paths the differential suite does
not reach (injected write faults, the suppressions file, the action ring) and the main lifecycle.
The C++ twin is ``room_memory_cpp/tests/test_operator.cpp``."""

import numpy as np
import pytest

import scenes
from room_memory import operator as op
from room_memory.memory import MemoryConfig, RoomMemory, VoxelLayer, load_reference, reference_path, region_keys
from room_memory.messages import (
    EventKind, EventState, Header, MapBricks, MapKind, MemoryState, OperatorAction, OperatorActionKind, OperatorOutcome,
    Time,
)

START = Time(1760400000, 0)
GENERATION = 1760400000000
ROOM = scenes.room_shell()
BOX = scenes.box_shell((1.0, 0.5, 0.3))
WITH_BOX = np.concatenate([ROOM, BOX])
BEHIND = ROOM[ROOM[:, 0] < -0.5]


def stamp(second):
    return Time(START.sec + second, 0)


def config(state_dir=None, sensors=("mid360",), **fields):
    return MemoryConfig(**{**dict(sensors=sensors, max_range=None, reference_windows=3, recent_windows=1, occupied_min_hits=1,
                                  occupied_min_ratio=0.0, free_min_seen=1, candidate_windows=1, expire_windows=1,
                                  fresh_min_voxels=1, fresh_min_fraction=0.0, activity_radius_m=0.0,
                                  suppress_clear_windows=3, state_dir=None if state_dir is None else str(state_dir)),
                           **fields})


def window(memory, cloud, second, sensor="mid360"):
    memory.add_points(sensor, cloud)
    return memory.close_window(stamp(second))


def track(memory, reference=ROOM):
    for second in range(3):
        for sensor in memory.config.sensors:
            memory.add_points(sensor, reference)
        assert memory.close_window(stamp(second)) == []
    assert memory.state == MemoryState.TRACKING
    assert memory.map_update(stamp(3)) is not None


def act(kind, event, expected, epoch=1, n=0, generation=GENERATION, text=""):
    return OperatorAction(Header("lab", epoch, n, stamp(100 + n)), kind, event, expected, generation, text, "trevor",
                          f"00000000-0000-4000-8000-{n:012d}")


def candidate(memory):
    track(memory)
    events = window(memory, WITH_BOX, 4)
    assert [(e.kind, e.state) for e in events] == [(EventKind.APPEARED, EventState.CANDIDATE)]
    return events[0].event_id


def outcome(memory, action, second=5):
    return memory.operate(action, stamp(second))[0]


@pytest.fixture
def no_faults():
    yield
    op.WRITE_FAULT = None


def test_confirm_then_seen_back_reverts_and_occlusion_does_not():
    memory = RoomMemory(config(), START, GENERATION)
    event_id = candidate(memory)
    result, events, _, _ = memory.operate(act(OperatorActionKind.CONFIRM, event_id, EventState.CANDIDATE, n=1), stamp(5))
    assert result == OperatorOutcome.APPLIED and [e.state for e in events] == [EventState.CONFIRMED]
    for second in range(6, 10):
        assert window(memory, BEHIND, second) == []
    assert len(memory.channels["mid360"].tracker.open) == 1
    back = window(memory, ROOM, 10)
    assert [(e.event_id, e.state) for e in back] == [(event_id, EventState.REVERTED)]


def test_dismiss_suppresses_until_seen_clear(tmp_path):
    memory = RoomMemory(config(tmp_path), START, GENERATION)
    event_id = candidate(memory)
    result, events, _, _ = memory.operate(act(OperatorActionKind.DISMISS, event_id, EventState.CANDIDATE, n=1), stamp(5))
    assert result == OperatorOutcome.APPLIED and [e.state for e in events] == [EventState.REVERTED]
    memory.activity(stamp(5))
    for second in range(6, 20):
        assert window(memory, WITH_BOX, second) == [] and not memory.activity(stamp(second)).active
    for second in range(20, 25):
        assert window(memory, BEHIND, second) == []
    assert len(memory.suppressions("mid360")) == 1
    for second in range(25, 28):
        assert window(memory, ROOM, second) == []
    assert memory.suppressions("mid360") == []
    assert (tmp_path / "lab" / "suppressed-mid360").read_bytes() == b"room_memory suppressed 1\n"
    assert [e.state for e in window(memory, WITH_BOX, 28)] == [EventState.CANDIDATE]


def test_accept_holds_accepted_for_its_map(tmp_path):
    memory = RoomMemory(config(tmp_path), START, GENERATION)
    event_id = candidate(memory)
    _, confirmed, _, _ = memory.operate(act(OperatorActionKind.CONFIRM, event_id, EventState.CANDIDATE, n=1), stamp(5))
    assert memory.channels["mid360"].residual <= memory.config.learn_max_offset_m
    assert outcome(memory, act(OperatorActionKind.ACCEPT, event_id, EventState.CONFIRMED, n=2)) == OperatorOutcome.APPLIED
    published = VoxelLayer.from_bricks(confirmed[0].mask.bricks).occupied
    _, layer = load_reference(reference_path(str(tmp_path), "lab", "mid360"))
    assert np.isin(published, layer.occupied).all()
    assert memory.status(stamp(5), 1.0).n_open_events == 1 and memory.accepted_after_map(stamp(5)) == []
    assert outcome(memory, act(OperatorActionKind.DISMISS, event_id, EventState.CONFIRMED, n=3)) == OperatorOutcome.STATE_MISMATCH
    assert memory.map_update(stamp(6)) is None
    delta = MapBricks.from_cdr(memory.map_update(stamp(8)))
    assert delta.kind == MapKind.DELTA and delta.header.map_epoch == 2
    released = memory.accepted_after_map(stamp(8))
    assert [(e.state, e.header.map_epoch) for e in released] == [(EventState.ACCEPTED, 2)]
    assert memory.status(stamp(8), 1.0).n_open_events == 0
    assert window(memory, WITH_BOX, 9) == []


@pytest.mark.parametrize("stage", ["write", "sync", "rename"])
def test_a_write_that_fails_before_the_rename_changes_nothing(tmp_path, no_faults, stage):
    memory = RoomMemory(config(tmp_path), START, GENERATION)
    event_id = candidate(memory)
    memory.operate(act(OperatorActionKind.CONFIRM, event_id, EventState.CANDIDATE, n=1), stamp(5))
    files = {path.name: path.read_bytes() for path in (tmp_path / "lab").iterdir()}
    reference = memory.channels["mid360"].reference.occupied.copy()
    op.WRITE_FAULT = lambda at: at == stage
    for kind, n in ((OperatorActionKind.ACCEPT, 2), (OperatorActionKind.DISMISS, 3)):
        result, events, _, _ = memory.operate(act(kind, event_id, EventState.CONFIRMED, n=n), stamp(5))
        assert result == OperatorOutcome.STORAGE_FAILED and events == [] and memory.storage_error
        assert {path.name: path.read_bytes() for path in (tmp_path / "lab").iterdir()} == files
        assert np.array_equal(memory.channels["mid360"].reference.occupied, reference)
        assert len(memory.channels["mid360"].tracker.open) == 1 and memory.suppressions("mid360") == []
    op.WRITE_FAULT = None
    assert outcome(memory, act(OperatorActionKind.DISMISS, event_id, EventState.CONFIRMED, n=3)) == OperatorOutcome.APPLIED


def test_a_write_installed_without_its_directory_sync_is_applied(tmp_path, no_faults):
    memory = RoomMemory(config(tmp_path), START, GENERATION)
    event_id = candidate(memory)
    memory.operate(act(OperatorActionKind.CONFIRM, event_id, EventState.CANDIDATE, n=1), stamp(5))
    op.WRITE_FAULT = lambda at: at == "directory"
    assert outcome(memory, act(OperatorActionKind.ACCEPT, event_id, EventState.CONFIRMED, n=2)) == OperatorOutcome.APPLIED
    assert memory.storage_error
    _, layer = load_reference(reference_path(str(tmp_path), "lab", "mid360"))
    assert np.array_equal(layer.occupied, memory.channels["mid360"].reference.occupied)


def test_a_lift_whose_write_fails_is_retried_every_window(tmp_path, no_faults):
    memory = RoomMemory(config(tmp_path), START, GENERATION)
    event_id = candidate(memory)
    memory.operate(act(OperatorActionKind.DISMISS, event_id, EventState.CANDIDATE, n=1), stamp(5))
    path = tmp_path / "lab" / "suppressed-mid360"
    dismissed = path.read_bytes()
    op.WRITE_FAULT = lambda at: at == "rename"
    for second in range(6, 9):
        assert window(memory, ROOM, second) == []
    assert memory.suppressions("mid360") == [] and path.read_bytes() == dismissed and memory.storage_error
    assert [e.state for e in window(memory, WITH_BOX, 9)] == [EventState.CANDIDATE]
    op.WRITE_FAULT = None
    window(memory, WITH_BOX, 10)
    assert path.read_bytes() == b"room_memory suppressed 1\n" and memory.storage_error is None
    restarted = RoomMemory(config(tmp_path), stamp(20), GENERATION + 1)
    assert restarted.suppressions("mid360") == []
    assert [e.state for e in window(restarted, WITH_BOX, 21)] == [EventState.CANDIDATE]


@pytest.mark.parametrize("text", [
    "", "room_memory suppressed 2\n", "room_memory suppressed 1", "room_memory suppressed 1\n5 appeared 1 10 0",
    "room_memory suppressed 1\n5 appeared 0 0\n", "room_memory suppressed 1\n5 appeared 2 10 9 0\n",
    "room_memory suppressed 1\n5 appeared 1 10 1 11\n", "room_memory suppressed 1\n5 moved 1 10 0\n",
    "room_memory suppressed 1\n5 sideways 1 10 0\n", "room_memory suppressed 1\n5 appeared 1 10 0 7\n",
    "room_memory suppressed 1\n5  appeared 1 10 0\n", "room_memory suppressed 1\n05 appeared 1 10 0\n",
    "room_memory suppressed 1\n5 appeared 1 10 0\n5 appeared 1 11 0\n", "room_memory suppressed 1\n5 appeared 9 10 0\n",
])
def test_the_suppressions_file_is_strict(text):
    with pytest.raises(ValueError):
        op.parse_suppressions(text)


def test_the_suppressions_file_round_trips_and_a_new_reference_drops_it(tmp_path):
    s = op.Suppression(5, EventKind.MOVED, np.array([-3, 10], dtype=np.int64), np.array([20], dtype=np.int64))
    assert op.format_suppressions([s]) == "room_memory suppressed 1\n5 moved 2 -3 10 1 20\n"
    assert op.parse_suppressions(op.format_suppressions([s]))[0].from_keys.tolist() == [20]
    assert op.parse_suppressions("room_memory suppressed 1\n") == []
    track(RoomMemory(config(tmp_path), START, GENERATION))
    path = tmp_path / "lab" / "suppressed-mid360"
    path.write_text("room_memory suppressed 1\nnonsense\n")
    with pytest.raises(ValueError):
        RoomMemory(config(tmp_path), START, GENERATION)
    reference_path(str(tmp_path), "lab", "mid360").unlink()
    building = RoomMemory(config(tmp_path), START, GENERATION)
    assert building.state == MemoryState.BUILDING and not path.exists()


def test_refusals_change_nothing(tmp_path):
    memory = RoomMemory(config(tmp_path), START, GENERATION)
    event_id = candidate(memory)
    seq = memory.event_seq
    other = act(OperatorActionKind.CONFIRM, event_id, EventState.CANDIDATE, n=10)
    other = OperatorAction(Header("lab-b", 1, 10, stamp(5)), *[getattr(other, f) for f in (
        "action", "event_id", "expected_state", "expected_generation", "text", "operator_id", "action_id")])
    pause = OperatorAction(Header("lab", 1, 16, stamp(5)), OperatorActionKind.PAUSE, None, None, GENERATION, "", "trevor",
                           "00000000-0000-4000-8000-000000000016")
    cases = [
        (other, OperatorOutcome.WRONG_ROOM),
        (act(OperatorActionKind.CONFIRM, event_id, EventState.CANDIDATE, n=11, generation=GENERATION + 1),
         OperatorOutcome.STALE_GENERATION),
        (act(OperatorActionKind.CONFIRM, event_id + 1, EventState.CANDIDATE, n=12), OperatorOutcome.UNKNOWN_EVENT),
        (act(OperatorActionKind.DISMISS, event_id, EventState.CONFIRMED, n=13), OperatorOutcome.STATE_MISMATCH),
        (act(OperatorActionKind.ACCEPT, event_id, EventState.CONFIRMED, n=14), OperatorOutcome.STATE_MISMATCH),
        (pause, OperatorOutcome.UNSUPPORTED),
    ]
    for action, expected in cases:
        result, events, _, _ = memory.operate(action, stamp(5))
        assert (result, events) == (expected, []) and memory.event_seq == seq
    assert outcome(memory, act(OperatorActionKind.CONFIRM, event_id, EventState.CANDIDATE, n=17)) == OperatorOutcome.APPLIED
    assert outcome(memory, act(OperatorActionKind.ACCEPT, event_id, EventState.CONFIRMED, epoch=0, n=18)) == OperatorOutcome.STALE_EPOCH
    assert memory.close_window(stamp(6)) == []
    assert outcome(memory, act(OperatorActionKind.ACCEPT, event_id, EventState.CONFIRMED, n=19)) == OperatorOutcome.TARGET_CHANGED
    assert RoomMemory(config(), START, GENERATION).operate(
        act(OperatorActionKind.CONFIRM, 5, EventState.CANDIDATE, epoch=0, n=1), stamp(1))[0] == OperatorOutcome.NOT_TRACKING


def test_confirm_and_accept_need_the_target_in_the_last_window_itself(tmp_path):
    # Review round 2: the recent layer keeps a region matched (missed == 0) for recent_windows
    # windows after the object left or went out of view. Only the last window's own evidence counts.
    def confirmed(memory):
        event_id = candidate(memory)
        assert outcome(memory, act(OperatorActionKind.CONFIRM, event_id, EventState.CANDIDATE, n=1)) == OperatorOutcome.APPLIED
        return event_id

    still = RoomMemory(config(tmp_path / "still", recent_windows=3), START, GENERATION)
    event_id = confirmed(still)
    assert window(still, WITH_BOX, 5) == []
    assert outcome(still, act(OperatorActionKind.ACCEPT, event_id, EventState.CONFIRMED, n=2), 5) == OperatorOutcome.APPLIED

    for name, cloud in (("removed", ROOM), ("occluded", BEHIND)):
        memory = RoomMemory(config(tmp_path / name, recent_windows=3), START, GENERATION)
        event_id = confirmed(memory)
        assert window(memory, cloud, 5) == []
        assert memory.channels["mid360"].tracker.open[0].missed == 0, name
        files = {path.name: path.read_bytes() for path in (tmp_path / name / "lab").iterdir()}
        reference = memory.channels["mid360"].reference.occupied.copy()
        result, events, _, _ = memory.operate(act(OperatorActionKind.ACCEPT, event_id, EventState.CONFIRMED, n=2), stamp(5))
        assert (result, events) == (OperatorOutcome.TARGET_CHANGED, []), name
        assert np.array_equal(memory.channels["mid360"].reference.occupied, reference), name
        assert {path.name: path.read_bytes() for path in (tmp_path / name / "lab").iterdir()} == files, name
        assert memory._held == [], name
        # Seen again: the same action id may be sent afresh, since a refusal is not a failure.
        assert window(memory, WITH_BOX, 6) == []
        assert outcome(memory, act(OperatorActionKind.ACCEPT, event_id, EventState.CONFIRMED, n=3), 6) == OperatorOutcome.APPLIED

    gone = RoomMemory(config(recent_windows=3), START, GENERATION)
    event_id = candidate(gone)
    assert window(gone, ROOM, 5) == [] and gone.channels["mid360"].tracker.open[0].missed == 0
    assert outcome(gone, act(OperatorActionKind.CONFIRM, event_id, EventState.CANDIDATE, n=1)) == OperatorOutcome.TARGET_CHANGED
    assert not gone.channels["mid360"].tracker.open[0].confirmed


def test_a_disappearance_needs_its_voxels_seen_free_in_the_last_window():
    memory = RoomMemory(config(recent_windows=3), START, GENERATION)
    track(memory, WITH_BOX)
    events = window(memory, ROOM, 4)
    assert [(e.kind, e.state) for e in events] == [(EventKind.DISAPPEARED, EventState.CANDIDATE)]
    event_id = events[0].event_id
    assert window(memory, BEHIND, 5) == [] and memory.channels["mid360"].tracker.open[0].missed == 0
    assert outcome(memory, act(OperatorActionKind.CONFIRM, event_id, EventState.CANDIDATE, n=1)) == OperatorOutcome.TARGET_CHANGED
    assert window(memory, ROOM, 6) == []
    assert outcome(memory, act(OperatorActionKind.CONFIRM, event_id, EventState.CANDIDATE, n=2), 6) == OperatorOutcome.APPLIED


def test_a_moved_rig_refuses_accept():
    memory = RoomMemory(config(), START, GENERATION)
    event_id = candidate(memory)
    memory.operate(act(OperatorActionKind.CONFIRM, event_id, EventState.CANDIDATE, n=1), stamp(5))
    window(memory, scenes.shifted(WITH_BOX, 0.05), 6)
    assert memory.channels["mid360"].residual > memory.config.learn_max_offset_m
    assert memory.channels["mid360"].tracker.open[0].missed == 0
    assert outcome(memory, act(OperatorActionKind.ACCEPT, event_id, EventState.CONFIRMED, n=2), 6) == OperatorOutcome.RIG_UNSETTLED


def test_a_repeated_action_id_acts_once():
    memory = RoomMemory(config(), START, GENERATION)
    event_id = candidate(memory)
    confirm = act(OperatorActionKind.CONFIRM, event_id, EventState.CANDIDATE, n=1)
    assert len(memory.operate(confirm, stamp(5))[1]) == 1
    assert memory.operate(confirm, stamp(6)) == (OperatorOutcome.APPLIED, [], True, False)
    changed = OperatorAction(*[getattr(confirm, f) for f in (
        "header", "action", "event_id", "expected_state", "expected_generation")], "a note", "trevor", confirm.action_id)
    assert memory.operate(changed, stamp(6)) == (OperatorOutcome.APPLIED, [], True, True)
    for n in range(op.STORED_ACTIONS):
        memory.operate(act(OperatorActionKind.CONFIRM, event_id + 100, EventState.CANDIDATE, n=1000 + n), stamp(7))
    assert memory.operate(confirm, stamp(8))[:3] == (OperatorOutcome.STATE_MISMATCH, [], False)


def test_published_geometry_is_what_accept_absorbs():
    memory = RoomMemory(config(), START, GENERATION)
    event_id = candidate(memory)
    memory.operate(act(OperatorActionKind.CONFIRM, event_id, EventState.CANDIDATE, n=1), stamp(5))
    published = region_keys(memory.channels["mid360"].tracker.open[0].published)
    assert outcome(memory, act(OperatorActionKind.ACCEPT, event_id, EventState.CONFIRMED, n=2)) == OperatorOutcome.APPLIED
    assert np.isin(published, memory.channels["mid360"].reference.occupied).all()
