"""C++ against the Python oracle, byte for byte: the offline CLI on the golden scenes and on a
grid of flag variations, and RoomMemory window sequences (the memory tests' scenarios, the
three Codex regressions of 289afb6, two-sensor runs, persistence, every RequestSync answer with
its SyncEnd, restarts across publisher generations) through the replay tools.

When CHANGE_DETECT names a built Swift ``change-detect``, the offline cases that Swift
accepts are compared against it too.
"""

import contextlib
import io
import os
import pathlib
import subprocess
import sys

import numpy as np
import pytest

import goldens
import replay
import scenes
from room_memory import offline
from room_memory.messages import (
    Activity, ChangeEvent, EventKind, EventState, Header, MapBricks, MapKind, MemoryStatus, RequestSync, SyncEnd,
    SyncTopic, Time, ZERO_TOKEN,
)

OFFLINE = os.environ.get("ROOM_MEMORY_OFFLINE")
REPLAY = os.environ.get("ROOM_MEMORY_REPLAY")
SWIFT = os.environ.get("CHANGE_DETECT")
SNAPSHOTS = pathlib.Path(
    os.environ.get("ROOM_MEMORY_SNAPSHOTS", goldens.HERE.parents[2] / "Tests" / "Fixtures" / "room" / "snapshots")
)
# Codex P2: height = width = 2^22, point_step 2^20, row_step 0, no data. The fallback row_step
# (2^42) x height overflows 64 bits; both decoders must reject it at the data-size check.
OVERFLOWING_CLOUD = (
    "000100000000000000000000010000000000000000004000000040000300000002000000780000000000000007000000010000000200"
    "000079000000040000000700000001000000020000007a00000008000000070000000100000000000000000010000000000000000000"
    "01"
)

# The memory tests' single-window mechanics (every voxel a window sees settles, one supported voxel
# makes a region fresh, no activity holds a region back, every region is sent and expired at once); scenarios written for them prepend it to their config line.
SINGLE_WINDOW = ("recent_windows=1 occupied_min_hits=1 occupied_min_ratio=0 free_min_seen=1 candidate_windows=1 "
                 "expire_windows=1 fresh_min_voxels=1 fresh_min_fraction=0 activity_radius_m=0")

# The generation replay gives `start 1760400000 0` (its start time in ms), and a request token.
G = 1760400000000
TOKEN = bytes.fromhex("5a1e7c3d9b024f618e33c4d5e6f70819")

pytestmark = pytest.mark.skipif(not (OFFLINE and REPLAY), reason="ROOM_MEMORY_OFFLINE / ROOM_MEMORY_REPLAY not set")


def python_offline(args):
    out, err = io.StringIO(), io.StringIO()
    with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
        code = offline.main(list(args))
    return code, out.getvalue().encode()


def cpp_offline(args):
    result = subprocess.run([OFFLINE, *args], capture_output=True)
    return result.returncode, result.stdout


def without_timing(text):
    return b"\n".join(line for line in text.split(b"\n") if not line.startswith(b"elapsed_ms="))


def assert_same(args, text_mode=False):
    expected = python_offline(args)
    actual = cpp_offline(args)
    if text_mode:
        expected = (expected[0], without_timing(expected[1]))
        actual = (actual[0], without_timing(actual[1]))
    assert actual == expected, f"C++ differs from Python for {args}"
    if SWIFT and expected[0] == 0 and not text_mode and "--no-evidence" not in args:
        swift = subprocess.run([SWIFT, "--evidence", "--json", *args], capture_output=True)
        assert swift.stdout == expected[1], f"Swift differs for {args}"
    return expected


def write_pair(tmp_path, name, before, after):
    before_path = tmp_path / f"{name}-before.bin"
    after_path = tmp_path / f"{name}-after.bin"
    before_path.write_bytes(scenes.encode_pointcloud2(before))
    after_path.write_bytes(scenes.encode_pointcloud2(after))
    return str(before_path), str(after_path)


# MARK: Offline CLI


@pytest.mark.parametrize("name", sorted(goldens.SCENES))
def test_golden_scene_cli(tmp_path, name):
    build, flags = goldens.SCENES[name]
    paths = write_pair(tmp_path, name, *build())
    # The goldens hold change-detect's output, whose default is evidence off.
    defaults = [] if "--evidence" in flags else ["--no-evidence"]
    code, out = assert_same([*defaults, *flags, *paths])
    assert code == 0 and out == (goldens.SWIFT_DIR / f"{name}.json").read_bytes()


def clutter_pair(seed):
    """Room shell plus random boxes; the after cloud moves, removes and adds some, with noise."""
    rng = np.random.default_rng(seed)
    room = scenes.room_shell()
    boxes = [tuple(rng.uniform([-1.0, -1.0, -0.4], [2.0, 2.0, 0.8])) for _ in range(5)]
    sides = rng.uniform(0.15, 0.6, size=5)
    before = [room] + [scenes.box_shell(center, side) for center, side in zip(boxes, sides)]
    after = [room, scenes.box_shell(boxes[0], sides[0]), scenes.shifted(scenes.box_shell(boxes[1], sides[1]), 0.6)]
    after.append(scenes.box_shell(tuple(rng.uniform([-1.0, -1.0, -0.4], [2.0, 2.0, 0.8])), 0.3))
    noise = lambda cloud: (cloud + rng.normal(0, 0.008, cloud.shape)).astype(np.float32)
    return noise(np.concatenate(before)), noise(np.concatenate(after))


BASES = {
    "moved": goldens._moved_pair,
    "board": goldens._board_pair,
    "jitter": goldens._jitter_pair,
    "speck": goldens._speck_pair,
    "clutter-1": lambda: clutter_pair(1),
    "clutter-2": lambda: clutter_pair(2),
    "column": lambda: (scenes.column(list(range(8, 17)) + list(range(20, 28))), scenes.column([16, 30])),
}

FLAGS = [
    [], ["--no-evidence"], ["--tolerance", "0"], ["--tolerance", "2"], ["--tolerance", "4"],
    ["--min-points", "2"], ["--min-voxels", "1", "--min-extent", "1"], ["--unobserved"],
    ["--unobserved", "--tolerance", "0", "--min-extent", "1"], ["--ray-stride", "4"], ["--ray-stride", "64"],
    ["--max-range", "2.2"], ["--origin", "0.3,-0.2,0.1"], ["--origin", "-1.4,2.4,-0.65", "--unobserved"],
    ["--voxel-size", "0.1"], ["--voxel-size", "0.03", "--tolerance", "2"], ["--voxel-size", "0.25", "--min-voxels", "2"],
    ["--no-pair"], ["--max-move", "0.4"], ["--no-evidence", "--no-pair", "--tolerance", "3"],
    ["--min-points", "3", "--unobserved", "--ray-stride", "2", "--max-range", "3"],
]


@pytest.mark.parametrize("base", sorted(BASES))
def test_flag_grid(tmp_path, base):
    paths = write_pair(tmp_path, base, *BASES[base]())
    for flags in FLAGS:
        assert_same([*flags, *paths])


@pytest.mark.parametrize("flags", [[], ["--tolerance", "0", "--unobserved"], ["--ray-stride", "8", "--min-points", "2"],
                                   ["--voxel-size", "0.1", "--no-pair"]])
def test_real_fixtures(flags):
    for sensor in ("jt128", "mid360"):
        assert_same([*flags, str(SNAPSHOTS / "a" / f"{sensor}.bin"), str(SNAPSHOTS / "b" / f"{sensor}.bin")])
    if goldens.PHASE0_DIR.is_dir():
        for _, before, after in goldens.phase0_pairs():
            assert_same([*flags, str(goldens.PHASE0_DIR / before), str(goldens.PHASE0_DIR / after)])


def test_large_scenes(tmp_path):
    indoor = write_pair(tmp_path, "indoor", scenes.indoor(False), scenes.indoor(True))
    for flags in ([], ["--unobserved", "--tolerance", "0"], ["--no-evidence"]):
        assert_same([*flags, *indoor])
    long_ray = write_pair(tmp_path, "long-ray", *scenes.long_ray())
    for flags in (["--max-range", "15", "--ray-stride", "16"], ["--max-range", "15", "--ray-stride", "32", "--unobserved"]):
        assert_same([*flags, *long_ray])


def test_text_mode(tmp_path):
    paths = write_pair(tmp_path, "moved", *goldens._moved_pair())
    for flags in (["--text"], ["--text", "--no-evidence"], ["--text", "--min-voxels", "1", "--unobserved"]):
        assert_same([*flags, *paths], text_mode=True)
    assert_same(["--text", "--min-extent", "1", str(SNAPSHOTS / "a" / "mid360.bin"), str(SNAPSHOTS / "b" / "mid360.bin")], text_mode=True)


def test_errors_exit_alike(tmp_path):
    good = str(SNAPSHOTS / "a" / "mid360.bin")
    bad = tmp_path / "broken.bin"
    bad.write_bytes(b"\x00\x01\x00\x00\x01")
    cases = [[], [good], ["--bogus", good, good], ["--tolerance", "9", good, good], ["--ray-stride", "3", good, good],
             [str(tmp_path / "missing.bin"), good], [good, str(tmp_path / "cloud.pcd")], [good, str(bad)],
             ["--voxel-size", "nan", good, good], ["--origin", "1,2,x", good, good]]
    for args in cases:
        assert cpp_offline(args)[0] == python_offline(args)[0] == 1, args


# MARK: RoomMemory sequences


def floats(tmp_path, name, cloud):
    path = tmp_path / f"{name}.f32"
    np.asarray(cloud, dtype="<f4").reshape(-1, 3).tofile(path)
    return path


def shared_state(state):
    """Reference files both engines write. The durable event log (D47) is C++ only."""
    if not state.exists():
        return {}
    files = {}
    for path in sorted(state.rglob("*")):
        if not path.is_file():
            continue
        relative = path.relative_to(state)
        if "events" in relative.parts:
            continue
        files[relative] = path.read_bytes()
    return files


def run_both(tmp_path, script):
    """Runs ``script`` (with {state} for a per-side state dir) through C++ and Python."""
    outputs, states = [], []
    for side in ("cpp", "py"):
        state = tmp_path / f"state-{side}"
        text = script.replace("{state}", str(state))
        if side == "cpp":
            path = tmp_path / "script.txt"
            path.write_text(text)
            result = subprocess.run([REPLAY, str(path)], capture_output=True, check=True)
            outputs.append(result.stdout.decode())
        else:
            outputs.append(replay.run(text))
        states.append(shared_state(state))
    assert outputs[0] == outputs[1]
    assert states[0] == states[1]
    return outputs[1]


def test_engines_share_the_node_defaults(tmp_path):
    """Every MemoryConfig default, field by field (numbers compared as numbers)."""
    sides = []
    for text in (subprocess.run([REPLAY, str(_script(tmp_path, "defaults"))], capture_output=True, check=True).stdout.decode(),
                 replay.run("defaults")):
        sides.append(dict(line.split(" ", 2)[1:] for line in text.splitlines()))
    cpp, py = sides
    assert sorted(cpp) == sorted(py)
    for key, value in py.items():
        try:
            assert float(cpp[key]) == float(value), key
        except ValueError:
            assert cpp[key] == value, key


def _script(tmp_path, text):
    path = tmp_path / "defaults.txt"
    path.write_text(text + "\n")
    return path


def test_board_reference_and_lifecycle(tmp_path):
    before, after, _, _ = scenes.board_scene()
    b, a = floats(tmp_path, "before", before), floats(tmp_path, "after", after)
    out = run_both(tmp_path, f"""
config {SINGLE_WINDOW} sensors=mid360 max_range=none reference_windows=3 min_extent=1 state_dir={{state}}
start 1760400000 0
status 1760400000 0 1.0
points mid360 {b}
status 1760400000 0 1.0
points mid360 {b}
close 1760400000 0
points mid360 {b}
close 1760400001 0
points mid360 {b}
close 1760400002 0
status 1760400003 0 1.0
points mid360 {a}
close 1760400003 0
points mid360 {a}
close 1760400004 0
status 1760400004 0 2.0
points mid360 {b}
close 1760400005 0
status 1760400005 0 1.0
""")
    assert out.count("event ") == 6


def test_new_coverage_and_moves(tmp_path):
    room = scenes.room_shell()
    box = scenes.box_shell((0.0, 0.5, 0.30))
    paths = {
        "partial": floats(tmp_path, "partial", room[room[:, 0] < 2.4]),
        "room": floats(tmp_path, "room", room),
        "box": floats(tmp_path, "box", np.concatenate([room, box])),
        "moved": floats(tmp_path, "moved", np.concatenate([room, scenes.shifted(box, 1)])),
        "added": floats(tmp_path, "added", np.concatenate([room, scenes.box_shell((0.3, 1.2, 0.3))])),
    }
    lines = ["config sensors=mid360 max_range=none reference_windows=3 state_dir=none candidate_windows=3",
             "start 1760400000 0"]
    # The gate needs 3 settled windows: the box added at the end stays for three.
    sequence = ["partial", "partial", "partial", "room", "added", "box", "moved", "moved", "room", "added", "added",
                "added"]
    for second, name in enumerate(sequence):
        lines += [f"points mid360 {paths[name]}", f"close {1760400000 + second} 0"]
    out = run_both(tmp_path, "\n".join(lines))
    assert "event " in out


def test_persistence_and_reload(tmp_path):
    room = floats(tmp_path, "room", np.concatenate([scenes.room_shell(), scenes.box_shell((1.0, 0.5, 0.3))]))
    build = f"""
config sensors=mid360,jt128 max_range=none reference_windows=2 room_id=lab-7 state_dir={{state}}
start 1760400000 0
points mid360 {room}
points jt128 {room}
close 1760400001 0
points mid360 {room}
points jt128 {room}
close 1760400002 0
status 1760400002 0 1.0
"""
    reload = """
config sensors=mid360,jt128 max_range=none reference_windows=2 room_id=lab-7 state_dir={state}
start 1760400100 0
status 1760400100 0 1.0
config sensors=mid360 voxel_size=0.1
start 1760400100 0
config sensors=mid360 voxel_size=0.05 room_id=other
start 1760400100 0
config sensors=mid360,jt128 room_id=lab-7
start 1760400200 0
"""
    run_both(tmp_path, build)
    out = run_both(tmp_path, reload).splitlines()
    # Each load takes the next epoch and saves it; a refused load saves nothing.
    assert out[0] == "init TRACKING 2" and out[2] == "init error" and out[4] == "init TRACKING 3"


def test_sync_cursor_across_restarts(tmp_path):
    """Codex P2: seq restarts at 0, so a restart must not reissue an old (map_epoch, seq)
    cursor; the pre-restart cursor gets the post-restart event."""
    room = scenes.room_shell()
    plain = floats(tmp_path, "room", room)
    boxed = floats(tmp_path, "boxed", np.concatenate([room, scenes.box_shell((0.3, 1.2, 0.3))]))
    lines = [f"config {SINGLE_WINDOW} sensors=mid360 max_range=none reference_windows=3 state_dir={{state}}",
             "start 1760400000 0"]
    for second in range(3):
        lines += [f"points mid360 {plain}", f"close {1760400000 + second} 0"]
    lines += [f"points mid360 {boxed}", "close 1760400003 0", "start 1760400100 0", "start 1760400200 0",
              f"points mid360 {boxed}", "close 1760400201 0",
              f"sync 1760400202 0 {request('lab', (1, 0), SyncTopic.EVENTS)}"]
    out = run_both(tmp_path, "\n".join(lines)).splitlines()
    events = [ChangeEvent.from_cdr(bytes.fromhex(line.split()[1])) for line in out if line.startswith("event ")]
    assert [(e.header.map_epoch, e.header.seq) for e in events] == [(1, 0), (3, 0)]
    assert events[0].event_id != events[1].event_id
    assert [line for line in out if line.startswith("init ")] == ["init BUILDING 0", "init TRACKING 2", "init TRACKING 3"]
    # The old cursor belongs to the first run's generation: answered as SINCE_NONE in this one.
    assert out[-3:-1] == ["sync 1", f"data {events[1].to_cdr().hex()}"]
    end = SyncEnd.from_cdr(bytes.fromhex(out[-1].split()[1]))
    assert end.publisher_generation == 1760400200000 and end.delivered == ((0, 0),) and end.unavailable == ()


def test_sensor_subset_restarts(tmp_path):
    """Codex P2: a restart with sensors=mid360 saved only reference-mid360.cdr at the next epoch,
    so the saved references disagreed and a restart with both sensors refused the room. The room
    epoch file keeps every start one past the room's last epoch, whichever sensors ran."""
    room = scenes.room_shell()
    partial = floats(tmp_path, "partial", room[room[:, 0] < 1.5])
    full = floats(tmp_path, "room", room)
    lines = [f"config {SINGLE_WINDOW} sensors=mid360,jt128 max_range=none reference_windows=3 learn_windows=30 "
             "state_dir={state}",
             "start 1760400000 0"]
    for second in range(3):
        lines += [f"points mid360 {partial}", f"points jt128 {partial}", f"close {1760400000 + second} 0"]
    # The mid360-only run learns the far wall, so its reference moves ahead of jt128's.
    lines += ["config sensors=mid360 delta_period_s=1", "start 1760400100 0", "map 1760400110 0",
              f"points mid360 {full}", "close 1760400111 0", "map 1760400111 0",
              "config sensors=mid360,jt128", "start 1760400200 0", "start 1760400300 0"]
    # A room with only a jt128 reference: the mid360-only build continues the room epoch.
    lines += ["config room_id=lab-2 sensors=jt128", "start 1760400400 0"]
    for second in range(3):
        lines += [f"points jt128 {full}", f"close {1760400400 + second} 0"]
    lines += ["config sensors=mid360", "start 1760400500 0"]
    for second in range(3):
        lines += [f"points mid360 {full}", f"close {1760400500 + second} 0"]
    lines += ["config sensors=mid360,jt128", "start 1760400600 0"]
    out = run_both(tmp_path, "\n".join(lines)).splitlines()
    assert [line for line in out if line.startswith("init ")] == [
        "init BUILDING 0", "init TRACKING 2", "init TRACKING 4", "init TRACKING 5",
        "init BUILDING 0", "init BUILDING 1", "init TRACKING 3"]
    maps = [MapBricks.from_cdr(bytes.fromhex(line.split()[1])) for line in out if line.startswith("map ")]
    assert [m.header.map_epoch for m in maps] == [2, 3]
    assert [line for line in out if line.startswith("close ")][-1] == "close 0 TRACKING 2"


def test_degraded_and_dropped(tmp_path):
    room = scenes.room_shell()
    cdr = tmp_path / "room.bin"
    cdr.write_bytes(scenes.encode_pointcloud2(room))
    r = floats(tmp_path, "room", room)
    run_both(tmp_path, f"""
config state_dir=none reference_windows=1 occupied_min_hits=1
start 1760400000 0
points mid360 {r}
hex jt128 00010000
close 1760400000 0
status 1760400001 0 1.0
cdr mid360 {cdr}
cdr jt128 {cdr}
close 1760400001 0
status 1760400002 0 1.0
status 1760400003 0 1.0
hex mid360 0001
hex jt128 {OVERFLOWING_CLOUD}
close 1760400004 0
status 1760400004 0 0.5
status 1760400005 0 0
""")


def test_codex_reference_needs_usable_data(tmp_path):
    room = scenes.room_shell()
    paths = [floats(tmp_path, "empty", np.zeros((0, 3), dtype=np.float32)),
             floats(tmp_path, "nan", np.full((50, 3), np.nan, dtype=np.float32)),
             floats(tmp_path, "far", room + 50), floats(tmp_path, "room", room),
             floats(tmp_path, "added", np.concatenate([room, scenes.box_shell((0.3, 1.2, 0.3))]))]
    lines = [f"config {SINGLE_WINDOW} sensors=mid360 max_range=15 reference_windows=3 state_dir={{state}}",
             "start 1760400000 0"]
    for second, path in enumerate(paths[:3] + [paths[3]] * 3 + [paths[4]]):
        lines += [f"points mid360 {path}", f"close {1760400000 + second} 0", f"status {1760400000 + second} 0 1.0"]
    out = run_both(tmp_path, "\n".join(lines))
    assert "close 1 TRACKING 1" in out


def test_codex_per_sensor_freeze(tmp_path):
    room = scenes.room_shell()
    r = floats(tmp_path, "room", room)
    boxed = floats(tmp_path, "boxed", np.concatenate([room, scenes.box_shell((0.3, 1.2, 0.3))]))
    out = run_both(tmp_path, f"""
config {SINGLE_WINDOW} state_dir=none max_range=none reference_windows=1
start 1760400000 0
points mid360 {r}
close 1760400000 0
points mid360 {boxed}
points jt128 {r}
close 1760400001 0
points mid360 {boxed}
close 1760400002 0
points mid360 {boxed}
points jt128 {boxed}
close 1760400003 0
status 1760400003 0 1.0
""")
    assert "close 1 TRACKING 1" in out


def test_codex_recent_windows_age_through_gaps(tmp_path):
    room = scenes.room_shell()
    r = floats(tmp_path, "room", room)
    boxed = floats(tmp_path, "boxed", np.concatenate([room, scenes.box_shell((0.3, 1.2, 0.3))]))
    lines = [f"config {SINGLE_WINDOW} sensors=mid360 max_range=none reference_windows=3 recent_windows=3 state_dir=none",
             "start 1760400000 0"]
    for second in range(3):
        lines += [f"points mid360 {r}", f"close {1760400000 + second} 0"]
    lines += [f"points mid360 {boxed}", "close 1760400003 0", "close 1760400004 0", "hex mid360 00010000",
              "close 1760400005 0", f"points mid360 {r}", "close 1760400006 0", f"points mid360 {boxed}",
              f"points mid360 {r}", "close 1760400007 0", "close 1760400008 0", "close 1760400009 0",
              "close 1760400010 0"]
    out = run_both(tmp_path, "\n".join(lines))
    assert out.count("event ") >= 2


def test_codex_tracking_waits_for_reference_occupancy(tmp_path):
    """Codex P2: one reference window under occupied_min_hits=2 settles nothing; both engines keep
    building until the reference has occupancy, then track (and detect) as usual."""
    room = scenes.room_shell()
    r = floats(tmp_path, "room", room)
    boxed = floats(tmp_path, "boxed", np.concatenate([room, scenes.box_shell((0.3, 1.2, 0.3))]))
    lines = ["config sensors=mid360 state_dir={state} max_range=none reference_windows=1 candidate_windows=3",
             "start 1760400000 0"]
    for second, path in enumerate([r, r, boxed, boxed, boxed]):
        lines += [f"points mid360 {path}", f"close {1760400000 + second} 0", f"status {1760400000 + second} 0 1.0"]
    out = run_both(tmp_path, "\n".join(lines))
    closes = [line for line in out.splitlines() if line.startswith("close ")]
    assert closes == ["close 0 BUILDING 0"] + ["close 0 TRACKING 1"] * 3 + ["close 1 TRACKING 1"]


def test_codex_candidates_need_fresh_windows(tmp_path):
    """Codex P2: a box in one window stays in the 3-window recent layer but is no candidate; a box
    in three windows is, in both engines."""
    room = scenes.room_shell()
    r = floats(tmp_path, "room", room)
    boxed = floats(tmp_path, "boxed", np.concatenate([room, scenes.box_shell((0.3, 1.2, 0.3))]))
    lines = [f"config {SINGLE_WINDOW} sensors=mid360 max_range=none reference_windows=3 recent_windows=3 "
             "candidate_windows=3 expire_windows=3 state_dir=none", "start 1760400000 0"]
    for second, path in enumerate([r, r, r, boxed, r, r, r, r, boxed, boxed, boxed, r]):
        lines += [f"points mid360 {path}", f"close {1760400000 + second} 0"]
    out = run_both(tmp_path, "\n".join(lines))
    events = [int(line.split()[1]) for line in out.splitlines() if line.startswith("close ")]
    assert events == [0] * 10 + [1, 0]


def test_codex_disappearances_need_fresh_rays(tmp_path):
    """Codex P2: free space seen in one window stays in the 3-window recent layer, but only windows
    whose own rays cross the removed box count; three such windows make a candidate, in both engines."""
    room = scenes.room_shell()
    r = floats(tmp_path, "room", room)
    half = floats(tmp_path, "half", room[room[:, 1] < 0])
    boxed = floats(tmp_path, "boxed", np.concatenate([room, scenes.box_shell((0.3, 1.2, 0.3))]))
    lines = [f"config {SINGLE_WINDOW} sensors=mid360 max_range=none reference_windows=3 recent_windows=3 "
             "candidate_windows=3 expire_windows=3 state_dir=none", "start 1760400000 0"]
    for second, path in enumerate([boxed, boxed, boxed, r, half, half, half, r, r, r, half]):
        lines += [f"points mid360 {path}", f"close {1760400000 + second} 0"]
    out = run_both(tmp_path, "\n".join(lines))
    events = [int(line.split()[1]) for line in out.splitlines() if line.startswith("close ")]
    assert events == [0] * 9 + [1, 0]


def test_a_move_seen_as_its_halves_stays_one_move(tmp_path):
    """A corner of the moved box is too small for the move-count gate (48 vs 148 voxels), so the diff
    reports an APPEARED and a DISAPPEARED region; both engines keep the open MOVED alive on them,
    send nothing new, and expire it once the box is back."""
    room = scenes.room_shell()
    box = scenes.box_shell((0.0, 0.5, 0.30))
    moved = scenes.shifted(box, 1)
    low = moved.min(axis=0)
    corner = moved[(moved[:, 1] <= low[1] + 0.1) & (moved[:, 2] <= low[2] + 0.1)]
    paths = {name: floats(tmp_path, name, np.concatenate([room, cloud]))
             for name, cloud in (("box", box), ("moved", moved), ("corner", corner))}
    lines = [f"config {SINGLE_WINDOW} sensors=mid360 max_range=none reference_windows=3 expire_windows=2 state_dir=none",
             "start 1760400000 0"]
    for second, name in enumerate(["box", "box", "box", "moved", "corner", "corner", "box", "box"]):
        lines += [f"points mid360 {paths[name]}", f"close {1760400000 + second} 0"]
    out = run_both(tmp_path, "\n".join(lines))
    events = [int(line.split()[1]) for line in out.splitlines() if line.startswith("close ")]
    assert events == [0, 0, 0, 1, 0, 0, 0, 1]


def test_codex_a_lone_half_keeps_no_move_alive(tmp_path):
    """Codex P2: a box moved and then removed from its new pose left only the DISAPPEARED at the old
    one, which kept the MOVED open forever; both engines now expire it and report the removal."""
    room = scenes.room_shell()
    box = scenes.box_shell((0.0, 0.5, 0.30))
    paths = {name: floats(tmp_path, name, cloud) for name, cloud in (
        ("box", np.concatenate([room, box])), ("moved", np.concatenate([room, scenes.shifted(box, 1)])), ("room", room))}
    lines = [f"config {SINGLE_WINDOW} sensors=mid360 max_range=none reference_windows=3 recent_windows=3 "
             "candidate_windows=3 expire_windows=3 state_dir=none", "start 1760400000 0"]
    for second, name in enumerate(["box"] * 3 + ["moved"] * 3 + ["room"] * 13):
        lines += [f"points mid360 {paths[name]}", f"close {1760400000 + second} 0"]
    out = run_both(tmp_path, "\n".join(lines))
    events = [int(line.split()[1]) for line in out.splitlines() if line.startswith("close ")]
    assert events == [0] * 5 + [1] + [0] * 4 + [1] + [0] * 2 + [1] + [0] * 5


@pytest.mark.parametrize("variant", [
    "tolerance=1 recent_windows=2",
    "tolerance=0 recent_windows=1 min_extent=1",
    "tolerance=2 recent_windows=3 voxel_size=0.1",
    "tolerance=1 recent_windows=1 max_range=2.5 min_voxels=3",
])
def test_two_sensor_sequences(tmp_path, variant):
    rng = np.random.default_rng(42)
    room = scenes.room_shell()
    box_a, box_b = scenes.box_shell((0.2, 0.4, 0.2)), scenes.box_shell((1.4, 1.6, 0.1), 0.3)
    scenes_by_name = {
        "base": [room, box_a],
        "added": [room, box_a, box_b],
        "moved": [room, scenes.shifted(box_a, 0.8), box_b],
        "removed": [room, box_b],
    }
    # Motion next to box_b holds it back (activity); the settled tail promotes it.
    sequence = ["base", "base", "base", "added", "added", "moved", "moved", "removed", "base", "base"] + ["added"] * 5
    lines = [f"config state_dir=none reference_windows=2 candidate_windows=3 {variant}", "start 1760400000 500000000"]
    for second, name in enumerate(sequence):
        cloud = np.concatenate(scenes_by_name[name])
        for sensor, sigma in (("mid360", 0.004), ("jt128", 0.009)):
            for frame in range(2):
                pick = cloud[rng.random(len(cloud)) < 0.8]
                noisy = (pick + rng.normal(0, sigma, pick.shape)).astype(np.float32)
                lines.append(f"points {sensor} {floats(tmp_path, f'{second}-{sensor}-{frame}', noisy)}")
        lines += [f"close {1760400001 + second} 0", f"status {1760400001 + second} 0 1.0",
                  f"activity {1760400001 + second} 0"]
    out = run_both(tmp_path, "\n".join(lines))
    activity = [Activity.from_cdr(bytes.fromhex(line.split()[1])) for line in out.splitlines() if line.startswith("activity ")]
    assert any(message.active for message in activity)
    # recent_windows 1 with 80 % of the points per window rarely holds one shape: mostly activity.
    assert "event " in out or "recent_windows=1" in variant


@pytest.mark.parametrize("variant", [
    "candidate_windows=6",
    "candidate_windows=6 activity_radius_m=0.3 stable_min_fraction=1.0",
    "candidate_windows=4 fresh_min_voxels=1 fresh_min_fraction=0.8 tolerance=2",
])
def test_settle_gate_and_activity(tmp_path, variant):
    """The node defaults but a shorter gate: a box held back while a second box keeps moving next to
    it, then counted once that stops; a move of the box, seen whole or as halves; Activity each second."""
    rng = np.random.default_rng(11)
    room = scenes.room_shell()
    box = scenes.box_shell((0.3, 1.2, 0.3))
    person = [scenes.box_shell((0.9, 1.0 + 0.25 * (n % 3), 0.3), 0.3) for n in range(3)]
    sequence = [[room]] * 3 + [[room, box, person[n % 3]] for n in range(6)] + [[room, box]] * 9
    sequence += [[room, scenes.shifted(box, 0.6)]] * 9
    lines = [f"config state_dir=none reference_windows=3 max_range=none {variant}", "start 1760400000 0"]
    for second, parts in enumerate(sequence):
        cloud = np.concatenate(parts)
        for sensor, sigma in (("mid360", 0.004), ("jt128", 0.008)):
            pick = cloud[rng.random(len(cloud)) < 0.85]
            noisy = (pick + rng.normal(0, sigma, pick.shape)).astype(np.float32)
            lines.append(f"points {sensor} {floats(tmp_path, f'{second}-{sensor}', noisy)}")
        stamp = f"{1760400001 + second} 0"
        lines += [f"close {stamp}", f"status {stamp} 1.0", f"activity {stamp}"]
    out = run_both(tmp_path, "\n".join(lines))
    activity = [Activity.from_cdr(bytes.fromhex(line.split()[1])) for line in out.splitlines() if line.startswith("activity ")]
    assert any(message.active for message in activity[3:9]) and not any(message.active for message in activity[:3])
    assert "event " in out


def test_bench_scale_window(tmp_path):
    """One sensor pair at the benchmark's scale: 4 x 80,000-point frames per window."""
    rng = np.random.default_rng(7)
    still, moved = scenes.indoor(False), scenes.indoor(True)
    lines = [f"config {SINGLE_WINDOW} state_dir=none reference_windows=1", "start 0 0"]
    for window, cloud in enumerate([still, still, moved, moved]):
        for frame in range(4):
            pick = cloud[rng.choice(len(cloud), 80_000, replace=False)]
            path = floats(tmp_path, f"{window}-{frame}", (pick + rng.normal(0, 0.01, pick.shape)).astype(np.float32))
            lines += [f"points mid360 {path}", f"points jt128 {path}"]
        lines.append(f"close {window + 1} 0")
    out = run_both(tmp_path, "\n".join(lines))
    assert out.count("event ") == 2


def request(room, since, topic, seq=0, expected=G, token=TOKEN):
    """A v2 request in generation ``expected``; ``expected=None`` makes a legacy one."""
    epoch = (1 << 64) - 1 if since is None else since[0]
    return RequestSync(Header(room, epoch, seq, Time(1760400000, 0)), since, topic, expected,
                       None if expected is None else token).to_cdr().hex()


def answers(out):
    """Each ``sync`` answer: (messages as bytes, SyncEnd or None), in order; ``sync invalid`` is skipped."""
    found, current = [], None
    for line in out.splitlines():
        word, _, rest = line.partition(" ")
        if word == "sync" and rest != "invalid":
            current = []
        elif word == "data":
            current.append(bytes.fromhex(rest))
        elif word == "end":
            found.append((current, None if rest == "none" else SyncEnd.from_cdr(bytes.fromhex(rest))))
    return found


@pytest.mark.parametrize("variant", [
    "keyframe_period_s=7 keyframe_delta_ratio=0.5 candidate_windows=2 expire_windows=2",
    "keyframe_period_s=100 keyframe_delta_ratio=0.02 candidate_windows=3 expire_windows=1",
    "keyframe_period_s=3 keyframe_delta_ratio=5 candidate_windows=1 expire_windows=3 map_sensor=mid360",
])
def test_settled_reference_map_stream_and_sync(tmp_path, variant):
    """Count-thresholded references, new coverage joining them as deltas, keyframes, the
    candidate gate, and every RequestSync answer path."""
    rng = np.random.default_rng(5)
    room = scenes.room_shell()
    box = scenes.box_shell((0.3, 1.2, 0.3))
    clouds = {"partial": room[room[:, 0] < 1.5], "room": room, "box": np.concatenate([room, box])}
    lines = [f"config sensors=mid360,jt128 max_range=none reference_windows=4 occupied_min_hits=2 "
             f"occupied_min_ratio=0.3 free_min_seen=2 free_max_ratio=0.2 map_sensor=jt128 delta_period_s=2 learn_windows=30 "
             f"room_id=lab state_dir={{state}} {variant}", "start 1760400000 0"]
    sequence = ["partial"] * 4 + ["room"] * 5 + ["box"] * 4 + ["room"] * 4 + ["box", "room", "box"] + ["room"] * 3
    syncs = {6: [(None, SyncTopic.MAP_BRICKS)], 12: [("map", SyncTopic.MAP_BRICKS), (None, SyncTopic.EVENTS)],
             18: [("event", SyncTopic.EVENTS), ("last event", SyncTopic.EVENTS), ((1, 99), SyncTopic.MAP_BRICKS)]}
    for second, name in enumerate(sequence):
        for sensor, sigma in (("mid360", 0.004), ("jt128", 0.008)):
            cloud = clouds[name]
            pick = cloud[rng.random(len(cloud)) < 0.7]
            noisy = (pick + rng.normal(0, sigma, pick.shape)).astype(np.float32)
            lines.append(f"points {sensor} {floats(tmp_path, f'{second}-{sensor}', noisy)}")
        stamp = f"{1760400000 + second} 0"
        lines += [f"close {stamp}", f"map {stamp}", f"status {stamp} 1.0"]
        for n, (since, topic) in enumerate(syncs.get(second, [])):
            if isinstance(since, str):
                # (map_epoch, seq) of the first map message, or of the first or last event so far.
                prefix = replay.run("\n".join(lines).replace("{state}", str(tmp_path / f"probe-{second}-{n}")))
                kind = "map" if since == "map" else "event"
                found = [line.split()[1] for line in prefix.splitlines() if line.startswith(kind + " ") and line != "map none"]
                cls = MapBricks if kind == "map" else ChangeEvent
                header = cls.from_cdr(bytes.fromhex(found[-1 if since == "last event" else 0])).header
                since = (header.map_epoch, header.seq)
            lines.append(f"sync {stamp} {request('lab', since, topic, n)}")
        if second == 18:
            lines += [f"sync {stamp} {request('other', None, SyncTopic.MAP_BRICKS, 6)}", f"sync {stamp} 0001000001"]
    lines += ["config state_dir={state}", "start 1760400100 0", "status 1760400100 0 1.0", "map 1760400100 0"]
    out = run_both(tmp_path, "\n".join(lines))
    maps = [MapBricks.from_cdr(bytes.fromhex(line.split()[1])) for line in out.splitlines()
            if line.startswith("map ") and line != "map none"]
    kinds = {message.kind for message in maps}
    assert MapKind.SNAPSHOT in kinds and (MapKind.DELTA in kinds or "ratio=0.02" in variant)
    assert [m.header.seq for m in maps[:-1]] == list(range(len(maps) - 1))
    answers = [line for line in out.splitlines() if line.startswith("sync ")]
    # Replays: the map since the first message, the events since the first one and since the latest (nothing missed).
    assert answers[4] == "sync 0" and answers[-2:] == ["sync 0", "sync invalid"] and answers[0] == "sync 1"


def _steps(out):
    """Each close, the events it emitted, and the map line that follows it."""
    steps = []
    current = None
    for line in out.splitlines():
        if line.startswith("close "):
            current = {"events": [], "map": None}
            steps.append(current)
        elif line.startswith("event ") and current is not None:
            current["events"].append(ChangeEvent.from_cdr(bytes.fromhex(line.split()[1])))
        elif line.startswith("map ") and current is not None:
            payload = line.split()[1]
            current["map"] = None if payload == "none" else MapBricks.from_cdr(bytes.fromhex(payload))
    return steps


def _both_points(paths, name):
    return [f"points mid360 {paths[name]}", f"points jt128 {paths[name]}"]


def test_learn_budget_freezes_both_sensors_and_keeps_keyframes(tmp_path):
    """Budget 1, due tick 3 s later: the new wall is published on that tick, then keyframes repeat it."""
    room = scenes.room_shell()
    partial = room[room[:, 0] < 1.5]
    paths = {
        "partial": floats(tmp_path, "partial", partial),
        "room": floats(tmp_path, "room", room),
        "boxed": floats(tmp_path, "boxed", np.concatenate([room, scenes.box_shell((0.3, 1.2, 0.3))])),
        "far": floats(tmp_path, "far", np.concatenate([room, scenes.box_shell((8.0, 8.0, 0.5))])),
    }
    lines = [f"config {SINGLE_WINDOW} sensors=mid360,jt128 max_range=none reference_windows=1 learn_windows=1 "
             "delta_period_s=3 keyframe_period_s=4 keyframe_delta_ratio=100 state_dir={state}",
             "start 1760400000 0"]
    for second, name in ((0, "partial"), (1, "room"), (2, "room"), (3, "room"), (4, "room"), (8, "room"),
                         (9, "boxed"), (10, "far"), (12, "room")):
        stamp = f"{1760400000 + second} 0"
        lines += _both_points(paths, name) + [f"close {stamp}", f"map {stamp}"]
    steps = _steps(run_both(tmp_path, "\n".join(lines)))
    assert steps[0]["map"].kind == MapKind.SNAPSHOT and steps[0]["map"].header.map_epoch == 1
    assert steps[1]["map"] is None and steps[2]["map"] is None
    assert steps[3]["map"].kind == MapKind.DELTA and steps[3]["map"].header.map_epoch == 2
    learned = steps[4]["map"]
    assert learned.kind == MapKind.SNAPSHOT and learned.header.map_epoch == 2
    assert len(learned.bricks) > len(steps[0]["map"].bricks)
    assert steps[5]["map"].kind == MapKind.SNAPSHOT and steps[5]["map"].bricks == learned.bricks
    assert steps[5]["map"].header.map_epoch == 2
    assert steps[6]["map"] is None and any(
        event.kind == EventKind.APPEARED and event.state == EventState.CANDIDATE for event in steps[6]["events"])
    # The open box expires. The outside cluster starts no candidate.
    assert steps[7]["map"] is None and all(event.state != EventState.CANDIDATE for event in steps[7]["events"])
    assert steps[8]["map"].kind == MapKind.SNAPSHOT and steps[8]["map"].header.map_epoch == 2
    assert steps[8]["map"].bricks == learned.bricks


def test_empty_and_shifted_windows_spend_the_learn_budget(tmp_path):
    room = scenes.room_shell()
    partial = room[room[:, 0] < 1.5]
    shifted = (partial + np.float32([0.1, 0.05, 0.1])).astype(np.float32)
    paths = {"partial": floats(tmp_path, "partial", partial), "room": floats(tmp_path, "room", room),
             "shifted": floats(tmp_path, "shifted", shifted)}
    config = (f"config {SINGLE_WINDOW} sensors=mid360,jt128 max_range=none reference_windows=1 learn_windows=1 "
              "delta_period_s=3 keyframe_period_s=4 keyframe_delta_ratio=100 state_dir={state}")

    def script(middle):
        lines = [config, "start 1760400000 0"]
        for second, name in [(0, "partial"), *middle, (3, "room"), (4, "room")]:
            stamp = f"{1760400000 + second} 0"
            if name is not None:
                lines += _both_points(paths, name)
            lines += [f"close {stamp}", f"map {stamp}"]
        return "\n".join(lines)

    for name, middle in (("empty", [(1, None)]), ("shifted", [(1, "shifted")])):
        side = tmp_path / name
        side.mkdir()
        steps = _steps(run_both(side, script(middle)))
        opening = steps[0]["map"]
        assert opening.kind == MapKind.SNAPSHOT and opening.header.map_epoch == 1
        assert steps[1]["map"] is None and steps[2]["map"] is None
        key = steps[3]["map"]
        assert key.kind == MapKind.SNAPSHOT and key.header.map_epoch == 1 and key.bricks == opening.bricks


def test_learn_windows_zero_reloads_without_learning(tmp_path):
    room = scenes.room_shell()
    partial = room[room[:, 0] < 1.5]
    paths = {"partial": floats(tmp_path, "partial", partial), "room": floats(tmp_path, "room", room)}
    lines = [f"config {SINGLE_WINDOW} sensors=mid360,jt128 max_range=none reference_windows=1 learn_windows=0 "
             "delta_period_s=1 keyframe_period_s=100 state_dir={state}", "start 1760400000 0"]
    lines += _both_points(paths, "partial") + ["close 1760400000 0", "map 1760400000 0",
                                                "config state_dir={state}", "start 1760400010 0", "map 1760400010 0"]
    lines += _both_points(paths, "room") + ["close 1760400011 0", "map 1760400011 0"]
    out = run_both(tmp_path, "\n".join(lines))
    assert "init TRACKING 2" in out
    maps = [None if line.split()[1] == "none" else MapBricks.from_cdr(bytes.fromhex(line.split()[1]))
            for line in out.splitlines() if line.startswith("map ")]
    assert maps[0].kind == MapKind.SNAPSHOT and maps[0].header.map_epoch == 1
    assert maps[1].header.map_epoch == 2 and maps[1].bricks == maps[0].bricks
    assert maps[2] is None
    closes = [line.split() for line in out.splitlines() if line.startswith("close ")]
    assert closes[-1][1] == "0" and closes[-1][3] == "2"


EXCERPT = goldens.HERE.parents[1] / "room_memory_cpp" / "tests" / "fixtures" / "static5m-mid360-excerpt.mcap"


def test_real_mid360_frames(tmp_path):
    """Eight real Mid-360 frames (tools/bag_excerpt.py from the static recording, 0.8 s, about
    a fifth of the points zero-range non-returns) through both engines in 0.2 s windows: the
    count rule, learning, the map stream and both sync answers on real data."""
    pytest.importorskip("mcap", reason="the mcap package reads the excerpt (pip install mcap)")
    sys.path.insert(0, str(goldens.HERE.parents[1] / "room_memory_cpp" / "tools"))
    import bag_stream

    frames = [data for topic, _, data in bag_stream.messages([EXCERPT]) if topic == "/livox/lidar"]
    assert len(frames) == 8
    lines = ["config sensors=mid360 window_s=0.2 reference_windows=2 recent_windows=2 learn_windows=30 occupied_min_hits=2 "
             "occupied_min_ratio=0.5 free_min_seen=2 candidate_windows=1 expire_windows=2 delta_period_s=0.2 "
             "keyframe_period_s=0.6 keyframe_delta_ratio=5 state_dir={state}", "start 1792014385 0"]
    for n, data in enumerate(frames):
        path = tmp_path / f"frame-{n}.cdr"
        path.write_bytes(data)
        lines.append(f"cdr mid360 {path}")
        if n % 2:
            stamp = f"1792014385 {(n + 1) * 100_000_000}"
            lines += [f"close {stamp}", f"map {stamp}", f"status {stamp} 0.2"]
    stamp = "1792014386 0"
    lines += [f"sync {stamp} {request('lab', None, SyncTopic.MAP_BRICKS)}",
              f"sync {stamp} {request('lab', None, SyncTopic.EVENTS, 1)}"]
    out = run_both(tmp_path, "\n".join(lines))
    maps = [MapBricks.from_cdr(bytes.fromhex(line.split()[1])) for line in out.splitlines()
            if line.startswith("map ") and line != "map none"]
    # The reference settles after two windows and keeps learning: a snapshot, then deltas.
    assert [(m.kind, m.header.map_epoch) for m in maps] == [(MapKind.SNAPSHOT, 1), (MapKind.DELTA, 2), (MapKind.DELTA, 3)]
    tail = out.splitlines()[-5:]
    assert tail[0] == "sync 1" and tail[1].startswith("data ") and tail[3] == "sync 0"
    (_, map_end), (_, events_end) = answers(out)[-2:]
    assert map_end.delivered == ((map_end.bound, map_end.bound),) and events_end.bound is None


def test_sync_answers_every_case(tmp_path):
    """Every protocol 2.3 answer with its SyncEnd, byte for byte: the BUILDING map, events known
    empty, the map replay, the "nothing missed" map, fresh snapshots (since unknown, another
    generation, generation 0, a legacy request), the events replay and fallback with unavailable
    ranges, and a foreign room. v2 requests expect the node's generation (77), another one, 0, or
    are legacy."""
    room = scenes.room_shell()
    box, speck = scenes.box_shell((0.3, 1.2, 0.3)), scenes.box_shell((-0.8, -0.8, 0.3))
    clouds = {name: floats(tmp_path, name, cloud) for name, cloud in (
        ("room", room), ("boxed", np.concatenate([room, box])), ("both", np.concatenate([room, box, speck])))}
    lines = [f"config {SINGLE_WINDOW} sensors=mid360 max_range=none reference_windows=3 learn_windows=0 "
             "delta_period_s=1 keyframe_period_s=1 state_dir=none", "start 1760400000 0 77"]
    second = 0

    def window(name):
        nonlocal second
        stamp = f"{1760400000 + second} 0"
        lines.extend([f"points mid360 {clouds[name]}", f"close {stamp}", f"map {stamp}", f"status {stamp} 1.0"])
        second += 1

    def sync(since, topic, expected=77, room_id="lab"):
        lines.append(f"sync {1760400000 + second} 0 {request(room_id, since, topic, len(lines), expected)}")

    sync(None, SyncTopic.MAP_BRICKS)  # 0: BUILDING, no map
    sync(None, SyncTopic.EVENTS)  # 1: nothing published
    for _ in range(3):
        window("room")
    sync(None, SyncTopic.EVENTS)  # 2: TRACKING, still no event
    sync(None, SyncTopic.EVENTS, None)  # 3: the same, legacy
    window("boxed")  # event seq 0 (candidate), map seq 1
    window("room")  # event seq 1 (expired), map seq 2
    sync((1, 2), SyncTopic.MAP_BRICKS)  # 4: nothing missed
    sync((1, 0), SyncTopic.MAP_BRICKS)  # 5: replay of 1, 2
    for expected in (0, 76, None):
        sync((1, 0), SyncTopic.MAP_BRICKS, expected)  # 6-8: fresh snapshots, since not honoured
    sync((1, 40), SyncTopic.MAP_BRICKS)  # 9: since not in the log
    sync((1, 0), SyncTopic.EVENTS)  # 10: replay of 1
    sync((1, 1), SyncTopic.EVENTS)  # 11: nothing missed
    sync((1, 0), SyncTopic.EVENTS, 0)  # 12: since not honoured: everything
    sync(None, SyncTopic.MAP_BRICKS, 77, "other")  # 13: another room, no answer
    # The box stays open while the speck comes and goes, until the box's only message has left the
    # 100-message log.
    window("boxed")
    for n in range(124):
        window("both" if n % 2 == 0 else "boxed")
    sync(None, SyncTopic.EVENTS)  # 14: fallback
    sync((1, 5), SyncTopic.EVENTS)  # 15: fallback, since evicted
    sync((1, 5), SyncTopic.EVENTS, 0)  # 16: the same since in generation 0
    sync((1, 5), SyncTopic.EVENTS, None)  # 17: legacy
    out = run_both(tmp_path, "\n".join(lines))
    replies = answers(out)
    assert len(replies) == 18
    ends = [end for _, end in replies]
    assert ends[13] is None and replies[13][0] == []
    for n, end in enumerate(ends):
        if end is not None:
            assert end.publisher_generation == 77 and end.header.seq == n - (n > 13)
            assert end.token == (ZERO_TOKEN if n in (3, 8, 17) else TOKEN)
    count = [len(messages) for messages, _ in replies]
    assert count[:4] == [0, 0, 0, 0] and all(end.bound is None and end.delivered == () for end in ends[:4])
    assert count[4] == 0 and ends[4].bound == 2 and ends[4].delivered == ()
    assert count[5] == 2 and ends[5].delivered == ((1, 2),)
    for n in (6, 7, 8, 9):
        (snapshot,) = replies[n][0]
        message = MapBricks.from_cdr(snapshot)
        assert message.kind == MapKind.SNAPSHOT and message.header.seq == 2 and message.publisher_generation == 77
        assert ends[n].delivered == ((2, 2),)
    assert count[10] == 1 and ends[10].delivered == ((1, 1),) and ends[10].bound == 1
    assert count[11] == 0 and ends[11].delivered == ()
    assert count[12] == 2 and ends[12].delivered == ((0, 1),) and ends[12].unavailable == ()
    bound = ends[14].bound
    oldest = bound - 99
    opened = ChangeEvent.from_cdr(replies[14][0][0]).header.seq
    assert opened < oldest and count[14] == 101
    assert ends[14].delivered == ((opened, opened), (oldest, bound))

    def missing(first):
        """[first, bound] minus what the fallback sent, as canonical ranges."""
        seqs = sorted(set(range(first, bound + 1)) - {opened} - set(range(oldest, bound + 1)))
        ranges = []
        for seq in seqs:
            if ranges and seq == ranges[-1][1] + 1:
                ranges[-1] = (ranges[-1][0], seq)
            else:
                ranges.append((seq, seq))
        return tuple(ranges)

    assert ends[14].unavailable == missing(0) and ends[15].unavailable == missing(6) != missing(0)
    assert ends[16].unavailable == ends[17].unavailable == ends[14].unavailable


def test_sync_across_restarts_with_two_generations(tmp_path):
    """Two starts with two explicit generations (5, then 9) over one state dir: every message of a
    start carries its generation, the first run's cursor is not honoured in the second, a refused
    generation (0, negative, past uint64) leaves the state untouched, and requests expecting the
    new generation, the old one, 0, or legacy are answered alike in both engines."""
    room = scenes.room_shell()
    plain = floats(tmp_path, "room", room)
    boxed = floats(tmp_path, "boxed", np.concatenate([room, scenes.box_shell((0.3, 1.2, 0.3))]))
    lines = [f"config {SINGLE_WINDOW} sensors=mid360 max_range=none reference_windows=3 state_dir={{state}}",
             "start 1760400000 0 5"]
    for second in range(3):
        lines += [f"points mid360 {plain}", f"close {1760400000 + second} 0", f"map {1760400000 + second} 0"]
    lines += [f"points mid360 {boxed}", "close 1760400003 0", "map 1760400003 0", "status 1760400003 0 1.0",
              f"sync 1760400004 0 {request('lab', (1, 0), SyncTopic.EVENTS, 0, 5)}"]
    lines += ["start 1760400100 0 0", "start 1760400100 0 -3", "start 1760400100 0 18446744073709551616",
              "start 1760400100 0 9", f"points mid360 {boxed}", "close 1760400101 0", "map 1760400101 0",
              "status 1760400101 0 1.0"]
    for expected in (9, 5, 0, None):
        lines.append(f"sync 1760400102 0 {request('lab', (2, 0), SyncTopic.EVENTS, 1, expected)}")
        lines.append(f"sync 1760400102 0 {request('lab', (2, 0), SyncTopic.MAP_BRICKS, 2, expected)}")
    out = run_both(tmp_path, "\n".join(lines)).splitlines()
    assert [line for line in out if line.startswith("init ")] == [
        "init BUILDING 0", "init error", "init error", "init error", "init TRACKING 2"]
    statuses = [MemoryStatus.from_cdr(bytes.fromhex(line.split()[1])) for line in out if line.startswith("status ")]
    assert [s.publisher_generation for s in statuses] == [5, 9]
    maps = [MapBricks.from_cdr(bytes.fromhex(line.split()[1])) for line in out if line.startswith("map ") and line != "map none"]
    assert {m.publisher_generation for m in maps[:-1]} == {5} and maps[-1].publisher_generation == 9
    replies = answers("\n".join(out))
    assert [end.publisher_generation for _, end in replies] == [5] + [9] * 8
    assert [end.header.seq for _, end in replies] == [0] + list(range(8))  # the SyncEnd seq restarts with the generation
    events, maps_end = replies[1::2], replies[2::2]
    # Generation 9: the event at (2, 0) is its first message, so the honoured since holds everything.
    assert len(events[0][0]) == 0 and events[0][1].delivered == () and events[0][1].bound == 0
    for messages, end in events[1:]:
        assert len(messages) == 1 and end.delivered == ((0, 0),)
        assert ChangeEvent.from_cdr(messages[0]).publisher_generation == 9
    # The map since (2, 0) is the newest message of generation 9: nothing missed, else a snapshot.
    assert maps_end[0][0] == [] and maps_end[0][1].delivered == ()
    for messages, end in maps_end[1:]:
        assert len(messages) == 1 and end.delivered == ((0, 0),) and MapBricks.from_cdr(messages[0]).publisher_generation == 9
