"""Regression on the real 5-minute static recordings of both lidars (rosbag2 mcap, zstd), replayed
through room_memory_bag at their own timestamps with the node's default parameters.

The recordings are not in git. They are read from ROOM_MEMORY_RECORDING (by default
~/Developer/lc-tasks/recordings/static5m) and ROOM_MEMORY_RECORDING_B (by default
~/Developer/lc-tasks/recordings/static5m-b), and each one's tests skip when it is absent.

Nothing in the room moves in either, except Trevor working at a desk about 1.3 m from both
sensors: in static5m from 86 s, in static5m-b throughout. His motion is activity (the Activity
message), never a change event (spark/room_memory_cpp/PLAN.md, "Activity").

- Each recording: no change event at all, also over 30 minutes of it played back to back, and
  every output within the protocol's 5 KB/s budget; activity while he moves, none before.
- Boxes injected into one sensor's frames by ray casting (tools/bag_stream.py), far ones and ones
  1.0-1.6 m from the rig, early (just after tracking starts) and late: an appearance, a
  disappearance and a move must each be detected, as a CANDIDATE of the right kind near the box,
  within candidate_windows * window_s + 8 s, and nothing else.
"""

import json
import math
import os
import pathlib
import subprocess
import sys

import pytest

HERE = pathlib.Path(__file__).resolve().parent
STREAM = HERE.parents[1] / "tools" / "bag_stream.py"
BAG = os.environ.get("ROOM_MEMORY_BAG")
RECORDINGS = {
    "static5m": pathlib.Path(os.environ.get("ROOM_MEMORY_RECORDING",
                                            "~/Developer/lc-tasks/recordings/static5m")).expanduser(),
    "static5m-b": pathlib.Path(os.environ.get("ROOM_MEMORY_RECORDING_B",
                                              "~/Developer/lc-tasks/recordings/static5m-b")).expanduser(),
}

pytest.importorskip("mcap", reason="the mcap package reads the recording (pip install mcap zstandard)")

pytestmark = pytest.mark.skipif(not BAG, reason="ROOM_MEMORY_BAG (the room_memory_bag binary) not set")

# The node defaults (room_memory_bag runs them): a change is a CANDIDATE once it has stayed settled
# for candidate_windows (40) windows of window_s (1 s); the recent layer and the diff add a few.
CANDIDATE_S = 40.0
LATENCY_BOUND_S = CANDIDATE_S + 8.0
EARLY, LATE, FAR = 45.0, 200.0, 180.0
BUDGET_BYTES_PER_S = 5000
LONG_RUN_S = 1800.0
STATIC5M_MOTION_FROM_S = 86.0


@pytest.fixture(scope="module", params=sorted(RECORDINGS))
def recording(request):
    path = RECORDINGS[request.param]
    if not list(path.glob("*.mcap")):
        variable = "ROOM_MEMORY_RECORDING" + ("_B" if request.param.endswith("-b") else "")
        pytest.skip(f"static recording not found at {path} (set {variable})")
    return path


def replay(recording, *injections, config=(), loop=1):
    """(events, summary) of one replay with the node defaults plus `config` (key=value words).
    The summary also holds the map messages and the active Activity messages."""
    stream = subprocess.Popen([sys.executable, str(STREAM), str(recording), f"--loop={loop}",
                               *[f"--inject={j}" for j in injections]], stdout=subprocess.PIPE)
    result = subprocess.run([BAG, *config], stdin=stream.stdout, capture_output=True, check=True)
    stream.stdout.close()
    assert stream.wait() == 0
    events, maps, activity, summary = [], [], [], None
    for line in result.stdout.decode().splitlines():
        words = line.split()
        if words[0] == "event":
            events.append({"t": float(words[1]), "kind": words[2], "state": words[3], "sensors": int(words[6]),
                           "voxels": int(words[8]), "centroid": tuple(float(v) for v in words[10:13])})
        elif words[0] == "map":
            maps.append({"t": float(words[1]), "kind": words[2], "epoch": int(words[4]), "bricks": int(words[6])})
        elif words[0] == "activity":
            activity.append({"t": float(words[1]), "regions": int(words[4]), "voxels": int(words[6]),
                             "sensors": int(words[8])})
        elif words[0] == "summary":
            summary = json.loads(line[len("summary "):])
    summary["maps"], summary["activity"] = maps, activity
    return events, summary


@pytest.fixture(scope="module")
def unmodified(recording):
    return replay(recording)


def distance(a, b):
    return sum((x - y) ** 2 for x, y in zip(a, b)) ** 0.5


def test_no_events_on_the_static_room(unmodified):
    events, summary = unmodified
    assert summary["tracking_from_s"] > 0 and summary["tracked_s"] > 200
    assert events == [], f"{len(events)} events: {events[:5]}"
    # Every pending region stayed well short of the gate (PLAN.md: the longest run is 24 windows).
    assert max(summary["max_run"].values()) <= CANDIDATE_S - 10, summary["max_run"]


def test_motion_next_to_the_rig_is_activity(recording, unmodified):
    _, summary = unmodified
    times = [a["t"] for a in summary["activity"]]
    assert summary["messages"]["activity"] == summary["messages"]["status"]
    assert len(times) >= 30, f"{len(times)} active seconds"
    if recording.name == "static5m":
        # Nothing moves before Trevor does at 86 s; his motion shows within seconds of it.
        assert STATIC5M_MOTION_FROM_S <= min(times) <= STATIC5M_MOTION_FROM_S + 5, min(times)
    else:
        assert min(times) <= summary["tracking_from_s"] + 30, min(times)


def test_no_events_over_half_an_hour(recording, unmodified):
    """The recording played back to back for at least LONG_RUN_S with one engine state."""
    _, once = unmodified
    loop = math.ceil(LONG_RUN_S / once["duration_s"])
    events, summary = replay(recording, loop=loop)
    assert summary["duration_s"] >= LONG_RUN_S
    assert events == [], f"{len(events)} events: {events[:5]}"
    assert max(summary["max_run"].values()) <= CANDIDATE_S - 10, summary["max_run"]


# Where Trevor works, in each sensor's frame (sensor bit: center); his BUILDING-time pose is there.
DESK = {1: (-1.2, -0.7, 0.0), 2: (0.35, 1.1, 0.0)}


def test_a_saved_union_reference_loads_and_only_his_old_pose_shows(recording, tmp_path):
    """The first deployed node saved union-of-5-window references, learning while it tracked; the
    defaults load them as they are and track at once. Such a reference holds Trevor as he sat
    while it was built (every voxel hit once in 5 windows), so where he sat can later show as
    DISAPPEARED once he has moved and stayed still past the gate (PLAN.md, "Activity": a reference
    that captured a person needs rebuilding). Nothing else may show."""
    legacy = ("reference_windows=5", "occupied_min_hits=1", "occupied_min_ratio=0", "free_min_seen=1",
              "recent_windows=1", "candidate_windows=1", "expire_windows=1", "learn_windows=1000000")
    replay(recording, config=(*legacy, f"state_dir={tmp_path}"))
    events, summary = replay(recording, config=(f"state_dir={tmp_path}",))
    assert summary["tracking_from_s"] <= 1.0
    elsewhere = [e for e in events if e["kind"] != "DISAPPEARED" or distance(e["centroid"], DESK[e["sensors"]]) > 0.8]
    assert elsewhere == [], f"{len(elsewhere)} events: {elsewhere[:5]}"
    assert sum(e["state"] == "CANDIDATE" for e in events) <= 1


def test_output_within_the_byte_budget(unmodified):
    _, summary = unmodified
    total = sum(summary["bytes"].values())
    assert total / summary["duration_s"] <= BUDGET_BYTES_PER_S, summary["bytes"]
    assert summary["messages"]["snapshot"] >= 1 and summary["messages"]["status"] >= summary["duration_s"] - 1


def test_tracking_teaches_the_reference_nothing(unmodified):
    """learn_windows 0: the map stays the tracking snapshot. One epoch, keyframes only, one size."""
    _, summary = unmodified
    maps = summary["maps"]
    assert maps[0]["kind"] == "SNAPSHOT" and len(maps) >= 4
    assert {item["epoch"] for item in maps} == {maps[0]["epoch"]}
    assert {item["kind"] for item in maps} == {"SNAPSHOT"}
    assert len({item["bricks"] for item in maps}) == 1


def test_learning_budget_stops_the_epoch_clock(recording):
    """learn_windows 30: an epoch increase by tracking_from_s + learn_windows * window_s +
    delta_period_s, and after that one epoch, snapshots only, with one brick count."""
    _, summary = replay(recording, config=("learn_windows=30",))
    maps = summary["maps"]
    opening = maps[0]
    assert opening["kind"] == "SNAPSHOT"
    horizon = summary["tracking_from_s"] + 30 * 1.0 + 5.0
    early = [item for item in maps if item["t"] <= horizon + 1e-6]
    late = [item for item in maps if item["t"] > horizon + 1e-6]
    assert any(item["epoch"] > opening["epoch"] for item in early)
    assert len(late) >= 2
    assert len({item["epoch"] for item in late}) == 1
    assert {item["kind"] for item in late} == {"SNAPSHOT"}
    assert len({item["bricks"] for item in late}) == 1


MID360, JT128 = 1, 2


def cube(sensor, kind, t, center, to=None):
    """(spec, sensor bit, event kind, where the detection must be, injection time); 0.3 m cubes."""
    name = "mid360" if sensor == MID360 else "jt128"
    text = f"{name}:{kind}:{t}:{','.join(map(str, center))}:0.3" + (f":{','.join(map(str, to))}" if to else "")
    return text, sensor, {"appear": "APPEARED", "disappear": "DISAPPEARED", "move": "MOVED"}[kind], to or center, t


# One replay per group; the boxes of a group are in different sensors or meters apart. Ranges from
# the sensor: far 2.2-4.6 m; near 1.0-1.6 m, more than activity_radius_m clear of where Trevor works.
GROUPS = {
    "far-appear": [cube(MID360, "appear", FAR, (-2.2, 1.4, 0.0)), cube(MID360, "appear", FAR, (4.6, -0.4, 0.0)),
                   cube(JT128, "appear", FAR, (-2.4, 0.6, 0.2))],
    "far-disappear": [cube(MID360, "disappear", FAR, (-2.2, 1.4, 0.0)), cube(JT128, "disappear", FAR, (-2.4, 0.6, 0.2))],
    "far-move": [cube(MID360, "move", FAR, (2.2, -0.6, 0.0), (2.2, 0.0, 0.0)),
                 cube(JT128, "move", FAR, (-2.4, 0.4, 0.2), (-2.6, 1.0, 0.2))],
}
for _t, _when in ((EARLY, "early"), (LATE, "late")):
    GROUPS[f"near-appear-{_when}"] = [cube(MID360, "appear", _t, (1.4, 0.3, 0.3)),
                                      cube(JT128, "appear", _t, (-1.2, -0.8, 0.45))]
    GROUPS[f"near-disappear-{_when}"] = [cube(MID360, "disappear", _t, (1.0, 0.0, 0.0)),
                                         cube(JT128, "disappear", _t, (-1.5, 0.3, 0.2))]
    GROUPS[f"near-move-{_when}"] = [cube(MID360, "move", _t, (-0.4, 1.4, 0.3), (0.2, 1.4, 0.3)),
                                    cube(JT128, "move", _t, (-0.9, -1.2, 0.45), (-1.4, -0.6, 0.45))]


def detection(events, sensor, kind, center):
    """First CANDIDATE of `kind` from `sensor` with its centroid within 0.25 m of `center`."""
    for e in events:
        if (e["state"] == "CANDIDATE" and e["kind"] == kind and e["sensors"] == sensor
                and distance(e["centroid"], center) <= 0.25):
            return e
    return None


@pytest.mark.parametrize("group", sorted(GROUPS))
def test_injected_changes_are_detected(recording, group):
    cubes = GROUPS[group]
    events, _ = replay(recording, *[c[0] for c in cubes])
    found = []
    for spec, sensor, kind, center, t in cubes:
        hit = detection(events, sensor, kind, center)
        assert hit is not None, f"{spec}: no {kind} candidate near {center}"
        latency = hit["t"] - t
        assert 0 < latency <= LATENCY_BOUND_S, f"{spec}: latency {latency} s"
        found.append(hit)
    others = [e for e in events if all(e is not hit for hit in found)]
    assert others == [], f"{group}: other events {others[:5]}"
