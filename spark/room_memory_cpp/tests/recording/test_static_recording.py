"""Regression on the real 5-minute static recordings of both lidars (rosbag2 mcap, zstd), replayed
through room_memory_bag at their own timestamps with the node's default parameters.

The recordings are not in git. They are read from ROOM_MEMORY_RECORDING (by default
~/Developer/lc-tasks/recordings/static5m) and ROOM_MEMORY_RECORDING_B (by default
~/Developer/lc-tasks/recordings/static5m-b), and each one's tests skip when it is absent.

Nothing in the room moves in either, except next to the rig: about 1.3 m from both sensors
something low (z up to ~0.8 m) shifts every few seconds to minutes in both recordings and live,
which the node does not report (near_range_m; spark/room_memory_cpp/PLAN.md has the evidence).

- Each recording: no change event at all, also over 30 minutes of it played back to back, and
  every output within the protocol's 5 KB/s budget.
- Boxes injected into one sensor's frames by ray casting (tools/bag_stream.py): an appearance,
  a disappearance and a move must each be detected, as a CANDIDATE of the right kind near the
  box, within the latency bound, and nothing else.
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

INJECT_AT = 180.0
LATENCY_BOUND_S = 8.0
BUDGET_BYTES_PER_S = 5000
LONG_RUN_S = 1800.0


@pytest.fixture(scope="module", params=sorted(RECORDINGS))
def recording(request):
    path = RECORDINGS[request.param]
    if not list(path.glob("*.mcap")):
        variable = "ROOM_MEMORY_RECORDING" + ("_B" if request.param.endswith("-b") else "")
        pytest.skip(f"static recording not found at {path} (set {variable})")
    return path


def replay(recording, *injections, config=(), loop=1):
    """(events, summary) of one replay with the node defaults plus `config` (key=value words)."""
    stream = subprocess.Popen([sys.executable, str(STREAM), str(recording), f"--loop={loop}",
                               *[f"--inject={j}" for j in injections]], stdout=subprocess.PIPE)
    result = subprocess.run([BAG, *config], stdin=stream.stdout, capture_output=True, check=True)
    stream.stdout.close()
    assert stream.wait() == 0
    events, maps, summary = [], [], None
    for line in result.stdout.decode().splitlines():
        words = line.split()
        if words[0] == "event":
            events.append({"t": float(words[1]), "kind": words[2], "state": words[3], "sensors": int(words[6]),
                           "voxels": int(words[8]), "centroid": tuple(float(v) for v in words[10:13])})
        elif words[0] == "map":
            maps.append({"t": float(words[1]), "kind": words[2], "epoch": int(words[4]), "bricks": int(words[6])})
        elif words[0] == "summary":
            summary = json.loads(line[len("summary "):])
    summary["maps"] = maps
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


def test_no_events_over_half_an_hour(recording, unmodified):
    """The recording played back to back for at least LONG_RUN_S: the reference keeps learning the
    whole time, and every pass starts again from the room as the first one found it."""
    _, once = unmodified
    loop = math.ceil(LONG_RUN_S / once["duration_s"])
    events, summary = replay(recording, loop=loop)
    assert summary["duration_s"] >= LONG_RUN_S
    assert events == [], f"{len(events)} events: {events[:5]}"


def test_a_saved_union_reference_still_gives_no_events(recording, tmp_path):
    """The first deployed node saved union-of-5-window references; the defaults load them as they are."""
    legacy = ("reference_windows=5", "occupied_min_hits=1", "occupied_min_ratio=0", "free_min_seen=1",
              "recent_windows=1", "candidate_windows=1", "expire_windows=1")
    replay(recording, config=(*legacy, f"state_dir={tmp_path}"))
    events, summary = replay(recording, config=(f"state_dir={tmp_path}",))
    assert summary["tracking_from_s"] <= 1.0
    assert events == [], f"{len(events)} events: {events[:5]}"


def test_output_within_the_byte_budget(unmodified):
    _, summary = unmodified
    total = sum(summary["bytes"].values())
    assert total / summary["duration_s"] <= BUDGET_BYTES_PER_S, summary["bytes"]
    assert summary["messages"]["snapshot"] >= 1 and summary["messages"]["status"] >= summary["duration_s"] - 1


MID360, JT128 = 1, 2
CASES = {
    # name: (spec, sensor bit, kind, center the detection must be near); the cubes are 0.3 m
    "mid360-appear-2.6m": (f"mid360:appear:{INJECT_AT}:-2.2,1.4,0.0:0.3", MID360, "APPEARED", (-2.2, 1.4, 0.0)),
    "mid360-appear-4.6m": (f"mid360:appear:{INJECT_AT}:4.6,-0.4,0.0:0.3", MID360, "APPEARED", (4.6, -0.4, 0.0)),
    "mid360-disappear": (f"mid360:disappear:{INJECT_AT}:-2.2,1.4,0.0:0.3", MID360, "DISAPPEARED", (-2.2, 1.4, 0.0)),
    "mid360-move": (f"mid360:move:{INJECT_AT}:2.2,-0.6,0.0:0.3:2.2,0.0,0.0", MID360, "MOVED", (2.2, 0.0, 0.0)),
    "jt128-appear": (f"jt128:appear:{INJECT_AT}:-2.4,0.6,0.2:0.3", JT128, "APPEARED", (-2.4, 0.6, 0.2)),
    "jt128-disappear": (f"jt128:disappear:{INJECT_AT}:-2.4,0.6,0.2:0.3", JT128, "DISAPPEARED", (-2.4, 0.6, 0.2)),
    "jt128-move": (f"jt128:move:{INJECT_AT}:-2.4,0.4,0.2:0.3:-2.6,1.0,0.2", JT128, "MOVED", (-2.6, 1.0, 0.2)),
}


def detection(events, sensor, kind, center):
    """First CANDIDATE of `kind` from `sensor` with its centroid within 0.25 m of `center`."""
    for e in events:
        if (e["state"] == "CANDIDATE" and e["kind"] == kind and e["sensors"] == sensor
                and distance(e["centroid"], center) <= 0.25):
            return e
    return None


@pytest.mark.parametrize("name", sorted(CASES))
def test_injected_change_is_detected(recording, name):
    spec, sensor, kind, center = CASES[name]
    events, _ = replay(recording, spec)
    found = detection(events, sensor, kind, center)
    assert found is not None, f"{name}: no {kind} candidate near {center}"
    latency = found["t"] - INJECT_AT
    assert 0 < latency <= LATENCY_BOUND_S, f"{name}: latency {latency} s"
    others = [e for e in events if e is not found]
    assert others == [], f"{name}: other events {others[:5]}"


def test_learning_budget_stops_the_epoch_clock(unmodified):
    """Defaults: an epoch increase by tracking_from_s + learn_windows * window_s + delta_period_s,
    and after that one epoch, snapshots only, with one brick count."""
    _, summary = unmodified
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
