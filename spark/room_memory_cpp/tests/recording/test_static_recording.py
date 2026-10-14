"""Regression on the real 5-minute static recording of both lidars (rosbag2 mcap, zstd), replayed
through room_memory_bag at its own timestamps with the node's default parameters.

The recording is not in git. It is read from ROOM_MEMORY_RECORDING, by default
~/Developer/lc-tasks/recordings/static5m, and every test here skips when it is absent.

- The unmodified recording: no change event outside the one real change it contains (from 86 s
  on, something low moves about 1.2 m from the JT128 and 1.4 m from the Mid-360, seen by both;
  spark/room_memory_cpp/PLAN.md has the evidence), and every output within the protocol's
  5 KB/s budget.
- Boxes injected into one sensor's frames by ray casting (tools/bag_stream.py): an appearance,
  a disappearance and a move must each be detected, as a CANDIDATE of the right kind near the
  box, within the latency bound.
"""

import json
import os
import pathlib
import subprocess
import sys

import pytest

HERE = pathlib.Path(__file__).resolve().parent
STREAM = HERE.parents[1] / "tools" / "bag_stream.py"
BAG = os.environ.get("ROOM_MEMORY_BAG")
RECORDING = pathlib.Path(os.environ.get("ROOM_MEMORY_RECORDING", "~/Developer/lc-tasks/recordings/static5m")).expanduser()

pytest.importorskip("mcap", reason="the mcap package reads the recording (pip install mcap zstandard)")

pytestmark = [
    pytest.mark.skipif(not BAG, reason="ROOM_MEMORY_BAG (the room_memory_bag binary) not set"),
    pytest.mark.skipif(not list(RECORDING.glob("*.mcap")),
                       reason=f"static recording not found at {RECORDING} (set ROOM_MEMORY_RECORDING)"),
]

# The real change (module docstring): from REAL_CHANGE_FROM_S on, within REAL_CHANGE_RADIUS_M of
# a center in each sensor's frame (sensor bit -> center).
REAL_CHANGE_FROM_S = 86.0
REAL_CHANGE_RADIUS_M = 0.8
REAL_CHANGE_AT = {1: (-1.2, -0.7, 0.0), 2: (0.35, 1.1, 0.0)}
INJECT_AT = 180.0
LATENCY_BOUND_S = 8.0
BUDGET_BYTES_PER_S = 5000


def replay(*injections, config=()):
    """(events, summary) of one replay with the node defaults plus `config` (key=value words)."""
    stream = subprocess.Popen([sys.executable, str(STREAM), str(RECORDING), *[f"--inject={j}" for j in injections]],
                              stdout=subprocess.PIPE)
    result = subprocess.run([BAG, *config], stdin=stream.stdout, capture_output=True, check=True)
    stream.stdout.close()
    assert stream.wait() == 0
    events, summary = [], None
    for line in result.stdout.decode().splitlines():
        words = line.split()
        if words[0] == "event":
            events.append({"t": float(words[1]), "kind": words[2], "state": words[3], "sensors": int(words[6]),
                           "voxels": int(words[8]), "centroid": tuple(float(v) for v in words[10:13])})
        elif words[0] == "summary":
            summary = json.loads(line[len("summary "):])
    return events, summary


@pytest.fixture(scope="module")
def unmodified():
    return replay()


def distance(a, b):
    return sum((x - y) ** 2 for x, y in zip(a, b)) ** 0.5


def real_change(event):
    return (event["t"] >= REAL_CHANGE_FROM_S
            and distance(event["centroid"], REAL_CHANGE_AT[event["sensors"]]) <= REAL_CHANGE_RADIUS_M)


def test_no_false_events_after_the_reference(unmodified):
    events, summary = unmodified
    assert summary["tracking_from_s"] > 0 and summary["tracked_s"] > 200
    false = [e for e in events if not real_change(e)]
    assert false == [], f"{len(false)} events outside the real change: {false[:5]}"
    assert any(real_change(e) for e in events), "the real change was not detected"


def test_a_saved_union_reference_still_gives_no_false_events(tmp_path):
    """The deployed node saved union-of-5-window references; the new defaults load them as they are."""
    legacy = ("reference_windows=5", "occupied_min_hits=1", "occupied_min_ratio=0", "free_min_seen=1",
              "recent_windows=1", "candidate_windows=1", "expire_windows=1")
    replay(config=(*legacy, f"state_dir={tmp_path}"))
    events, summary = replay(config=(f"state_dir={tmp_path}",))
    assert summary["tracking_from_s"] <= 1.0
    assert [e for e in events if not real_change(e)] == []


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
def test_injected_change_is_detected(name):
    spec, sensor, kind, center = CASES[name]
    events, _ = replay(spec)
    found = detection(events, sensor, kind, center)
    assert found is not None, f"{name}: no {kind} candidate near {center}"
    latency = found["t"] - INJECT_AT
    assert 0 < latency <= LATENCY_BOUND_S, f"{name}: latency {latency} s"
    others = [e for e in events if e is not found and not real_change(e)]
    assert others == [], f"{name}: other events {others[:5]}"
