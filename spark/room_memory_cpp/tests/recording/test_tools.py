"""The recording tools themselves, on synthetic input: bag_stream.py's injected cubes and
room_memory_bag's timer schedule. No recording needed; the bag test needs ROOM_MEMORY_BAG."""

import os
import pathlib
import shutil
import struct
import subprocess
import sys

import numpy as np
import pytest

TOOLS = pathlib.Path(__file__).resolve().parents[2] / "tools"
sys.path.insert(0, str(TOOLS))
import bag_stream  # noqa: E402

BAG = os.environ.get("ROOM_MEMORY_BAG")


def test_bag_stream_finds_the_oracle_outside_the_repository(tmp_path):
    """Codex P1: the image copies the oracle to /oracle/room_memory, not next to
    /ws/src/room_memory_cpp, and bag_stream.py only looked next to itself. It takes the package
    from ROOM_MEMORY_PYTHON_PACKAGE (CTest sets it), in the tests and in their subprocesses."""
    tools = tmp_path / "src" / "room_memory_cpp" / "tools"
    tools.mkdir(parents=True)
    shutil.copy(TOOLS / "bag_stream.py", tools)
    package = os.environ.get("ROOM_MEMORY_PYTHON_PACKAGE", str(TOOLS.parents[1] / "room_memory"))
    env = {**os.environ, "ROOM_MEMORY_PYTHON_PACKAGE": package}
    env.pop("PYTHONPATH", None)
    result = subprocess.run([sys.executable, str(tools / "bag_stream.py"), "--help"], env=env, cwd=tmp_path,
                            capture_output=True)
    assert result.returncode == 0, result.stderr.decode()


def test_split_bags_replay_in_segment_order(tmp_path):
    """Codex P2: segments sorted by name put _10.mcap between _1 and _2, and the replay's timers
    then ran across a jump back in time."""
    for n in (2, 10, 0, 11, 1, 9):
        (tmp_path / f"static5m_{n}.mcap").touch()
    (tmp_path / "metadata.yaml").touch()
    assert [f.name for f in bag_stream.segments(tmp_path)] == [f"static5m_{n}.mcap" for n in (0, 1, 2, 9, 10, 11)]
    assert bag_stream.segments(tmp_path / "static5m_10.mcap") == [tmp_path / "static5m_10.mcap"]


def test_loop_plays_the_recording_back_to_back(monkeypatch):
    """--loop: each pass starts one frame period after the previous one's last lidar message, and
    other topics neither play nor stretch a pass."""
    second = 10**9
    recorded = [("/tf", 5 * second, b"t"), ("/livox/lidar", 7 * second, b"a"), ("/lidar_points", 8 * second, b"b"),
                ("/livox/lidar", 9 * second, b"c"), ("/tf", 12 * second, b"t")]
    monkeypatch.setattr(bag_stream, "messages", lambda files: iter(recorded))
    played = [(topic, t / second, data) for topic, t, data in bag_stream.passes([], 3)]
    assert played == [("/livox/lidar", 7.0, b"a"), ("/lidar_points", 8.0, b"b"), ("/livox/lidar", 9.0, b"c"),
                      ("/livox/lidar", 9.1, b"a"), ("/lidar_points", 10.1, b"b"), ("/livox/lidar", 11.1, b"c"),
                      ("/livox/lidar", 11.2, b"a"), ("/lidar_points", 12.2, b"b"), ("/livox/lidar", 13.2, b"c")]


@pytest.mark.parametrize("background_m", [10.0, 4.0])
def test_injected_returns_get_the_documented_range_noise(monkeypatch, background_m):
    """Codex P2: the noise term was scaled by the intersection fraction, so a return 10 m out whose
    cube entry is about 2 m away got about 2 mm of noise instead of 1 cm."""
    rng = np.random.default_rng(1)
    center = np.array([2.0, 0.5, 0.3])
    axis = center / np.linalg.norm(center)
    # Rays in a narrow cone around the cube center (never axis aligned), all entering its front.
    directions = axis + rng.normal(0.0, 0.01, size=(4000, 3))
    directions /= np.linalg.norm(directions, axis=1, keepdims=True)
    points = (directions * background_m).astype(np.float32)
    noisy = bag_stream.draw_cube(points, center, 0.3, np.random.default_rng(2))
    monkeypatch.setattr(bag_stream, "RANGE_NOISE_M", 0.0)
    clean = bag_stream.draw_cube(points, center, 0.3, np.random.default_rng(2))
    assert np.all(np.linalg.norm(clean, axis=1) < 2.2)  # every ray hit the cube
    noise = np.linalg.norm(noisy.astype(np.float64), axis=1) - np.linalg.norm(clean.astype(np.float64), axis=1)
    assert abs(noise.std() - 0.01) < 0.0005 and abs(noise.mean()) < 0.001


def records(seconds, hz=10):
    """Mid-360 float records of a 2 m wall at `hz` from t = 1000 s."""
    y, z = np.meshgrid(np.linspace(-1, 1, 40), np.linspace(-0.5, 0.5, 20))
    wall = np.stack([np.full(y.size, 2.0), y.ravel(), z.ravel()], axis=1).astype("<f4").tobytes()
    out = b""
    for n in range(seconds * hz):
        out += struct.pack("<BBQI", 1, 0, 1000 * 10**9 + n * 10**9 // hz, len(wall)) + wall
    return out


@pytest.mark.skipif(not BAG, reason="ROOM_MEMORY_BAG (the room_memory_bag binary) not set")
def test_statuses_tick_every_second_independently_of_the_window():
    """Codex P2: with window_s=2 both statuses of a window were sent after it closed, the first with
    two seconds of frames and the second with none (DEGRADED every other second)."""
    result = subprocess.run([BAG, "sensors=mid360", "window_s=2", "reference_windows=2"], input=records(12),
                            capture_output=True, check=True)
    lines = [line.split() for line in result.stdout.decode().splitlines()]
    timed = [(float(words[2] if words[0] == "window" else words[1]), words[0], words) for words in lines
             if words[0] in ("window", "status")]
    statuses = [words for _, kind, words in timed if kind == "status"]
    assert all(words[2] != "DEGRADED" for words in statuses), statuses
    assert [float(words[1]) for words in statuses] == [float(s) for s in range(1, 12)]
    assert sum(kind == "window" for _, kind, _ in timed) == 5
    # Window and status lines come in time order.
    times = [t for t, _, _ in timed]
    assert times == sorted(times)
