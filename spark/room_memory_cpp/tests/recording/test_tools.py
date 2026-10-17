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

HERE = pathlib.Path(__file__).resolve().parent
TOOLS = HERE.parents[1] / "tools"
sys.path.insert(0, str(TOOLS))
import bag_stream  # noqa: E402

BAG = os.environ.get("ROOM_MEMORY_BAG")
# What CTest's static_recording runs (CMake sets it from the same variable), else that file here.
STATIC_RECORDING = os.environ.get("ROOM_MEMORY_STATIC_RECORDING_TEST", str(HERE / "test_static_recording.py"))
# Set in the pytest runs below, so this file's own run-pytest tests do not run inside them.
PROBE = "ROOM_MEMORY_RECORDING_PROBE"


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


def _samples(low, high):
    """float32 grid matching scenes.cpp samples(): start + i * step while value < high."""
    step = np.float32(0.02)
    start = np.float32(low) + np.float32(0.007)
    values = []
    i = 0
    while True:
        value = np.float32(start + np.float32(i) * step)
        if value >= np.float32(high):
            return np.asarray(values, dtype=np.float32)
        values.append(value)
        i += 1


def _room_and_box():
    """The C++ room shell and one box, so one window of each emits a change."""
    x0, x1 = np.float32(-1.5), np.float32(2.5)
    y0, y1 = np.float32(-1.5), np.float32(2.5)
    z0, z1 = np.float32(-0.7), np.float32(1.3)
    xs, ys, zs = _samples(x0, x1), _samples(y0, y1), _samples(z0, z1)
    phase = np.float32(0.007)
    floor_z = z0 + phase
    gx, gy = np.meshgrid(xs, ys, indexing="ij")
    floor = np.stack([gx.ravel(), gy.ravel(), np.full(gx.size, floor_z, dtype=np.float32)], axis=1)
    x_lo, x_hi = x0 + phase, x1 - phase
    gy, gz = np.meshgrid(ys, zs, indexing="ij")
    walls_x = np.stack([
        np.concatenate([np.full(gy.size, x_lo, dtype=np.float32), np.full(gy.size, x_hi, dtype=np.float32)]),
        np.concatenate([gy.ravel(), gy.ravel()]),
        np.concatenate([gz.ravel(), gz.ravel()]),
    ], axis=1)
    y_lo, y_hi = y0 + phase, y1 - phase
    gx, gz = np.meshgrid(xs, zs, indexing="ij")
    walls_y = np.stack([
        np.concatenate([gx.ravel(), gx.ravel()]),
        np.concatenate([np.full(gx.size, y_lo, dtype=np.float32), np.full(gx.size, y_hi, dtype=np.float32)]),
        np.concatenate([gz.ravel(), gz.ravel()]),
    ], axis=1)
    room = np.concatenate([floor, walls_x, walls_y], axis=0)
    cx, cy, cz, half = np.float32(0.3), np.float32(1.2), np.float32(0.3), np.float32(0.2)
    bxs, bys, bzs = _samples(cx - half, cx + half), _samples(cy - half, cy + half), _samples(cz - half, cz + half)
    faces = []
    gy, gz = np.meshgrid(bys, bzs, indexing="ij")
    for x in (cx - half + phase, cx + half - phase):
        faces.append(np.stack([np.full(gy.size, x, dtype=np.float32), gy.ravel(), gz.ravel()], axis=1))
    gx, gz = np.meshgrid(bxs, bzs, indexing="ij")
    for y in (cy - half + phase, cy + half - phase):
        faces.append(np.stack([gx.ravel(), np.full(gx.size, y, dtype=np.float32), gz.ravel()], axis=1))
    gx, gy = np.meshgrid(bxs, bys, indexing="ij")
    for z in (cz - half + phase, cz + half - phase):
        faces.append(np.stack([gx.ravel(), gy.ravel(), np.full(gx.size, z, dtype=np.float32)], axis=1))
    return room, np.concatenate([room, *faces], axis=0)


def _frame(points, t_ns):
    raw = np.ascontiguousarray(points, dtype="<f4").tobytes()
    return struct.pack("<BBQI", 1, 0, t_ns, len(raw)) + raw


@pytest.mark.skipif(not BAG, reason="ROOM_MEMORY_BAG (the room_memory_bag binary) not set")
def test_a_second_bag_run_allocates_a_new_generation(tmp_path):
    """Two processes on one state_dir each publish events. Reusing generation 1 appends seq 0
    after the first run's high water and aborts (static_recording's second replay)."""
    room, boxed = _room_and_box()
    t0 = 1000 * 10**9
    payload = _frame(room, t0) + _frame(boxed, t0 + 10**9) + _frame(boxed, t0 + 2 * 10**9)
    config = ["sensors=mid360", "max_range=none", "reference_windows=1", "recent_windows=1",
              "occupied_min_hits=1", "occupied_min_ratio=0", "free_min_seen=1", "candidate_windows=1",
              "expire_windows=1", "fresh_min_voxels=1", "fresh_min_fraction=0", "activity_radius_m=0",
              f"state_dir={tmp_path}"]

    def run():
        result = subprocess.run([BAG, *config], input=payload, capture_output=True, check=True)
        lines = result.stdout.decode().splitlines()
        events = [line for line in lines if line.startswith("event ")]
        assert events, result.stdout.decode()
        return int((tmp_path / "lab" / "publisher_generation").read_text().strip())

    first = run()
    second = run()
    assert second > first


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


def run_pytest(tmp_path, *args, environment=()):
    """pytest as CTest runs it, with neither recording present."""
    nowhere = tmp_path / "no-recording"
    nowhere.mkdir(exist_ok=True)
    env = {**os.environ, "ROOM_MEMORY_RECORDING": str(nowhere), "ROOM_MEMORY_RECORDING_B": str(nowhere), PROBE: "1",
           **dict(environment)}
    result = subprocess.run([sys.executable, "-m", "pytest", "-q", "-rs", "-p", "no:cacheprovider", *args],
                            env=env, cwd=tmp_path, capture_output=True)
    return result.returncode, result.stdout.decode() + result.stderr.decode()


@pytest.mark.skipif(PROBE in os.environ, reason="inside a run_pytest run")
def test_static_recording_reports_skipped_without_a_recording(tmp_path):
    """Codex P2: static_recording ran this whole directory, and this file's tests need no
    recording, so without one it passed ("3 passed, 32 skipped"). It runs test_static_recording.py
    alone, and a run in which every test skipped exits 77, CTest's SKIP_RETURN_CODE for it."""
    code, out = run_pytest(tmp_path, STATIC_RECORDING)
    assert code == 77, out
    assert " passed" not in out, out


@pytest.mark.skipif(PROBE in os.environ, reason="inside a run_pytest run")
def test_a_run_that_passed_anything_is_not_reported_skipped(tmp_path):
    """The other side of exit 77: one pass among skips and deselections still exits 0."""
    code, out = run_pytest(tmp_path, str(HERE / "test_tools.py"), "-k", "split_bags or statuses_tick",
                           environment={"ROOM_MEMORY_BAG": ""})
    assert code == 0, out
    assert "1 passed, 1 skipped" in out, out


def without_mcap(tmp_path, error):
    """The environment of a run in which `import mcap` raises `error` (a statement)."""
    shadow = tmp_path / "shadow"
    shadow.mkdir(exist_ok=True)
    (shadow / "mcap.py").write_text(error + "\n")
    path = os.pathsep.join(p for p in (str(shadow), os.environ.get("PYTHONPATH")) if p)
    return {"PYTHONPATH": path, "ROOM_MEMORY_BAG": BAG or "room_memory_bag"}


@pytest.mark.skipif(PROBE in os.environ, reason="inside a run_pytest run")
def test_static_recording_reports_skipped_without_the_mcap_package(tmp_path):
    """Codex P2: without mcap (the image has none) test_static_recording.py skips at collection,
    nothing is collected and pytest exits 5, not 0, so CTest reported static_recording failed.
    A run that collected nothing and skipped something exits 77 too."""
    code, out = run_pytest(tmp_path, STATIC_RECORDING,
                           environment=without_mcap(tmp_path, "raise ModuleNotFoundError(\"No module named 'mcap'\", name='mcap')"))
    assert code == 77, out
    assert "the mcap package reads the recording" in out, out


@pytest.mark.skipif(PROBE in os.environ, reason="inside a run_pytest run")
def test_static_recording_fails_when_mcap_is_broken(tmp_path):
    """The other side: an mcap that is there but fails to import is a collection error, not a skip."""
    code, out = run_pytest(tmp_path, STATIC_RECORDING,
                           environment=without_mcap(tmp_path, "raise RuntimeError('mcap is broken')"))
    assert code not in (0, 77), out
    assert "mcap is broken" in out, out


@pytest.mark.skipif(PROBE in os.environ, reason="inside a run_pytest run")
def test_a_run_that_collected_and_skipped_nothing_still_fails(tmp_path):
    """Exit 5 alone (every test deselected, none skipped) is not reported skipped. On this file:
    test_static_recording.py skips at collection where mcap is missing, before -k deselects."""
    code, out = run_pytest(tmp_path, str(HERE / "test_tools.py"), "-k", "no_such_test")
    assert code == 5, out
