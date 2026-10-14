"""Writes the frames of ``spark/room_memory/bench/bench_memory.py`` for the C++ benchmark, and
the Python engine's event bytes on the same run, so ``bench_memory`` replays identical data.

    spark/.venv-local/bin/python spark/room_memory_cpp/bench/dump_frames.py <out_dir> [windows]

Writes ``still.f32`` and ``moved.f32`` (10 frames x 80,000 points x float32 x, y, z, little
endian), ``python_events.hex`` (one ChangeEvent CDR per line, in publish order) and
``python_timing.txt`` (the Python benchmark's per-window milliseconds on this run).
"""

import pathlib
import sys
import time

HERE = pathlib.Path(__file__).resolve().parent
PYTHON_PACKAGE = HERE.parents[1] / "room_memory"
sys.path.insert(0, str(PYTHON_PACKAGE))
sys.path.insert(0, str(PYTHON_PACKAGE / "bench"))
sys.path.insert(0, str(PYTHON_PACKAGE / "tests"))

import numpy as np  # noqa: E402

import bench_memory  # noqa: E402
from room_memory.memory import MemoryConfig, RoomMemory  # noqa: E402
from room_memory.messages import Time  # noqa: E402
from scenes import indoor  # noqa: E402


def main():
    out = pathlib.Path(sys.argv[1])
    windows = int(sys.argv[2]) if len(sys.argv) > 2 else 6
    out.mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(7)
    pool = {
        "still": bench_memory.frames(indoor(False), rng, bench_memory.HZ),
        "moved": bench_memory.frames(indoor(True), rng, bench_memory.HZ),
    }
    for name, frames in pool.items():
        np.concatenate(frames).astype("<f4").tofile(out / f"{name}.f32")

    memory = RoomMemory(MemoryConfig(reference_windows=1, occupied_min_hits=1, occupied_min_ratio=0.0, free_min_seen=1,
                                       candidate_windows=1, expire_windows=1, state_dir=None), Time(0, 0))
    lines, per_window = [], []
    for window in range(windows):
        start = time.perf_counter()
        for frame in pool["still" if window < 2 else "moved"]:
            memory.add_points("mid360", frame)
            memory.add_points("jt128", frame)
        events = memory.close_window(Time(window + 1, 0))
        if window >= 1:
            per_window.append((time.perf_counter() - start) * 1000)
        lines += [event.to_cdr().hex() for event in events]
    (out / "python_events.hex").write_text("".join(line + "\n" for line in lines))
    (out / "python_timing.txt").write_text(" ".join(f"{ms:.1f}" for ms in per_window) + "\n")
    print(f"wrote {out}: {len(lines)} events, Python median {np.median(per_window):.1f} ms per window")


if __name__ == "__main__":
    main()
