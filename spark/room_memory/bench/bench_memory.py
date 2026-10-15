"""Realistic room_memory workload: two sensors, ~80k points per frame, 10 Hz, 1 s windows, ray evidence on.

Run from spark/room_memory: python bench/bench_memory.py [windows]
"""
import sys
import time

import numpy as np

sys.path.insert(0, ".")
sys.path.insert(0, "tests")
from room_memory.memory import MemoryConfig, RoomMemory  # noqa: E402
from room_memory.messages import Time  # noqa: E402
from scenes import indoor  # noqa: E402

HZ = 10
POINTS = 80_000


def frames(cloud, rng, count):
    out = []
    for _ in range(count):
        pick = cloud[rng.choice(len(cloud), POINTS, replace=False)]
        out.append((pick + rng.normal(0, 0.01, pick.shape)).astype(np.float32))
    return out


def main():
    windows = int(sys.argv[1]) if len(sys.argv) > 1 else 6
    rng = np.random.default_rng(7)
    pool = {"still": frames(indoor(False), rng, HZ), "moved": frames(indoor(True), rng, HZ)}
    memory = RoomMemory(MemoryConfig(reference_windows=1, occupied_min_hits=1, occupied_min_ratio=0.0, free_min_seen=1,
                                       candidate_windows=1, expire_windows=1, state_dir=None,
                                       # the indoor move lies within the default near field
                                       tolerance=1, min_voxels=6, near_range_m=0.0), Time(0, 0))
    add_ms, close_ms, events = [], [], 0
    for window in range(windows):
        scene = pool["still" if window < 2 else "moved"]
        start = time.perf_counter()
        for frame in scene:
            memory.add_points("mid360", frame)
            memory.add_points("jt128", frame)
        mid = time.perf_counter()
        events += len(memory.close_window(Time(window + 1, 0)))
        end = time.perf_counter()
        if window >= 1:  # skip the reference-building window
            add_ms.append((mid - start) * 1000)
            close_ms.append((end - mid) * 1000)
    per_window = np.array(add_ms) + np.array(close_ms)
    n = 2 * HZ
    print(f"windows timed: {len(per_window)}  frames/window: {n} x {POINTS} pts  events: {events}")
    print(f"close_window ms: median {np.median(close_ms):.1f}  max {np.max(close_ms):.1f}")
    print(f"per-window ms (1 s of data): median {np.median(per_window):.1f}  max {np.max(per_window):.1f}")
    print(f"ms per frame: {np.median(per_window) / n:.2f}")
    print(f"ms per 10 Hz tick (both sensors): {2 * np.median(per_window) / n:.2f}")


if __name__ == "__main__":
    main()
