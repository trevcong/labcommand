# room_memory_cpp: plan and results (Phase 2, production C++)

The production room-memory core and node for the DGX Spark (aarch64, Ubuntu 24.04, ROS 2 Jazzy). `spark/room_memory` (Python) stays the reference oracle and is not changed; every output here is byte-identical to it, and through the shared goldens to the Swift `change-detect`. Contracts: `docs/CHANGE-DETECTION.md` (v2) and `docs/ROOM-MEMORY-PROTOCOL.md` on the Phase 0 branch.

## Layout

| path | role | depends on |
|---|---|---|
| `include/room_memory/keys.hpp`, `src/keys.cpp` | packed Int64 keys, index boxes, sorted-set algebra (parallel merges), LSD radix sort | — |
| `include/room_memory/key_set.hpp`, `src/key_set.cpp` | `KeyIndex` (open addressing on packed keys), `BitGrid` (bits over an index box), `Membership` (grid or hash), `Tuning` | OpenMP |
| `include/room_memory/change_detection.hpp`, `src/occupancy.cpp` | config validation, NEON quantizer, `CellSummary` aggregation (count + representative return), occupancy maps | OpenMP, NEON |
| `src/rays.cpp`, `include/room_memory/ray_backend.hpp` | the spec DDA (4-lane NEON walker), the `RayBackend` seam, `ray_marks` / `cast_rays` | OpenMP, NEON |
| `src/diff.cpp` | diff, tolerance dilation, 26-connected clustering, min-extent gate, moved pairing, v2 order | — |
| `src/json.cpp` | canonical `labcommand.change-detection.v2` record | — |
| `include/room_memory/pointcloud2.hpp`, `src/pointcloud2.cpp` | CDR `sensor_msgs/PointCloud2` decode (and the test-scene encoder) | — |
| `include/room_memory/messages.hpp`, `src/messages.cpp` | CDR codecs for `MapBricks`, `ChangeEvent`, `MemoryStatus`, `Epoch`, `RequestSync`; zlib; LZFSE | zlib, Apple Compression (optional) |
| `include/room_memory/memory.hpp`, `src/memory.cpp` | voxel layers, bricks, reference persistence, candidate tracker, `RoomMemory` | — |
| `tools/offline*.cpp` | `room_memory_offline`: the C++ `python -m room_memory.offline` | core |
| `tools/replay.cpp` | `room_memory_replay`: scripted `RoomMemory` runs printing CDR hex (differential tests) | core |
| `node/room_memory_node.cpp`, `launch/`, `package.xml`, `Dockerfile` | thin rclcpp node, same topics/QoS/parameters as `node.py` | rclcpp (Docker only) |
| `bench/` | `bench_memory.cpp` (the Python benchmark's workload) and `dump_frames.py` (its exact frames) | core, Python |
| `tests/` | CTest suites, the C++ port of `scenes.py`/`goldens.py`, the pytest differential bridge | core, Python (bridge) |

The core is plain C++20 with no ROS dependency. `CMakeLists.txt` builds core, CLIs, tests and benchmark with plain CMake (Mac: Apple clang + Homebrew libomp, or GCC + libgomp); with `ament_cmake` and `rclcpp` present (colcon in `ros:jazzy`) it also builds the node and calls `ament_package()`. The ROS package is `room_memory_cpp`, so it coexists with the Python `room_memory` in one workspace.

## Build, test, benchmark

```sh
# Mac (brew install cmake libomp; CXX=g++-16 for the GCC/libgomp build)
cmake -S spark/room_memory_cpp -B build/room_memory_cpp -DCMAKE_BUILD_TYPE=Release
cmake --build build/room_memory_cpp
ctest --test-dir build/room_memory_cpp          # 15 entries; the differential ones use spark/.venv-local
CHANGE_DETECT=$PWD/.build/release/change-detect ctest --test-dir build/room_memory_cpp -R differential  # + Swift
spark/.venv-local/bin/python spark/room_memory_cpp/bench/dump_frames.py /tmp/frames
OMP_NUM_THREADS=1 build/room_memory_cpp/bench_memory /tmp/frames [--full-diff]
build/room_memory_cpp/room_memory_offline before.bin after.bin   # = python -m room_memory.offline
```

`-DROOM_MEMORY_PYTHON=` (empty) skips the differential tests, `-DROOM_MEMORY_SIMD=OFF` builds the portable lane code, `-DROOM_MEMORY_APPLE_LZFSE=OFF` drops the Apple LZFSE decoder. On the Spark: `docker build -f spark/room_memory_cpp/Dockerfile .` from the repository root.

## Parity rules (what makes the bytes identical)

- **Binary32 everywhere the spec says so**, in the spec's operation order. `-ffp-contract=off -fno-fast-math` are **PUBLIC** compile options of the core target: GCC on aarch64 fuses `a*b + c` by default, which would change `(x*x + y*y) + z*z`, the DDA and the centroid distance. NEON code uses `vmulq`/`vaddq`, never `vfmaq`. Centroids are binary64 `((sum / n) + 0.5) * size` rounded once, as in Python.
- **Quantizer**: `q = p * inverse` (binary32), the `|q| < 2^20` guard, floor by `vcvtmq_s32_f32` (round toward −∞, exact inside the guard); scalar tail uses `std::floor`. A test checks SIMD lanes against the scalar path point by point.
- **Representative return**: the lexicographic minimum of the per-axis order keys (`bits ^ sign-fixup`), exactly Python's segmented minima, so `-0.0 < +0.0` and every coordinate round-trips.
- **DDA**: Python's lockstep loop, per ray: `tMax < 0 → 0`, the NaN drop, `at_end` before marking, per-axis clamp to `+inf`, minimum of tMax, every tied axis still short of its end steps, `tMax += tDelta` by repeated binary32 addition. The Manhattan cap is provably never binding (every iteration steps an axis) and is not coded. The 4-lane walker takes the minimum with `vminq_f32`; it can differ from the strict-`<` chain only in the sign of a zero, and `±0` compare equal, so the same axes step. Keep-mode early exit is Python's rule restated as per-axis "cells past the entry face" counters, plus an exact up-front rejection of rays whose start–end box misses the keep box (the walk never leaves that box).
- **Order**: sorted unique `int64` keys are the only set type that reaches an output; regions sort by `(kind, −count, min_key, from_min_key)`; pairs by `(distance, min keys)`, all stable.
- **Text**: JSON is hand-written like the Swift encoder; floats as `%08x` bits. CDR encoders write little endian; `zlib` payloads use `deflateInit2(6, Z_DEFLATED, −15, 8, default)` fed like Python's `compressobj` (all input, then `Z_FINISH`), which gives the same bytes with the same zlib (1.2.12 here, the system zlib on both sides).

## Architecture of a window

`RoomMemory::close_window` handles each configured sensor in order (event ids and sequence numbers are assigned serially in that order, as in Python); each stage inside is parallel:

1. **summarize** (`occupancy.cpp`), 800k points per sensor-window in the benchmark:
   - NEON quantize 4 points per iteration into packed keys, with vector min/max of the index box and the accepted count;
   - keys become box-relative codes (i, j, k bit fields, i on top, so code order is key order); a per-chunk histogram on the top ≤12 code bits gives ≤4096 **range partitions**;
   - scatter `{code, count, order keys}` records contiguously per partition (no index chasing later);
   - each partition aggregates in a per-thread table: **direct-addressed** by the low code bits when the partition spans ≤2¹³ codes (room-scale boxes: no hashing, voxels come out in order by scanning touched-bit words), otherwise **open addressing** (Fibonacci hash, linear probing) plus a small sort;
   - partitions are in key order, so concatenating them is the sorted voxel set. Large intermediate buffers are thread-local and grow-only, so a window's ~25 MB are not page-faulted in every second.
2. **occupancy_from**: the `min_points` cut and the endpoint floats, parallel by chunk.
3. **diff** in the engine's *regions* scope (below): sorted-set differences/intersection (parallel merges split at value ranges), tolerance dilation against a `Membership` (dense bit grid over the set's box, or `KeyIndex`), rays, clustering, pairing.
4. **tracker and events**: serial; masks are zlib `MapBricks` snapshots.

Recent windows are kept as `CellSummary` values (per voxel: point count and minimum order-key triple), not points. Merging summaries is exact for the occupancy of the concatenated points (sum and lexicographic minimum are associative and commutative), so `recent_windows > 1` costs a merge instead of re-quantizing every stored point. A window without usable points ages the deque with an empty summary and skips the tracker, as in the third Codex fix.

### Engine diff scope (exact)

`RoomMemory` reads only `report.regions`, and `report_unobserved` is off. Dilation tests each voxel on its own, and the rays only filter, so `disappeared = surviving(cast_rays(keep = only_before)) = cast_rays(keep = surviving(only_before))`. `DiffScope::regions` therefore dilates the before-only voxels first and casts rays only against the survivors. On real data the noise voxels of `only_before` sit next to after-occupied surface voxels and are suppressed, so the keep set shrinks to the actual change, and almost every ray is rejected up front by its start–end box. Only `unknown_voxels` differs, so the scope leaves it unset, and `to_json` refuses such a report. The offline CLI and every golden use `DiffScope::full`. Proof by test: `regions_scope_matches_the_full_diff` (every golden scene), `moved_box_is_one_moved_candidate` (both scopes in the engine), and the benchmark reproduces Python's event bytes in both scopes (`--full-diff`).

### Determinism

Every parallel result is a set union, a sum, or a minimum, or else is written to index ranges fixed before the parallel step: point aggregation, mark grids (per-thread grids ORed together, or one grid taking relaxed atomic ORs), hit flags, partitioned merges, and chunked copies. Nothing depends on scheduling, chunking or the thread count. CTest runs every golden, core and memory suite at `OMP_NUM_THREADS` 1, 3 and 18 (the core suite also loops 1/2/5/max threads in-process), and the differential suite at 1 and 18.

### GPU seam

`RayBackend` (`ray_backend.hpp`) takes a `RayBatch` (endpoint keys and floats, origin in voxels, start cell, inverse) and returns sorted marks, optionally restricted to a keep set. `cast_rays`, `ray_marks`, `DiffOptions::backend` and the `RoomMemory` constructor accept one; null means `cpu_ray_backend()`. The rays are the only stage worth offloading, and the contract is spelled out there; there is no GPU code.

## Optimizations, measured (Apple M5 Max, per sensor-window of 800k points unless noted)

| step | 1 thread | 18 threads |
|---|---|---|
| first build: hash-partitioned aggregation, scalar per-ray DDA | 32.2 ms/window (both sensors) | 11.0 ms |
| `summarize`: contiguous partition records instead of index chasing | 11.9 → 8.4 ms | 2.8 → 2.35 ms |
| `summarize`: direct-addressed partitions, vector box/keys in the quantizer, thread-sized chunking | 8.4 → 5.3 ms | 2.35 → 1.1 ms |
| DDA (full keep set, 99k rays): 4-lane NEON walker (state in registers) | 49 → 16 ms | 3.3 → 1.6 ms |
| DDA, all marks (reference build): per-thread private grids instead of one atomically shared grid | 29 → 16 ms | 8.3 → 2.1 ms |
| reject keep-mode rays from their integer start/end box before the float setup; parallel set merges and `occupancy_from` | diff 2.8 → 2.4 ms | diff 1.9 → 1.6 ms |

Things tried and dropped: four scalar rays interleaved per thread (no gain: the walk was instruction-bound, not latency-bound), auto-vectorized lane loops (the compiler refused; hence the intrinsics), and binary or galloping search for the small-against-huge known-set intersection (the merge streams 1.5M keys in 0.3 ms; the searches were no faster).

## Results

Benchmark: `bench/dump_frames.py` writes the exact frames of `spark/room_memory/bench/bench_memory.py` (numpy PCG64, seed 7) and the Python engine's event bytes. `bench_memory` replays the same loop: two sensors, 10 frames × 80,000 points per sensor per window, 1 s windows, ray evidence on, `reference_windows=1`, median of windows 1–5. Every run below reproduced Python's 2 `ChangeEvent`s byte for byte. Medians of 3 runs:

| | ms per window (1 s, both sensors) | ms per 10 Hz tick | vs Python |
|---|---|---|---|
| Python (numpy 2.0.2) | 611 | 61.1 | 1× |
| C++ clang + libomp, 1 thread | 16.5 | 1.65 | 37× |
| C++ clang + libomp, 6 threads | 6.5 | 0.65 | 94× |
| C++ clang + libomp, 18 threads | 6.4 | 0.64 | 95× |
| C++ GCC 16 + libgomp, 1 thread | 15.2 | 1.52 | 40× |
| C++ GCC 16 + libgomp, 18 threads | 8.2 | 0.82 | 75× |

Target: < 20 ms per tick (< 200 ms per window). Met at one thread with 12× headroom. Beyond 6 threads the window no longer speeds up: the M5 Max has 6 cores in its top performance level and 12 in the lower one, and what is left is serial (sensor order, the tracker, allocation).

Other paths:

| | 1 thread | 18 threads |
|---|---|---|
| tracking window with `--full-diff` (rays against every before-only voxel) | 46 ms | 9.1 ms |
| reference-building window, both sensors (rays from every voxel, all marks kept) | 52 ms | 15.6 ms |
| offline CLI, indoor 89,320-point pair (process wall time; Python 100 ms, Swift release 74 ms) | 28 ms | 27 ms |
| offline CLI, long-ray stride 1 to 15 m (Python 1,697 ms, Swift release 211 ms) | 75 ms | 41 ms |

On the Spark itself the GB10's cores differ from the M5's; the code has no Apple-specific path except the optional LZFSE decoder. The Dockerfile builds with `-mcpu=native` on the Spark.

## Tests and gate

- **CTest (15 entries)**: `goldens`, `core` and `memory` at 1, 3 and 18 threads; `messages`; `cli`; the installed `room_memory_offline` binary byte-compared on both change-detection fixtures; `differential` (pytest) at 1 and 18 threads. 73 C++ test cases in total:
  - **goldens** (8): all 37 Swift goldens (27 scenes generated by the C++ port of `scenes.py`, 10 Phase 0 pairs, required when the Phase 0 fixtures exist at configure time) and both `Tests/Fixtures/change-detection` records on the shared `Tests/Fixtures/room/snapshots/{a,b}` pairs. Every one again with the sparse paths forced (no dense grids, hashed aggregation). Plus the spec clear-line literal, shuffled input, and the CLI;
  - **messages** (18): all 12 hand CDR goldens decode to their values, and the 7 exact ones re-encode byte for byte; zlib header/payload; every truncation; trailing padding; the 16 MiB limit before inflating; stream checks; all of Python's rejection tables; LZFSE;
  - **core** (24): the Python unit tests (quantizer, drops, representatives, `min_points`, invalid configs, synthetic scenes, occluded back face, octant sweep, long-ray stride table, extent gate, tie rules), plus SIMD-vs-scalar quantizer lanes, thread-count determinism, regions-vs-full scope, summary merging, and key sort/membership;
  - **memory** (16): all of `test_memory.py`, including the three Codex regressions;
  - **cli** (7): `test_offline.py` (text block, usage and input errors, flag mapping, decimal rounding).
- **Differential (53 pytest cases)**:
  - the C++ CLI against `python -m room_memory.offline` on every golden scene, a 21-flag grid over 7 scene families, the real fixtures with 4 flag sets, the large scenes, text mode, and error exit codes;
  - with `CHANGE_DETECT` set to a Swift release build, also against Swift; run here, every accepted case agreed;
  - `RoomMemory` window sequences, as replay output and saved reference files: board lifecycle, new coverage and moves, persistence and reload (incl. refused loads), degraded/dropped windows, the three Codex scenarios, 4 two-sensor variants, and a benchmark-scale window.
- Verified here: CTest 15/15 with Apple clang + libomp, with GCC 16 + libgomp (the Spark's compiler family), and with `-DROOM_MEMORY_SIMD=OFF` (the portable lane code other architectures use); `pytest` in `spark/room_memory`: 193 passed, 1 skipped (rclpy); `swift run LabCommandCoreTestsRunner`: PASS (180 groups); no TODO/FIXME.
- Not built here: the node (no rclcpp on the Mac) and the Docker image (pulls `ros:jazzy`). The Dockerfile runs `colcon build` and `colcon test` (all CTest suites, with the Python oracle copied in for the differential tests; Phase 0 cases skip there because the fixtures are not in the build context).

## Node

`room_memory_node` (node name `room_memory`) declares the Python node's parameters with the same defaults (`room_id`, `sensors`, `voxel_size`, `max_range` (≤ 0: none), `tolerance`, `min_voxels`, `min_extent`, `window_s`, `reference_windows`, `recent_windows`, `state_dir` (empty: none, `~` expanded)). It subscribes to `/livox/lidar` and `/lidar_points` as generic (serialized) subscriptions with the sensor-data QoS, so the raw CDR goes through `decode_pointcloud2`, and publishes the codec bytes with generic publishers on `/room_memory/events` (reliable, depth 100) and `/room_memory/status` (reliable, depth 1, 1 Hz). Windows and status run on node-clock timers and log wall/CPU time per window like `node.py`. The type support for `sensor_msgs` and `labcommand_msgs` is needed only at run time, so the Dockerfile still needs `ros/labcommand_msgs` from the Phase 0 branch in the build context.

## Deviations from the Python oracle

- **LZFSE**: as in Python, uncompressed `bvx-` blocks are decoded natively, and the encoder writes only NONE/ZLIB. Compressed blocks (`bvx1`/`bvx2`/`bvxn`) are decoded with Apple's Compression framework when `ROOM_MEMORY_APPLE_LZFSE` is on (the default on macOS; there a corrupt compressed block is `Invalid`). Elsewhere, including the Spark, they are `Unsupported` as in Python. The differential tests contain no compressed LZFSE.
- **Header strings**: encoders also reject a `room_id` that is not UTF-8, which Python strings cannot be; decoders reject the same, so validation stays symmetric. A `PointCloud2` `frame_id` with invalid UTF-8 is kept as bytes (Python substitutes U+FFFD); it reaches no output.
- **CLI**: integer flags outside int64 are a usage error (Python accepts big integers). The usage text names `room_memory_offline`. Error messages follow Python's wording but are not byte-compared (only stdout and exit codes are).
- **Text mode** `elapsed_ms` is this process's own timing, as in Python and Swift.
