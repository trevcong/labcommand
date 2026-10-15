# room_memory_cpp: plan and results (Phase 2, production C++)

The production room-memory core and node for the DGX Spark (aarch64, Ubuntu 24.04, ROS 2 Jazzy). `spark/room_memory` (Python) stays the reference oracle. It changes only where the semantics are shared (thresholds, persistence, message content), so every output here stays byte-identical to it, and through the shared goldens to the Swift `change-detect`. Contracts: `docs/CHANGE-DETECTION.md` (v2, unchanged: everything below is engine-side) and `docs/ROOM-MEMORY-PROTOCOL.md` (D23–D26 and the §3.3 definitions are this round's).

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
| `include/room_memory/memory.hpp`, `src/memory.cpp` | voxel layers, bricks, window counts (`VoxelStats`), reference persistence, gated candidate tracker, map stream and sync log, coverage and pose estimate, `RoomMemory` | — |
| `tools/offline*.cpp` | `room_memory_offline`: the C++ `python -m room_memory.offline` | core |
| `tools/replay.cpp`, `tools/config_keys.hpp` | `room_memory_replay`: scripted `RoomMemory` runs (windows, status, `map`, `sync`) printing CDR hex (differential tests) | core |
| `tools/bag.cpp`, `tools/bag_stream.py`, `tools/bag_excerpt.py` | `room_memory_bag`: a recording replayed at its own timestamps (window and 1 Hz status deadlines on their own schedules, as the node's timers), every output and its size; the mcap streamer (with ray-cast box injection) that feeds it; the excerpt cutter | core; Python `mcap` |
| `node/room_memory_node.cpp`, `launch/`, `package.xml`, `Dockerfile` | thin rclcpp node, same topics/QoS/parameters as `node.py`; the launch file also starts the read-only foxglove_bridge sidecar on :8767 | rclcpp, foxglove_bridge (Docker only) |
| `bench/` | `bench_memory.cpp` (the Python benchmark's workload) and `dump_frames.py` (its exact frames) | core, Python |
| `tests/` | CTest suites, the C++ port of `scenes.py`/`goldens.py`, the pytest differential bridge, the static-recording regression (`tests/recording`, gated), the committed real excerpt (`tests/fixtures/static5m-mid360-excerpt.mcap`) | core, Python (bridge) |

The core is plain C++20 with no ROS dependency. `CMakeLists.txt` builds core, CLIs, tests and benchmark with plain CMake (Mac: Apple clang + Homebrew libomp, or GCC + libgomp); with `ament_cmake` and `rclcpp` present (colcon in `ros:jazzy`) it also builds the node and calls `ament_package()`. The ROS package is `room_memory_cpp`, so it coexists with the Python `room_memory` in one workspace.

## Build, test, benchmark

```sh
# Mac (brew install cmake libomp; CXX=g++-16 for the GCC/libgomp build)
cmake -S spark/room_memory_cpp -B build/room_memory_cpp -DCMAKE_BUILD_TYPE=Release
cmake --build build/room_memory_cpp
ctest --test-dir build/room_memory_cpp          # 16 entries; the differential ones use spark/.venv-local
CHANGE_DETECT=$PWD/.build/release/change-detect ctest --test-dir build/room_memory_cpp -R differential  # + Swift
spark/.venv-local/bin/python spark/room_memory_cpp/bench/dump_frames.py /tmp/frames
OMP_NUM_THREADS=1 build/room_memory_cpp/bench_memory /tmp/frames [--full-diff]
build/room_memory_cpp/room_memory_offline before.bin after.bin   # = python -m room_memory.offline
# A recording through the node's engine (pip install mcap zstandard into spark/.venv-local):
spark/.venv-local/bin/python spark/room_memory_cpp/tools/bag_stream.py ~/Developer/lc-tasks/recordings/static5m \
    [--inject mid360:appear:180:-2.2,1.4,0.0:0.3] | build/room_memory_cpp/room_memory_bag [key=value ...]
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
3. **pose and learning** (tracking windows): `pose_offset` of the window against the reference (27 membership probes per occupied voxel, parallel integer counts), then, while it is at most `learn_max_offset_m`, all of the window's rays (`VoxelLayer::observe`) to count the voxels the reference has not settled (a sorted merge).
4. **diff** in the engine's *regions* scope (below): sorted-set differences/intersection (parallel merges split at value ranges), tolerance dilation against a `Membership` (dense bit grid over the set's box, or `KeyIndex`), rays, clustering, pairing.
5. **tracker and events**: serial; masks are zlib `MapBricks` snapshots. Every event message is logged for `RequestSync`.
6. **map tick** (`map_update`, once the delta period has passed): newly settled voxels join the references, which are saved, then a delta or a snapshot of the map sensor's bricks.

Recent windows are kept as `CellSummary` values (per voxel: point count and minimum order-key triple), not points. Merging summaries is exact for the occupancy of the concatenated points (sum and lexicographic minimum are associative and commutative), so `recent_windows > 1` costs a merge instead of re-quantizing every stored point. A window without usable points ages the deque with an empty summary and skips the tracker, as in the third Codex fix.

### Engine diff scope (exact)

`RoomMemory` reads only `report.regions`, and `report_unobserved` is off. Dilation tests each voxel on its own, and the rays only filter, so `disappeared = surviving(cast_rays(keep = only_before)) = cast_rays(keep = surviving(only_before))`. `DiffScope::regions` therefore dilates the before-only voxels first and casts rays only against the survivors. On real data the noise voxels of `only_before` sit next to after-occupied surface voxels and are suppressed, so the keep set shrinks to the actual change, and almost every ray is rejected up front by its start–end box. Only `unknown_voxels` differs, so the scope leaves it unset, and `to_json` refuses such a report. The offline CLI and every golden use `DiffScope::full`. Proof by test: `regions_scope_matches_the_full_diff` (every golden scene), `moved_box_is_one_moved_candidate` (both scopes in the engine), and the benchmark reproduces Python's event bytes in both scopes (`--full-diff`).

### Determinism

Every parallel result is a set union, a sum, or a minimum, or else is written to index ranges fixed before the parallel step: point aggregation, mark grids (per-thread grids ORed together, or one grid taking relaxed atomic ORs), hit flags, partitioned merges, and chunked copies. Nothing depends on scheduling, chunking or the thread count. CTest runs every golden, core and memory suite at `OMP_NUM_THREADS` 1, 3 and 18 (the core suite also loops 1/2/5/max threads in-process), and the differential suite at 1 and 18.

### GPU seam

`RayBackend` (`ray_backend.hpp`) takes a `RayBatch` (endpoint keys and floats, origin in voxels, start cell, inverse) and returns sorted marks, optionally restricted to a keep set. `cast_rays`, `ray_marks`, `DiffOptions::backend` and the `RoomMemory` constructor accept one; null means `cpu_ray_backend()`. The rays are the only stage worth offloading, and the contract is spelled out there; there is no GPU code.

## Reference model, change gate, map stream (this round)

### Reference: per-voxel window counts

A sensor's reference is no longer the union of its first windows. `VoxelStats` counts, per voxel, the windows that observed it (`seen`: occupied, or crossed by a ray) and the windows that hit it (`hits`). `settled(rule)` turns the counts into evidence:
- **occupied** when `hits >= occupied_min_hits` (2) and `hits >= occupied_min_ratio × seen` (0.3);
- **free** when `seen >= free_min_seen` (2) and `hits <= free_max_ratio × seen` (0: never hit);
- anything else (a voxel seen once, or one that flickers) stays **UNKNOWN**. It can then be neither DISAPPEARED (it is not occupied in the reference) nor APPEARED (it is not known).

BUILDING counts `reference_windows` (30) windows per sensor and publishes the settled set as the reference, recomputed each window so status shows the coverage growing. A sensor's reference is ready once it has its windows and at least one settled occupied voxel; until then it keeps counting windows (with `reference_windows` below `occupied_min_hits` nothing settles in the quota, and tracking an empty reference could never recover: `pose_offset` has nothing to register on, so no window may teach it). TRACKING starts when every sensor is ready. At BUILDING → TRACKING the counts of the settled voxels are dropped. The unsettled ones keep counting in TRACKING (new coverage), and voxels that settle join the reference at the next map tick, with `map_epoch` + 1 and the references saved. Settled voxels are never revised, so a real change is never learned away. Persistence is unchanged: a saved reference is the settled layer as a `MapBricks` snapshot, and a reload restarts the counts empty and takes `map_epoch` + 1, saved at once, so `(map_epoch, seq)` sync cursors never repeat across restarts. Every save also writes the room's epoch file, `<state_dir>/<room_id>/map_epoch` (decimal and a newline). A run with a sensor subset saves only its own references, so the others' saved epochs can lag behind it: a load takes one past the highest of the epoch file and the loaded references, and saved references need not agree on `map_epoch`. A sensor with no saved reference builds from the room's epoch, not from 0. A tracking window learns only while its pose estimate is at most `learn_max_offset_m` (0.02 m). Without that check, a moved rig's shifted surfaces settle in voxels the reference never saw (behind walls), and the reference half-adopts the new pose: on a 14 cm JT128 shift the estimate decayed from 0.16 m to 0.013 m within a minute.

The recent layer is the merge of the last `recent_windows` (3) window summaries. A reference voxel counts as DISAPPEARED only if none of the last three windows hit it.

### Change gate

`CandidateTracker(candidate_windows, expire_windows)`. A region first matches an open candidate, else a pending region, else it starts a pending one; matching means same kind, a shared voxel, at most one region per entry per window. Only fresh windows count toward promotion: the recent layer keeps a return for 3 windows, so a box seen once would otherwise match three reports in a row and pass the gate alone. An appeared or moved region is fresh when the window itself occupies one of its (new-pose) voxels; a disappeared one when the window's own rays traverse one of its voxels (one keep-mode cast over the disappeared regions' keys), since the merged layer keeps a window's free space for 3 windows too. A stale region keeps a pending or open match alive but neither counts nor starts a pending region. A pending region becomes a CANDIDATE, with its event id and `first_seen` of its first fresh window, after `candidate_windows` (3) fresh windows in one unbroken run of matched windows. It is dropped silently the first window it is not matched. An APPEARED or DISAPPEARED region that matches no open entry of its kind but shares a voxel with the same side of an open MOVED candidate (APPEARED its new pose, DISAPPEARED its old one) is a half of that move and starts nothing; a window with both halves counts as the move's match. A lone half keeps nothing alive: a move removed from its new pose leaves only the DISAPPEARED at the old one, so the move expires after `expire_windows` and the removal then starts its own candidate (Codex P2; before, the lone half reset the move forever). The diff's move gates (`move_count_ratio` 2) can split one move into its two halves from window to window, and on the injected Mid-360 move with 1 cm noise (old pose 156 voxels in the 30-window reference, new pose 73–84 in the recent layer) the ratio sat on the gate and the move expired and reopened as APPEARED + DISAPPEARED every few seconds. An open candidate goes EXPIRED after `expire_windows` (3) consecutive windows without a match. With both at 1 this is the old tracker exactly; the mechanics tests (`mid360_config`, `SINGLE_WINDOW`) run that way.

### Map stream and RequestSync

`RoomMemory::map_update(stamp)` runs after every `close_window` and returns the live `MapBricks` CDR, if one is due:
- nothing while BUILDING;
- the first call in TRACKING: a snapshot of the map sensor's reference (`map_sensor`, default the JT128 when configured, D25);
- afterwards, once `delta_period_s` (5) has passed since the last map message: voxels settled since join the references. If any did, `map_epoch` + 1, the references are saved, and the changed bricks go out as a delta (`base_epoch` = the last message's epoch; a brick that became all-unknown would be a removed index, which the grow-only reference never produces).
- a snapshot instead when `keyframe_period_s` (60) has passed since the last one (an unchanged map keeps its epoch), or when the deltas since it would exceed `keyframe_delta_ratio` (0.5) of its size (D24).

The node logs every map message (`kMapLogDepth` 10) and every event message (`kEventLogDepth` 100), plus the latest message of each open event. `RoomMemory::sync(request, stamp)` answers by protocol §5:
- maps: replay every logged message after `since` when it is logged with that epoch and something follows. Otherwise a fresh snapshot of the last live state under the latest live `seq`, not logged and not advancing `seq`. Nothing before the first live map message.
- events: replay after `since` when it is logged (possibly nothing). Otherwise the bounded fallback: the open events' latest messages older than the log, in `seq` order, then the whole log.
- another `room_id`: nothing. The node drops (and logs) a request that does not decode.

### Coverage and pose (MemoryStatus)

- **`coverage_known_frac`**: the map sensor's known voxels inside the index box of its occupied voxels, divided by the box's voxel count. Two integers and one division, cached when the reference changes, reported in BUILDING too.
- **`pose_residual_m`**: `pose_offset`, per sensor on the latest window, then the largest over sensors. For each axis, the window's occupied voxels are shifted −4 … +4 voxels and counted where they hit reference-occupied voxels. The peak is refined by the parabola through it and its neighbours, and the result is the norm of the three per-axis shifts. Integer counts, then one fixed sequence of binary64 operations, so C++ and Python agree to the bit. Cost: 27 membership probes per occupied voxel. Measured on the recording, JT128 shifted from 150 s:

  | applied shift | static | (1, 0, 0) cm | (2, 2, 0) cm | (5, 5, 0) cm | (10, 10, 0) cm | (0, 0, 3) cm |
  |---|---|---|---|---|---|---|
  | `pose_residual_m`, median after the shift | 2.8 mm (max 3.4) | 5.0 mm | 26 mm | 74 mm | 162 mm | 35 mm |

  A first version, the trimmed mean of nearest-reference distances, read 6 mm for a 14 cm horizontal shift: floors and ceilings do not move under it, and they dominate any per-voxel average. RIG_MOVED is still not entered (§4.1, PR 2.3); the estimate is reported, and it gates learning.

## False change events on a static room

### Before

The deployed node (union of the first 5 windows, events per single window) on the real recording (`~/Developer/lc-tasks/recordings/static5m`: 299.6 s, rosbag2 mcap with zstd chunks, 2,996 Mid-360 frames of ~20,000 points and 2,987 JT128 frames of ~115,000 points), replayed by `room_memory_bag` at its log times: **587 ChangeEvent messages in 294 s of TRACKING, 119.8 per minute**. That matches the ~2/s, 230–256 B seen live. On the first 86 s alone, before anything in the room moved, it is 116 messages in 81 s (86 per minute). By kind and sensor: 467 DISAPPEARED Mid-360, 8 DISAPPEARED JT128, 100 APPEARED (30 Mid-360, 70 JT128) and 12 MOVED. Almost every CANDIDATE is followed by its EXPIRED one window later.

### Cause: the Mid-360's sparse non-repetitive scan, stored as occupancy by a union reference

Per-voxel statistics over 60 Mid-360 windows (`hits/seen`: the share of the windows observing a voxel in which it was hit):

| range | voxels ever hit | median hits/seen | share < 0.3 | hit in one window only |
|---|---|---|---|---|
| 0–2 m | 10,408 | 1.00 | 0.16 | 0.05 |
| 2–4 m | 17,163 | 0.88 | 0.21 | 0.06 |
| 4–6 m | 6,240 | 0.56 | 0.26 | 0.07 |
| 6–9 m | 6,119 | 0.40 | 0.39 | 0.11 |
| 9–15 m | 699 | 0.53 | 0.35 | 0.20 |

The same over 40 JT128 windows (repeating 128-line pattern): median hits/seen 1.00 in every band and 6–15 % below 0.3. Its first 40 windows give no change region at all.

The 162 voxels of the 44 DISAPPEARED regions the old engine finds in Mid-360 windows 5–59:
- 78 % were hit in only 1 of the 5 reference windows (126; 27 in 2, 9 in 3).
- Over the 60 windows they were hit in a median 17 % of the windows that observed them (80 % below 0.3), while rays crossed them in most windows (median `seen` 50 of 60).
- They sit at a median range of 7.8 m (p10 3.2 m, p90 8.8 m), low in the frame (median z −0.25 m): far surfaces at grazing incidence, at the bottom of the Mid-360's −7° … 52° field of view.

The mechanism: the Mid-360 hits a far or grazing surface voxel in only some windows, and rays to neighbouring returns cross it in the others. A union reference stores it as occupied after a single hit. Every later window whose rays cross it without a hit within one voxel then reports it gone, as a cluster of a few such voxels. The next window hits it again, so the CANDIDATE expires after one window. The APPEARED share is the mirror image: a voxel the reference saw crossed once (free) is hit in a later window.

### Fix and thresholds

The count rule above, with the gate and a 3-window recent layer. Each knob was set from the data. These are event messages on the first 86 s of the recording (81 s of TRACKING for the old engine, 56 s for 30 reference windows), with the other knobs at their defaults:

| configuration | events |
|---|---|
| old engine: union of 5 windows, single window, no gate | 116 |
| count rule, `reference_windows` 10, `recent_windows` 1, no gate | 8 |
| the same with 20 or 30 reference windows | 2 |
| 30 windows, `occupied_min_ratio` 0.2 | 22 |
| 30 windows, `occupied_min_ratio` 0.1 | 236 |
| 30 windows, 3 hits, ratio 0.5, `free_min_seen` 3 | 0 |
| 30 windows, `recent_windows` 3, no gate | 0 |
| node defaults (also the 3-window gate) | 0 |

- **`reference_windows` 30.** 10 windows left 8 messages and 20 left 2, and more windows make the hit share of a sparsely sampled voxel more certain. 30 s of BUILDING costs nothing at run time.
- **`occupied_min_ratio` 0.3 with 2 hits** rather than 0.5 with 3: both clear the static prefix, but 0.3 sits below the 0.40 median hit share of real Mid-360 surfaces at 6–9 m, so those far surfaces stay occupied and a disappearance there remains detectable.
- **`free_max_ratio` 0.** A voxel that was ever hit is not free.
- **`recent_windows` 3** removes the last two messages, a 6-voxel floor patch at 6.3 m reported gone for one window: a reference voxel counts as gone only when three windows in a row miss it.
- **`candidate_windows` 3, `expire_windows` 3.** A margin on top. On the real change below, they also cut the churn from 89 to 49 messages.

### After

The node defaults on the same recording: **0 events in 30–86 s and 0 false events in the whole 269 s of TRACKING (0 per minute)**. The 49 messages that remain (25 CANDIDATE, 24 EXPIRED, first at 89 s) all lie within 0.69 m of one spot in each sensor's frame: (0.35, 1.1, 0) for the JT128 and (−1.2, −0.7, 0) for the Mid-360, about 1.2 m and 1.4 m from the sensors. That is a real change: something low moves next to the rig from 86 s on.
- **Raw returns.** Without any reference involved, the returns in a 0.9 × 0.9 m column (z −0.06 … 0.3 m) at each spot are flat for 80 s (Mid-360 1,524–1,536/s; JT128 5,533–5,642/s). Then both rise together, peak at 130–140 s (Mid-360 1,743/s, +14 %; JT128 7,143/s, +27 %), and ease back by 290 s.
- **JT128 at (0.25, 1.0).** It sees only floor returns at z −0.078 m (2–8 per s) until 85 s. From 86 s it sees returns up to z +0.15 m, 12 to 846 per second.
- **New occupancy.** Voxels occupied now that the 30-window reference holds as free, at the JT128 spot: exactly 0 in every window from 30 to 85 s, then 12, 132, 73 … with bursts at 87–90, 133–136 and 202–206 s. The Mid-360's events cluster in the same seconds (88–90, 127–138, 204–216 s).
- **The rig did not move.** The pose estimate stays at 2.8–3.4 mm throughout.

The regression test treats exactly that region and time as the real change (`REAL_CHANGE_*` in `tests/recording/test_static_recording.py`) and requires 0 events anywhere else.

### Synthetic changes in the real recording

`bag_stream.py --inject`: a 0.3 m cube drawn by ray casting into one sensor's real frames, at 180 s, in observed free space (2-voxel free margin). Each real return whose ray enters the cube first is moved to the entry point, plus 1 cm range noise (added to the range, not scaled by the entry fraction: before the fix a 10 m return entering at 2 m got 2 mm). So the cube occludes what is behind it and is sampled at the sensor's own density, and when it leaves, the real returns behind it are free-space evidence. Detection is the first CANDIDATE of the right kind and sensor within 0.25 m of the cube. Latency is counted from the injection to the end of the window that sends it.

| case | sensor | cube center (sensor frame) | range | detected | latency | other events |
|---|---|---|---|---|---|---|
| appear | Mid-360 | (−2.2, 1.4, 0.0) | 2.6 m | APPEARED, 157 voxels | 3 s | 0 |
| appear | Mid-360 | (4.6, −0.4, 0.0) | 4.6 m | APPEARED, 97 voxels | 3 s | 0 |
| disappear (present from the start) | Mid-360 | (−2.2, 1.4, 0.0) | 2.6 m | DISAPPEARED, 154 voxels | 5 s | 0 |
| move 0.6 m | Mid-360 | (2.2, −0.6, 0) → (2.2, 0, 0) | 2.2 m | MOVED, 80 voxels | 5 s | 0 |
| appear | JT128 | (−2.4, 0.6, 0.2) | 2.5 m | APPEARED, 133 voxels | 3 s | 0 |
| disappear | JT128 | (−2.4, 0.6, 0.2) | 2.5 m | DISAPPEARED, 136 voxels | 5 s | 0 |
| move 0.63 m | JT128 | (−2.4, 0.4, 0.2) → (−2.6, 1.0, 0.2) | 2.5 m | MOVED, 151 voxels | 5 s | 0 |

The latency is the gate: an appearance is in every window from the first, so it needs 3 windows. A disappearance needs the 3-window recent layer to lose the cube first, then the gate's 3 windows (one of them shared): 5 windows.

### Output bandwidth on the recording (node defaults)

| stream | messages | bytes | mean |
|---|---|---|---|
| MapBricks snapshots (JT128 map, 1,838–1,869 bricks, 43 KB) | 6 | 259,350 | 867 B/s |
| MapBricks deltas (new coverage, median 1.25 KB) | 48 | 91,619 | 306 B/s |
| MemoryStatus | 299 | 20,332 | 68 B/s |
| ChangeEvent | 49 | 12,034 | 40 B/s |
| **all room-memory topics** | | 383,335 in 299 s | **1.28 KB/s** (budget 5 KB/s) |

Cadence sweep on the same recording (mean of all topics):

| map sensor | delta / keyframe / ratio | mean |
|---|---|---|
| Mid-360 (29 KB snapshot) | 5 s / 60 s / 0.5 | 1.93 KB/s |
| JT128 (43 KB snapshot) | 5 s / 60 s / 0.5 | 1.28 KB/s |
| JT128 | 10 s / 120 s / 1 | 0.83 KB/s |
| JT128 | 30 s / 300 s / 1 | 0.61 KB/s |

Every setting is far under the budget, so the defaults trade bytes for latency instead. A (re)connecting Mac has a map within 60 s even without `RequestSync`, which the read-only bridge cannot carry, and a newly learned brick reaches the mirror within 5 s. Before this round the node sent 0.53 KB/s (status and events only, no map).

### Window cost on the recording (Apple M5 Max, 18 threads, both sensors)

| | median | max |
|---|---|---|
| BUILDING window | 7.4 ms | 13.5 ms |
| TRACKING window (diff, learning rays, pose, map tick) | 9.3 ms | 36.9 ms (a tick with a JT128 snapshot and save) |
| before this round, TRACKING | 7.5 ms | 23.4 ms |

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

Benchmark: `bench/dump_frames.py` writes the exact frames of `spark/room_memory/bench/bench_memory.py` (numpy PCG64, seed 7) and the Python engine's event bytes. `bench_memory` replays the same loop: two sensors, 10 frames × 80,000 points per sensor per window, 1 s windows, ray evidence on, median of windows 1–5. One reference window settles everything it saw and the gate is off (`candidate_windows` 1), so tracking starts at window 1; the recent layer (3 windows), the learning rays and the pose estimate are the node's.

**This round** (Apple clang + libomp; Python's 6 `ChangeEvent`s reproduced byte for byte in every run):

| | ms per 10 Hz tick (both sensors) | reference-building window |
|---|---|---|
| Python (numpy 2.0.2) | 198 | — |
| C++, 1 thread | 7.3 | 88 ms |
| C++, 6 threads | 2.3 | 33 ms |
| C++, 18 threads | 1.9 | 30 ms |

The target is < 20 ms per tick, met at one thread with 2.7× headroom (12× before this round). The extra cost is the per-window learning: every tracking window now casts all its rays, as a reference-building window does, to count the voxels the reference has not settled. The recent-layer merge and the 27 probes per voxel of the pose estimate add the rest. On the real recording, where the JT128 brings 1.15 M points per window and the Mid-360 0.2 M, a tracking window takes 9.3 ms median at 18 threads, 0.93 ms per tick.

**Previous round**, for reference (`reference_windows=1`, single-window recent layer, no learning; 2 events). Medians of 3 runs:

| | ms per window (1 s, both sensors) | ms per 10 Hz tick | vs Python |
|---|---|---|---|
| Python (numpy 2.0.2) | 611 | 61.1 | 1× |
| C++ clang + libomp, 1 thread | 16.5 | 1.65 | 37× |
| C++ clang + libomp, 6 threads | 6.5 | 0.65 | 94× |
| C++ clang + libomp, 18 threads | 6.4 | 0.64 | 95× |
| C++ GCC 16 + libgomp, 1 thread | 15.2 | 1.52 | 40× |
| C++ GCC 16 + libgomp, 18 threads | 8.2 | 0.82 | 75× |

Beyond 6 threads the window no longer speeds up: the M5 Max has 6 cores in its top performance level and 12 in the lower one, and what is left is serial (sensor order, the tracker, allocation).

Other paths:

| | 1 thread | 18 threads |
|---|---|---|
| tracking window with `--full-diff` (rays against every before-only voxel) | 46 ms | 9.1 ms |
| reference-building window, both sensors (rays from every voxel, all marks kept) | 52 ms | 15.6 ms |
| offline CLI, indoor 89,320-point pair (process wall time; Python 100 ms, Swift release 74 ms) | 28 ms | 27 ms |
| offline CLI, long-ray stride 1 to 15 m (Python 1,697 ms, Swift release 211 ms) | 75 ms | 41 ms |

On the Spark itself the GB10's cores differ from the M5's; the code has no Apple-specific path except the optional LZFSE decoder. The Dockerfile builds with `-mcpu=native` on the Spark.

## Tests and gate

- **CTest (16 entries)**: `goldens`, `core` and `memory` at 1, 3 and 18 threads; `messages`; `cli`; the installed `room_memory_offline` binary byte-compared on both change-detection fixtures; `differential` (pytest) at 1 and 18 threads; `static_recording` (pytest, gated). 85 C++ test cases in total:
  - **goldens** (8): all 37 Swift goldens (27 scenes generated by the C++ port of `scenes.py`, 10 Phase 0 pairs, required when the Phase 0 fixtures exist at configure time) and both `Tests/Fixtures/change-detection` records on the shared `Tests/Fixtures/room/snapshots/{a,b}` pairs. Every one again with the sparse paths forced (no dense grids, hashed aggregation). Plus the spec clear-line literal, shuffled input, and the CLI;
  - **messages** (19): all 12 hand CDR goldens decode to their values, and the 7 exact ones re-encode byte for byte; zlib header/payload; every truncation; trailing padding; the 16 MiB limit before inflating; stream checks; all of Python's rejection tables; LZFSE;
  - **core** (25): the Python unit tests (quantizer, drops, representatives, `min_points`, invalid configs, synthetic scenes, occluded back face, octant sweep, long-ray stride table, extent gate, tie rules), plus SIMD-vs-scalar quantizer lanes, thread-count determinism, regions-vs-full scope, summary merging, and key sort/membership;
  - **memory** (26): all of `test_memory.py`, including the three Codex regressions. New this round: window counts and the settle rule, the brick delta, the candidate gate with expiry hysteresis, the map stream (snapshot, a delta after `delta_period_s`, nothing while unchanged, a keyframe after `keyframe_period_s`; snapshot + delta mirror the keyframe and the reference), a snapshot when deltas outgrow the keyframe, every sync answer path, the event fallback after 100 more transitions, coverage and the pose estimate (a (0.1, 0.05, 0.1) m shift reads 0.15 m and teaches nothing), and the new config rejections;
  - **cli** (7): `test_offline.py` (text block, usage and input errors, flag mapping, decimal rounding).
- **Differential (57 pytest cases)**:
  - the C++ CLI against `python -m room_memory.offline` on every golden scene, a 21-flag grid over 7 scene families, the real fixtures with 4 flag sets, the large scenes, text mode, and error exit codes;
  - with `CHANGE_DETECT` set to a Swift release build, also against Swift;
  - `RoomMemory` window sequences, as replay output (events, status, map messages, sync answers) and saved reference files: board lifecycle, new coverage and moves, persistence and reload (incl. refused loads), degraded/dropped windows, the three Codex scenarios, 4 two-sensor variants, a benchmark-scale window. Those written for single-window mechanics run with `SINGLE_WINDOW`; the others run on the node defaults.
  - New this round: 3 variants of a two-sensor sequence through the count rule, learning, deltas, keyframes, the ratio rule and the gate, with every `RequestSync` path (map replay and snapshot, events since none / since the first / since the latest, a stale `since`, another room, an undecodable request) and a reload. Also eight real Mid-360 frames (`tests/fixtures/static5m-mid360-excerpt.mcap`, 1.9 MB, cut by `tools/bag_excerpt.py` from the recording at 40 s) through both engines: settling, learning deltas and both sync topics on real data, byte for byte. That test skips with a message when the `mcap` package is absent (the Jazzy image).
- **Static recording (10 pytest cases, `tests/recording`)**: skipped, with the path in the message, unless the recording is at `ROOM_MEMORY_RECORDING` (default `~/Developer/lc-tasks/recordings/static5m`; `*.mcap` and `/recordings/` are git-ignored, and kept out of the Docker context, except the excerpt) and `mcap` is importable.
  - The unmodified recording must give no event outside the real change and must detect the real change.
  - It must stay within 5 KB/s, with a snapshot and a status every second.
  - The node defaults loading a saved union-of-5-windows reference (what the deployed node wrote) must still give no event outside the real change.
  - Each of the 7 injections must be detected within 8 s, with no other event.
- Verified here:
  - CTest 16/16 with Apple clang + libomp, the recording present.
  - 15/15 (every entry but the recording) with GCC 16 + libgomp, the Spark's compiler family, and with `-DROOM_MEMORY_SIMD=OFF`, the portable lane code.
  - `pytest` in `spark/room_memory`: 213 passed, 1 skipped (rclpy).
  - `swift run LabCommandCoreTestsRunner`: PASS (193 groups).
  - No TODO/FIXME.
- Not built here: the node (no rclcpp on the Mac; it was compiled with `-fsyntax-only` against stub headers of the rclcpp API it uses) and the Docker image (pulls `ros:jazzy`). The Dockerfile runs `colcon build` and `colcon test` (all CTest suites, with the Python oracle copied in for the differential tests; there the Phase 0, excerpt and recording cases skip).

## Node

`room_memory_node` (node name `room_memory`) declares the Python node's parameters with the same defaults, which are `MemoryConfig`'s (a pytest checks both launch files against it):
- the earlier ones: `room_id`, `sensors`, `voxel_size`, `max_range` (≤ 0: none), `tolerance`, `min_voxels`, `min_extent`, `window_s`, `reference_windows` (now 30), `recent_windows` (now 3), `state_dir` (empty: none, `~` expanded);
- the count rule: `occupied_min_hits` 2, `occupied_min_ratio` 0.3, `free_min_seen` 2, `free_max_ratio` 0;
- the gate: `candidate_windows` 3, `expire_windows` 3;
- learning: `learn_max_offset_m` 0.02;
- the map stream: `map_sensor` (empty: the JT128 when configured), `delta_period_s` 5, `keyframe_period_s` 60, `keyframe_delta_ratio` 0.5.

It subscribes to `/livox/lidar` and `/lidar_points` as generic (serialized) subscriptions with the sensor-data QoS, so the raw CDR goes through `decode_pointcloud2`. It publishes the codec bytes with generic publishers:
- `/room_memory/events`: reliable, depth 100;
- `/room_memory/status`: reliable, depth 1, 1 Hz;
- `/room_memory/map_bricks`: reliable, depth 10, after each window when `map_update` returns a message;
- `/room_memory/sync/map_bricks` and `/room_memory/sync/events`: reliable, depth 10 and 200.

It answers each `/room_memory/request_sync` (reliable, depth 10) on the sync topic of its `topic`, and logs and drops requests that do not decode. Windows and status run on node-clock timers and log wall/CPU time per window and the map bytes. The type support for `sensor_msgs` and `labcommand_msgs` is needed only at run time. `Epoch` (0.2 Hz) is not published yet: with no CONFIRMED state its counts would always be 0.

**Sidecar.** The launch file also starts `foxglove_bridge` (`ros-jazzy-foxglove-bridge`, added to the image) as node `room_memory_bridge`. Its launch arguments are `foxglove_bridge` (default `true`), `foxglove_port` (8767), `foxglove_address` (`0.0.0.0`) and `foxglove_client_publish` (`false`). Its parameters:
- `topic_whitelist` `['^/room_memory/.*']`: the raw clouds are never relayed;
- `capabilities` `['connectionGraph']`, read-only. The list is not empty because ROS 2 parameter files cannot carry an empty list. With `foxglove_client_publish:=true` it adds `clientPublish`.
- `client_topic_whitelist` `['^/room_memory/request_sync$']`. The Phase 3 design read of foxglove_bridge 3.4.1 (`docs/phase3-live-integration.md` on its branch, [F2]) found this parameter parsed but not enforced, which is why client publishing is off by default.

The bridge finds `labcommand_msgs` in the same workspace, so Foxglove on the Mac decodes the custom messages. The lidar containers and their bridge on :8765 are not touched.

Not verified, since nothing here runs ROS:
- the parameter names against the installed bridge version (they are the ones the 3.4.1 README lists);
- that `ParameterValue(..., value_type=List[str])` takes the `PythonExpression` list;
- that the image's apt `ros-jazzy-foxglove-bridge` is 3.4.1. It is not pinned.

The first launch on the Spark should check the bridge's startup log for the whitelist and the capabilities.

**Deploying over the old node.** The live node has saved union-of-5-window references to `~/room-memory/lab/`. The new node loads them as they are and goes straight to TRACKING; the file format did not change. On the recording that still gives no false event (37 messages in 298 s, all at the real change; `test_a_saved_union_reference_still_gives_no_false_events`), because the 3-window recent layer and the gate absorb the noise. But that reference keeps the flickering voxels as occupied and has none of the count rule's coverage. Delete `~/room-memory/lab/reference-*.cdr` before the first start: the node rebuilds the reference in 30 s, with nothing in front of the sensors moving.

## Deviations from the Python oracle

- **LZFSE**: as in Python, uncompressed `bvx-` blocks are decoded natively, and the encoder writes only NONE/ZLIB. Compressed blocks (`bvx1`/`bvx2`/`bvxn`) are decoded with Apple's Compression framework when `ROOM_MEMORY_APPLE_LZFSE` is on (the default on macOS; there a corrupt compressed block is `Invalid`, and so are bytes after the end-of-stream block, which Apple's decoders ignore: the payload must not decode without its last byte). Elsewhere, including the Spark, they are `Unsupported` as in Python. The differential tests contain no compressed LZFSE.
- **Header strings**: encoders also reject a `room_id` that is not UTF-8, which Python strings cannot be; decoders reject the same, so validation stays symmetric. A `PointCloud2` `frame_id` with invalid UTF-8 is kept as bytes (Python substitutes U+FFFD); it reaches no output.
- **CLI**: integer flags outside int64 are a usage error (Python accepts big integers). The usage text names `room_memory_offline`. Error messages follow Python's wording but are not byte-compared (only stdout and exit codes are).
- **Text mode** `elapsed_ms` is this process's own timing, as in Python and Swift.
