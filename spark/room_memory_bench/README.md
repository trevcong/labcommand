# Room-memory replay benchmark

A local, ROS-free load and candidate-detection benchmark for the unchanged Python
reference in `spark/room_memory`. Clouds are generated lazily and sent directly to
an adapter: no point-cloud recording is written. Defaults replay **one hour at
10 Hz per sensor, 80,000 XYZ float32 points per frame** (72,000 frames; 5.76 billion
points). This is a synthetic regression/stress workload, not sensor calibration,
real-world accuracy evidence, or a real-time scheduler test.

## Run

From the repository root, using the existing local virtual environment:

```sh
spark/.venv-local/bin/python -m spark.room_memory_bench --duration 60 \
  --output spark/room_memory_bench/results/python-60s
spark/.venv-local/bin/python -m spark.room_memory_bench --duration 600 \
  --output spark/room_memory_bench/results/python-600s
spark/.venv-local/bin/python -m spark.room_memory_bench --duration 3600 \
  --output /tmp/room-memory-hour
spark/.venv-local/bin/python -m pytest spark/room_memory_bench/tests -q
```

`--duration` is a positive whole number of **replay seconds**, not wall seconds.
Replay is unpaced: every tick is processed in order without dropping frames or
sleeping. `--points`, `--seed` (default 7), and `--noise-m` (range-noise standard
deviation, default 0.01 m) select load and repeatable scene noise.
`--rss-interval` defaults to 10 replay seconds. An hour run is supported but has
not been executed as part of the measurements below.

Each output directory contains:

- `report.json` and `report.md`: completion/skip/failure status, configuration,
  environment, timing percentiles, detection metrics and RSS summary.
- `ground_truth.json`: exact event times, types, geometric bounding boxes in meters,
  move source boxes, initial geometry, and reference-restoration times.
- `detections.jsonl`: normalized lifecycle messages and matching disposition.
- `ticks.jsonl`: every measured tick and generation duration for independent audits.
- `rss_samples.json`: replay timestamp, elapsed wall timestamp, current harness RSS,
  and separate child RSS for a subprocess backend.
- `backend.stderr.log`: subprocess diagnostics, when a binary is launched.

A rerun replaces files in the chosen output directory. Use a separate directory to
preserve a run. A failed run returns exit 1 and retains its partial tick/event/RSS
records; a missing optional binary returns exit 0 with `status: skipped`. Inspect
`status` before comparing reports. Errors from an existing binary are failures,
not skips.

## Workload and reference settings

The room is 8 × 8 × 3 m around a synthetic origin. Rays hit the nearest opaque
axis-aligned box or room wall; background returns behind boxes are occluded.
Independent Gaussian **range** noise is added after intersection. JT128-style
frames use 128 elevation channels with rotating azimuth; Mid-360-style frames use
independent broad solid-angle samples. These are simplified sampling patterns,
not manufacturer scan trajectories or measured mounting/extrinsic parameters.
Both sensors use the same ideal origin and geometry, independently sampled and
independently evaluated. Arrays are contiguous `(N, 3)` float32 XYZ meters.

Every 60-second cycle starts with two baseline 0.6 m boxes:

| Cycle offset | Action | Expected candidate |
| --- | --- | --- |
| 10 s | Insert a third box | appeared |
| 25 s | Remove a baseline box | disappeared |
| 40 s | Translate the other baseline box by 1 m | moved |
| 60 s | Restore the baseline scene | no new candidate; prior candidates expire |

Changes happen instantaneously at frame timestamps, not over an interpolated
trajectory. The expected move destination and source are both in ground truth.
Restoration is an explicit scenario control, excluded from scored changes because
the reference compares against a frozen map, not the preceding frame. A candidate
emitted at a restoration without an eligible target is still a false alarm.

The adapter uses the reference's default detection configuration: 5 cm voxels,
15 m maximum range, tolerance 1, minimum 6 voxels / extent 2, evidence of absence
and move pairing enabled, all evidence rays, one-second windows, five reference
windows, one recent window. Only the sensor ordering is explicit and persistence
is disabled (`state_dir=None`). Both sensors receive all ten frames before each
window closes; `status` is also called once per second. No reference source is
modified, and ROS/CDR decoding, publication, disk persistence, and network costs
are outside this benchmark.

The generator holds only one tick's two clouds. The reference holds its own
one-second frame window and allocates occupancy/ray intermediates. Total RSS can
therefore be much larger than the generator's ~1.9 MB output pair. Small timing
arrays and matching metadata grow with duration; no history of clouds is retained
by the harness. Frame RNG streams derive from `(seed, tick_index, sensor_index)`,
so changing duration does not change the shared prefix of a replay.

## Metrics and matching rules (version 1)

A tick is **one frame for each sensor plus window closure/status if due**.
`tick_ms` measures wall time inside `backend.tick`, including normalization; it
excludes generation, report I/O, RSS sampling, initialization and shutdown.
Subprocess timings additionally include framing, pipe transport and JSON decoding;
these are end-to-end adapter timings, not isolated C++ compute times. `wall_s`
includes initialization, replay, sampling/logging, and shutdown; final report
serialization is excluded. Generation and closing-tick distributions, cumulative
backend seconds and the number of ticks above the 100 ms input period are also
reported. All ticks, including reference construction, enter the timing metrics.
Percentiles use NumPy's default linear interpolation. Empty distributions are JSON
`null`, never NaN.

Matching is online and deterministic:

1. Score only the first `candidate` message for each `(sensor, event_id)`.
   Later repeats of that ID and other lifecycle states are ignored and counted.
   A different candidate ID at the same location is **not** silently deduplicated.
2. A ground-truth event has one target **per sensor**. A candidate must have the
   same kind and sensor, and be emitted between event time and event time + 10 s,
   inclusive, before that event's scene interval ends.
3. Expand both destination AABBs by 0.10 m on all faces; require 3D IoU ≥ 0.05.
   Padding permits visible-face / voxelized boxes without requiring full hidden
   surface coverage. A moved candidate must additionally supply `from_centroid`
   within 0.50 m Euclidean distance of the ground-truth source-box center. This is
   a deliberately coarse box-event test, not a shape-reconstruction metric.
4. Select the highest IoU unmatched eligible target, breaking ties by earliest
   event time, then truth ID. Each target and candidate can match only once.
5. Every remaining candidate is a false alarm, including wrong-kind, late, spatial
   duplicates with new IDs, and unmatched fragments. A split appeared/disappeared
   pair does not count as a move. The move remains a miss.
6. Hit rate = matched / eligible sensor targets. Targets with less than the full
   10-second horizon remaining at replay end are reported as censored and excluded
   from hit-rate/latency denominators. They can still absorb a correct candidate,
   avoiding an artificial false alarm. Latency = emission time minus event time,
   in replay seconds; first-seen timestamps are not substituted for emission time.
7. False alarms/hour = total unmatched candidates across **both sensors** × 3600 /
   `(duration - 5 reference seconds)`. This is alarms per room replay hour, not per
   sensor hour or wall hour; the denominator includes change and restoration
   intervals. Nonpositive exposure and no eligible targets produce null rates.

Current RSS comes from macOS `proc_pidinfo(PROC_PIDTASKINFO)` or Linux
`/proc/<pid>/statm`; it is **not** peak `ru_maxrss`. Samples occur after backend
initialization, after closing each sampling-interval window, and at replay end,
while the backend is still alive. Reports include start/end/delta, sampled maximum,
and least-squares slope in bytes per replay second. Both replay and wall times are
stored for further analysis. Sampling can miss transient allocation peaks.
Python RSS includes the generator, harness and reference together; subprocess RSS
is separate from harness RSS and excludes grandchildren. Startup growth includes
reference construction and allocator retention and must not be called a leak
without inspecting the later samples. No garbage collection is forced.

## Pluggable backends and C++ stdio contract

`backends.Backend` has `tick(Tick) -> list[dict]`, `close()`, and an optional child
`pid` for RSS sampling. `PythonBackend` invokes `RoomMemory` directly. A future
binary can implement the following protocol without importing this harness:

```sh
spark/.venv-local/bin/python -m spark.room_memory_bench --duration 60 \
  --backend cpp --binary /path/to/room_memory_cpp \
  --output /tmp/room-memory-cpp
```

The default binary path is `spark/room_memory_cpp/build/room_memory_bench`, relative
to this checkout. It is expected to be absent on this branch. There is no C++
performance claim here. The subprocess is launched directly, without a shell, as:

```text
PATH --room-memory-bench-stdio-v1
```

Requests on **stdin** have a four-byte **little-endian uint32** JSON-header byte
length, exactly that many UTF-8 JSON bytes (no required newline), followed by the
binary payload described by that header. Responses on **stdout** are one UTF-8
JSON object per newline, at most 1 MiB including newline. Flush each reply. All
logging goes to stderr. Only one request is outstanding. A backend should consume
the complete request before replying. Requests/responses are:

```json
{"op":"init","version":1,"config":{"sensors":["jt128","mid360"],"voxel_size":0.05,"max_range":15.0,"tolerance":1,"min_voxels":6,"min_extent":2,"window_s":1.0,"reference_windows":5,"recent_windows":1,"state_dir":null}}
{"op":"ready","version":1}
```

Initialization creates a fresh in-memory reference engine at time zero, with no
persistence. Both sensors keep separate references. No payload follows init.

```json
{"op":"tick","index":9,"time_s":0.9,"end_s":1.0,"close_window":true,"frames":[{"sensor":"jt128","points":80000,"bytes":960000},{"sensor":"mid360","points":80000,"bytes":960000}]}
{"op":"events","index":9,"events":[]}
```

The tick payload concatenates each listed frame in header order, exactly `bytes`
bytes each. Each point is three **little-endian IEEE754 float32** values, XYZ;
there is no padding, timestamp channel or intensity. `bytes == points * 12`.
`time_s` is acquisition time and `end_s` is the tick end. Ingest both frames,
close the accumulation window at `end_s` only when `close_window` is true, and
perform the equivalent status bookkeeping. Reply even when no events exist.
All events emitted during that operation carry `time_s == end_s`:

```json
{"op":"events","index":109,"events":[{"time_s":11.0,"sensor":"jt128","event_id":0,"state":"candidate","type":"appeared","bbox":[[0.95,-0.85,-0.65],[1.65,-0.15,0.0]],"from_centroid":null}]}
```

`event_id` is a stable nonnegative integer unique per sensor for the run. `sensor`
is `jt128` or `mid360`; `type` is `appeared`, `disappeared` or `moved`; `state` is
`candidate`, `confirmed`, `accepted`, `reverted` or `expired`. `bbox` is `[min,max]`
of the destination (or removed region) with finite ordered XYZ coordinates.
`from_centroid` is null/absent except for moves, which need a finite XYZ source
centroid to qualify for a hit. Candidate detection is scored identically for both
backends; terminal transitions are retained in the audit log.

```json
{"op":"shutdown"}
{"op":"bye"}
```

Shutdown has no payload; flush the reply and exit zero. Errors may be reported as
`{"error":"description"}` and cause the run to fail. Malformed messages, mismatched
indices, EOF, nonzero shutdown and stalled reads/writes also fail. `--timeout`
(default 120 wall seconds) bounds each entire request/reply exchange and shutdown
wait. On failure the harness kills and reaps **only its own spawned child**.
Protocol tests use a temporary Python executable to verify transport and failure
handling; that fixture is not a substitute C++ backend or a performance result.

## Measured Python results

Completed locally on 2026-10-14: macOS 26.5.1 arm64, Python 3.9.6, NumPy 2.0.2.
Both runs used 80,000 points per sensor frame, 10 Hz, seed 7, 1 cm range noise,
default reference settings and ten-second RSS sampling. These are single runs on
a shared Mac; brief pytest runs overlapped early replay. They are diagnostic
measurements, not isolated machine-capacity comparisons.

| Metric | 60 s smoke | 600 s stress |
| --- | ---: | ---: |
| Wall seconds | 73.023 | 666.232 |
| Ticks / sensor frames | 600 / 1,200 | 6,000 / 12,000 |
| Tick p50, ms | 0.008 | 0.006 |
| Tick p95, ms | 998.984 | 848.359 |
| Tick p99, ms | 1393.130 | 1205.735 |
| Tick max, ms | 1804.447 | 1648.171 |
| Closing-tick p50, ms | 1000.333 | 848.396 |
| Ticks over 100 ms | 60 | 600 |
| Backend processing seconds | 62.717 | 564.650 |
| Sensor targets hit / expected | 6 / 6 | 60 / 60 |
| Hit rate | 100.0% | 100.0% |
| False alarms / exposure seconds | 0 / 55 | 0 / 595 |
| False alarms per replay hour | 0.00 | 0.00 |
| Latency p50 / p95 / p99 / max, replay seconds | 1.0 / 1.0 / 1.0 / 1.0 | 1.0 / 1.0 / 1.0 / 1.0 |
| RSS start / end / sampled peak, MiB | 27.50 / 436.41 / 1782.02 | 27.38 / 467.56 / 1727.77 |
| RSS end minus start, MiB | 408.91 | 440.19 |
| RSS fitted slope, MiB/replay second | -7.588 | -0.118 |

The ten-minute run processed **960 million points**. Both sensors matched every
scripted appearance, disappearance and move under the documented rules; zero
false alarms on this small synthetic scene does not establish a real-world rate.
The 1 s latency is replay/window latency, not an end-to-end real-time guarantee.
The tiny median tick reflects frame queuing; all 600 window-closing ticks exceeded
the 100 ms input period. The full replay took 666.2 wall seconds for 600 input seconds.

RSS during the ten-minute run ranged from 370.69 to 467.56 MiB after replay second 60.
Its large startup peak and end-minus-start growth include reference construction
and allocator retention. The samples do not establish a memory leak or prove an
hour-long plateau. The fitted slope includes startup and is not a leak estimate.
An actual one-hour run remains unmeasured.

Raw artifacts: [60 s report](results/python-60s/report.json),
[600 s report](results/python-600s/report.json),
[60 s RSS samples](results/python-60s/rss_samples.json),
[600 s RSS samples](results/python-600s/rss_samples.json).
The adjacent ground-truth, event and tick logs allow recomputing all reported
metrics. Both summaries were checked against those logs after completion.

Validation: **17 harness tests passed**; the unchanged reference suite reported
**193 passed, 1 skipped**. The absent default C++ binary was also exercised through
the CLI and produced `status: skipped`; no C++ implementation was benchmarked.
