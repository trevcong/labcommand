# Spatio x1 bag timing findings — 2026-10-19 CDT

**UNCOMMITTED: sandbox refused Git's worktree index.lock. Real-data slices and full passes NOT RUN: sandbox refused the mandatory shared lock. This task is incomplete against the real-data and local-commit criteria.** Code and these findings remain in the assigned worktree for Claude/Trevor to recover. No permission workaround was attempted.

## 1. What I built

A read-only Python inspector in `spatio/bagtiming/`. It writes `bagtiming.json` and `bagtiming.md`, with a compatibility table, schema definitions, frame ids, counts, rates, gaps, backward jumps, resets, zeros, separate receipt/header/point statistics, header-minus-receipt quantiles and drift, lidar-minus-nearest-IMU header offsets, numerical scan coverage, and odometry/input rate comparisons.

`core.py` has no file IO and accepts `(topic, receipt_ns, decoded_msg_like)` records. `reader.py` streams MCAP in storage order. `report.py` defines the measurements and renders Markdown. `__main__.py` supplies the CLI. `tests/` generates only tiny synthetic inputs. No other workstream imports or files are involved.

Reader evidence: installed mcap 1.4.0 and mcap-ros2-support 0.5.7. mcap_ros2 successfully decoded embedded schemas for synthetic Imu, PointCloud2 and Livox CustomMsg, including a full CLI roundtrip. A limited CDR1 CustomMsg fallback and header-only sampling parser pass independent hand-buffer and decoder-comparison tests. Actual recording-schema decoding remains unverified because the first probe could not acquire the required lock. No actual decoder failure has been established.

## 2. How to run it

From `~/Developer/spatio-x1-bagtiming`:

```sh
python3 -m spatio.bagtiming BAG_DIR_OR_MCAP --out DIR [--max-seconds 30] [--topics /livox/lidar /livox/imu /Odometry] [--gap-factor 3.0]
```

The optional `--evidence FILE.json` supplies evidence-backed PointCloud2 units/origin and clock domains; see `spatio/bagtiming/README.md`. Without evidence, units and clock domains remain unknown. Livox point-time units have copied-source evidence built in. Exit 0 means inspection completed, even with warnings/errors; 2 means unreadable/invalid input; 1 means output-write failure.

Required real-data procedure, once the environment permits it: run each 30-second slice under the shared lock, record wall time, extrapolate to full duration, then run full passes sequentially under the same lock. Do not run unlocked. Example Mid-360 slice:

```sh
/usr/bin/lockf -k -t 1800 ~/Developer/lc-tasks/.spatio-bigio.lock nice -n 10 python3 -m spatio.bagtiming ~/Developer/spatio-data/recordings/session_20260808_211213/bag --out ~/Developer/spatio-data/derived/x1-bagtiming/session_20260808_211213/slice30 --max-seconds 30
```

Example JT128 slice, after the Mid-360 slice finishes:

```sh
/usr/bin/lockf -k -t 1800 ~/Developer/lc-tasks/.spatio-bigio.lock nice -n 10 python3 -m spatio.bagtiming ~/Developer/spatio-data/recordings/20260826T221936Z_JT128_room_walk/bag --out ~/Developer/spatio-data/derived/x1-bagtiming/20260826T221936Z_JT128_room_walk/slice30 --max-seconds 30
```

Full runs use the same commands without `--max-seconds 30`, writing into each recording's output directory rather than `slice30/`. If projected runtime exceeds about 1,200 s, select `--point-timing-sampled-every K`. All message headers/receipts remain included; only every Kth point array is decoded. Unknown coverage of unsampled scans stays explicit. No K or runtime estimate was selected here, since neither slice ran.

The sandbox also refused the requested derived-output directory. The authorized fallback is `spatio/bagtiming/_out/`, with recording-specific subdirectories for any later permitted outputs. No real reports were fabricated there.

## 3. Tests

Exact command:

```sh
cd ~/Developer/spatio-x1-bagtiming && nice -n 10 python3 -m unittest discover -s spatio/bagtiming/tests -t . -v
```

Final result: **ran 38 / passed 38 / failed 0 / skipped 0**, unittest elapsed **0.473 s**, exit 0. All 17 task-mandated test names exist exactly and pass. No skipped, expected-failure, or external-data tests. Additional tests cover endian/row padding, malformed layout, nonfinite times, unsigned subtraction, sampling, receipt storage order, bounded gap examples, drift, explicit clock evidence, missing IMU, schema validation, and point epoch resets.

The shell printed `nice: setpriority: Operation not permitted`. The test command nevertheless ran and passed; reduced priority was not established. Tests were sequential, and no worker pool was used. No packages were installed.

Swift suite not run: no Swift files changed.

## 4. Real-data results

**No bag payloads were inspected. No measured scan duration, per-topic rate, header offset, clock drift, discontinuity count, or IMU coverage result is available. No 30-second runtime or full-pass extrapolation exists. This is a lock-access refusal, not a lock timeout.**

The following are rechecked filesystem/metadata facts only, not independent message counts. Source root: `/Users/tconf/Developer/spatio-data/recordings/`.

| Recording | File bytes (stat) | Metadata duration | Metadata starting time (ns) | Metadata total messages |
|---|---:|---:|---:|---:|
| `session_20260808_211213/bag/bag_0.mcap` | 1,570,788,339 | 383.058498782 s | 1786223548870054793 | 88,328 |
| `20260826T221936Z_JT128_room_walk/bag/bag_0.mcap` | 1,285,874,848 | 182.651983165 s | 1787794728266184044 | 77,111 |

Sources: each recording's `bag/metadata.yaml`; file sizes were obtained using stat without opening the large MCAP files. `~/Developer/lc-tasks/spatio-copy.log` contains `COPY_DONE 0`.

Metadata topic inventory:

| Recording | Topic | Exact declared type | Declared count |
|---|---|---|---:|
| Mid-360 | `/livox/lidar` | `livox_ros_driver2/msg/CustomMsg` | 3,845 |
| Mid-360 | `/livox/imu` | `sensor_msgs/msg/Imu` | 76,799 |
| Mid-360 | `/tf` | `tf2_msgs/msg/TFMessage` | 3,842 |
| Mid-360 | `/Odometry` | `nav_msgs/msg/Odometry` | 3,842 |
| JT128 | `/slam/points` | `sensor_msgs/msg/PointCloud2` | 1,818 |
| JT128 | `/slam/imu` | `sensor_msgs/msg/Imu` | 36,446 |
| JT128 | `/lidar_imu` | `sensor_msgs/msg/Imu` | 36,420 |
| JT128 | `/Odometry` | `nav_msgs/msg/Odometry` | 555 |
| JT128 | `/path` | `nav_msgs/msg/Path` | 55 |
| JT128 | `/lidar_packets_loss` | `hesai_ros_driver/msg/LossPacket` | 1,817 |

JT128 `meta.yaml` records the walk at 2026-08-26 22:19:36–22:22:42 UTC, duration 182.323 s, PTP off. `bag/metadata.yaml` starts at 2026-08-27 01:38:48.266184044 UTC, **11,952.266184044 s after the stated walk start**. `postprocess.yaml` records postprocessing at 01:42:36 UTC, `use_timestamp_type: 0 # sensor clock`, stride 4, FAST-LIO lidar_type 2. These metadata records identify the bag log time as postprocess receipt, not walk acquisition. They do not prove `/slam/points` or `/slam/imu` header/point clock semantics after the adapter.

Mid-360 `provenance.txt` declares FAST_LIO_ROS2 2fffc57, livox_ros_driver2 4a1def9, `ptp4l running: 0`. `mid360.yaml` selects CustomMsg input `/livox/lidar`, `/livox/imu`, lidar_type 1, timestamp_unit 3, time_sync_en false, time_offset_lidar_to_imu 0.0. The copied driver can use sensor or host timestamps; PTP-not-running alone cannot decide the message clock domain.

`/Users/tconf/Developer/spatio-data/spark-src/launchers/jt128_slam_adapter.py` is **0 bytes** by stat. No claim is made about the Spark original. The JT metadata has no raw `/lidar_points` topic.

### Compatibility table — provisional metadata/source evidence, NOT a completed bag inspection

All receipt units are ns by MCAP log_time contract. Standard ROS Header units are seconds + nanoseconds by schema contract; actual bag schemas/header values remain untested. Clock-domain labels below distinguish receipt from header/points. “Unknown” is a deliberate result of missing evidence, not a guessed sensor domain.

| Topic | Type | Time unit | Clock domain + evidence | Per-point timing present? | Scan duration | IMU coverage % | FAST-LIO input path | Unknowns |
|---|---|---|---|---|---|---|---|---|
| Mid `/livox/lidar` | `livox_ros_driver2/msg/CustomMsg` | receipt ns; header s+ns; point timebase/offset ns by copied source | receipt unknown; header/points unknown: copied driver has device and host fallback branches | Defined by copied schema; actual records unverified | unknown | unknown | CustomMsg direct, lidar_type 1; mid360.yaml and copied preprocess.cpp | actual schemas, frame ids, rates, offsets, selected clock branch, coverage |
| Mid `/livox/imu` | `sensor_msgs/msg/Imu` | receipt ns; header s+ns | receipt/header unknown; provenance PTP off does not establish clock origin | n/a | n/a | unknown paired coverage | direct IMU topic selected by mid360.yaml | actual frame, timing, physical synchronization; units/axes owned by g6 |
| Mid `/tf` | `tf2_msgs/msg/TFMessage` | receipt ns; nested header s+ns | unknown; no top-level header clock | n/a | n/a | n/a | output | nested clocks not combined into one header timeline |
| Mid `/Odometry` | `nav_msgs/msg/Odometry` | receipt ns; header s+ns | unknown; no publisher stamp evidence | n/a | n/a | n/a | output | actual output rate and stamp semantics |
| JT `/slam/points` | `sensor_msgs/msg/PointCloud2` | receipt ns; header s+ns; point units unknown | receipt postprocess-receipt from walk/bag/postprocess metadata; header/points unknown because adapter copy is empty | unknown: fields not decoded | unknown | unknown | adapter → PointCloud2 lidar_type 2 declared in postprocess.yaml; x2 must establish transformation | timing field, datatype/range, origin/units, adapter semantics, effective time-unit config |
| JT `/slam/imu` | `sensor_msgs/msg/Imu` | receipt ns; header s+ns | receipt postprocess-receipt; header unknown: adapter transformation unavailable | n/a | n/a | unknown paired coverage | declared paired input; transformation owned by x2 | original vs transformed headers; g6 units/axes |
| JT `/lidar_imu` | `sensor_msgs/msg/Imu` | receipt ns; header s+ns | receipt postprocess-receipt; header unknown: sensor-clock parameter is supporting provenance, not decoded proof | n/a | n/a | unknown paired coverage | raw IMU candidate; actual FAST-LIO pairing/config unverified | actual stamps and relation to `/slam/imu` |
| JT `/Odometry` | `nav_msgs/msg/Odometry` | receipt ns; header s+ns | receipt postprocess-receipt; header unknown | n/a | n/a | n/a | output | output rate/stamp semantics and reason for fewer outputs |
| JT `/path` | `nav_msgs/msg/Path` | receipt ns; header s+ns | receipt postprocess-receipt; header unknown | n/a | n/a | n/a | output | actual schema/frames; nested poses not a single header time base |
| JT `/lidar_packets_loss` | `hesai_ros_driver/msg/LossPacket` | receipt ns; other fields unknown | receipt postprocess-receipt; any internal clock unknown | n/a | n/a | n/a | diagnostic, not lidar/IMU input | actual schema and header availability |

The metadata count ratios are **3842/3845 = 0.999219766** for Mid-360 and **555/1818 = 0.305280528** for JT128. These are output/input count ratios only. They are not per-topic rate ratios, completeness guarantees, or evidence of FAST-LIO backlog.

### Errors and warnings / blocked evidence

- **ERROR — shared lock refused:** `/usr/bin/lockf -k -t 1800 /Users/tconf/Developer/lc-tasks/.spatio-bigio.lock ...` exited **73**: `cannot open ... Operation not permitted`. Therefore **0/2 slices and 0/2 full passes** ran. No wait-for-lock timeout occurred.
- **ERROR — commit refused:** `git add spatio/bagtiming/ && git commit ... [skip ci]` exited **128**: cannot create `/Users/tconf/Developer/labcommand/.git/worktrees/spatio-x1-bagtiming/index.lock`, `Operation not permitted`. The failure happened during git add; no commit exists.
- **WARN — external writes refused:** requested plan and derived directory returned `Operation not permitted`; findings use the same task-authorized owned `_out/` fallback. Real outputs do not exist.
- **WARN — reduced priority not established:** `nice` reported `setpriority: Operation not permitted`; synthetic tests still ran and returned 0.
- **WARN — JT receipt/acquisition mismatch:** metadata start is **11,952.266184044 s** after walk start. Receipt timestamps must not be used as firing times.
- **WARN — adapter unavailable:** copied adapter is **0 bytes**. PointCloud2 timing unit/origin and transformed-header semantics are unknown.
- **WARN — clock synchronization unproved:** both recordings' metadata says PTP was off; no physical synchronization conclusion follows from that alone.
- **WARN — real timing inspection absent:** all real scan durations, per-topic rates, header offsets/drifts, reset/gap counts, frames and IMU coverage percentages remain unknown. No per-message error/warn findings were emitted because no real bag was read.

## 5. Measured vs guessed

Measured locally: test results, synthetic decoding/roundtrip behavior, copied source text, metadata text, file byte counts, sandbox failures. The compatibility table labels metadata/source-derived claims explicitly. No real message timing was measured. No timestamps were invented or shifted.

Copied source evidence: `livox_ros_driver2/msg/CustomMsg.msg` defines `uint64 timebase` (first point); `CustomPoint.msg` defines `uint32 offset_time` relative to base. Neither comment names ns. `src/lddc.cpp:367-373` sets header from timebase using rclcpp::Time / ROS1 division by 1e9; line 395 subtracts base_time. `FAST_LIO_ROS2/src/preprocess.cpp:180-181` divides offset_time by 1e6 for milliseconds. This establishes the ns contract used by the inspector. `livox_ros_driver2/src/comm/pub_handler.cpp:278-289` selects sensor time for PTP/GPS or a host high-resolution-clock fallback. It prevents a blanket “sensor clock” classification.

Not established: real recording decoder compatibility, live timestamp source, clock sync, adapter correctness, FAST-LIO backlog, replay success, or physical accuracy. No replay was run here; therefore this work does not independently establish “replay demonstrated; physical accuracy not evaluated.” Historical replay evidence belongs to the lead's separate artifacts.

## 6. Known limits

- This handoff is incomplete until locked real runs and the local commit can be made. No measured compatibility verdict is possible yet.
- MCAP/CDR ros2msg only; no SQLite, IDL/XCDR2, or ROS runtime. CustomMsg fallback supports only the validated copied schema. Unknown encodings/schema differences return 2 rather than skipping messages.
- Clock domains and PointCloud2 unit/origin contracts require explicit evidence; sidecar YAML is not automatically parsed. Magnitudes/datatypes do not supply semantics. The synthetic float32/float64 tests supply explicit producer contracts.
- Physical synchronization cannot be established from timestamp overlap. Nearest-IMU offsets are cadence dependent and not clock calibrations. No unit/axis/gravity or trajectory evaluation was performed.
- Sampled scans have unknown bounds/coverage; point-clock diagnostics apply to the sampled sequence. Header/receipt statistics still cover every selected message. A slice stops at its first out-of-window stored receipt and cannot establish whole-bag discontinuity absence.
- Rates are span rates; null across backward jumps. Gap threshold uses the median stored-order interval and is undefined if that median is nonpositive. Numerical coverage is conservative across missing/zero/reset IMU headers and gaps intersecting scan boundaries.
- Per-point negative intervals describe storage order, not necessarily hardware clock reversals. Float timestamps retain source precision; rounded integer ns do not add precision.
- ROS TFMessage has no top-level header. Nested frame ids are inventoried, but transform header timestamps are not combined into an invented topic clock.

## 7. Needs Trevor's approval

No new product/design approval is needed for the existing authorized scope. A permitted environment is needed for the shared lock, derived outputs, external findings path, and Git worktree metadata. The current approval policy cannot grant those writes from this session. Re-run the documented locked checks and commit locally after those permissions are available. Do not bypass the shared lock or run on the Spark.

Claude's one independent review is still pending. I did not review my own implementation or request another reviewer. Spark execution, live sampling, physical accuracy evaluation and other workstreams remain outside this task.

## 8. Design decisions

- Preserve integer ns until subtraction; never estimate, shift, or correct timestamps.
- Keep receipt, header, timebase, absolute scan bounds and within-scan storage-order diagnostics separate.
- Use mcap_ros2 embedded-schema decoding as the default, with a bounded tested CustomMsg fallback and exact header-only parsing for unsampled lidar records.
- Require explicit evidence for PointCloud2 time units and origin. Unknown stays unknown even when a number looks like an epoch or scan-relative seconds.
- Keep numerical coverage and physical synchronization separate; include boundary-crossing gaps and unknown denominators.
- Stream records; retain only timestamp vectors and scan summaries. Single-process analysis, no installations or pools.
- Preserve source order to expose receipt clock reversals. All schema variants are retained in JSON.
- Use the task's `_out/` fallback after refused writes. `_out/*.md` is unignored for a later commit; generated outputs and caches remain ignored. No file was created outside `spatio/bagtiming/` except disposable synthetic test temporary directories.
- Both shared plans were read fully, branch/base were verified as `spatio-x1-bagtiming` / `d675f8a2d4941647bcbef536ce0ca30838cdf352`. Only the owned folder changed. No Spark access, container operations, app launches, AppleScript, process killing, Swift build, gh, push, PR or Actions trigger occurred.
