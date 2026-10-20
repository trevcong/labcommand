# Spatio x1 bag timing findings — 2026-10-19 CDT

The real Mid-360 schema is now supported. Both 30-second slices and both complete-message passes finished with exit 0 under the shared lock. Mid-360 full-pass point timing is sampled every second scan; all selected headers and receipts are included. This supersedes the earlier real-data-blocked findings. The implementation and original regressions were subsequently committed by the orchestrator in `aaa9670`. The real-data results below are retained from that run; this review-fix pass reran the package tests and checked the saved full-report message counts, without rerunning the recordings.

## Change and evidence

The bug was whitespace-sensitive schema validation, not a different wire layout. The recorded `ros2msg` definition contains `uint8  lidar_id`, `uint8[3]  rsvd`, and unqualified `CustomPoint[]`. All six root fields and seven CustomPoint fields equal the copied driver declarations after comment/whitespace normalization. The validator now normalizes whitespace while retaining exact field types, names, array sizes, and order. Original schema text remains in reports. The exact recorded definition and provenance are saved in `spatio/bagtiming/tests/fixtures/`.

Nanoseconds are source-backed: copied `livox_ros_driver2/src/lddc.cpp:367-395` uses timebase for the ROS stamp and subtracts it for offsets; copied `FAST_LIO_ROS2/src/preprocess.cpp:180-181` divides offsets by 1e6 to get milliseconds. The `.msg` comments do not specify units. Recording provenance names driver `4a1def9` and FAST-LIO `2fffc57`; this is not independent deployed-binary proof. `src/comm/pub_handler.cpp:278-289` has device-time and host-clock branches, so clock selection remains unknown.

The actual JT adapter copy is `~/Developer/spatio-data/spark-src/hesai_ws/jt128_slam_adapter.py` (SHA-256 `66cbd90b3d976dacf9e0509ecaa69951854d294e987e88cea38562087e1f61c7`). Lines 94-102 write FLOAT32 seconds relative to the first kept point and clip negative offsets; line 205 preserves the input header. This does not prove that the header equals the first-point timestamp. No header-origin contract was supplied. The empty `launchers/` copy is not evidence about this adapter. Adapter correctness/IMU units are other workstreams.

## Runs and reproduction

From `~/Developer/spatio-x1-bagtiming`, for each RUN below:

```sh
/usr/bin/lockf -k -t 1800 ~/Developer/lc-tasks/.spatio-bigio.lock nice -n 10 python3 -m spatio.bagtiming ~/Developer/spatio-data/recordings/RUN/bag --out ~/Developer/spatio-data/derived/x1-bagtiming/RUN/slice30 --max-seconds 30
# Full: change output slice30 to full and omit --max-seconds 30.
# Mid-360 full additionally uses --point-timing-sampled-every 2.
```

RUN = `session_20260808_211213` or `20260826T221936Z_JT128_room_walk`. Every output directory contains `bagtiming.json` and `bagtiming.md`. No topic filter or evidence override was used. `spatio-copy.log` contains `COPY_DONE 0`. Every real bag probe and run held `.spatio-bigio.lock`; runs were sequential. `nice` reported `setpriority: Operation not permitted`, so reduced priority was not established.

| Recording / scope | Messages decoded or header-parsed | Point sampling K | Analysis wall time (s) |
|---|---:|---:|---:|
| Mid-360 / slice30 | 7126 | 1 | 98.608 |
| Mid-360 / full | 88328 | 2 | 627.189 |
| JT128 / slice30 | 12719 | 1 | 1.180 |
| JT128 / full | 77111 | 1 | 7.094 |

Slice-based unsampled full estimates were 1259.086 s (Mid) and 7.182 s (JT). Mid exceeded the documented ~1,200 s threshold, so K=2 was selected; halving point decoding suggested roughly 630 s plus overhead, not a guarantee. Actual times above exclude final report serialization and lock wait. Full durations: 383.058498782 s and 182.651983165 s. Full decoded/header-parsed counts match MCAP summary counts exactly; every topic decoded successfully through mcap_ros2 0.5.7, with no manual fallback.

## Compatibility table — measured full-pass values

Rates are span rates. All receipt timestamps use MCAP nanoseconds; headers use ROS seconds + nanoseconds. Mid receipt/header/point clock provenance is unknown. JT receipt time is postprocessing receipt by `meta.yaml`, `postprocess.yaml`, and bag dates; JT header/point physical clock identity remains unknown. These sidecars are interpreted here; the CLI does not auto-import them.

| Recording / topic | Type | Frame | Count | Header / receipt Hz | Point timing, scan duration, numerical IMU coverage | FAST-LIO path / limit |
|---|---|---|---:|---:|---|---|
| Mid-360 `/livox/imu` | `sensor_msgs/msg/Imu` | livox_frame | 76799 | 199.994079 / 200.486349 | n/a; paired coverage reported with lidar | configured IMU input |
| Mid-360 `/livox/lidar` | `livox_ros_driver2/msg/CustomMsg` | livox_frame | 3845 | 9.96629584 / 10.0367373 | timebase + UINT32 offset ns; median 100.216885 ms; 1916/1923 = 99.635985% evaluable, 1922 unknown | CustomMsg direct, lidar_type 1; physical sync unproved |
| Mid-360 `/Odometry` | `nav_msgs/msg/Odometry` | camera_init | 3842 | 9.96624931 / 10.0301059 | n/a | output/diagnostic; not lidar input |
| Mid-360 `/tf` | `tf2_msgs/msg/TFMessage` | camera_init | 3842 | unknown / 10.0301282 | n/a | output/diagnostic; not lidar input |
| JT128 `/slam/points` | `sensor_msgs/msg/PointCloud2` | hesai_lidar | 1818 | 9.97235988 / 9.94832761 | FLOAT32 time at byte 16; raw range 0–0.1004050001502037, 0 nonfinite; seconds/first-kept-point by source; absolute bounds, duration and coverage unknown in CLI | adapter → lidar_type 2; header/first-point relation and effective time-unit config unverified |
| JT128 `/slam/imu` | `sensor_msgs/msg/Imu` | hesai_lidar | 36446 | 199.903371 / 199.540458 | n/a; paired coverage reported with lidar | configured IMU input |
| JT128 `/lidar_packets_loss` | `hesai_ros_driver/msg/LossPacket` | none | 1817 | unknown / 9.94445557 | n/a | output/diagnostic; not lidar input |
| JT128 `/lidar_imu` | `sensor_msgs/msg/Imu` | hesai_lidar | 36420 | 199.902785 / 199.430964 | n/a; paired coverage reported with lidar | raw IMU candidate; adapter input |
| JT128 `/Odometry` | `nav_msgs/msg/Odometry` | camera_init | 555 | 3.04941163 / 3.04211219 | n/a | output/diagnostic; not lidar input |
| JT128 `/path` | `nav_msgs/msg/Path` | camera_init | 55 | unknown / 0.303897458 | n/a | output/diagnostic; not lidar input |

JT seconds are supported by the supplied adapter source, but its exact execution and upstream epoch relationship are not established by the bag. The conservative CLI `UNKNOWN_TIME_UNIT` finding combines missing units/origin; it does not refute this source evidence. Neither the 0.100405 raw maximum nor header cadence is substituted for an absolute scan interval. Coverage against both JT IMU topics is unknown for all 1,818 scans, not 0% coverage.

## Timing findings

| Recording / topic | Header gap count / max interval s | Receipt gap count / max interval s | Header − receipt median s | Offset drift s/s |
|---|---:|---:|---:|---:|
| Mid-360 `/livox/imu` | 0 / 0.012586343 | 0 / 0.012653832 | -0.000291365 | 1.80690725e-05 |
| Mid-360 `/livox/lidar` | 1 / 1.39978272 | 0 / 0.102529856 | -0.103675778 | 0.000118293877 |
| Mid-360 `/Odometry` | 1 / 1.40186501 | 0 / 0.110752959 | -0.018104463 | 8.22373599e-05 |
| JT128 `/slam/points` | 0 / 0.100624 | 0 / 0.11588862 | -1.787794e+09 | -0.00245471067 |
| JT128 `/slam/imu` | 0 / 0.01005 | 3365 / 0.107395254 | -1.787794e+09 | -0.00245659738 |
| JT128 `/lidar_imu` | 0 / 0.01005 | 1827 / 0.10767225 | -1.787794e+09 | -0.00245149886 |
| JT128 `/Odometry` | 77 / 4.112815 | 78 / 4.12171546 | -1.787794e+09 | -0.00246057702 |
| JT128 `/path` | 0 / 0 | 0 / 7.45362807 | -94.1023377 | -1 |

Mid scan duration distribution over 1923/3845 scans: min 98.782975, p5 99.237266, median 100.216885, p95 100.488230, max 101.420400 ms. Sampled header-minus-minimum-point time is zero throughout. There are 259866 negative within-scan point intervals and 0 within-scan resets; storage order is not firing order. Unsampled point behavior remains unknown.

Mid slice coverage was 301/315 (95.555556%); its first lidar header precedes the first IMU header by 1.863188315 s. Full sampled coverage is 1916/1923, with 7 uncovered and 1922 unknown scans. The initial 1.399782717 s lidar header gap is real timestamp evidence, despite smooth receipt cadence. No timestamps were shifted or repaired.

JT `/slam/points` headers span 730.175829–912.379442 s, whereas receipt times are Unix-epoch values. Large negative header-minus-receipt offsets must not be interpreted as transport latency. The bag begins 11,952.266184044 s after the walk start recorded in metadata. IMU receipt gaps describe batched/postprocessed arrival; their headers have no gap above 3× median. `/path` repeats header 1787794728019273971 ns across 55 messages (54 duplicates); its top-level header is not a trajectory time axis. The 555/1,818 odometry/input count ratio is 0.305280528; it does not establish the cause or prove processing backlog.

Mid-360: `/livox/lidar` minus nearest `/livox/imu` header: median -0.000281401 s, p5 -0.0026041738, p95 0.0017038634. This cadence-dependent statistic is not physical clock calibration.

JT128: `/slam/points` minus nearest `/slam/imu` header: median -0.000219 s, p5 -0.002188, p95 0.002191. This cadence-dependent statistic is not physical clock calibration.
JT128: `/slam/points` minus nearest `/lidar_imu` header: median -0.000219 s, p5 -0.002188, p95 0.002191. This cadence-dependent statistic is not physical clock calibration.

No observed topic header/receipt backward jumps, resets, or zero stamps in either full pass. TF and LossPacket lack top-level headers and are not treated as zero-stamped messages. Gaps use 3× the stored-order median interval, not a fixed failure tolerance.

## Validation and limits

`python3 -m unittest discover -s spatio/bagtiming/tests -t . -v`: **41 run, 41 passed, 0 failed, 0 skipped**, 0.587 s. Three added tests cover the exact recorded schema through CLI/sampling, forced fallback parity, and strict rejection of type/name/order/array changes. All 38 existing tests remain. `git diff --check` passed for implementation/docs; the verbatim recorded schema intentionally retains its original trailing space on line 44.

`swift run LabCommandCoreTestsRunner`: exit 1 before tests (0 executed). Initial default-cache denial also produced SDK/compiler errors; retry with writable `CLANG_MODULE_CACHE_PATH` and `SWIFTPM_MODULECACHE_OVERRIDE` failed at `sandbox-exec: sandbox_apply: Operation not permitted`. Logs: `/private/tmp/spatio-x1-swift-tests.log` and `/private/tmp/spatio-x1-swift-tests-retry.log`. No Swift files changed, no test weakened or skipped, and no app build/launch occurred.

This is a bag timing inspection, not a new FAST-LIO replay, trajectory evaluation, or physical acceptance result. Historical replay artifacts must retain the label **“replay demonstrated; physical accuracy not evaluated”**; this task independently proves decoding/timing inspection only. Physical synchronization, mount/IMU axes, deployed configuration, and <10 cm closure remain unproved. MCAP ros2msg/CDR only; no SQLite/IDL support added.

No additional product/design approval is needed for this fix. Any Spark execution, live capture, or physical test still needs separate authorization. No Spark access, container operations, other-workstream edits, app input, push, PR, gh, or Actions trigger occurred.

## Review fixes

The review in `sp-rv-x1-final.md` names `1591f89` and contains one P1, with no P2 findings. The P1 is valid: repeated spaces in the real Mid-360 schema caused rejection before decode. At the start of this pass, the branch already contained `aaa9670`, which fixes the central `schema_sections` parser by normalizing declaration whitespace. Exact field names, types, array sizes and order remain enforced. No additional production change is needed, and no specification or existing test was loosened.

Retained the three regressions from that fix: recorded-schema CLI/sampling, forced fallback parity, and rejection of incompatible layouts. Added one regression that pins the recorded schema SHA-256 from the review and requires normal `mcap_ros2` decoding without manual fallback. It verifies receipt time, the exact integer timebase `1700000000000000017` ns, offsets `[5, 19]` ns, and scan bounds `1700000000000000022`–`1700000000000000036` ns. Substituting the original `1591f89` schema gate in memory reproduces the exact pre-decode `InputError`; the current gate passes. No source files were reverted for this check.

Run the full package suite sequentially from this worktree:

```sh
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 -m unittest discover -s spatio/bagtiming/tests -t . -v
```

Current result: **42 tests run, 42 passed, 0 failures, 0 errors, 0 skipped**, in 0.587 s. All 41 prior tests remain unchanged. `git diff --check` passes for this follow-up.

Swift attempt: `swift run -j 1 LabCommandCoreTestsRunner`, with writable module caches under `/private/tmp`, exited 1 during manifest evaluation: `sandbox-exec: sandbox_apply: Operation not permitted`. **0 Swift tests executed**; this is blocked validation, not a passing suite. Log: `/private/tmp/spatio-x1-review-swift-tests.log`. No Swift code changed and no app was built or launched.

The saved full reports were read again: Mid-360 has **88,328** inspected messages, matching its MCAP summary; JT128 has **77,111**, also matching. These are prior-run report values, not fresh bag runs. The new regression uses synthetic point data with the verbatim recorded schema. Neither establishes physical accuracy or synchronization.

The delegate-approved 5 cm wall gate, 5 s window, 1 s gap, must-move minimums, memory-design numbers, and 1 cm placement tolerance remain **provisional**. They are outside this schema fix; x1 does not implement those acceptance gates. No numeric parser changes or scattered numeric guards were introduced.

The implementation fix is already locally committed in `aaa9670`. The requested follow-up commit was attempted with message `Verify x1 recorded Livox schema review fix [skip ci]`, but staging failed: `Unable to create /Users/tconf/Developer/labcommand/.git/worktrees/spatio-x1-bagtiming/index.lock: Operation not permitted`. The new regression and findings remain uncommitted; no permission workaround was attempted. A permitted environment is needed to stage these two worktree files and make that local commit, and to run the Swift suite. The requested findings file outside the worktree was also updated. No further design approval is needed for the P1 fix.
