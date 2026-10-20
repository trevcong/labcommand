# Spatio x2 — JT128 adapter characterization

**UNCOMMITTED: sandbox refused Git index writes. Real-data characterization is incomplete: sandbox refused the mandatory shared lock, so neither a slice nor a full bag pass ran.** The requested external plan, findings and derived-output paths are also blocked. All created files are inside `spatio/jt128adapter/`. No bypass was attempted.

## 1. What I built

- Byte-identical vendor copies of the staged adapter and its upstream tests, with source paths, sizes, UTC mtimes and hashes in `vendor/PROVENANCE.md`.
- Synthetic characterization of point layout, ring retention, stride, relative timing/precision, scan boundaries, nonmonotonic times, invalid geometry, gyro conversion and static acceleration/header behavior.
- `bagcheck.py`: a ROS-free system `mcap`/`mcap_ros2` reader, per-scan measurements, one-to-one raw/adapted IMU pairing, ratios, signed-axis comparisons, and JSON/Markdown verdict reports. No other workstream imports.
- An executable tiny synthetic MCAP fixture and CLI tests. A reproducible demonstration is in `_out/synthetic/adaptercheck.json` and `.md`; these are **synthetic**, not real-data measurements, and are ignored by Git.

Provenance caveat: **not verified to be the version that produced the 20260826T221936Z bag**. The staged adapter is 9,255 bytes, mtime 2026-08-26T23:23:14.799647Z, SHA256 `1eee0c0173e0fe622f2964e378426de0be9fec29678041564171a492e73c3df8`. Staged upstream tests are 4,166 bytes, mtime 2026-08-26T23:18:20.978159Z, SHA256 `6ae8bf50835379384cbd22b01b9c921e6e3e03c832dd4cda87b964e833be6311`. Both Desktop hashes were confirmed by read only. The Spark-copy adapter is still 0 bytes by local stat. Nothing ran or was written inside Desktop; no Spark access occurred.

## 2. How to run it

From this worktree, use the exact test command in section 3. For a bounded real-data pass:

```sh
cd ~/Developer/spatio-x2-jt128-adapter
/usr/bin/lockf -k -t 1800 ~/Developer/lc-tasks/.spatio-bigio.lock nice -n 10 python3 -m spatio.jt128adapter.bagcheck ~/Developer/spatio-data/recordings/20260826T221936Z_JT128_room_walk --out ~/Developer/spatio-data/derived/x2-jt128-adapter/ --max-seconds 1
```

Remove `--max-seconds 1` for the full pass. If only output writes are blocked, use `--out spatio/jt128adapter/_out/real-data`; the shared lock remains mandatory. Do not use a private lock or run unlocked. Outputs are `adaptercheck.json` and `adaptercheck.md`. The current sandbox cannot run this command because it cannot open the shared lock. Complete the requested initial local commit before a later real-data run.

No dependencies were installed. System Python 3.9.6, numpy 2.0.2, mcap 1.4.0 and mcap_ros2 are sufficient. Actual MCAP/CDR decoding was verified using the synthetic fixture, not the protected large recording.

## 3. Tests

Exact command:

```sh
cd ~/Developer/spatio-x2-jt128-adapter && nice -n 10 python3 -m unittest discover -s spatio/jt128adapter/tests -t . -v
```

**29 ran / 28 passed / 0 failed / 1 skipped.** Every named done-criteria test exists and passes. Full output is in `_out/tests.log` (ignored). `nice` prints `setpriority: Operation not permitted`; the test process still executes. There are no parallel runners or worker pools.

Upstream per-test outcomes, executed from the byte-identical vendored file through a loader:

| Upstream test | Outcome |
|---|---|
| `test_cloud_time_is_seconds_from_scan_start` | PASSED |
| `test_gyro_manual_example_is_deg_s_not_rad_s` | PASSED |
| `test_adapter_on_real_jt128_pcap_points` | SKIPPED: `jt128_one_frame.pcap` was not copied to this Mac |

The harness provides an empty `jt128_decode` module only because the upstream module imports it eagerly; no decoder functions are implemented or called. The unavailable-PCAP test alone is skipped by a harness subclass; vendor bytes are unchanged. Adapter imports themselves require neither ROS nor a stub. A guarded-import test rejects any attempted rclpy/sensor_msgs import. The named accel test checks the actual AST assignment, not a simulated ROS callback. ROS main() was not executed and is not claimed as runtime coverage.

The initial precision assertion used an incorrect expected measurement; it was corrected to the measured 0.11825561523765016 microseconds without widening its tolerance. The final suite is green with the single authorized skip.

**Swift suite not run: no Swift files changed.** No app build or launch occurred.

## 4. Real-data results

**No real bag messages were decoded.** At approximately 2026-10-20T02:23Z (2026-10-19 CDT), the bounded-read command exited 73 with:

```text
lockf: cannot open /Users/tconf/Developer/lc-tasks/.spatio-bigio.lock: Operation not permitted
```

This is a sandbox denial, **not a lock timeout**. The full pass was not attempted after that failure. The external derived directory creation also returned `Operation not permitted`. No real-data `adaptercheck.json` was fabricated.

Small sidecar files were read, and the MCAP file was only statted. Source directory: `/Users/tconf/Developer/spatio-data/recordings/20260826T221936Z_JT128_room_walk/`.

| Source | Rechecked fact | Evidence boundary |
|---|---|---|
| `bag/bag_0.mcap` stat | 1,285,874,848 bytes | Size only; not decoded |
| `bag/metadata.yaml` | 77,111 total; /slam/points 1,818; /slam/imu 36,446; /lidar_imu 36,420; /Odometry 555; /path 55; /lidar_packets_loss 1,817 | Sidecar counts, not independently counted from MCAP |
| `bag/metadata.yaml` | No /lidar_points entry | No raw point mapping available in declared topics |
| `bag/metadata.yaml` | Start 1787794728266184044 ns; duration 182.651983165 s | Recorder timeline, not walk acquisition time |
| `meta.yaml` | Walk 2026-08-26T22:19:36Z to 22:22:42Z; duration_s 182.323; ptp_on false | Recorded metadata statements |
| `postprocess.yaml` | 2026-08-27T01:42:36Z; use_timestamp_type 0 (sensor clock); adapter_stride 4; lidar_type 2 | Configuration statements, not proof of actual adapter version or firing times |
| `/Users/tconf/Developer/lc-tasks/spatio-copy.log` | COPY_DONE 0 | Copy completion marker, not source completeness proof |

The adapted-minus-raw IMU count difference is **26**, from sidecar counts. Its cause and even the real pairing rate remain unmeasured. Startup/shutdown subscriber differences or recorder loss are possibilities, not findings. The implemented report uses exact matches first, then nearest within 1 ms, and records unmatched timestamps and boundary/interior counts without inventing a transport cause.

### Consistency verdict table

Because the shared lock was refused, these are **pending bag verdicts**, not results emitted from a full pass. They must not be read as evidence that the bag is consistent or inconsistent. Unknown properties remain unknown even after a pass when raw inputs are absent.

| Property | Verdict | Numbers / evidence |
|---|---|---|
| Field layout | not determinable from this bag (not read: shared-lock denial) | Source/static + synthetic: x/y/z/intensity/time float32 at offsets 0/4/8/12/16; ring uint16 at 20; point_step 24, dense true, little endian. Recorded layout unmeasured. |
| Time origin | not determinable from this bag (raw points absent; shared-lock denial) | Synthetic: zero at first kept point, not necessarily scan acquisition start or header stamp. Real zero-start fraction unmeasured. |
| Time unit | not determinable from this bag (raw timing absent; shared-lock denial) | Synthetic absolute float64 seconds become relative float32 seconds. Over 0.1 s: 0 us additional cast error, 0.118255615 us max versus ideal input grid. Real durations unmeasured. |
| Stride | not determinable from this bag (raw points absent) | Source/test stride 4 keeps indices 0,4,8,12,16 from 19 inputs. postprocess.yaml declares 4, but mapping is not independently established. |
| Ring handling | not determinable from this bag (raw points absent; shared-lock denial) | Selected uint16 rings preserved including 65535. Controlled 128-ring interleaving retains exactly 32 rings: 0,4,...,124. Real ring histogram unmeasured. |
| Gyro conversion | not determinable from this bag (shared-lock denial) | Source helper multiplies each axis by pi/180 = 0.017453292519943295; synthetic MCAP has 4/4 exact matched pairs with that ratio on every axis. Real ratios unmeasured. |
| Accel handling | not determinable from this bag (shared-lock denial) | Static main copies vector and covariance; synthetic MCAP ratios 1.0 on all axes. Actual raw physical units and real ratios unmeasured. |
| Header stamp source | not determinable from this bag (shared-lock denial; raw cloud absent) | Static main copies both input headers. Synthetic IMU 4/4 exact pairs, delta 0 ns; synthetic cloud header-minus-receipt -1000 s. Those numbers are fixture facts only. |

## 5. Measured vs guessed

The source and synthetic results above are measured locally. Sidecar metadata was reread; it is explicitly not a bag decode. The actual bag-to-vendor consistency claim remains unestablished. Matching file hashes and plausible metadata do not establish which process produced a recording.

The controlled firing pattern is ten azimuth blocks, each with rings 0 through 127 in order: 1,280 source points become 320 at stride 4. **This does not establish the real driver's firing order.** The adapter retains selected ring values; it does not renumber them.

Over 12,801 ideal samples spanning 0.1 s at epoch 1,787,700,000 seconds, float64 epoch spacing is 0.238418579 microseconds. Relative subtraction lands on a grid exactly representable by float32 over that interval, giving zero extra cast loss. The total max error against the ideal grid is 0.118255615 microseconds, below 1 microsecond. No general precision claim is made for longer scans or other epochs.

Historical outputs without physical reference must retain the label **replay demonstrated; physical accuracy not evaluated**. This task did not itself replay FAST-LIO or evaluate physical accuracy.

## 6. Known limits

### What would change FAST-LIO's input

- **Stride changes coverage:** in the controlled interleaving, 96 of 128 rings disappear even though surviving ring values are copied correctly. Real coverage must be measured from the bag.
- **Time origin:** each input message becomes an output scan; times reset to its first kept point. Header is copied unchanged. If that header represents a different instant, point offsets and header may disagree. Raw headers and point times are needed to measure that discrepancy.
- **Scan end:** returned dt is last-kept minus first-kept, not the last original point; from a ten-point fixture at stride 4 it ends at source index 8, omitting index 9. This dt is only logged in main; emitted last-point timing is also truncated.
- **Nonmonotonic timestamps:** source does not sort. Fixture [1000,1000.2,1000.1,999.9] emits [0,0.2,0.1,0] and returns dt about -0.1 s. Negative relative times are clamped; positive reversals remain. Observed-behavior tests preserve this behavior.
- **Invalid geometry:** zero and NaN XYZ survive while main declares is_dense true. Missing ring defaults to zero. Neither behavior was fixed.
- **Precision:** no greater-than-1-us loss was observed in the tested 0.1 s epoch case. This is a bounded measurement, not a promise for every input.
- **Gyro:** scalar multiplication by pi/180 changes all axes without sign/permutation. Physical correctness depends on the actual raw unit. Main copies angular-velocity covariance unchanged; whether downstream code uses that covariance was not established here.
- **Acceleration:** main passes values through, so a raw g-valued stream remains in g. The copied FAST-LIO source explicitly normalizes acceleration at `/Users/tconf/Developer/spatio-data/spark-src/FAST_LIO_ROS2/src/IMU_Processing.hpp:260`: `acc_avr = acc_avr * G_m_s2 / mean_acc.norm();`. SHA256 of that file: `2b786c50c255a118fe95c9b017ef13af9e27a79b0624a718d4e192c3e5ca138d`. This supports why g input can be acceptable to that source; it is not a runtime calibration or deployed-version proof.

No ROS callback was run. Python fallback and numpy conversion paths agree on the tested finite fixture. Big-endian, padded input rows, arbitrary field datatype declarations, sensor calibration and all malformed vendor inputs are not exhaustively characterized. The checker validates output buffer dimensions and reads declared endian/offset/row strides; it requires MCAP summary statistics. It retains compact IMU and per-scan records, not full clouds. Nearest pairing is deterministic greedy matching after all exact matches; it is not a global minimum-cost assignment under ambiguous duplicate/nearly-identical stamps.

## 7. Needs Trevor's approval

- A separately approved source export is needed to obtain the **actual nonempty adapter version used for postprocessing** and establish provenance; no Spark command is authorized or attempted here.
- No new task-scope approval is needed for the local tests or already-requested bag read/commits. The execution environment must permit the shared lock, external outputs (or the documented output fallback), and worktree Git metadata before the unfinished authorized steps can run. Approval policy here is never; this agent cannot escalate permissions.
- If exact point-input timing/stride provenance is required, an approved copy of raw points or the relevant PCAP plus driver/config identity is needed. Current synthetic results cannot replace that evidence.

First commit attempt after passing tests used the requested owned-folder add and a message ending `[skip ci]`. It failed before staging:

```text
fatal: Unable to create '/Users/tconf/Developer/labcommand/.git/worktrees/spatio-x2-jt128-adapter/index.lock': Operation not permitted
```

The environment cannot make either requested local commit. No push, gh, PR, Actions, SSH, container operation, app launch, keystrokes, or process kill occurred.

## 8. Design decisions

- Kept the adapter and upstream tests immutable; tests assert observed bugs rather than repairing or weakening the source.
- Used a PEP 420 parent package; no `spatio/__init__.py` or files outside the owned folder were created.
- Used the installed system decoder and numpy instead of adding packages. The reader is tested through real synthetic MCAP/CDR serialization, including intentionally inconsistent conversions and nearest matches.
- Kept source-level checks of ROS layout/accel/header assignments explicitly static. No fake ROS callback coverage is claimed.
- Classified point origin/unit/stride/ring/header provenance conservatively; zero-relative time and plausible cloud sizes cannot prove raw mapping.
- Paired IMU stamps as integer nanoseconds, globally reserving exact matches before nearest matching. Reported rates for both streams, ratios only for nonzero raw components, residuals for all finite components, and signed-permutation ambiguity.
- Followed the specified fallback after actual sandbox refusals. Plan: `_out/codex-spatio-x2-plan.md`; findings: this file. Real-data outputs are absent because the shared lock failed. Synthetic outputs and logs are ignored; top-level `_out/*.md` files are eligible for the future commit.
