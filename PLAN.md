# Rig-bump guard and acceptance suite (room_memory), v1

A nudged tripod must not flag the whole room. The node detects a rig move (Mid-360 IMU + ICP), re-aligns the sensor pose, or relearns if it cannot, and reports RIG_MOVED. Plan only: no code, no commit, no Spark. Long version: `~/Developer/lc-tasks/plans/rigguard.full.md`.

## Base

- Code is on `room-memory-cpp` @ 5291df5 (`~/Developer/labcommand-spark`); paths are relative to it. Branch `rig-guard` from it.
- C++ node `spark/room_memory_cpp` is primary; Python `spark/room_memory` is the reference.
- Facts: per-sensor references; 1 s windows, 40 s candidate gate; `pose_offset` is translation-only; `STATE_RIG_MOVED=2` is reserved; `static5m{,-b}` lack `/livox/imu`.

## Design

**1. Re-align the pose, not the map.** Each channel gets `T_c` (sensor → reference frame), identity at build. Points are range-clipped, transformed by `T_c`; rays start at `t_c`. Reference and `map_epoch` are untouched, so re-alignment sends no map. At identity the transform is skipped (byte-identical to today).

**2. Triggers** (start a check; never evidence): IMU shock (`||a|/g0−1|` or gyro over threshold), IMU tilt vs baseline `g_base` (gravity at the last decision: taken while building, or the first quiet reading in place when there is none), `pose_offset` > 1.5 cm on 2 windows, periodic every 30 s, node start with saved state, sensor back after ≥ 5 s silence. The IMU is optional (no samples → ICP-only).

**3. KISS-ICP-style point-to-point ICP** (Vizzo et al. 2023), no new dependencies.
- Target: per-sensor registration cloud of real points at 2 cm, built only during BUILDING, only in reference-occupied voxels, ≤ 300 k.
- Source: window at 10 cm, ≤ 50 k points.
- Seed: rotation from gyro/tilt, else a ±10° yaw sweep; translation from a per-axis `pose_shift`.
- Iterate: adaptive threshold τ = clamp(3σ, 5–50 cm), shrinking to 3× inlier RMS per stage; Geman–McClure kernel (κ = τ/3); weighted Horn step.
- **Fit** = `inlier_frac` ≥ 0.9 × static baseline, RMS ≤ 2 cm, and a degeneracy check (λ_min of the point-to-plane information matrix). Fit and ≥ 1.5 cm or 0.25° → `MOVED`, else `IN_PLACE`; unfit → `REJECTED`. Budget ≤ 150 ms per check.

**4. Decisions.**
- While checking (≤ 10 s): CANDIDATE promotion held, learning off.
- Move evidence → **RIG_MOVED**: a fit `MOVED`; IMU tilt ≥ 0.25° once quiet; or gross mismatch (every available channel's `inlier_frac` < 0.5 × baseline on 2 windows, and no quiet IMU watched throughout). Moved channels expire open CANDIDATEs and stop diffing.
- **Rejection is never evidence.** Rejected without evidence → `INCONCLUSIVE`: pose kept, hold released, never relearns.
- Re-align: 3 fit estimates agreeing within 5 mm / 0.1°, inside the absolute envelope 0.30 m / 10° → applied, `REALIGNED`, TRACKING.
- Relearn: evidence but no applicable estimate for 60 s (`auto_relearn`) → archive state to `prev/`, rebuild; first map is a snapshot. A sensor silent ≥ 5 s does not hold the rebuild: it builds its reference when it returns (a move during that build restarts it), and the new frame is committed to disk only then.
- Precedence RIG_MOVED > DEGRADED > TRACKING; silent channels never block a decision.
- Check period + 2 windows ≤ 35 s (validated), so a yaw only the periodic check sees resolves before a false 40 s run promotes.

**5. Wire.** `MemoryStatus.state = RIG_MOVED`, plus `labcommand_msgs/RigPose` on `/room_memory/rig` (transient_local, on change + every 10 s): outcome (IN_PLACE, CHECKING, MOVED, REALIGNED, RELEARNING, INCONCLUSIVE), causes, per-sensor pose and Δ, `imu_ok`. A new topic: a new ChangeEvent kind or status field would break deployed Macs (D3, §1.1). Codecs + goldens in both engines; amend protocol §4.1, D17.

**6. Persistence.** With `rig_guard=true`, save `registration-<sensor>.bin` and a `rig_state` file (poses, gravity, baselines), atomically; a mismatched set throws on load. Legacy state is archived and rebuilt. `rig_guard=false` writes exactly today's files.

**7. Parity.** Guard off: C++/Python byte-identical. Guard on, never re-aligned: `MemoryStatus`, events and MapBricks byte-identical, `RigPose` outcomes exact. **v1 relaxation:** `RigPose` floats within 0.1 mm / 0.001°; after a re-align, events match by kind/sensor/position within 5 cm.

## Tests

- **Unit** (ctest `guard`, `memory`, `messages`; pytest `test_guard.py`, same cases): convergence grid (≤ 0.30 m, 10° yaw, 3° tilt) to 5 mm / 0.05°, with 20 % outliers; corridor/plane → `REJECTED`, never RIG_MOVED; IMU shock/tilt/gap; state machine incl. hold and expiry; relearn; `rig_shifted` golden (+5 cm → 0 regions after re-align); codec goldens.
- **Default off:** existing tests and differential byte-identical to 5291df5.
- **Replay** (`test_rig_guard.py`; skip if `~/Developer/lc-tasks/recordings/static5m{,-b}` absent). `bag_stream.py` gains `--rig-move`, `--imu-synth`, `--drop`, `--strip-imu`; `bag.cpp` gains an IMU record kind, `state_from=<dir>`, and a read-only `probe=<box>` (per window: reference voxels occupied / free / unknown).
  - Static, guard on: 0 events, 0 RIG_MOVED, 0 INCONCLUSIVE.
  - 2–10 cm, 0.5–5° yaw: RIG_MOVED → REALIGNED, 0 events, error ≤ 5 mm / 0.05°; a cube injected after it is found within 0.25 m.
  - 0.5 m + 20°: RELEARNING. Shock only: CHECKING → IN_PLACE. Cube moved on an unmoved rig: never RIG_MOVED.
  - Non-vacuity: with `rig_guard=false` the moves give ≥ 10 CANDIDATEs.

## Physical acceptance suite

**Recorded:** one mcap per scenario in `~/Developer/lc-tasks/recordings/acceptance/<scenario>/` (lidars, IMU, outputs), plus `initial_state/` and `truth.json` (Spark-clock marks, taped box positions). Replayed by `room_memory_bag state_from=initial_state`, judged by `tests/acceptance/evaluate.py` (ctest `acceptance_replay`).

**Live:** back up `~/room-memory/lab`, build a fresh reference with the room empty (D33), deploy the `rig-guard` room-memory image. Per scenario: stop node, archive state, start recorder, start node, wait TRACKING + 60 s, act and mark. Each run is kept as a regression.

**Global** (unless a row says otherwise): no CANDIDATE beyond truth; no RIG_MOVED, RELEARNING or INCONCLUSIVE; `map_epoch` unchanged. Box tolerance 0.5 m physical, 0.25 m synthetic.

| scenario | procedure | pass criteria |
|---|---|---|
| **A. Removed** | ≥ 0.3 m box, present since the build, carried out | DISAPPEARED at the box within 48 s (+2 s live); nothing else |
| **B. Hidden** | 0.6 × 0.9 m board ≥ 1.2 m in front of X for 120 s, then removed | Precondition: X's reference voxels mostly seen occupied before. **0 DISAPPEARED** in X's box; probe: ≥ 80 % of X's voxels *unknown* (unverified), ≤ 10 % *free*; board may APPEAR, EXPIRES within 8 s of removal |
| **C. Person still > 40 s** | stand still 2–3 m away 90 s; repeat seated | nothing while walking; any CANDIDATE is APPEARED at the person, EXPIRES within 8 s of leaving (D33: no CONFIRMED yet); **0 DISAPPEARED** |
| **D. Unplugged** | Mid-360 Ethernet out 60 s, replug; then JT128 | DEGRADED and 0 Hz within 3 s; other sensor tracks; Mid-360: `imu_ok` false, no shock; replug + 5 s: TRACKING, resume check IN_PLACE; 0 CANDIDATEs |
| **E. Nudged** | E1 tap leg; E2 push 3–5 cm; E3 twist 1–2° (laser spot); E4 back; E5 carry 1 m + 30°, leave view within 30 s, stay out until TRACKING + 60 s | E1: CHECKING → IN_PLACE. E2–E4: RIG_MOVED within `imu_quiet_s`+2 s (IMU); ICP-only 3 s (E2, E4), 34 s (E3, periodic); then REALIGNED, Δ within 1 cm / 0.15°, 0 CANDIDATEs, a later box found. E5: RELEARNING at +60 s, new snapshot, `map_epoch` +1, 0 CANDIDATEs |

"Unverified" in B is test-side evidence only; v1 publishes no such status. Each recording is judged with IMU and with `--strip-imu` (ICP-only); IMU qualification needs a real `/livox/imu`.

## Phases

| phase | gate |
|---|---|
| 0. Measure: ICP noise, `pose_offset` yaw response, thresholds, cost; `/livox/imu`? | defaults in `spark/room_memory_cpp/PLAN.md`; a gross-mismatch threshold separates 0.5 m + 20° from a person 0.8 m away and 30 % rearranged, else drop it and E5 is IMU-only |
| 1. Python: `guard.py`, `imu.py`, codec | pytest green |
| 2. C++: `guard.*`, `imu.*`, `memory.*`, `messages.*`, `RigPose.msg`, node, launch, protocol | ctest green; guard-off byte-identical; parity per Design 7 |
| 3. Replay tooling and `test_rig_guard.py` | green on both recordings |
| 4. Five recordings and live runs | every criterion PASSes; a FAIL blocks |

## Safety

Only room-memory changes (plus a subscribe-only recorder); never `lidar-center`, `lidar-dual` or `ptp-master`; old state is archived.

## Open questions

1. `auto_relearn` default true? 2. Does `lidar-dual` publish `/livox/imu`? 3. Envelope 0.30 m / 10° right? 4. May the recorder container run on the Spark? 5. OK that "unverified" (B) is test-side only in v1?

## Later

- Transactional state manifest, crash injection; bit-identical ICP, threshold-boundary differential.
- Re-matched observability test.
- `g_install` vs `g_base`, fallen-tripod block, gradual tilt, IMU gaps.
- Nudge while unplugged (E7), cold nudge across restart (E6), repeated nudges, cumulative envelope, `imu_rpy`.
- Fidelity-checked live probe, PASS/FAIL/VOID, clock-offset fallback.
- Wire "unverified" status; Mac `RigPose` display; JT128 ↔ Mid-360 extrinsic (D30).
