# Spatio current state (ledger)

**Verified against `origin/main` 4de8ec7 (merge of #84), 2026-10-20.** Every row below was checked against the code at that commit. A row about a run names the commit the run used, which may be older.

The same file is kept in two places: `docs/CURRENT-STATE.md` in the repo and `~/Developer/lc-tasks/plans/SPATIO-current-state.md`. Paths under `~/Developer/spatio-data/` and `~/Developer/lc-tasks/plans/` are **local, not in the repo**. They are not version-controlled and may move.

## Bottom line

1. **The milestone-1 physical marked-loop walk has not been done, so it has not passed.** No Spatio claim on the walk-day path has evidence from a physical measurement made for that purpose.
2. **The walk-day software is merged** (#84): one command takes a recording to a per-run verdict and a three-run series. It has software tests and synthetic runs only.
3. **No recorded run would count today.** The real Aug 8 and Aug 26 recordings are test samples only. The synthetic dry run predates the current wall rule. Under that rule, runs scored on auto-suggested wall patches are NOT-EVALUABLE.
4. **The JT128 lap "drift" (1.82 m, −16°) was mostly cart placement, not estimator drift.** That comes from a real recording, with no tape ground truth. The estimator error is a few cm where measured, and it is not bounded.
5. **Spatio memory v1 is not on main.** The memory on main is the stationary-rig room memory (`spark/`), which has its own candidate-only acceptance harness. Cross-walk memory needs relocalization, and none exists.
6. **No automated gate runs the ~900 Spatio Python tests.** CI runs pytest only inside `spark/`.

## How to read this

Columns:
- **Claim**: one statement, as of 4de8ec7.
- **Artifact**: the file, test, or report that supports it. Line numbers are at 4de8ec7. A test is named as `file::test`.
- **Rev**: the commit that introduced the behaviour, or the commit a run used. "4de8ec7" means the state was read at that commit.
- **Evidence**: one of the four categories below.
- **Superseded by / status**: "current" if nothing replaces it. Otherwise, what replaces it.

Evidence categories:
- **software test**: a test in the repo asserts the behaviour.
- **synthetic**: an end-to-end run on generated data (synthetic recordings or scenes).
- **physical measurement**: computed from a real sensor recording. **None of these have tape-measured ground truth** unless the row says so.
- **none yet**: no evidence. The row describes a design, an assumption, a missing piece, or a step that has not happened.

For a "not done" or "does not exist" row, the category describes the evidence for the missing capability. That is usually "none yet".

---

## 1. Physical validation

| # | Claim | Artifact | Rev | Evidence | Superseded by / status |
|---|---|---|---|---|---|
| P1 | The milestone-1 marked-loop walk (three consecutive hand-pushed loops, each ending < 10 cm from the start, with sharp walls) has **not been performed**. No run has passed and no series has passed. | `lc-tasks/plans/SPATIO-board.md` 05:41 (local); `lc-tasks/plans/SPATIO-walkday-trevor-todo.md` (local) | 4de8ec7 | none yet | current |
| P2 | There is no frozen walk spec. The repo holds only a draft with 11 unfilled `TODO(Trevor)` markers (`grep -c`). The m1 note says 12 fields. | `spatio/walkday/spec/SPEC-mid360-walk.DRAFT.json` (the only `SPEC-*` file) | 4de8ec7 | none yet | current |
| P3 | Four walk-checker settings and the hold detection settings are explicitly **NOT APPROVED**. Endpoint spread 0.05 m and endpoint speed 0.10 m/s were picked after looking at the Aug 8 replays. | `spatio/walkday/walkday.conf:11-15`, `:17-27` | a095804 | none yet | current (needs Trevor) |
| P4 | Real Mid-360 parked jitter has never been measured. The hold-speed setting comes from synthetic data only, and the preflight walk is meant to measure it. | `spatio/walkday/walkday.conf:18-25` | a095804 | none yet | current |
| P5 | The Aug 8 Mid-360 and Aug 26 JT128 recordings are **test samples only**. Nothing may be tuned or hardcoded to them, and neither is a milestone-1 attempt. | `SPATIO-board.md` "DIRECTION FROM TREVOR (22:31)" (local) | — | none yet (a rule, not a result) | current |
| P6 | Aug 8 Mid-360 recording through the walk-day pipeline: **NOT-EVALUABLE**. The cart moved after its last hold (`motion_after_end_hold`), so the return error is withheld. FAST-LIO's own start-to-end pose is 223.8 cm and +99.3° (diagnostic only). Whole-map wall p95 is 5.27 cm, over the 5 cm gate. | `lc-tasks/plans/SPATIO-m1-integration.md` "Results" (local); session `~/Developer/spatio-data/walkday/m1-aug08-mid360-final2/` (local) | a095804 (m1 branch, before 3f6d1d6, b1b7922, b6bf877, bf0ecd7) | physical measurement (real recording, test sample, no tape truth) | Not re-run on 4de8ec7. It would still not PASS: it was scored with auto-suggested patches, which are now NOT-EVALUABLE (W5) |
| P7 | Aug 26 JT128 recording: stops at input gate **B5**. x1 found no defensible absolute point time in any of the 1,818 `/slam/points` scans (its `UNKNOWN_SCAN_BOUNDS` finding), so no scan has point-time bounds that can be checked against the IMU. The JT128 is outside the Mid-360 acceptance path. | `spatio/walkday/gate.py:13-14`, `:65-68` (B5); `spatio/bagtiming/core.py:310`; `SPATIO-m1-integration.md` (local); `~/Developer/spatio-data/walkday/m1-jt128-explicit-config/` (local) | a095804 | physical measurement (real recording) | current |
| P8 | Synthetic dry run: pass-a, pass-b and pass-c PASS at a 5 cm true return; fail FAIL at 15 cm; series REPLAY_ONLY. | `spatio/walkday/synth/dryrun.sh`; `SPATIO-m1-integration.md` "Results" (local); `~/Developer/spatio-data/walkday-dryrun/sessions/dry-20261020T014026-*` (local) | a095804 | synthetic | **Superseded** by b1b7922/9c3b3dd (zero verified preselected patches = NOT_EVALUABLE). `dryrun.sh` passes no `--patches` (`process.sh:231-235` then auto-suggests), so on 4de8ec7 each pass run should be NOT-EVALUABLE. Not re-run (it needs Docker). The comment `dryrun.sh:12` "Expected: pass-* PASS" is stale |
| P9 | With measured placements on preselected patches, three scored runs flow through to a series PASS. One placement over tolerance, or auto-suggested patches, blocks the series PASS. | `spatio/walkday/tests/test_placement_path.py::test_measured_placements_on_preselected_patches_flow_through_to_series_pass`, `::test_auto_suggested_patches_block_series_pass` | aee4b80, b1b7922 | software test (no Docker; real CLIs on fabricated sessions) | current |
| P10 | Room-memory (stationary rig) physical acceptance is a **candidate score** only. The node emits `CANDIDATE` and never `CONFIRMED`, so `confirmed_product_bar` stays `not_evaluated`. The repo records no acceptance result. | `README.md:66`, `:68`; `docs/acceptance-runbook.md:9-11`, `:387` | 4de8ec7 | none yet (harness only) | current |

## 2. Localization

| # | Claim | Artifact | Rev | Evidence | Superseded by / status |
|---|---|---|---|---|---|
| L1 | The estimator is FAST-LIO2 (ROS 2 port) with the stock Mid-360 config, frozen as `mid360_walk.yaml`. Revision 2fffc57 is pinned **only** in the draft spec. The replay image is built from whatever HEAD the local FAST-LIO checkout has, and `process.sh` refuses a mismatch only when a spec is given. | `spatio/walkday/configs/mid360_walk.yaml`; `spatio/walkday/spec/SPEC-mid360-walk.DRAFT.json:5`; `spatio/replay/build_image.sh:22-26`; `spatio/replay/Dockerfile:30`; `spatio/walkday/process.sh:198-204` | 7a74a1c | synthetic (dry runs used it) | current |
| L2 | There is **no loop closure and no relocalization** anywhere in the repo. Each session starts at its own origin. | `grep -rniE 'loop.?clos\|relocali\|scan.?context' spatio`: 0 hits | 4de8ec7 | none yet | current. Roadmap F2a/F2b/F2c (local) |
| L3 | Replay takes topics from the FAST-LIO config and the sensor kind from the bag's message types. It has no hardcoded Livox topic names. A synthetic recording with other topics (`/synth/lidar_points`, `/synth/imu_data`, 250 Hz IMU) passes through every stage. | `spatio/replay/recording.py`; `spatio/replay/replay.sh`; `spatio/walkday/synth/dryrun.sh:6-7` | 7a74a1c | software test + synthetic | current |
| L4 | The recorder sometimes stores two neighbouring `/Odometry` messages swapped. `undo_adjacent_swaps` swaps back only a neighbouring pair whose swap restores increasing time, and records that it did. | `spatio/replay/tum.py` `undo_adjacent_swaps`; tests in `spatio/replay/tests/test_tum.py` | 5efc022 | software test + synthetic (seen in synthetic replays) | current |
| L5 | **JT128 lap "drift" is mostly placement.** Raw-scan registration of the end hold against the start hold gives 1.812 m and −15.2° yaw. FAST-LIO gives 1.822 m and −15.5°. They differ by 3.4 cm and 0.64°. The gyro (−14.4° yaw) and the accelerometer (7.37° tilt change) agree on rotation. The translation rests on lidar ICP alone. | `lc-tasks/plans/SPATIO-drift-investigation.md` M1–M3, M16, M17 (local); scripts and outputs in `~/Developer/spatio-data/derived/drift-investigation/` (local, **not in repo**) | n/a (offline scripts) | physical measurement (real recording, no tape truth) | current. Supersedes the board 22:40 framing ("end 1.82 m from start … lap did not return") |
| L6 | The cart came to rest about 5 cm from where it entered the lap (t ≈ 89 s). It was repositioned for about 50 s after recording started. Whether Trevor meant the lap entry as "start" is a **guess**. | `SPATIO-drift-investigation.md` M4, "Needs Trevor" Q1 (local) | n/a | physical measurement (position); none yet (intent) | Open: Trevor Q1. Do not read the board 23:27 "lap closed to ~5 cm" as a closure result |
| L7 | FAST-LIO estimator error is **not bounded**. FAST-LIO and ICP disagree by 3.4 cm / 0.64° (JT128) and 4.1 cm / 0.72° (Aug 27 home test 2). Along-walk self-consistency reaches 8.1 cm / 1.7° on home test 2's gap-free keyframes (63% scan loss). These are disagreements, not error bounds, and they come from **JT128 data, not Mid-360**. | `SPATIO-drift-investigation.md` M5, M14, M14b, "Ranked causes" §2 (local) | n/a | physical measurement (real recordings, no tape truth) | current |
| L8 | Raw-scan start-to-end registration is ported into the pipeline as **information only**. It never corrects or changes the score. On Aug 8 its fit was 3%, so its number was meaningless there. | `spatio/walkday/startend.py:1` ("INFORMATION ONLY: never scored"); `spatio/walkday/result.py:214` (status fixed) vs `:292` (`raw_start_end` attached after) | 078c665 | software test (`spatio/walkday/tests/test_startend.py` covers the registration; the information-only status is by code reading) | current |
| L9 | Start and end are detected as sustained holds (#60). walkcheck's own default hold speed (0.02 m/s per step) found **no hold** on a parked synthetic FAST-LIO cart, so walk day overrides it with 0.10 m/s (NOT APPROVED). The minimum hold is 8 s and the protocol hold is 15 s. | `spatio/walkcheck/holds.py:12-13`; `spatio/walkday/walkday.conf:17-27`; `spatio/walkday/spec_draft.py:24-27` (`HOLD_S = 15`) | 741077d, a095804, 569352f | synthetic | current (needs Trevor) |
| L10 | The input gate blocks on B1 (a config topic missing or empty), B2 (an x1 error such as a clock reset), B3 (header time going backwards, resetting, or zero), B4 (only part of the bag inspected) and B5 (no checkable point-time bounds). Scoring needs a clean gate. | `spatio/walkday/gate.py:8-14`; `spatio/walkday/result.py:106` `input_gate_blockers`, `:215-219`; `spatio/walkday/tests/test_gate.py` | 7a74a1c, b6bf877 | software test | current |
| L11 | Time sync between lidar, IMU and cameras is **unproven**. The historical recordings had PTP off. The JT128 Aug 26 bag starts 3 h 19 min later than its folder name says, and nobody knows which clock is wrong. | `lc-tasks/plans/SPATIO-northstar.md` §2 layer 0 (local); `SPATIO-board.md` 02:48 and 05:41 (local) | — | none yet | Open: Trevor (clock offset) |
| L12 | JT128 with default DDS lost about 70% of scans on the original run (home test 2: about 63%). This did not move the Aug 26 endpoint. | `SPATIO-drift-investigation.md` "Risks" and M15 (local) | n/a | physical measurement (real recordings) | current (live-run risk) |
| L13 | Processing time: x1 took 1,256 s on the 383 s Aug 8 bag at full per-point decoding (`BAGTIMING_SAMPLE_EVERY=1`). "25–35 min per 5-min walk" is a projection, not a benchmark. | `SPATIO-m1-integration.md` "Known limits" (local); `spatio/walkday/walkday.conf:36` | a095804 | physical measurement (one run) | current |

## 3. Wall scoring

| # | Claim | Artifact | Rev | Evidence | Superseded by / status |
|---|---|---|---|---|---|
| W1 | One shared wall rule serves both walk-day `result.py` and `spatio.series` adapt. | `spatio/wall_scoring.py:1-6`, `score_walls` `:317`, `wall_verdict` `:343`; `spatio/walkday/tests/test_wall_parity.py` | b1b7922 | software test | current |
| W2 | The wall gate is 5 cm (p95 point-to-plane, and the separation of a doubled wall). It is "provisional (approved for now, revisit after real data)". | `spatio/walkday/walkday.conf:30-31`; `spatio/walkday/result.py:522` (`--wall-gate-m` default 0.05) | 7a74a1c | software test | current |
| W3 | The 5 cm gate is defined separately in several modules, and `WALL_GATE_M` from the settings file reaches **only** `spatio.walkday.result`. walkcheck and wallpatch get no gate argument. | `spatio/walkcheck/defaults.py:42`; `spatio/walkday/process.sh:218-221` (walkcheck, no gate), `:241-243` (wallpatch measure, no gate), `:253` (result gets `--wall-gate-m`) | 4de8ec7 | none yet (no test ties them together) | Open: solidity job S4 |
| W4 | **Guard band (provisional, delegated by Trevor 05:13):** p95 and both 80% bands ≤ 4.0 cm PASS; above 4.0 and up to 5.0 cm NOT-EVALUABLE; p95 > 5.0 cm FAIL. A band never fails on its own. | `spatio/wall_scoring.py:17-22` (`PASS_MAX_M = 0.040`), `effective_gate_check` `:54`; `spatio/walkday/tests/test_wall_parity.py::test_guard_band_boundaries`; `spatio/wallpatch/tests/test_wallpatch.py` `test_guard_band_boundaries` | bf0ecd7 | software test | current (provisional; revisit after real walks) |
| W5 | **Auto-suggested patches, or zero verified preselected patches, make the walls NOT-EVALUABLE.** They never count toward a series, even when the trajectory passes. | `spatio/wall_scoring.py:333-336`, `acceptance_blockers` `:249`; `spatio/walkday/tests/test_result.py::test_auto_suggested_patches_never_count_through_series_sh` | b1b7922 | software test | current. Supersedes the m1 rule that let auto patches decide PASS (P8) |
| W6 | A run counts only with origin `preselected`, a frozen spec naming at least 2 patches, and every named patch verified (exactly one MEASURED patch of that name, not doubled, both gates passed, and the spec's box when the spec fixes one). | `spatio/wall_scoring.py:182-268`; `spatio/walkday/tests/test_result.py::test_preselected_patch_not_verified_blocks_series` | b1b7922, b6bf877 | software test | current |
| W7 | "Preselected" is asserted, not recorded. `process.sh` sets origin `preselected` simply because `--patches` was given, and the `cp` of that file is unchecked. | `spatio/walkday/process.sh:227-230` | 7a74a1c, aee4b80 | none yet | Open: solidity job S2 |
| W8 | The whole-map doubled-wall flag is a **warning only**. It cannot tell a ghost wall from two faces of a pillar or crate. Doubled walls are decided on the patches. | `spatio/wall_scoring.py:1-6`, `:122-127`; `spatio/walkday/tests/test_wall_parity.py::test_whole_map_doubled_flag_is_only_a_warning`; `spatio/walkday/tests/test_result.py::test_solid_pillar_is_a_warning_not_a_failure` | bf0ecd7 | software test | current. Supersedes board 04:53 ("whole-map doubled-wall flag now FAILs runs") |
| W9 | Wall patches with an approximate hint or ambiguous geometry are **never a pass**; they are NOT-EVALUABLE. That is how the g8 doubled-wall P1 was closed: the case is routed to not-evaluable, not resolved geometrically. | `spatio/wallpatch/FINDINGS-g8safe.md`; `spatio/wallpatch/tests/test_wallpatch.py` (`test_plane_perpendicular_to_hint_does_not_pass_doubled_wall`, `test_partial_overlap_pooled_tilt_does_not_pass_doubled_wall`, `test_thick_band_blocks_only_a_would_be_pass`) | fb2a929, e0f1004 | software test | current. Supersedes roadmap §1 and §11 "g8 has an open P1 … parked" |
| W10 | A wall-patch report must hash-match the run's own `map.pcd`, or scoring rejects it as bad input (exit 3). | `spatio/wall_scoring.py:297-314` `_check_map` | c7a7100 | software test (`test_wall_parity.py`) | current |
| W11 | The patch tool's verdict is re-checked against the stricter of the requested gate and the tool's own gate. | `spatio/wall_scoring.py:54-100` | 13664e6 | software test (`test_wall_parity.py::test_strict_gate_band_wider_than_gate_is_not_evaluable`) | current |
| W12 | **Wall scoring has no physical validation.** The only real-recording wall numbers are from the Aug 8 test sample (whole-map p95 5.27 cm; patch 1 unmeasurable, patch 2 undetermined), which was scored before the current rule. | P6 artifacts | a095804 | none yet | current |

## 4. Walk-day pipeline and series

| # | Claim | Artifact | Rev | Evidence | Superseded by / status |
|---|---|---|---|---|---|
| S1 | `process.sh` runs 9 stages: copy check, config, x1 timing + input gate, FAST-LIO replay, physical reference (g5), manifest (g2), walk check, wall patches, then raw-scan start→end and the result. Exit codes: 0 PASS, 1 FAIL, 2 NOT-EVALUABLE, 3 usage. | `spatio/walkday/process.sh:1-41` (header; exit codes `:37`); `spatio/walkday/tests/test_process_sh.py` | 7a74a1c | software test + synthetic | current |
| S2 | A rejected re-score moves the earlier `walk_result.json` aside, so `series.sh` never counts an old verdict. | `spatio/walkday/result.py:505-516` `_retire` | 3f6d1d6 | software test (`test_result.py`) | current |
| S3 | `walk_result.json` is written in place (`open(..., "w")`, no temp file and rename) with `json.dump`'s default `allow_nan`. walkcheck's `plain()` writes non-finite floats as `None`. | `spatio/walkday/result.py:495-501`; `spatio/walkcheck/report.py:26-30` | 7a74a1c | none yet | Open: solidity job S2 |
| S4 | A changed setting gives a new settings identity, which starts a new series. The frozen spec's sha256 is part of series identity. | `spatio/walkday/result.py:140-157` `_settings_used`; `spatio/walkday/seriesgate.py` | 569352f, b6bf877 | software test | current |
| S5 | Series precedence, strongest first: input error, NOT_EVALUABLE, REPLAY_ONLY, PASS, INCOMPLETE, FAIL. If no run has physical accuracy evaluated, the verdict is REPLAY_ONLY. A synthetic recording never counts as physical. | `spatio/series/report.py:3`, `:50`, `:56`; `spatio/walkday/result.py:395` | #58, 7ed27fa | software test | current |
| S6 | `series/adapter.py` defaults `gate_state="proposed"`, while `walkday.conf` sets `WALL_GATE_STATE=approved`. The walk-day flow is unaffected: `series.sh` uses `spatio.series compare`, not `adapt`. | `spatio/series/adapter.py:25`; `spatio/walkday/walkday.conf:31`; `spatio/walkday/series.sh:57` | 4de8ec7 | none yet | Open: solidity job S4 |

## 5. Memory

| # | Claim | Artifact | Rev | Evidence | Superseded by / status |
|---|---|---|---|---|---|
| M1 | **There is no `spatio/memory/` package on main.** The Spatio memory v1 (SQLite store with `CURRENT` / `LAST_SEEN` / `NOT_SEEN_ON_RECHECK` / `UNKNOWN`, and observations kept apart from interpretations) exists only on the unmerged local branches `spatio-memory` (head 90ae03e) and `spatio-g4-memory-acceptance` (head 45146e1). Its last reviews were REVISE, and it is deprioritized. | `git ls-tree -r origin/main` (no `spatio/memory`); `git ls-tree spatio-memory` (`spatio/memory/store.py`, `DESIGN.md`); `SPATIO-board.md` table rows C2 and g4 (local) | 90ae03e (not on main) | software test (on the branch only) | current. Corrects the north star layer 3 "Exists today" and B3's premise "spatio/memory/store.py … as they exist" |
| M2 | The memory on main is the Spark **room memory**: the production C++ core and node in `spark/room_memory_cpp`, plus the Python reference oracle in `spark/room_memory`. Differential tests compare the two byte for byte. | `README.md:44`; `spark/room_memory_cpp/PLAN.md:3` | 4de8ec7 | software test (pytest, ctest differential) | current |
| M3 | Change detection exists in three engines (Python core, C++ port, Swift `change-detect`): 5 cm voxels, with an optional pairing of a disappearance and an appearance as `moved`. It compares clouds in one sensor's own frame (`README.md:62`), not across moving-cart sessions. | `spark/room_memory/room_memory/core/change_detection.py:25`, `:48`; `Sources/LabCommandCore/ChangeDetection.swift:82-84`, `:92`, `:182` | 4de8ec7 | software test (Swift runner, pytest, `scripts/check-shared-goldens.sh`) | current |
| M4 | The room-memory tracker publishes only `CANDIDATE` and `EXPIRED`, never `CONFIRMED`. | `README.md:66` | 4de8ec7 | software test | current |
| M5 | Room-memory events are lost when the node restarts. Durable event history is open draft PR #32 and is not on this tree. | `README.md:64`; `docs/known-gaps.md:9` (`commit()` path is in #32) | 4de8ec7 | none yet | Open: PR #32 |
| M6 | Each lidar keeps its own frame. The JT128 to Mid-360 extrinsic has not been measured. | `README.md:62` | 4de8ec7 | none yet | current |
| M7 | The C++ node constructs `RoomMemory` outside the constructor's `GenerationError` try, and `main` catches only `GenerationError`. Any other construction exception ends in `std::terminate`. | `spark/room_memory_cpp/node/room_memory_node.cpp:152-159`, `:345` | 4de8ec7 | none yet | Open: solidity job S5 |
| M8 | The rosbags CDR cross-check is skipped in CI (`importorskip`; rosbags is not in `spark/requirements-ci.txt`) and covers only `RigPose`. The protocol doc records a hand run of "52 passed". | `spark/room_memory/tests/test_rosbags.py:15`; `docs/ROOM-MEMORY-PROTOCOL.md:1066`; `.github/scripts/ci_checks.py:139` (`--require-rosbags`, never passed by `ci.yml`) | 9371475 | software test (hand run only) | Open: issue #73, solidity job S3 |
| M9 | **Cross-walk memory does not exist.** Comparing one walk with the last needs relocalization into a saved map (L2) and memory schema v2 (many sessions per map), which needs Trevor's approval. | `SPATIO-northstar.md` "The hinge: relocalization" (local); `SPATIO-feature-roadmap.md` F2, F5 (local) | — | none yet | Design: B3 `docs/MEMORY-V2-DESIGN.md` |

## 6. Clearance check (F1)

| # | Claim | Artifact | Rev | Evidence | Superseded by / status |
|---|---|---|---|---|---|
| C1 | A single-walk clearance check exists. It works from ray-cast free / occupied / unknown evidence with sensor origins, not from holes in `map.pcd`. It includes a one-walk report (F7a) and a numbered recheck list (F8a). | `spatio/clearance/` (`grid.py`, `classify.py`, `report.py`); `spatio/clearance/README.md` | 551974d, 1f85225, 60f7d01, f1bec04 | synthetic + software test | current. Supersedes north star layer 1b "Exists today: Nothing" |
| C2 | Each zone gets one of four statuses: PASS, FAIL, NOT_MEASURED or INDETERMINATE. | `spatio/clearance/report.py:15`; `spatio/clearance/classify.py` | 551974d | software test (60 test functions in `spatio/clearance/tests`) | current |
| C3 | F1 runs **only on synthetic scenes**. It is not wired into `process.sh`. No real scan, mock-up or tape-measured clearance has been run. The rule table is not approved. | `spatio/clearance/README.md:86` ("Synthetic tests cover fidelity and deterministic formatting, not physical accuracy"); no file outside `spatio/clearance` imports it | 4de8ec7 | none yet (physical) | current |

## 7. Cameras, thermal, images

| # | Claim | Artifact | Rev | Evidence | Superseded by / status |
|---|---|---|---|---|---|
| K1 | The repo has no camera or thermal code. The FPV and thermal cameras are unidentified (Trevor's label photos are pending). Whether the thermal camera is radiometric is unknown. | grep of the repo for `thermal`, `/dev/video`, `image policy`: 0 hits; `SPATIO-walkday-trevor-todo.md` §D (local) | 4de8ec7 | none yet | current. Roadmap F3 |
| K2 | The camera image policy is an **unapproved draft**: the header says "DRAFT … needs Trevor's approval", and the decision boxes are unticked. | `lc-tasks/plans/SPATIO-image-policy-DRAFT.md:1` (local) | — | none yet | current. Gates roadmap F4 |

## 8. Tests and gates

| # | Claim | Artifact | Rev | Evidence | Superseded by / status |
|---|---|---|---|---|---|
| T1 | CI runs pytest only with `working-directory: spark`. Nothing under `.github/` mentions `spatio`, and the README "Tests" section never mentions it. So no automated gate runs the Spatio tests. | `.github/workflows/ci.yml:59-64`; `README.md:74-118` | 4de8ec7 | none yet | Open: solidity job S1 |
| T2 | `python3 -m pytest -q spatio .github/scripts` (system Python 3.9.6) at 4de8ec7: **1025 passed, 1 skipped** in 116 s. | run by this job | 4de8ec7 | software test | current. Supersedes the board baseline of 907 passed / 1 skipped |
| T3 | The orchestrator's full local gate covers the spark pytest, the C++ build and ctest, and the Swift runner. It does not run the Spatio tests, so they need the separate T2 command. Each job's gate result is in its PR note, not here. | `~/Developer/lc-tasks/gate-full.sh` (local) | — | none yet (for Spatio coverage) | Open: solidity job S1 |
| T4 | The merges since #50 were pushed with `[skip ci]`; the board records 0 Actions runs for them. | `git log origin/main` subjects; `SPATIO-board.md` 22:48–05:41 (local) | cc51e08..4de8ec7 | none yet | current |

## 9. Open decisions (Trevor)

| Decision | State | Source |
|---|---|---|
| Spark recorder proposal and read-only preflight (incl. confirming live `/livox/lidar` is `CustomMsg`) | Not approved; nothing exists on the Spark | `SPATIO-m1-integration.md` (local) |
| Walk spec values, then freeze | 11 `TODO(Trevor)` markers in the draft | P2 |
| Endpoint spread / speed, hold speed 0.10 m/s, minimum hold 8 s | NOT APPROVED | `walkday.conf:11-27` |
| Hold-speed default mismatch: bare walkcheck CLI 0.02 m/s vs conf 0.10 m/s | Document only (solidity S4) | L9 |
| Wall guard band 4/5 cm; whole-map doubled = warning; stricter noisy-wall rule | **Provisional, delegated** by Trevor 05:13; revisit after real walks | `SPATIO-board.md` 05:13 (local); W4, W8 |
| Wall gate 5 cm, and other walkcheck defaults | "provisional (approved for now)" | `walkday.conf:7-10`, `:30` |
| How preselected wall patches get boxed on each run's map | Open | `SPATIO-m1-integration.md` "Needs Trevor" (local) |
| JT128 3 h 19 min clock offset | Open | L11 |
| Drift Q1–Q3: where the start mark was; the walk protocol change; adding raw start→end as a diagnostic column | Q3 implemented as information only (L8); Q1, Q2 open (runbook already says start on the mark) | `SPATIO-drift-investigation.md` "Needs Trevor" (local) |
| Camera label photos; image policy; clearance rule table; memory schema v2 | Open | K1, K2, C3, M9 |

## 10. Reconciliation: claims in the plan docs that main supersedes

| Source claim | Now (at 4de8ec7) | Why |
|---|---|---|
| Board 02:25: "synthetic dry run 3x PASS at 5 cm" | Each pass run would be NOT-EVALUABLE (auto-suggested patches). Not re-run | P8, W5 (b1b7922) |
| Board 04:53: whole-map doubled-wall flag FAILs runs | Warning only | W8 (bf0ecd7) |
| Roadmap §1, §11: g8 doubled-wall P1 open, parked | Ambiguous or approximate-hint patches are NOT-EVALUABLE; merged in #84 | W9 (fb2a929, e0f1004) |
| Roadmap §1: the integration note's Results/Tests/Review read `FILL` | They are filled | `SPATIO-m1-integration.md` (local) |
| North star layer 1b: clearance "Exists today: Nothing" | F1 exists, synthetic only | C1, C3 |
| North star layer 3: memory v1 "exists" (design + Grok implementation) | Not on main; local branches only | M1 |
| Board 22:40: JT128 "lap did not return to its start" (read as drift) | Mostly placement; estimator error a few cm, unbounded | L5, L7 |
| Board 23:27: "lap closed to ~5 cm of its entry point" | Position measured; the intent is a guess (Trevor Q1) | L6 |
| m1 note: the whole-map doubled flag decides PASS; auto patches can PASS | Patches decide; auto patches never PASS | W5, W8 |
| m1 note: g3 epoch tolerance 10 µs | 3 µs (provisional, accepted at board 02:48) | 4e3527c |
