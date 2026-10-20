# Shared wall-scoring findings

Implemented on `spatio-m1-wallunify`, starting at `6de077e`. Changes are uncommitted.
The 12-line plan was written before code changes.

`spatio/wall_scoring.py` now owns wall evidence, patch provenance, acceptance
blockers, and the combined trajectory/wall verdict. Walk-day result and
`series adapt` call it; the walk-day series gate imports its provenance check.
There are no recording-specific paths, topics, sensor settings, or fitted thresholds.

The shared rule requires at least two uniquely named, verified preselected
patches for PASS, matching the spec's names and boxes when fixed. Verified keeps
the existing meaning: measured, not doubled, and both patch gates passed.
Auto-suggested patches never enter the verified list. Zero verified patches
means NOT_EVALUABLE, including when all patches failed; failure diagnostics remain
visible. Incomplete required patches also block PASS. With verified evidence,
known wall failures still fail; whole-map p95 and existing patch gates remain.

I kept series adapt's stricter whole-map doubled-wall failure. A solid pillar
can trigger this detector, but a general report does not prove that every such
flag is furniture rather than map doubling. Both paths therefore FAIL that case
and show the detector's limitation. No detector thresholds or geometry changed.

Run from this worktree, supplying the actual session and frozen spec:

```sh
python3 -m spatio.walkday.result SESSION --patch-origin preselected --spec SPEC.json --wall-gate-m 0.05 --wall-gate-state approved
python3 -m spatio.series adapt SESSION/walkcheck.json --metadata SESSION/series-metadata.json --wallpatch SESSION/wallpatch/wallpatch.json --patches SESSION/wallpatch/patches.json --patch-origin preselected --spec SPEC.json --wall-gate-m 0.05 --wall-gate-state approved --out SESSION/adapted.json
python3 -m pytest -q spatio .github/scripts
```

Series metadata requirements are documented in `spatio/series/README.md`.
Physical-reference metadata must name the same spec hash. Missing optional wall
inputs produce NOT_EVALUABLE, not inferred evidence. `adapt` exit 0 still means
conversion succeeded; use `explain`/`compare` for verdict exit codes.

Synthetic results (not physical accuracy measurements):

| Evidence | Walk-day | Series adapt |
| --- | --- | --- |
| 5 cm and different 7 cm loops, sharp verified walls | PASS | PASS |
| 15 cm return error | FAIL | FAIL |
| 0.50 m pillar; whole-map separation above 45 cm; sharp outer patches | FAIL | FAIL |
| Whole-map p95 6 cm with 5 cm gate | FAIL | FAIL |
| Whole-map p95 exactly 5 cm with 5 cm gate | PASS | PASS |
| No report, empty report, zero verified patches, or auto suggestions | NOT_EVALUABLE | NOT_EVALUABLE |
| Missing named patch, mismatched box, duplicate, or undetermined patch | NOT_EVALUABLE | NOT_EVALUABLE |
| Trajectory time gap | NOT_EVALUABLE | NOT_EVALUABLE |

`test_wall_parity.py` runs 24 session cases through both public CLIs and compares
status, reasons, wall fields, and patch provenance. Eight further cases check
malformed patch reports and invalid gate inputs. Sessions are generated locally;
the existing adapter tests also vary trajectory geometry, clock, rate, and speed.

Final validation: `python3 -m pytest -q spatio .github/scripts` exited 0:
**862 passed, 1 skipped, 0 failed, 16 warnings in 100.57 seconds**.
The existing skip is `test_adapter_on_real_jt128_pcap_points`: the
`jt128_one_frame.pcap` input was not copied to this Mac. Warnings are dependency
deprecations and an existing invalid regex escape in `spatio/bagtiming/reader.py`.
`git diff --check` passed. The earlier focused run passed 89 tests.
No tests were deleted, weakened, or newly skipped.

Existing assertions changed:

- `ResultTest.test_solid_pillar_is_a_warning_not_a_failure` was renamed to
  `test_solid_pillar_keeps_the_stricter_shared_failure`: exit 0 → 1, suspected
  false → true, plus an explicit FAIL assertion. The warning assertions remain.
- `ResultTest.test_auto_suggested_patches_never_count_through_series_sh`:
  run exit 0 → 2, underlying series PASS → FAIL, and report text now checks
  NOT_EVALUABLE instead of the obsolete “spatio.series alone said PASS”.
- `ResultTest.test_preselected_patch_not_verified_blocks_series`:
  run exit 0 → 2 and underlying series PASS → FAIL. Missing evidence now blocks
  the per-run result, not just the later walk-day series gate.
- `PlacementPathTest.test_auto_suggested_patches_block_series_pass`:
  run exit 0 → 2 and underlying series PASS → FAIL for the same reason.
- Other expected verdicts stay unchanged. Passing fixtures now explicitly supply
  a test spec and preselected patches. The 8 cm doubled-wall fixture gains a clean
  second patch so it still tests a measured failure with verified evidence.
  Adapter fixtures use rooms with measured patches rather than a lone plane;
  their expected PASS/FAIL/NOT_EVALUABLE assertions remain intact.

Limits: this unifies conversion rules; it does not repair the whole-map detector,
validate a physical walk, or establish milestone acceptance. Old inputs without
patch verification now need explicit evidence before they can PASS. Python only:
no Swift code changed, so no Swift build or app launch was needed. No Spark,
containers, historical recordings, GitHub, or other workstreams were touched.
No additional approval is needed for these edits. Waiving the whole-map flag
would be a separate rule change requiring evidence and Trevor's decision.

## Fix round 1: bind wall reports to the run map

Addressed the single P2 at the end of `sp-rv-wallunify.txt`. The shared
`score_walls` now checks every supplied wall report's `map_sha256` against
the actual map bytes before using its wall measurements. Series adapt reads
`map.pcd` beside its input walkcheck report; walk-day reads `SESSION/map.pcd`.
The commands above still apply; keep the run's map beside `walkcheck.json`.
Map hashing reads 1 MiB chunks rather than loading the full map into memory.

A mismatch, missing/malformed hash, or unreadable map rejects the input:
walk-day exits 3 and retires an earlier `walk_result.json`; series adapt exits 1
without writing a new result. An absent optional wall report still follows the
existing NOT_EVALUABLE rule. No wall thresholds or existing verdict assertions
were loosened.

Eight new synthetic parity cases cover another session's report, changed map
bytes, missing map, a directory in place of the map, and missing, null,
non-hexadecimal, or non-string hashes. The cross-session case uses two different
generated maps with exactly the same frozen spec, each independently passing
with its own evidence. Adapt receives the other session's report path directly;
walk-day receives a copy at its usual session path. Both must reject with the
same diagnostic. Tests also check retirement of walk-day's earlier PASS.

Before the fix, the cross-session regression failed as intended:
**1 failed, 39 deselected, 14 warnings in 8.04 seconds**; walk-day incorrectly
returned PASS. After the fix, the focused rejection suite passed:
**8 passed, 32 deselected, 14 warnings in 10.11 seconds**.

Full requested validation, `python3 -m pytest -q spatio .github/scripts`,
exited 0: **870 passed, 1 skipped, 0 failed, 16 warnings in 104.33 seconds**.
The existing skip remains `test_adapter_on_real_jt128_pcap_points`, whose
`jt128_one_frame.pcap` fixture is unavailable on this Mac. Warnings remain
dependency deprecations and the existing invalid regex escape in
`spatio/bagtiming/reader.py`. `git diff --check` passed. No tests were deleted,
weakened, or newly skipped.

This verifies map-content binding on synthetic local inputs, not physical
accuracy. A matching hash identifies identical map bytes; it does not
authenticate a hand-edited report or establish when the walkcheck report was
generated. No historical recording-specific settings were added. No Swift
changes, build, app launch, Spark access, or container operations were needed.
No additional Trevor approval is needed. Changes remain uncommitted as requested.
