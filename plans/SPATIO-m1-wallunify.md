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
