# P1: reject empty and undersized maps

Read both shared Spatio plans. Work only in this worktree and package.

1. Run the existing package tests and reproduce empty/tiny-map false PASS
   with regressions before changing the evaluator.
2. Require at least 25 finite XYZ points by default, independently of the
   wall-quality verdict. This is a PROVISIONAL input-sufficiency floor,
   not a calibrated room-coverage or accuracy threshold. Expose a positive
   integer API/CLI override and record it in the report.
3. Report `NOT-EVALUABLE` with `too_few_map_points`, the actual count,
   and the configured minimum. Keep trajectory diagnostics available.
4. Deliberate spec correction: the existing three-point-map PASS expectation
   contradicts P1 and becomes NOT-EVALUABLE. Give trajectory-only tests
   adequate synthetic maps without weakening their existing assertions.
   Cover empty/tiny maps, finite-point counting, the boundary, overrides,
   invalid thresholds, report output, and CLI exit status.
5. Run `python3 -m pytest -q spatio/walkcheck/tests` from the repo root,
   inspect the diff, and write `FINDINGS-P1.md` with exact results and limits.
   Attempt a local commit; if Git metadata is read-only, leave it uncommitted.

No old recordings, Spark, containers, app launches, other packages, pushes,
PRs, or GitHub Actions. No Swift changes; Swift tests/build are not relevant.
Independent review belongs to another team member.
