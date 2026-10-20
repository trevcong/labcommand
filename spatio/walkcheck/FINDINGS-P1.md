# P1 findings: empty and undersized maps

Built on `spatio-fix-wcempty`, starting at `bdf6fbf` (the local `origin/main`
ref). Read both shared plans and wrote `PLAN-P1.md` before implementation.
Only `spatio/walkcheck` changed.

The evaluator now returns NOT-EVALUABLE with `too_few_map_points` when the
map has fewer than 25 finite XYZ points. NaN/infinity do not count. The report
retains trajectory diagnostics and records the actual point count and minimum.
The CLI returns 2 and still writes JSON, Markdown, and PNG reports.

Run from the repository root:

```sh
python3 -m spatio.walkcheck /path/to/session
python3 -m spatio.walkcheck /path/to/session --min-map-points 50
python3 -m pytest -q spatio/walkcheck/tests
```

Python callers can use `evaluate_session(path, min_map_points=50)`.
The override must be a positive integer; even a minimum of 1 rejects empty
maps. The default 25 is a PROVISIONAL coarse input floor, matching the existing
wall fitter's minimum support count numerically but kept as a separate setting.
It is not fitted to old recordings and is not a room-coverage requirement.
The wall-quality verdict remains independent.

Exact local test results:

- Baseline package: **264 passed, 14 warnings**, 13.84 seconds.
- Before the fix, new insufficient-map regressions:
  **16 failed, 25 deselected, 14 warnings**, 7.66 seconds.
  Command: `python3 -m pytest -q spatio/walkcheck/tests/test_map_minimum.py -k insufficient_map --tb=short`.
- After the fix, full package: **305 passed, 14 warnings**, 14.40 seconds.
  No failures or skips. All 41 new cases passed. Warnings are existing
  matplotlib/pyparsing deprecations. `git diff --check` passed.

Observed synthetic results: ASCII and binary maps with 0, 1, 3, or 24 points
are NOT-EVALUABLE with both 5 cm and 15 cm trajectory return errors. Before the
fix these returned PASS and FAIL respectively. Maps with 25 or 32 finite
points preserve the passing trajectory result even without wall patches.
Of 30 stored points, only 0 or 24 finite points is NOT-EVALUABLE; 25 is
sufficient for the default input gate. A 32-point map becomes NOT-EVALUABLE
with an explicit minimum of 33. CLI/report output and invalid overrides are
also covered. The regressions use an independent synthetic 1.7 m loop at
7 Hz with epoch 123.4 seconds and 11/13-second holds.

Deliberate spec correction: the previous three-point-map test expected PASS.
It now requires NOT-EVALUABLE and the specific reason. Trajectory/hold tests
using one- or three-point placeholder maps now use 32-point synthetic maps;
their trajectory assertions and thresholds are unchanged. No tests were
removed, skipped, or weakened. The older `FINDINGS-robustness.md` one-point-map
example now produces NOT-EVALUABLE; its earlier FAIL description predates P1.

Limits: point count alone does not establish distinct points, useful geometry,
sharp walls, adequate room coverage, or physical accuracy. No historical
recordings were read or tuned to. No FAST-LIO, physical walk, Spark, container,
app, or Swift validation was performed; this is Python-only scorer evidence.
No additional Trevor approval is needed for this local fix. Physical
acceptance and freezing operational thresholds remain separate work.
Independent review remains another team member's responsibility.

Commit status: Codex could not write Git metadata in its sandbox; the orchestrator committed this work (e62a26b). No push by Codex.
