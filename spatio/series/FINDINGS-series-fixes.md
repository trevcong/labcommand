# Series report fixes

Built only in `spatio/series` on branch `spatio-fix-series`, starting at
`bdf6fbf` (also the local `origin/main` ref). No remote refresh was performed.
The plan was written before implementation in `PLAN-series-fixes.md`.

## Changes and evidence

- The positive-duration/null-endpoint-window case was already fixed on this
  main snapshot. A remaining zero-duration case was reproduced: a null window
  disabled the paired-span check, and the positive-length opposite window
  escaped validation. The same per-window check now applies to zero duration.
  Four regressions cover either null window with FAIL and NOT_EVALUABLE.
- Reports return distinct exits: PASS 0, rejected input 1, unreadable input or
  output/usage 2, FAIL 3, INCOMPLETE 4, NOT_EVALUABLE 5, REPLAY_ONLY 6.
  Non-PASS reports are still written. `adapt` uses 0 for conversion success.
- `adapt` consumes real `spatio.walkcheck.v1` output plus explicit session
  metadata and writes the existing validated `spatio.walk_result/1` format.
  Tests call the actual walkcheck evaluator on varied synthetic recordings.
  No walkcheck files were edited. Missing wall evidence becomes NOT_EVALUABLE;
  a suspected doubled wall prevents a series PASS even when walkcheck's
  trajectory verdict is PASS. Original producer verdict and source are saved.
- Comparisons now sort by explicit timezone-aware `recorded_at`, normalized
  to UTC. Missing or tied times are rejected. The report table, counted triple
  and default series ID agree. A regression proves that moving a failed
  attempt to the end of CLI arguments cannot create a false passing triple.

Deliberate spec corrections: non-PASS report exits are no longer 0, and CLI
argument order no longer establishes chronology. Existing fixtures gained
explicit synthetic times; the Markdown golden gained a time column and the
corrected rule. The old FAIL-exit assertion changed from 0 to 3. No tests were
removed, skipped or weakened. Existing measurement gates were retained.

## How to run

From the repository root:

```sh
python3 -m spatio.series adapt session/walkcheck.json --metadata session/series-metadata.json --out session/walk_result.json
python3 -m spatio.series compare session-a/walk_result.json session-b/walk_result.json session-c/walk_result.json --out series-report
```

See `README.md` for the metadata fields, unit mappings and complete exit-code
contract. Explicit metadata is necessary because walkcheck omits revision,
configuration hash and physical reference details. Its `created_at` is report
generation time, not recording time.

## Exact validation results

Commands ran with `python3` from the repository root:

- Baseline: `python3 -m pytest -q spatio/series/tests` — **35 passed**.
- Before fixes, the first regression set — **17 failed, 2 passed**, reproducing
  the duration, exit and chronology problems.
- Final: `python3 -m pytest -q spatio/series/tests` — **65 passed, 14 warnings**,
  9.47 seconds; no failures or skips.
- Compatibility: `python3 -m pytest -q spatio/walkcheck/tests` —
  **264 passed, 14 warnings**, 15.25 seconds; no failures or skips.
- `git diff --check` passed.

Warnings are installed matplotlib/pyparsing deprecations. No Swift code changed,
so no Swift suite or app build was needed.

Actual synthetic results: return displacements of 3, 5 and 7 cm produced the
expected chronological three-run PASS with synthetic physical-reference
metadata, even with reversed CLI arguments. A 15 cm return remained FAIL.
Missing map, trajectory gaps, insufficient wall evidence and doubled walls
were exercised. These are software results, not measured cart accuracy.

## Limits and approvals

No historical recordings were used for tuning. No physical scans, Spark work,
containers, app launches, external apps, GitHub operations or pushes occurred.
The adapter cannot discover omitted attempts or prove that supplied physical
measurements are true. Its clock-discontinuity count is not a new pose-reset
detector. Old reports need actual recording timestamps before comparison.
Historical runs remain labelled "replay demonstrated; physical accuracy not
evaluated". Placement and wall thresholds remain provisional.

No new Trevor approval is needed for these local fixes. Physical acceptance
and final threshold approval remain future work. This is implementation and
test evidence, not an independent review of the code.

## Commit status

Codex could not write Git metadata in its sandbox, so the orchestrator committed this work as 7ed27fa. Codex made no commit or push.

Note: `explain` exits with the run's own status, so a replay-only PASS exits 0; automation must use `compare` for series acceptance (see README).
