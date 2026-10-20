# Series reports

Run from the repository root:

```sh
python3 -m spatio.series explain session/walk_result.json
python3 -m spatio.series compare session-a/walk_result.json session-b/walk_result.json session-c/walk_result.json --out series-report
```

`compare` writes `series.md` and `series.json`, including failed attempts.
Supply **all attempts**: this tool cannot discover omitted recordings.
Inputs remain `spatio.walk_result/1`; existing validation and provisional
thresholds remain in force.

## Recording order

Every input to `compare` must now include `recorded_at`, an ISO 8601 recording
start timestamp with a timezone, for example `2030-01-01T10:30:00-06:00`.
Times are compared in UTC. The table, earliest consecutive passing triple,
and default series ID all use this order. Missing or tied times are rejected.
Use the actual recording start from acquisition metadata, not a guessed time.

This is a deliberate correction to the old CLI-order rule. Old files can still
be explained without this field, but must gain explicit recording metadata
before comparison. Run names, file times, relative trajectory clocks and report
generation times are not substitutes.

## Using walkcheck output

Walkcheck produces `spatio.walkcheck.v1`, which differs from the series input.
Use the adapter; no edits to walkcheck are needed:

```sh
python3 -m spatio.series adapt session/walkcheck.json --metadata session/series-metadata.json --wallpatch session/wallpatch/wallpatch.json --patches session/wallpatch/patches.json --spec frozen-spec.json --patch-origin preselected --wall-gate-m 0.05 --wall-gate-state approved --out session/walk_result.json
python3 -m spatio.series explain session/walk_result.json
```

`--metadata` is a JSON object with these required keys:

| Key | Source / meaning |
| --- | --- |
| `run_id` | Session manifest; must match the walkcheck manifest ID when present. |
| `recorded_at` | Actual recording start, timezone included. |
| `fastlio_revision` | Nonempty revision from session provenance. |
| `config_sha256` | Actual configuration hash, 64 lowercase hex characters. |
| `physical_accuracy_evaluated` | Boolean, consistent with the walkcheck manifest. True requires that manifest to say true too. |
| `physical_reference` | Null for historical/unmeasured runs; otherwise the measurement object below. |

A session manifest can itself serve as `--metadata` if it contains all these
fields. Walkcheck's embedded manifest omits revision, configuration hash and
physical references. Its top-level `created_at` is the **report generation
time**, so the adapter never uses it to order recordings. No metadata is
inferred from `session_dir`, paths, names or old sample recordings.

For a physically evaluated run, `physical_reference` contains
`start_placement_residual_m`, `end_placement_residual_m`, `heading_check`
(`ok`, `off`, or `not_checked`), and `placement_tolerance_m` (`0.01`).
Residuals must come from physical measurements; null means unmeasured and
cannot count toward a series. Do not set the physical flag merely because
the trajectory closes. When supplying a spec for a physically evaluated run,
the reference must also contain its `spec_sha256`; it must match that spec.

`spatio.wall_scoring` implements the wall rule for both this adapter and
walk-day. Supply the measured patch report and the frozen spec explicitly;
the adapter does not discover them from neighboring files. `--patches` retains
the source patch-file hash. `--patch-origin` defaults to `auto_suggested`.
Auto suggestions, no spec, zero verified patches, or an incomplete required
patch set cannot PASS. At least two uniquely named preselected patches must
match the spec (including boxes where fixed), be measured, and pass their gates.

The adapter maps:

| Walkcheck field | Series field |
| --- | --- |
| `result` | `status` (`NOT-EVALUABLE` becomes `NOT_EVALUABLE`) |
| `return_to_start.error_3d_m` | `return_error.d3_m` |
| `error_horizontal_cm`, `error_vertical_cm` within `return_to_start` | Corresponding metre components, divided by 100 |
| Return windows and heading | Same windows and heading, no alignment |
| `timestamps.duration_s`, `max_dt_s` | Duration and largest gap |
| `timestamps.backward_steps` | Nonmonotonic-clock count in `validity.resets`; zero also means monotonic |
| `motion.enough_motion`, `path_length_m` | Moving flag and path length |
| `drift.return_error_pct_of_path` | Drift percentage |
| `walls.p95_point_to_plane_cm`, `doubled_wall` plus patch evidence | Shared wall residual and combined doubled-wall evidence |
| `--wall-gate-m`, `--wall-gate-state` | Explicit gate (default 0.05 m) and state (default proposed) |

`backward_steps` counts non-increasing timestamps; it is **not** an independent
estimator pose-reset detector. The adapter cannot add evidence absent from
walkcheck. It retains the original verdict in `walkcheck_result` and the
source path in `walkcheck_source`, preserves nulls and reasons, and validates
the converted result using the existing series rules.

Walkcheck's PASS does not gate on doubled walls. Both converters retain the
stricter whole-map doubled-wall failure as well as the patch gates. This can
fail a sharp room containing a solid pillar: the whole-map detector cannot
distinguish its faces from ghost walls. That limitation remains visible in
warnings; it is not silently waived. Zero verified preselected patches forces
NOT_EVALUABLE, with any diagnostic wall failures retained in the reasons.
Other missing required measurements also force
NOT_EVALUABLE. A producer failure is never upgraded to PASS. Contradictory or
malformed inputs are rejected. The adapter does not rescore trajectories.
Historical outputs retain the exact label:
"replay demonstrated; physical accuracy not evaluated".

## Exit codes

Reports are still written for valid non-PASS results.

| Code | Meaning |
| --- | --- |
| 0 | PASS for `compare` / `explain`; successful **conversion only** for `adapt` |
| 1 | Rejected input, including missing/ambiguous comparison chronology |
| 2 | Unreadable input/output or CLI usage error |
| 3 | FAIL |
| 4 | INCOMPLETE (`compare`) |
| 5 | NOT_EVALUABLE |
| 6 | REPLAY_ONLY (`compare`) |

This deliberately corrects the old “report written means exit 0” behavior.
`explain` exits by the individual run's status; it does not establish series
acceptance. `adapt` exits 0 for a successful conversion even if the converted
status is FAIL or NOT_EVALUABLE; run `compare` to obtain the series exit code.

## Validation and limits

```sh
python3 -m pytest -q spatio/series/tests
python3 -m pytest -q spatio/walkcheck/tests
```

Tests use the actual walkcheck evaluator on varied synthetic sessions (requires
walkcheck's numpy/matplotlib dependencies), not hand-written lookalike reports.
No historical thresholds, paths, sensor settings or topics are baked in.

The 0.10 m strict return rule remains unchanged; placement and wall thresholds
remain provisional. Wall p95 remains descriptive, with only the doubled-wall
flag used by the existing series pass rule. This package validates reports,
not physical accuracy or sharpness of a real room map.

For the null-window duration issue, main already checked each present window
for positive duration. The fix extends that same check to duration zero:
one null window no longer hides a positive-length opposite window. No minimum
duration or window length was introduced, and null is never treated as zero.
