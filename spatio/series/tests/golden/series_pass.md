# Spatio series golden-pass

Verdict: PASS

Counted runs: g-pass-1, g-pass-2, g-pass-3

## Rules

This report checks the producer's walk_result files again. It does not trust a PASS that the numbers do not support. Attempts are in the order given on the command line.

Precedence is input error (exit 1), then NOT_EVALUABLE, then REPLAY_ONLY, then PASS, then INCOMPLETE, then FAIL. A weaker verdict must not hide a stronger problem.

- R0. Validate each file against schema S4 and rules W1–W4. Reject a PASS that the numbers do not support. Reject a non-PASS with no reasons. The error names the file.
- R1. Every run must share config_sha256 and fastlio_revision. If not, the verdict is NOT_EVALUABLE because a changed configuration starts a new series. The table still lists every run.
- R2. A run counts only when its status is PASS, physical accuracy was evaluated, both placement residuals are measured and at or under 0.01 m, and the heading check is ok.
- R3. The series is PASS only when three consecutive attempts all count. The earliest three are named. Other non-counting attempts stay in the table and under Retained failed attempts.
- R4. If every run has physical accuracy not evaluated, the verdict is REPLAY_ONLY and the label is "replay demonstrated; physical accuracy not evaluated".
- R5. Fewer than 3 attempts, once the stronger verdicts above do not apply, is INCOMPLETE. A retained failure does not rule the series out before three attempts exist. Three or more attempts with no counting triple is FAIL.
- Boundary. A 3D return error of exactly 0.10 m is not under 0.10 m, so that run fails.
- Null. A null measurement is "not measured". It is never read as zero.
- Gates. The return threshold is 0.10 m and the placement tolerance is 0.01 m. A looser number in the file is rejected. Wall p95 is shown and is not an acceptance gate. W1 uses the doubled-wall flag only. A proposed wall gate is not yet approved by Trevor.

## Comparison

| attempt | run_id | status | return 3D (cm) | horizontal (cm) | vertical (cm) | heading (°) | path (m) | drift (% of path) | wall p95 (cm) | doubled wall? | placement start/end (cm) | physical accuracy evaluated? | counts toward series? | reasons |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 1 | g-pass-1 | PASS | 5.0 | 3.0 | 4.0 | 0.4 | 42.0 | 0.12 | 3.1 | no | 0.4 / 0.6 | yes | yes | - |
| 2 | g-pass-2 | PASS | 6.0 | 3.6 | 4.8 | 0.8 | 38.5 | 0.16 | 2.8 | no | 0.3 / 0.5 | yes | yes | - |
| 3 | g-pass-3 | PASS | 3.0 | 1.8 | 2.4 | 1.1 | 45.0 | 0.07 | 2.2 | no | 0.2 / 0.4 | yes | yes | - |

## Retained failed attempts

None.

The wall gate is proposed, not yet approved by Trevor.

## Reasons

- Three consecutive attempts count: g-pass-1, g-pass-2, g-pass-3.
