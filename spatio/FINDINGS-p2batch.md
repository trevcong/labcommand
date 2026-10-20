# Spatio P2 batch findings

All six fixes were implemented, in order, on `spatio-fix-p2batch`, starting at
`bdf6fbf`. The plan is `spatio/PLAN-p2batch.md`.

## Changes and results

- **#56:** Fixture regeneration permits absolute roundoff of at most `1e-12`
  in computed `EXPECTED.json` floats. Types, counts, strings, keys, and other
  files remain exact. The regression accepts a one-ULP change at 0.05 and rejects
  a `1e-9` change, NaN, changed counts/types/text, and changed binary data.
  This deliberately corrects the byte-equality requirement for computed metrics.
- **#51:** Quaternion and rotation-matrix differences use atan2 forms.
  The regression reproduced a false `2.41483654e-6` degree difference between
  identical nontrivial quaternions before the fix. It now yields exactly zero,
  while preserving `1e-9` radian differences, sign equivalence, and half turns.
- **#54:** A package conftest collects the immutable upstream tests through the
  existing offline harness. Both runnable upstream tests run exactly once.
  The existing unavailable-PCAP skip is retained explicitly. No new test is
  skipped, no decoder is faked, and neither vendor file changed.
- **#52:** A missing bag with a recorded SHA-256 now produces an M7 error and
  validation CLI exit 1. A deliberately unhashed missing bag retains its prior
  behavior. The nested-path collision test now requires an error rather than a
  warning: a deliberate strengthening of the specification. Other unreadable
  fingerprint behavior is unchanged.
- **#49:** The gate summary explicitly labels the independent rosbags check
  PASS, FAIL, or SKIPPED, including when no cases were collected.
  `--require-rosbags` fails the summary for absent, skipped, or failed coverage.
  The gate's default keeps this optional dependency optional but visible.
  The actual local cross-check passed all 52 cases; its generated JUnit report
  also passed the new required-coverage summary check. No workflow was triggered
  or edited, and no Spark runtime code changed.
- **#64:** Any longer-turn interpretation admitted by either existing unit band
  makes a sparse step ambiguous, regardless of distance to the band center.
  The supplied 13.43 rad/s synthetic repro failed before the fix by reporting
  deg/s. It now returns `undetermined`, no ratio, zero accepted windows, and four
  rejected ambiguous windows. A sparse genuine deg/s case is likewise refused,
  while additional intermediate poses retain a deg/s verdict. One existing
  full-IMU/two-pose assertion had the same ambiguity: it now requires rejection,
  with an added denser-pose positive control. This is a deliberate specification
  correction, not a relaxed threshold. All IMU thresholds remain provisional
  and unchanged.

## How to run and test results

Run from the repository root with `python3 -m pytest -q` followed by each path:

| Path | Passed | Failed | Skipped |
| --- | ---: | ---: | ---: |
| `spatio/io` | 26 | 0 | 0 |
| `spatio/replaycompare` | 17 | 0 | 0 |
| `spatio/jt128adapter` | 38 | 0 | 1 |
| `spatio/manifest` | 31 | 0 | 0 |
| `.github/scripts` | 13 | 0 | 0 |
| `spark/room_memory/tests/test_rosbags.py` | 52 | 0 | 0 |
| `spatio/imucheck` | 23 | 0 | 0 |

The JT128 skip is the pre-existing missing `jt128_one_frame.pcap` input.
JT128 and IMU each emitted one third-party `mcap_ros2.reader` deprecation warning.

Broader `spark/room_memory` suite: running; final result will be recorded before
handoff.

To check regenerated fixtures:
`python3 -m spatio.io.make_fixtures --out spatio/io/fixtures --check`.
For the gate's saved `pytest.xml` and `ctest.xml` reports:
`python3 .github/scripts/ci_checks.py summary <reports-directory> --require-rosbags`.
Omit the flag to report optional rosbags coverage without requiring it.

## Limits and handoff

These are local synthetic and codec checks. No historical recordings were read,
no physical accuracy was evaluated, and no physical localization result is
claimed. No thresholds or inputs were fitted to old walks. The IMU inference
remains supporting evidence, not independent proof of units; no new external
evidence interface was added.

Swift and C++ were unchanged, so their suites and app builds were not run.
No Spark, protected container, app, GitHub, push, PR, or other worktree operations
were performed. This is the builder's evidence packet, not an independent review.
No operational approval from Trevor is needed for these local fixes.

Commit status: Codex could not write Git metadata, so the orchestrator committed each fix separately (one commit per issue). Codex made no commit or push.
