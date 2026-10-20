# Walk-day robustness findings

Built in `spatio-robustness` on 2026-10-20. `PLAN.md` was written before code.
Only this package changed. Added `walking_loop()` and `jitter_timestamps()` to
`synthetic.py`, plus 179 tests in `tests/test_robustness.py`. No false PASS was
found in these cases, so no scorer code or existing tests changed.

The generator accepts geometry, four leg speeds, independent start/end holds,
three corner stops, initial reposition, sample rate, timestamp origin, heading,
closure offset, and drift vector/stretch. Positions remain continuous; drift
accumulates over a chosen fraction of route distance and stays at the end.
The expected displacement is supplied independently of the scorer.

Run from this worktree:

```sh
python3 -m pytest -q spatio/walkcheck/tests/test_robustness.py
cd spatio/walkcheck
python3 -m pytest -q
```

To generate an additional synthetic session from the repository root:

```python
from pathlib import Path
from spatio.walkcheck.synthetic import walking_loop, write_session

rows = walking_loop(
    side=2.4, speeds=(0.2, 1.5, 0.4, 1.1), holds=(11, 12),
    stops=(0, 11, 0), rate=17, origin=1234,
    drift=(0.15, 0, 0), drift_stretch=(0.4, 0.7),
)
write_session(
    Path("/tmp/spatio-synthetic-robustness"), rows, xyz=[[0, 0, 0]],
    manifest={"run_id": "synthetic-robustness", "sensor": "synthetic",
              "source_recording": None, "fastlio_revision": None,
              "config_file": None, "config_sha256": None, "image": None,
              "created_at": None, "notes": "Synthetic trajectory fault test",
              "physical_accuracy_evaluated": False},
)
```

Then run `python3 -m spatio.walkcheck /tmp/spatio-synthetic-robustness`.
This example should FAIL for 15 cm return error; its one-point map provides
only a readable input and cannot establish wall quality.

Exact local test results:

- Before changes: `python3 -m pytest -q spatio/walkcheck/tests`:
  **85 passed, 14 warnings**, 13.44 s.
- New module: **179 passed, 14 warnings**, 8.67 s.
- Full package, `python3 -m pytest -q` from `spatio/walkcheck`:
  **264 passed, 14 warnings**, 13.99 s.
- No failures or skips. The warnings are existing matplotlib/pyparsing
  deprecations. No Swift files changed, so no Swift tests or app build ran.

Observed synthetic results:

- Constant speeds 0.2, 0.5, 0.9, and 1.5 m/s, plus a mixed-speed profile:
  0, 5, and 9.9 cm closure PASS; 10, 10.1, and 15 cm FAIL.
  An 8 cm + 8 cm diagonal (11.3137 cm total) and a 15 cm vertical
  offset FAIL. Reported displacement matches the known input within 2 micrometres.
- Clean alternate loops with 1.3/4.7 m sides, 7/23 Hz sampling, and relative
  or epoch timestamps PASS. Generator checks confirm requested leg speeds.
- Short and long corner stops preserve endpoint selection and closure.
  Long stops produce the existing `possible_repositioning` warning.
- Ten initial-reposition cases are NOT-EVALUABLE. Sixteen missing/short-hold
  cases are NOT-EVALUABLE, including cases with long interior stops.
- Thirty seeded timestamp-jitter cases (up to +/-10 ms, 17 Hz) preserve
  correct PASS/FAIL outcomes. Six clock-gap/dropout cases across start,
  route, and end are NOT-EVALUABLE. Jitter that reverses time is rejected.
- Heading-only cases report +90, +20, -20, and -135 degrees and PASS the
  position rule, as required by the existing contract.
- Forty featureless-stretch proxy cases retain injected drift: 5 cm PASS;
  15 cm, 11.3137 cm diagonal, and 20 cm vertical FAIL. Drift accumulation
  occurs over either 10–30% or 55–90% of route distance.
- A CLI test tightens `--min-hold` from 8 to 12 seconds on a new 10-second-hold
  loop and verifies NOT-EVALUABLE with the override recorded in JSON.

Limits and decisions:

These finite synthetic cases do not prove that every possible trajectory is
safe, or that reported poses match a physical cart. "True closure" here means
the known separation of constant synthetic endpoint holds. Drift that cancels
before the endpoint, incorrect estimator poses, actual featureless geometry,
IMU physics, wall sharpness, and FAST-LIO execution are not established by
these new tests. Existing IMU/wall tests remain in the full package suite.
The generator has instantaneous speed changes at corners; it is a scoring
fixture, not a cart dynamics simulator.

No historical data was read or fitted. Existing PROVISIONAL hold/gap/motion
limits and their CLI overrides remain unchanged. The existing strict 10 cm
milestone closure rule remains fixed. No new scoring thresholds were added.
No Spark, container, app, GitHub, or other-package work was performed.
Independent review remains a separate team responsibility.

No additional Trevor approval is needed for these local changes. Physical
walks and acceptance remain separate, approval-dependent work.

Commit status: changes are **uncommitted**. The local `git add` attempt failed
with exit 128: Git could not create
`/Users/tconf/Developer/labcommand/.git/worktrees/spatio-robustness/index.lock`
(`Operation not permitted`). This session cannot write the worktree's Git
metadata. No permission bypass or push was attempted.
