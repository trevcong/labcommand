# F1 synthetic clearance findings

Built only `spatio/clearance`: a strict JSON zone/scan parser, NumPy voxel
ray traversal, four-state classifier, conflict reporting, synthetic scene,
tests, and JSON/Markdown CLI. The 23-line PLAN.md was written before code.
Boxes and plane-offset volumes are axis-aligned. Actual hit distances and
conservative voxel bounds are reported separately. Unknown cells cannot pass;
near-limit occupancy returns INDETERMINATE using explicit provisional uncertainty.

From the repository root, run:

```sh
python3 -m spatio.clearance --synthetic --voxel-size 0.1 \
  --uncertainty-m 0.02 --output /private/tmp/clearance-example
python3 -m pytest -q spatio/clearance
```

Choose a new output directory. README.md defines independent zone/scan inputs
and the `--zones`, `--scans`, `--grid-origin` options.

Measured local test result: **82 passed, 0 failed, 0 skipped**.
CLI synthetic and independent-input runs are also covered by subprocess tests.
The independent scene changes scale, position, sensor origins and timestamps.
No Swift files or shared components changed; the relevant suite is this Python
package. Swift tests and an app build were not run.

The CLI smoke run used 0.10 m voxels and ±0.02 m provisional uncertainty:

| Zone | Result | Measured min hit clearance | Free / occupied / unknown cells |
|---|---|---:|---:|
| Panel, box 0.80 m in front | FAIL | 0.80 m | 6 / 1 / 153 |
| Behind solid occluder | NOT_MEASURED | none | 0 / 0 / 80 |
| Behind narrow gap | NOT_MEASURED | 1.00 m | 5 / 0 / 95 |

There were three timestamped scans and zero conflicting cells in that scene.
Separate tests prove conflicts survive both observation orders, complete
coverage can PASS, and margins on both sides of the limit can abstain.
The synthetic generator calculates first intersections with solid surfaces;
the 0.80 m clearance is recovered from those returns.

Limits: synthetic evidence only; physical accuracy was not evaluated.
PASS is a voxel-resolution statement. Partial boundary cells can cause false
alarms; thin obstacles and sparse sampling need physical evaluation.
Origins/first returns must already be correctly transformed and timed.
No bag reader, deskew, oblique zones, calibrated uncertainty or large-walk
streaming is included. Example provisions are **EXAMPLE, NOT APPROVED**.
Trevor's future approval is needed for zone types, actual rule provisions and
applicability, and physical acceptance/uncertainty settings.

All changes remain in the assigned worktree on branch `spatio-f1-clearance`
at `/Users/tconf/Developer/labcommand/.git/worktrees/spatio-f1-clearance`;
No Spark, real recordings, protected containers, apps, GitHub or other
workstream files were used or changed.


Commit status: Codex could not write Git metadata; the orchestrator committed this work.
