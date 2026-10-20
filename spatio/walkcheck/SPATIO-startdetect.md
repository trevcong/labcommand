# Start/end hold detection findings

Built on branch `spatio-startdetect`, in its assigned worktree. Read both shared plans and the drift investigation. Only `spatio/walkcheck` changed.

The scorer now detects sustained stationary intervals and retains the first and last as the start/end holds. Closure uses the first five seconds inside the start hold and the last five seconds inside the end hold; there is no alignment or selection by lowest closure error. Reports include hold times, durations, samples, and flags. Missing/short holds are NOT-EVALUABLE. A single full-recording stationary hold retains the existing stationary FAIL.

Provisional detection defaults are eight seconds minimum, 2 cm/s step speed, 3 deg/s angular speed, 5 cm position spread, and 10 degrees rotation excursion. Significant motion before/after the holds (over 20 cm or 10 degrees) blocks evaluation. An intermediate hold at another pose is flagged as possible repositioning; intentional route stops are indistinguishable, so that flag alone does not fail the walk or redefine its start. Motion outside the holds cannot satisfy the route-length gate.

Run:

```bash
python3 -m spatio.walkcheck /path/to/session
python3 -m spatio.walkcheck /path/to/session --min-hold 8 --max-hold-speed 0.02
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 -m pytest -q spatio/walkcheck
```

All detection thresholds have CLI overrides; see README and `--help`. Optional `session/imu.csv` or `--imu-file PATH` uses header `t,gx,gy,gz,ax,ay,az`: trajectory-clock seconds, rad/s, and acceleration including gravity in m/s². Continuous quiet IMU must bracket each candidate trajectory step. Malformed input fails closed. No bag/topic, unit, clock-offset or extrinsic inference is performed.

Validation: baseline 45 passed. Final full package suite: **85 passed, 0 failed, 0 skipped, 14 warnings in 12.63 s**, one pytest process with BLAS/OpenMP threads limited to one. The warnings are existing matplotlib/pyparsing deprecations. Forty added cases cover hold selection/duration, missing holds, repositioning, pre/post motion, rotation, drift, quaternion signs, different sampling rates/time origins, optional IMU, coverage/gaps/spikes, invalid inputs/thresholds, CLI overrides and reporting. Existing assertions and tolerances were retained; passing synthetic fixtures were extended to meet the ten-second hold protocol. The gap-override fixture's gap was moved into the moving segment to keep testing gap policy independently of hold detection.

Measured synthetic results: the reposition-then-lap case detects three holds (0–25 s, 32–82 s, 169–182 s), flags possible repositioning, and reports **65 cm closure, FAIL**. Using the later pause would misleadingly yield 5 cm. Separate valid loops at different rates and timestamp origins retain 5 cm closure. No historical recordings or live hardware were evaluated; these tests establish software behavior, not physical accuracy. No Swift code changed, so no Swift build/test was run.

Limits: strict step-speed/spread checks can reject noisy holds or holds adjacent to slow transitions. Thresholds require validation on fresh taped walks; they were not fitted to old recordings. IMU export must supply documented units and a common clock. The detector cannot identify the taped pose or distinguish every reposition from a route stop. Wall scoring remains separate.

Trevor: no further approval needed for this local implementation. Freeze/validate provisional limits before physical acceptance. Spark execution remains outside this work. No SSH, container operations, app launch, push, PR, `gh`, or GitHub Actions.

Handoff: this sandbox exposes git metadata read-only and excludes the requested external findings directory from writable roots. This file is the worktree fallback for `/Users/tconf/Developer/lc-tasks/plans/SPATIO-startdetect.md`. The final reply records the external-write and commit outcomes. Suggested local commit message: `Detect sustained start and end holds in walkcheck [skip ci]`.
