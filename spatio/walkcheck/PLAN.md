# Walk-day synthetic robustness plan

Scope: exercise the existing hold-based walk scorer on new, deterministic
synthetic loops. Read both supplied Spatio plans before implementation.
Work only in `/Users/tconf/Developer/spatio-robustness`, branch
`spatio-robustness`, and only change this package.

Files:
- `synthetic.py`: add a small parameterized loop generator with per-leg speeds,
  independent endpoint holds, corner stops, an optional initial reposition,
  heading change, and persistent drift accumulated over a chosen route stretch;
  add deterministic timestamp jitter.
- `tests/test_robustness.py`: check known synthetic closure against the strict
  10 cm milestone gate, clean passes, invalid holds, stops, repositioning,
  timestamp jitter/gaps, heading-only error, and injected drift.
- `FINDINGS-robustness.md`: record implementation, commands, exact test counts,
  synthetic results, limitations, and any approval needs.

Use speeds from 0.2 through 1.5 m/s, including changing speeds within a loop,
and vary geometry, rate, epoch, offsets, and fault location. Expected closure
comes from the generator inputs, independently of the scorer. Here "true
closure" means the known displacement between stationary synthetic endpoints;
it does not mean independently measured physical cart error.

Keep existing thresholds unchanged. Hold, gap, and motion limits remain
PROVISIONAL and retain their existing CLI overrides; the strict 10 cm closure
rule is the existing milestone contract. Add no recording-derived settings,
paths, topics, rates, extrinsics, or fitted thresholds.

Validation:
1. Run the existing package baseline with
   `python3 -m pytest -q spatio/walkcheck/tests`.
2. Run the new module with
   `python3 -m pytest -q spatio/walkcheck/tests/test_robustness.py`.
3. Run the full package suite again and inspect the diff.
4. If a new test demonstrates a real false PASS, retain the failing regression,
   make the smallest scorer correction, explain it in the findings, and rerun
   the full suite. Never weaken or remove existing assertions.
5. Attempt a local commit. If `.git` is not writable, leave the changes
   uncommitted and report that restriction.

Will not do: change other packages, tune to old recordings, access the Spark,
operate containers, launch/build the Swift app, change wall-scoring policy,
add loop correction, claim physical accuracy or FAST-LIO robustness from
synthetic tests, push, use `gh`, open PRs, or trigger CI. Swift tests/builds are
not relevant to this Python-only package change.
