# Walk day

One command per recording, one per series. Plain-English runbook:
`~/Developer/lc-tasks/plans/SPATIO-walkday-runbook.md`.

```sh
spatio/walkday/process.sh <recording-dir> <run-id> [--start-placement-cm Y --placement-cm X --heading ok --spec FROZEN --patches FILE]
spatio/walkday/process.sh <recording-dir> <run-id> --config FASTLIO.yaml --driver-config DRIVER.yaml   # two-config run (JT128: jt128_fastlio.yaml + jt128_post.yaml)
spatio/walkday/series.sh <run-id> <run-id> <run-id>      # every attempt, failures included
spatio/walkday/synth/dryrun.sh                           # synthetic end-to-end dry run
python3 -m unittest discover -s spatio/walkday/tests -t .
```

`process.sh` runs these stages in order. It stops loudly (exit 2, NOT-EVALUABLE) at the first one that can't be trusted:

1. **copy check:** the bag files match `metadata.yaml` (`spatio/replay/recording.py complete`). The bag-level `starting_time` must also be present (`recording.py recorded-at`; stage `recording-time`). `result.py` writes it as `recorded_at`, which `spatio.series compare` uses to order runs. It is the recorder host's clock when the first message arrived, never the processing time.
2. **config:** topics come from the FAST-LIO config, which must agree with the bag's message types. `logs/provenance.json` then records the effective walk-day settings identity, the FAST-LIO config hash and, with `--driver-config`, the sensor driver config hash.
   - The driver config is upstream of FAST-LIO (for the JT128, the Hesai driver's `jt128_post.yaml`). Replay does not read it, because the bag already holds the driver's or adapter's output.
   - It must be readable text, must not be a FAST-LIO config or the `--config` file itself, and at least one ROS topic it declares must be in the bag. A copy is kept in `logs/`.
3. **spec:** the config sha matches the frozen spec, when a spec is given.
4. **x1 bag timing and input gate:** `spatio/walkday/gate.py`. Rules B1–B5 are in its docstring.
5. **FAST-LIO replay (A):** `spatio/replay/replay.sh`, one at a time per machine.
6. **physical reference (g5):** only with `--placement-cm`. It is never applied for a recording that contains `SYNTHETIC.json`.
7. **manifest (g2):** build plus validate. The replay's own manifest is kept as `logs/replay_manifest.json`.
8. **walk check (B):** settings come from `walkday.conf`.
9. **wall patches (g8):** `--patches FILE` holds the frozen spec's preselected patches (same names), boxed on this run's map. Without it, patches are auto-suggested and labelled NOT preselected. Such a run is still scored, but it never counts toward a series.
10. **result:** `result.py` writes `walk_result.json` (the `spatio.series` input), `walkday.md` and `walkday.json`. The settings identity it writes is checked against what was actually used: the walk check's recorded parameters (`walkcheck.json`) and its own `--wall-gate-m`/`--wall-gate-state`. If any differs (for example, re-scoring with `--wall-gate-m 0.08`), the identity is recomputed from the used values and `walkday.md` lists each change. That run then starts a new series.

`process.sh` exit codes: 0 PASS, 1 FAIL, 2 NOT-EVALUABLE, 3 usage.

`series.sh` runs `spatio.series compare` (its own output goes in `series/<id>/spatio_series/`). Then `seriesgate.py` writes the final `series/<id>/series.{md,json}`. Its two rules can only make the verdict stricter:

- G1: every run has the same walk-day settings identity and the same driver config; otherwise NOT_EVALUABLE.
- G2: a run counts only if both of the spec's preselected patches were verified.

Running `spatio.series compare` directly skips G1 and G2, so walk day always uses `series.sh`. `series.sh` uses `spatio.series compare`'s exit codes: 0 PASS, 1 inputs rejected (for example missing or tied `recorded_at`), 2 unreadable input (a listed run was never scored) or usage error, 3 FAIL, 4 INCOMPLETE, 5 NOT_EVALUABLE, 6 REPLAY_ONLY. So 3 means usage error for `process.sh` but FAIL for `series.sh`.

The run-status rule is in `result.py`'s docstring. It is stricter than the series parser's W1 rule: walls and patches must also pass.

Sessions go to `~/Developer/spatio-data/walkday/<run-id>/` (`--sessions-root` or `SPATIO_WALKDAY_ROOT`). g2 refuses to write under `spatio-data/sessions`, which holds the earlier replays of the test samples.

`synth/` builds synthetic recordings for the dry run:

- an analytic cart loop in a box room
- a Livox-style CustomMsg lidar with per-point time
- an IMU, by default 250 Hz on `/synth/*` topics

Each recording ships its own FAST-LIO config and a `SYNTHETIC.json` with the ground truth. `synth/truth.py` compares a session with that ground truth.
