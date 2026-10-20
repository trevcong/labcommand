# Walk day

One command per recording, one per series. Plain-English runbook:
`~/Developer/lc-tasks/plans/SPATIO-walkday-runbook.md`.

```sh
spatio/walkday/process.sh <recording-dir> <run-id> [--start-placement-cm Y --placement-cm X --heading ok --spec FROZEN]
spatio/walkday/series.sh <run-id> <run-id> <run-id>      # in walking order, failures included
spatio/walkday/synth/dryrun.sh                           # synthetic end-to-end dry run
python3 -m unittest discover -s spatio/walkday/tests -t .
```

`process.sh` runs these stages in order. It stops loudly (exit 2, NOT-EVALUABLE) at the first one that can't be trusted:

1. **copy check:** the bag files match `metadata.yaml` (`spatio/replay/recording.py complete`). The bag-level `starting_time` must also be present (`recording.py recorded-at`; stage `recording-time`). `result.py` writes it as `recorded_at`, which `spatio.series compare` uses to order runs. It is the recorder host's clock when the first message arrived, never the processing time.
2. **config:** topics come from the FAST-LIO config, which must agree with the bag's message types.
3. **spec:** the config sha matches the frozen spec, when a spec is given.
4. **x1 bag timing and input gate:** `spatio/walkday/gate.py`. Rules B1–B5 are in its docstring.
5. **FAST-LIO replay (A):** `spatio/replay/replay.sh`, one at a time per machine.
6. **physical reference (g5):** only with `--placement-cm`. It is never applied for a recording that contains `SYNTHETIC.json`.
7. **manifest (g2):** build plus validate. The replay's own manifest is kept as `logs/replay_manifest.json`.
8. **walk check (B):** settings come from `walkday.conf`.
9. **wall patches (g8):** `--patches FILE`, otherwise auto-suggested and labelled NOT preselected.
10. **result:** `result.py` writes `walk_result.json` (the `spatio.series` input), `walkday.md` and `walkday.json`.

`process.sh` exit codes: 0 PASS, 1 FAIL, 2 NOT-EVALUABLE, 3 usage.

`series.sh` passes through `spatio.series compare`'s exit codes: 0 PASS, 1 inputs rejected (for example missing or tied `recorded_at`), 2 unreadable input (a listed run was never scored) or usage error, 3 FAIL, 4 INCOMPLETE, 5 NOT_EVALUABLE, 6 REPLAY_ONLY. So 3 means usage error for `process.sh` but FAIL for `series.sh`.

The run-status rule is in `result.py`'s docstring. It is stricter than the series parser's W1 rule: walls and patches must also pass.

Sessions go to `~/Developer/spatio-data/walkday/<run-id>/` (`--sessions-root` or `SPATIO_WALKDAY_ROOT`). g2 refuses to write under `spatio-data/sessions`, which holds the earlier replays of the test samples.

`synth/` builds synthetic recordings for the dry run:

- an analytic cart loop in a box room
- a Livox-style CustomMsg lidar with per-point time
- an IMU, by default 250 Hz on `/synth/*` topics

Each recording ships its own FAST-LIO config and a `SYNTHETIC.json` with the ground truth. `synth/truth.py` compares a session with that ground truth.
