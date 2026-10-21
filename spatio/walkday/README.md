# Walk day

One command per recording, one per series. Plain-English runbook:
`~/Developer/lc-tasks/plans/SPATIO-walkday-runbook.md`.

```sh
spatio/walkday/process.sh <recording-dir> <run-id> [--start-placement-cm Y --placement-cm X --heading ok --spec FROZEN --patches FILE]
spatio/walkday/process.sh <recording-dir> <run-id> --config FASTLIO.yaml --driver-config DRIVER.yaml   # two-config run (JT128: jt128_fastlio.yaml + jt128_post.yaml)
spatio/walkday/process.sh --rescore SESSION --patches FILE [--spec FROZEN]
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
9. **wall patches (g8):** `--patches FILE` holds the frozen spec's preselected patches (same names), boxed on this run's map. Only for a recording marked `SYNTHETIC.json`, a file without `map_sha256` is bound to this run's `map.pcd` only when the frozen spec fixes every box and each box is the spec's (the dry run supplies such a file). Without it, patches are auto-suggested and labelled NOT preselected. Such a run gets diagnostic measurements but its result is NOT_EVALUABLE and it never counts toward a series.
10. **result:** `result.py` writes `walk_result.json` (the `spatio.series` input), `walkday.md` and `walkday.json`. The settings identity it writes is checked against what was actually used: the walk check's recorded parameters (`walkcheck.json`) and its own `--wall-gate-m`/`--wall-gate-state`. If any differs (for example, using the result CLI with the stricter `--wall-gate-m 0.03`), the identity is recomputed from the used values and `walkday.md` lists each change. That run then starts a new series. Before scoring, `result.py` moves any earlier `walk_result.json` aside (`walk_result.json.retired`). If the inputs are rejected (exit 3) or the run stops early, no `walk_result.json` remains, so an old PASS can't be counted: `series.sh` refuses the run and `walkday.md` gives the reason. A run is scored only on a clean input gate: if `logs/input_gate.json` is missing, blocked (for example CLOCK_RESET) or contradictory, the run is NOT-EVALUABLE with its return error withheld, so it never counts. Walls are scored by the shared `spatio.wall_scoring` rule (also used by `spatio.series adapt`); a wall-patch report measured on a different `map.pcd` is rejected (exit 3).

For a real walk, select the wall names and descriptions in the frozen spec
before recording. Export `SPATIO_WALK_SPEC=/absolute/path/to/spec.frozen.json`
(or pass `--spec` to both commands). First run `process.sh` with the measured
start/end placements and heading, without `--patches`. The resulting map can
then be boxed at those preselected walls; the initial auto-suggested result is
`NOT_EVALUABLE` and does not count.

```sh
export SPATIO_WALK_SPEC=/absolute/path/to/spec.frozen.json
spatio/walkday/process.sh <recording-dir> <run-id> --start-placement-cm Y --placement-cm X --heading ok
shasum -a 256 <session>/map.pcd
# Prepare FILE in the g8 format with the spec's patch names, each box_map_frame,
# origin "preselected", and top-level map_sha256 equal to the hash above.
spatio/walkday/process.sh --rescore <session> --patches FILE
```

`--rescore` copies FILE to `SESSION/wallpatch/patches.json`, checks patch origin
and names against the frozen spec and requires `map_sha256` to match this
session's `map.pcd`, then runs wallpatch measure and result. Missing or wrong
hashes are refused; rescore never fills in a missing hash, even for synthetic
sessions. Auto-suggested origins remain auto-suggested and cannot count.
Use the same frozen spec as the physical reference. The map, trajectory,
manifest, placements, input gate and walk check are reused unchanged. The
original source recording must remain available for its start time and
synthetic marker checks. The saved `logs/walkday.conf` supplies `WALL_GATE_M` and
`WALL_GATE_STATE`; `--settings FILE` overrides that source, subject to the
existing gate and settings identity checks. Settings for skipped stages are
not reapplied. Rescore also requires `logs/provenance.json` and
`logs/input_gate.json`. It accepts only `--patches`, `--spec` and `--settings`.
It retires the earlier result before replacing patch evidence; a failed copy,
origin check or measurement leaves no countable `walk_result.json`. Read the
new `walkday.md`, then rerun `series.sh` with every attempt included.

`process.sh` exit codes: 0 PASS, 1 FAIL, 2 NOT-EVALUABLE, 3 usage.

`series.sh` runs `spatio.series compare` (its own output goes in `series/<id>/spatio_series/`). Then `seriesgate.py` writes the final `series/<id>/series.{md,json}`. Its two rules can only make the verdict stricter:

- G1: every run has the same walk-day settings identity, the same driver config and the same frozen spec (`wall_patches.spec_sha256`); otherwise NOT_EVALUABLE.
- G2: a run counts only if both of the spec's preselected patches were verified.

Running `spatio.series compare` directly skips G1 and G2, so walk day always uses `series.sh`. `series.sh` uses `spatio.series compare`'s exit codes: 0 PASS, 1 inputs rejected (for example missing or tied `recorded_at`), 2 unreadable input (a listed run was never scored) or usage error, 3 FAIL, 4 INCOMPLETE, 5 NOT_EVALUABLE, 6 REPLAY_ONLY. So 3 means usage error for `process.sh` but FAIL for `series.sh`.

The wall rule is in `spatio/wall_scoring.py`, shared with `spatio.series adapt`.
Both require the spec's verified preselected patches, whole-map p95 and patch
gates, and no doubled wall on the patches. Zero verified patches is
NOT_EVALUABLE. The whole-map doubled-wall flag is a warning only (Trevor,
2026-10-20): a solid pillar can trigger it, so only the scoring patches decide
doubled walls. Provisional guard band per patch: p95 and both 80% bands
<= 4.0 cm (`wall_scoring.PASS_MAX_M`) can pass; above 4.0 cm and within the
5 cm gate the patch is not checkable (NOT_EVALUABLE, never PASS); p95 or
separation over 5 cm fails. A band of 5 cm or more stays not checkable.

Sessions go to `~/Developer/spatio-data/walkday/<run-id>/` (`--sessions-root` or `SPATIO_WALKDAY_ROOT`). g2 refuses to write under `spatio-data/sessions`, which holds the earlier replays of the test samples.

`synth/` builds synthetic recordings for the dry run:

- an analytic cart loop in a box room
- a Livox-style CustomMsg lidar with per-point time
- an IMU, by default 250 Hz on `/synth/*` topics

Each recording ships its own FAST-LIO config and a `SYNTHETIC.json` with the ground truth. `synth/truth.py` compares a session with that ground truth.
