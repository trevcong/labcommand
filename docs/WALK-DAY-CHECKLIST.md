# Walk-day checklist: Mid-360 marked loop, basement, 2026-10-21

One page. Values are filled only where a repo file settles them (cited `file:line`). Everything else is a blank below or a question at the end. No code, setting or test changed. Today `python3 -m spatio.expspec validate` on the draft spec gives 12 errors: the 11 `TODO(Trevor)` blanks plus the `spec_id` pattern. So no run can count until the spec is filled and frozen (`spatio/wall_scoring.py:261`, `docs/CURRENT-STATE.md:56`).

## 1. Already settled (fill as-is)
| Field | Value | Source |
|---|---|---|
| sensor | Mid-360 + built-in IMU, CustomMsg `/livox/lidar`, Imu `/livox/imu` | `spatio/walkday/spec/SPEC-mid360-walk.DRAFT.json:4`, `spatio/walkday/configs/mid360_walk.yaml:13-14` |
| fastlio_revision | `2fffc570a25d0df172720bac034fbdb6a13d2162` (matches the replay pin; the image build refuses any other checkout) | `SPEC-mid360-walk.DRAFT.json:5`, `spatio/replay/FASTLIO_REVISION:1`, `spatio/replay/build_image.sh:27-30` |
| config_file / config_sha256 | `spatio/walkday/configs/mid360_walk.yaml` / `0fba8e5d…0aa1` (re-hashed with `shasum -a 256`: matches) | `SPEC-mid360-walk.DRAFT.json:6-7` |
| frames.map / body / lidar_to_imu | `camera_init` / `body` / extrinsic_T [-0.011, -0.02329, 0.04412], R = I, est_en true | `SPEC-mid360-walk.DRAFT.json:22-24`, `mid360_walk.yaml:33-34` |
| route.duration_target_s | 180–300 s | `SPEC-mid360-walk.DRAFT.json:10-13` |
| stationary_windows_s (hold on the mark) | 15 s start, 15 s end (see Q3) | `SPEC-mid360-walk.DRAFT.json:39-42`, `spatio/walkday/spec_draft.py:27` |
| acceptance | return < 0.10 m (3D); placement ≤ 0.01 m; wall p95 and separation 0.05 m; 3 runs; state `proposed` (see Q4) | `SPEC-mid360-walk.DRAFT.json:44-49`, `spatio/thresholds.py:17,31`, `spatio/series/validate.py:25` |
| forbidden | end-onto-start alignment, post-hoc loop correction, smoothing away failures, sliding the window | `SPEC-mid360-walk.DRAFT.json:51-56`, `spatio/expspec/constants.py:32-41` |
| wall guard band | ≤ 4.0 cm can pass; 4–5 cm not checkable; > 5 cm fails | `spatio/thresholds.py:26`, `spatio/walkday/README.md:43-46` |
| walkday.conf (approved provisional) | window 5 s, gap 1.0 s, path ≥ 2.0 m, excursion ≥ 0.5 m, wall gate 0.05 m `approved` | `spatio/walkday/walkday.conf:7-10,36-37` |
| walkday.conf (NOT approved) | endpoint spread 0.05 m, endpoint speed 0.10 m/s, hold speed 0.10 m/s, min hold 8 s (Q1, Q2) | `walkday.conf:14-15,28-29` |
| frozen spec / env | `SPATIO_WALK_SPEC=<frozen spec>` is the `--spec` default and is required with `--placement-cm` | `spatio/walkday/process.sh:53,95` |
| sessions root | `~/Developer/spatio-data/walkday/<run-id>/` (`SPATIO_WALKDAY_ROOT` overrides); an existing run id is refused | `process.sh:56,119` |
| run id | must match `[A-Za-z0-9][A-Za-z0-9_.-]*`; one new id per recording | `process.sh:77` |
| exit codes | process.sh 0 PASS, 1 FAIL, 2 NOT-EVALUABLE, 3 usage; series.sh 0 PASS … 3 FAIL, 4 INCOMPLETE, 5 NOT_EVALUABLE, 6 REPLAY_ONLY | `spatio/walkday/README.md:29,36` |

Measured at the Spark preflight, not filled here: ROS domain id, network mode, recorder image, topic rates (runbook §3, `~/Developer/lc-tasks/plans/SPATIO-walkday-runbook.md:55-60`, local). The Spark recorder itself still needs Trevor's approval (same runbook, §6 item 1).

## 2. Blank: only Trevor can fill (no file decides these)
`spec_id` (:3, see Q5) · `route.description` (:9) · stops A, B (:15-16) · `reference_point` (:19) · `start_marks` (:20) · `frames.cart_mount` (:25) · patch A name + description (:29-30) · patch B name + description (:34-35). Line numbers are in `SPEC-mid360-walk.DRAFT.json`. `box_map_frame` may stay `null` (`spatio/expspec/validate.py:115-116`).

## 3. Order of the day
1. Fill section 2, then `python3 -m spatio.expspec validate <draft>` → 0 errors, then `python3 -m spatio.expspec freeze <draft> --out <frozen>` (`spatio/expspec/cli.py:135,144`).
2. Preflight walk: about 30 s parked, a few metres and one turn, back to the mark, 15 s hold. Run `process.sh` with no placement. Check `walkcheck.json` → `holds` finds both holds (`walkday.conf:26-27`).
3. Each scored run: cart on the mark **before** recording starts; hold 15 s; push slowly; return; hold 15 s; stop recording after the hold. Tape START and END residuals in cm; heading ok/off. Keep every attempt.
4. Export `SPATIO_WALK_SPEC=/absolute/path/to/spec.frozen.json` (the `--spec` default; required for placements and rescore). Run `spatio/walkday/process.sh <recording-dir> <run-id> --start-placement-cm Y --placement-cm X --heading ok` first. Its auto-suggested patches leave the run `NOT_EVALUABLE`. Box the spec's preselected walls on this session's `map.pcd`; prepare a g8 patch file with the spec's names, `box_map_frame`, origin `preselected`, and top-level `map_sha256` from `shasum -a 256 <session>/map.pcd`. Then run `spatio/walkday/process.sh --rescore <session> --patches <file>`. Rescore uses the saved settings and evidence, checks patch provenance, measures the walls and rebuilds the result without replay. Missing or mismatched hashes fail. Finally run `spatio/walkday/series.sh <run> <run> <run>` with failures included; exit **6 means REPLAY_ONLY**, not physical acceptance (other series exits: 0 PASS, 1 inputs rejected, 2 unreadable input/usage, 3 FAIL, 4 INCOMPLETE, 5 NOT_EVALUABLE). See `spatio/walkday/README.md` for the full rescore contract.

## 4. Trevor must confirm (yes/no)
- **Q1 Hold speed.** Walk day passes `MAX_HOLD_SPEED_M_S=0.10` (`walkday.conf:28`, `process.sh:233`). A bare `python3 -m spatio.walkcheck` uses 0.02 m/s (`spatio/walkcheck/holds.py:13`). The detector uses min(hold speed, endpoint speed) (`spatio/walkcheck/trajectory.py:226-228`), so raising one alone changes nothing. Use 0.10 m/s for scored runs?
- **Q2 Endpoint and hold limits.** Approve endpoint spread 0.05 m, endpoint speed 0.10 m/s and min hold 8 s (`walkday.conf:11-15,29`; picked after looking at the Aug 8 replays)?
- **Q3 Window wording vs scoring.** The rendered spec says "first/last 15 s of the run, pose nearest the middle" (`spatio/expspec/render.py:167-176`). The score is the mean of the first 5 s of the first detected hold vs the last 5 s of the last hold (`walkday.conf:7`, `trajectory.py:484-497`, `:272-274`). Freeze with 15/15 anyway?
- **Q4 Acceptance state.** Set `acceptance.state` to `approved_by_trevor` before freezing? It changes the printed banner only (`spatio/expspec/runsheet.py:33`, `render.py:94`); scoring does not read it.
- **Q5 Spec id.** Use the draft's example `mid360-walk-2026-10` (`SPEC-mid360-walk.DRAFT.json:3`)?
- **Q6 Patches for real runs.** Is hand-boxing the spec's preselected walls on each run's map, then using the supported `process.sh --rescore` route in step 4, the agreed walk-day procedure? The patch file must carry that exact map's sha256; missing provenance and auto-suggested origins cannot establish a counted pass.
