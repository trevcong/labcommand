# G5 physical acceptance runbook (D49, section 3.13)

This file is decision D49 and protocol section 3.13. `~/Developer/lc-tasks/NUMBERING.md` reserves that pair for this checkout. It is the operator procedure for one room visit. It does not add a message, a topic, or a threshold, and it does not amend `docs/ROOM-MEMORY-PROTOCOL.md`.

The scored session is PR #36, branch `g5-acceptance-harness`, commit `172518b` ("Physical acceptance-test harness and scorer"). The commands, label grammar, and exit codes below were read from that commit (`README.md`, `session.py`, `score.py`). If that PR changes, re-read those files before the visit. This checkout does not contain `spark/room_memory/acceptance/` and does not copy it.

`evaluate.py`, the `acceptance_replay` ctest, and the `bag_stream.py` rig-move flags are still deferred. This visit does not run them.

Two results stay separate:

- The harness `g5_acceptance` value `full_candidate` is a candidate score. It is not the product bar in `docs/phase3-live-integration.md` section 7.5 / Q3.
- The tripod bump is its own mark file. `bump_evaluation` of `not_evaluated` means the guard was off, unseen, or the evidence was incomplete. That is not a harness exit code and not a failed guard test.

`RIG_MOVED` is not a box detection. While the rig is moved, the node suppresses current-window change overlays and keeps confirmed history. Unverified space (a voxel the sensor did not see) is not `DISAPPEARED`. This visit does not run the hidden-board case from `PLAN-RIGGUARD.md`.

## 1. Rehearse on the Mac

From the repo root, stdin closed. Omitted mode is dry-run. `--dry-run` is the same thing. It finishes in well under a second, prints the visit in order, and writes a timing sheet. It does not sleep the hour, dial the Spark, or read the network.

```text
python3 scripts/room_acceptance_marks.py --out DIR
python3 scripts/room_acceptance_marks.py --dry-run --out DIR
```

Printed order:

1. `preflight`
2. `tripod_bump`
3. `harness_handoff`
4. `reference_setup` and the PR #36 protocol, hour last

Script-owned rows are `driver` `room_acceptance_marks`. Harness-owned rows are `driver` `harness` and `completion` `rehearsal`. A rehearsal row was not watched. Do not pass `DIR/scenario-marks.json` to `acceptance score`. The schema is `scenario-marks-v1`, not `room-memory-acceptance-marks-v1`. The dry-run clock is synthetic (`2026-01-01T00:00:00Z`, monotonic nanoseconds from 0). The harness clock is `receipt_mono` on the subscriber's parent process. They are not the same clock.

`--hour-s`, `--sit-s`, `--walk-s`, and `--watch-s` change only the planned seconds of the rows the harness would shorten. Protocol seconds stay 60 / 120 / 105 / 120 / 3600. A dry-run with `--hour-s 1` is a format check, not a G5 pass. `--watch-s` does not change the bump. `--bump-s` changes the bump's planned watch only. The bump protocol duration stays 30 s.

## 2. Preflight, before anyone moves

Do this in the room. This procedure does not deploy, restart, set a parameter, `docker exec`, or ssh.

- The room-memory container is already up. Leave it up.
- A display you already have (LabCommand Room, or the node log already on screen) is how you read state. This script does not open a socket.
- Write down what you actually see. Both launch files default `rig_guard` to false, `auto_relearn` to true, `relearn_after_s` to 60, `imu_quiet_s` to 2, and `check_max_s` to 10. Those are what an un-overridden launch would pass. They are not observations of this process. If you did not see the value, the answer is `unknown`.
- IMU availability is the same rule. `imu_topic` defaulting to `/livox/imu` does not mean samples are arriving. Answer `yes` only if you saw samples or `imu_ok`. Answer `no` if you saw that they are absent. Otherwise `unknown`.
- Three distinct objects, taped, with the real names in your notes:
  - PLACE: not in the frozen reference. The spot is empty in that reference.
  - REMOVE: a different object that is already in the reference.
  - MOVE: a third object that is already in the reference. You will move it about 1 m and stay under 1.5 m.
- `TRACKING` does not prove the reference contains those objects. A saved reference is not checked by the harness.

### Label worksheet

Resolve this before the scored session. PR #36's live runner does not print candidate coordinates (`candidates_fn` is not passed). The harness appends `HARNESS_DIR/topics.jsonl` during the run. At the label prompt, read that file from another terminal. It is append-only until the session ends. This one-liner only reads:

```text
python3 -c '
import json, sys
for line in open(sys.argv[1], encoding="utf-8"):
    line = line.strip()
    if not line:
        continue
    try:
        row = json.loads(line)
    except json.JSONDecodeError:
        continue
    if row.get("topic") != "/room_memory/events":
        continue
    msg = row.get("message") or {}
    if msg.get("state") != "candidate":
        continue
    aabb = msg.get("aabb") or [None, None]
    coords = " ".join(str(v) for v in list(aabb[0] or []) + list(aabb[1] or []))
    src = msg.get("from_centroid")
    tail = (" from " + " ".join(str(v) for v in src)) if src else ""
    print("region", msg.get("sensors"), coords + tail)
    print("event", row.get("publisher_generation"), msg.get("event_id"), msg.get("kind"))
' HARNESS_DIR/topics.jsonl
```

Use the candidate whose `kind` matches the step: `appeared`, `disappeared`, or `moved`.

Sensor numbers are the `ChangeEvent.sensors` bit, and the coordinates are in that sensor's frame:

| number | sensor | cloud topic | status field |
|---|---|---|---|
| 1 | Mid-360 | `/livox/lidar` | `mid360_hz` |
| 2 | JT128 | `/lidar_points` | `jt128_hz` |

Copy `message.sensors`. Do not translate a Mid-360 box into sensor 2, or the other way around. The two frames are not one frame.

Syntax the harness accepts (`session.py` `parse_label`). Several clauses are separated by semicolons. Anything else is asked again. The recording stays up.

```text
region 1 0 0 0 1 1 1
region 1 5 0 0 6 1 1 from 2.5 0.5 0.5
event 4 7
skip
```

- `region SENSOR x0 y0 z0 x1 y1 z1` is the target aabb. Copy `message.aabb`, which is `[[x0,y0,z0],[x1,y1,z1]]`.
- `from x y z` is required for a move. Copy `message.from_centroid`. The scorer matches a move only when that point is within 0.50 m of the event's `from_centroid`.
- The scorer's intersection-over-union minimum is 0.05, after it pads each box by 0.10 m. Copying the aabb is the reliable label. A hand measurement can miss.
- `event GEN ID` is the log line's `publisher_generation` and `message.event_id`. Kind still has to match the scenario.
- A true detection is a `region` or an `event` association. A blank line or `skip` is an unattributed interval, or at best a temporal match. Either one withholds `full_candidate`.
- The first matching candidate is the true detection. A later candidate from the same sensor, including the next expiry, is a false alarm. The other sensor can corroborate that same target once.
- Person rows are different. Before `person_walk` and `person_sit`, `skip` or a person region is allowed. A region that overlaps a candidate there is the detector treating the person as a change, which is not what you want. Prefer `skip` unless you are deliberately marking the person.

The move-source label is not a coordinate. When the harness asks, type `verified`, then this line and nothing else:

```text
new_object reference_object other_reference_object
```

The third token is the move source (`reference_setup.move_source`). The physical MOVE object is the reference object you will move, not the one you place and not the one you remove. Any other line blocks the three change scenarios. The harness does not restart the node and does not change parameters. Write the tape names in your notes beside PLACE, REMOVE, and MOVE. Do not type the tape names on that line.

## 3. Tripod bump, then stop

Do this before the harness, with the scene matching the reference and nobody in view. One leg, 3-5 cm (PLAN-RIGGUARD E2), then put the leg back (E4). Do not tap only, twist, or carry the rig out on this visit.

```text
python3 scripts/room_acceptance_marks.py --live --out DIR
```

`--live` refuses any scenario other than `tripod_bump`. It does not run the protocol and it does not sleep the hour. `--bump-s` shortens the planned watch. The protocol duration in the file stays 30 s.

1. Press Enter at the push. That timestamp is the start. It is not rewritten when the watch ends.
2. Press Enter when the leg is back down, or wait for the deadline. Enter early records `shortened`. Waiting out the deadline records the restore at the deadline.
3. Answer the observation prompts with what you saw:

| prompt | tokens |
|---|---|
| rig_guard | `on` / `off` / `unknown` |
| auto_relearn | `on` / `off` / `unknown` |
| relearn_after_s | a number, or `unknown` |
| IMU | `yes` / `no` / `unknown` |
| map_epoch before and after | integers, or `unknown` |
| frame_epoch before and after | integers, `unshown`, or `unknown` |
| RIG_MOVED | `seen` / `not_seen` / `unknown` |
| recovery | `realigned` / `in_place` / `still_moved` / `relearning` / `unknown` |
| unexpected candidates | `none`, a short description, or `unknown` |

The harness status lines do not carry `map_epoch`. Read it from the display or from the node log line that prints `map_epoch` at startup. `frame_epoch` is on `RigPose` (`/room_memory/rig`), which PR #36 does not subscribe. If that value is not on the display, type `unshown`.

Recovery matches `_track_moved` in `spark/room_memory/room_memory/memory.py`. Agreeing fits inside the align envelope adopt the displaced pose and return the channel to normal (`REALIGNED`, or `IN_PLACE` when the increment is below the move threshold). That is a successful alignment at the displaced pose. It does not by itself change `map_epoch`. Relearning runs only while that recovery is still unresolved and `auto_relearn` is enabled, after `relearn_after_s`. Do not hold the rig out of place to force a relearn. If you did not see `auto_relearn` on the running node, type `unknown`. Do not fill in `true` from the launch file.

`bump_evaluation` is `observed` only when the guard was seen `on`, configuration and IMU were answered, both map epochs are integers, both frame epochs are integers or `unshown`, `RIG_MOVED` is `seen` or `not_seen`, recovery is one of the four outcomes, and the candidate note is not empty or `unknown`. Every other case is `not_evaluated`:

| reason | meaning |
|---|---|
| `guard_disabled` | you saw the guard off |
| `guard_unobserved` | you did not see whether it was on |
| `configuration_unobserved` | `auto_relearn`, `relearn_after_s`, or IMU was `unknown` |
| `evidence_missing` | moved, recovery, epoch, or candidate note was not answered |
| `rehearsal` | dry-run timing sheet |
| `interrupted` / `eof` / `aborted` | the bump watch or its questions did not finish |

Optional local files are summaries stored next to the answers. They do not become the answers and they do not flip `not_evaluated` to `observed`.

```text
python3 scripts/room_acceptance_marks.py --live --status PATH --topics PATH --recording PATH --out DIR
```

Each path is opened read-only. A missing path exits 2 and writes no marks. `--status` must be a JSON object. Copied keys, when present, are `state`, `events_seq`, `map_epoch`, `frame_epoch`, `rig_guard`, `auto_relearn`, `relearn_after_s`, `imu_ok`, `outcome`, and `publisher_generation`. `--topics` stores the path, the line count, and counts of candidate event lines, `rig_moved` status lines, and lines that mention `imu_ok` or `/livox/imu`. `--recording` stores the path, the byte size (a directory sums files one level down, not deeper), and whether `metadata.yaml` is present.

The script refuses to overwrite `scenario-marks.json` or `scenario-marks.csv`, refuses an input path that is one of those files, and refuses an `--out` directory that is the recording or sits inside it. Use a new directory for a second run.

If recovery was `relearning`, do not start the harness on the new epoch until a reference is frozen again and the three objects are checked again. If it realigned, confirm the display is back to `TRACKING` after the leg is home, and check the three objects again. The physical spots moved with the tripod until you put it back.

## 4. Scored session

Stop the marks script. On the machine whose `docker` already sees the container (no `--ssh`, no default container name):

```text
PYTHONPATH=spark/room_memory python3 -m acceptance run --transport ros2 --container NAME --out HARNESS_DIR
PYTHONPATH=spark/room_memory python3 -m acceptance score --log HARNESS_DIR/topics.jsonl --marks HARNESS_DIR/marks.json --out HARNESS_DIR
```

`run` subscribes, inside the container, to `/room_memory/status`, `/room_memory/events`, and `/room_memory/activity`. It does not create a publisher. Stopping it signals that child PID only. `run` writes the report when the session ends. `score` replays a saved log. Pass `HARNESS_DIR/marks.json`, never `scenario-marks.json`.

Do not put the tripod bump inside this session. Do not shorten `--hour-s` or `--sit-s` if you want `full_candidate`. `--watch-s` shortens the baseline and the three change watches only and also withholds full acceptance.

Harness order, after the reference questions:

| id | protocol | what you do |
|---|---|---|
| `baseline_idle` | 60 s | Room matches the reference. Nobody in view. Enter to start. |
| `box_placed` | 120 s | Place the PLACE object. Do not move the reference objects. |
| `box_removed` | 120 s | Remove the REMOVE object. Do not remove the object you just placed. Removing the placed object expires the appearance. It does not produce `DISAPPEARED`. |
| `box_moved` | 120 s | Move the MOVE object about 1 m, under 1.5 m. Leave the placed object where it is. |
| `person_walk` | 105 s | Walk through and leave. The watch includes time after you leave. |
| `person_sit` | 120 s | Sit still in view. A still person can be promoted after about 40 s. The watch is the full two minutes. |
| `restore_reference` | 60 s | Put every reference object back and remove the placed object. This interval is not an expected detection. `skip` leaves the scene unrestored, and the hour is then not the reference room. |
| `hour_idle` | 3600 s | Leave the room static. |

Each change has two Enters. The first is physical completion (box down, gone, or at the new pose). The second is the clear mark, after you step at least 1 m away. A person within 1 m restarts the roughly 40 s settle. The watch starts at the clear mark. Enter during a watch ends it early and records `shortened`.

After a change watch, type the label from the worksheet. The prompt is:

```text
Label the target: region SENSOR x0 y0 z0 x1 y1 z1 [from x y z] | event GEN ID | skip
```

Type `abort` to stop. Later scenarios are `skipped` and are not scored as misses. `person_sit` is two minutes. Do not walk the hour short.

## 5. What to keep

- Harness `--out`: `topics.jsonl`, `marks.json`, `report.json`, `report.md`. A killed hour still has the checkpoints written as each scenario finished.
- This script's `--out`: `scenario-marks.json`, `scenario-marks.csv`.
- Tape names for PLACE, REMOVE, and MOVE, and whether you typed the token line exactly.
- The bump answers, `bump_evaluation`, and `bump_evaluation_reason`.
- `map_epoch` before the bump, after the bump, and at the harness start. A relearn means the harness reference is a new epoch and the object check has to be done again.
- Whether the guard and the IMU were actually on.

## 6. How to read the score

Open `report.json`. Quote both `g5_acceptance` and `confirmed_product_bar`.

| exit | meaning |
|---|---|
| 0 | `g5_acceptance` is `full_candidate`. Reference confirmed, every protocol scenario complete and verifiable, three true detections, both person scenarios showed activity and no candidate, idle candidate alarms in verifiable time are at most `--max-false-alarms-per-hour` (default 1), and `dropped_windows` did not change from ready through `session_end`. |
| 1 | The protocol was complete and verifiable, and that candidate bar failed. |
| 2 | Not a pass. A shortened sit or hour, unverifiable coverage, a session that never became protocol-complete, or lost windows. `candidate_result` may say `inconclusive` or `fail`. `g5_acceptance` is `denied` either way. |
| 3 | `score` raised. The traceback is on stderr. A crash is not a scored fail. |

`full_candidate` does not mean Q3 passed. `confirmed_product_bar` is `not_evaluated` when no `CONFIRMED` message was published, and also when `CONFIRMED` messages exist but the static intervals are not a full verifiable baseline plus hour. Today's node emits `CANDIDATE`. `not_evaluated` means the false-CONFIRMED-per-hour bar was not measured. When the field is `pass` or `fail`, each `CONFIRMED` received during a verifiable static interval counted, including one whose candidate was published before the subscription. The candidate score's idle rate is `false_candidate_alarms_per_hour`. The product metric is `false_confirmed_alarms_per_hour`. Do not report one as the other.

A miss with activity somewhere in the window is `miss_with_activity_observed`. No activity on a person scenario is `person_unverified`. It does not prove the detector discriminated. `bump_evaluation` is not in this report. Say it separately, and say `not_evaluated` when that is the value.

## 7. Stop

Ctrl-C the marks script in its own terminal, or `kill` that PID. It writes the rows already taken. The push time stays. The restore time stays if you had already pressed Enter for it. An open watch is `completion` `aborted`, `session.end_reason` is `interrupted`, and the process exits 130. EOF exits 2. Typing `abort` exits 0 with the bump `not_evaluated`.

Stop the harness by its own PID. It already checkpoints. Do not send keystrokes to LabCommand or to any other app. Do not restart the node to clean up.
