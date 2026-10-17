# G5 physical acceptance runbook (D49, section 3.13)

This file is decision D49 and protocol section 3.13. `~/Developer/lc-tasks/NUMBERING.md` reserves that pair for this checkout. It is the operator procedure for one room visit. It does not add a message, a topic, or a threshold, and it does not amend `docs/ROOM-MEMORY-PROTOCOL.md`.

The scored session is the harness in `spark/room_memory/acceptance/` (PR #36). Main has that harness at the merge `5a51554`. The commands, label grammar, and exit codes below were read from commit `1225db5`, subject "Do not deny acceptance for a lifetime dropped-window count" (`README.md`, `session.py`, `score.py`). If that harness changes, re-read those files before the visit. This procedure does not edit them. This checkout contains that harness. Run it from this tree.

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

Do this in the room. The marks script does not deploy, restart, set a parameter, ssh, or run `docker exec`. It only reads local files.

`docker inspect` and `docker logs` are read-only. You run them on the machine whose docker already sees the container, and you redirect them to files on this Mac. The script does not run them.

Section 4 is a different process. `acceptance run` starts one subscribe-only `docker exec -i` inside the container. That exec belongs to the harness, not to the marks script. The bump capture in section 3 is the same kind of subscribe-only exec, in its own terminal, and it uses the same `bash -lc` setup prefix as `acceptance/transport.py`. It does not restart the node and it does not set a parameter.

- The room-memory container is already up. Leave it up.
- A display you already have (LabCommand Room, or the node log already on screen) is how you read `TRACKING` and `RIG_MOVED`. This script does not open a socket. LabCommand Room does not subscribe to `/room_memory/rig`, and the node log does not print `auto_relearn`, `relearn_after_s`, or `imu_ok`. Those three come from the files in section 3, not from the display.
- Both launch files default `rig_guard` to false, `auto_relearn` to true, `relearn_after_s` to 60, `imu_quiet_s` to 2, and `check_max_s` to 10. A default counts as an observation only when the inspect file names `room_memory.launch.py` or `room_memory_cpp.launch.py`, that argument is absent, and the command has no params file. Remembering the default does not count. If the command does not name one of those two files, an absent argument stays `unknown`.
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
    kind = msg.get("kind")
    print("kind", kind)
    print("event", row.get("publisher_generation"), msg.get("event_id"))
' HARNESS_DIR/topics.jsonl
```

Paste the `region` line or the `event` line. `event` is three tokens: `event`, the generation, and the id. The `kind` line is how you pick the candidate. Do not append the kind to the event line. The harness asks again when that line is not exactly three tokens.

Use the candidate whose `kind` matches the step: `appeared`, `disappeared`, or `moved`.

Sensor numbers are the `ChangeEvent.sensors` bit, and the coordinates are in that sensor's frame:

| number | sensor | cloud topic | status field |
|---|---|---|---|
| 1 | Mid-360 | `/livox/lidar` | `mid360_hz` |
| 2 | JT128 | `/lidar_points` | `jt128_hz` |

Copy `message.sensors`. Do not translate a Mid-360 box into sensor 2, or the other way around. The two frames are not one frame.

`message.sensors` is a bitmask. 3 means both sensors. Do not paste region 3. Split it into region 1 and region 2, or use event GEN ID. The one-liner prints the integer as-is, so a both-sensor event comes out as `region 3 …`. `parse_label` accepts any integer, including 3. The scorer matches a region only when that number equals `message.sensors`, so `region 1` does not match an event whose sensors value is 3. For that event, paste the `event` line. It does not use the bitmask.

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
- `event GEN ID` is the log line's `publisher_generation` and `message.event_id`. Kind still has to match the scenario. Kind is not a fourth token on that line.
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

Start the capture in the next paragraphs before this script. `--live` refuses any scenario other than `tripod_bump`. It does not run the protocol and it does not sleep the hour. `--bump-s` shortens the planned watch. The protocol duration in the file stays 30 s. The command, with the three local files, is after the capture.

1. Press Enter at the push. That timestamp is the start. It is not rewritten when the watch ends.
2. Press Enter when the leg is back down. That is the normal path and records `completion` `complete`, including when it is sooner than 30 s. If the deadline passes with no Enter, `restore_mono_ns` and `wall_end_utc` stay empty and `completion` is `restore_unmarked`. The script does not fill those fields with the deadline. The questions still follow. Finishing them exits 0. `restore_unmarked` is not a harness exit code.
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

Type `unknown` for a field you are not copying from the source named below. A value you type is kept, so a remembered launch default typed by hand is recorded as if you had seen it. Type `unknown` and let the file fill the field.

| field | what counts as an observation |
|---|---|
| `rig_guard` | A node-log line `rig guard: N ICP estimates` means the guard is on. Both nodes print that line only while the guard is running (`node.py`, `room_memory_node.cpp`). Seeing it on screen and typing `on` counts. Passing the saved log as `--log` counts, including when inspect would have said off. Absence of the line does not mean off. Off counts only from the inspect command (`rig_guard:=false`, or the launch default when inspect shows that file and no override). |
| `auto_relearn`, `relearn_after_s` | The inspect command. An explicit `name:=value` is source `command`. If inspect names `room_memory.launch.py` or `room_memory_cpp.launch.py` and does not pass that argument or a params file, the launch default counts and is source `launch_default` (`auto_relearn` on, `relearn_after_s` 60). No other source has these values. The startup log does not print them. |
| IMU | `imu_ok` on `/room_memory/rig` in the topics capture below. `true` is `yes`, `false` is `no`. PR #36's `topics.jsonl` does not subscribe to that topic. On that file `imu_lines` is 0 and `imu_subscribed` is false. That zero is not an observation that the IMU is off. |
| `map_epoch` before and after | Two or more `map_epoch` values on `/room_memory/status` in the topics capture (or on the rig lines, if status has fewer than two). The first is before, the last is after. One sample does not fill both. The startup log prints `map_epoch` once and is not the after-bump value. Harness status lines do not carry `map_epoch`. |
| `frame_epoch` before and after | Two or more `frame_epoch` values on `/room_memory/rig`. The capture replaces a typed `unshown`. Without a rig capture, `unshown` is the answer when the display does not show it. PR #36 does not subscribe to `/room_memory/rig`. |
| `RIG_MOVED` | A status line `state` `rig_moved`, or a rig `outcome` of `moved`, in the topics capture. Two or more status samples, or two or more rig outcomes, with none of those, is `not_seen`. The window log's `state RIG_MOVED` is the same fact if you saw it and type `seen`. |
| recovery | The last rig `outcome` in the capture: `realigned`, `in_place`, `relearning`, or `moved` (stored as `still_moved`). `checking` and `inconclusive` do not fill it. |
| unexpected candidates | The events topic in the capture. Zero candidate lines is `none`. A count is the note. The harness log is not this capture. |

These files count. A visit that passes them and types `unknown` can finish `bump_evaluation` `observed`. A visit that does not have them stays `not_evaluated`. That is not a harness exit code and not a failed guard test.

Save the two process files any time the container is up. They do not describe the push. The topics file does, so it has to keep running across the push and be read after you close it.

```text
docker inspect NAME > inspect.json
docker logs NAME > node.log
```

Start the topics capture before you start the marks script, so the file exists. In another terminal, on the machine whose docker already sees the container. `docker exec` does not run the image entrypoint, and the entrypoint is what sources ROS. A bare `python3` raises `ModuleNotFoundError` on `labcommand_msgs` (and on `rclpy`). The traceback stays on the terminal and `bump-topics.jsonl` stays empty. Use the same prefix as the harness (`acceptance/transport.py`): `bash -lc` with `source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && python3 -u -`.

```text
docker exec -i NAME bash -lc 'source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && python3 -u -' > bump-topics.jsonl <<'PY'
import json, sys, time
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from labcommand_msgs.msg import ChangeEvent, MemoryStatus, RigPose

STATE = {0: "building", 1: "tracking", 2: "rig_moved", 3: "degraded", 4: "paused"}
OUTCOME = {0: "in_place", 1: "checking", 2: "moved", 3: "realigned", 4: "relearning", 5: "inconclusive"}
EVENT = {0: "candidate", 1: "confirmed", 2: "accepted", 3: "reverted", 4: "expired"}
TOPICS = ["/room_memory/status", "/room_memory/events", "/room_memory/rig"]

def latched():
    return QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)

def volatile():
    return QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)

def emit(payload):
    stamped = dict(payload)
    stamped["mono_ns"] = time.monotonic_ns()
    stamped["wall_ns"] = time.time_ns()
    sys.stdout.write(json.dumps(stamped, separators=(",", ":")) + "\n")
    sys.stdout.flush()

class Dropped:
    def publish(self, *_a, **_k):
        return None

class Reader(Node):
    def __init__(self):
        super().__init__(
            "room_memory_bump_capture",
            enable_rosout=False,
            start_parameter_services=False,
            enable_logger_service=False,
            parameter_overrides=[
                Parameter("use_sim_time", Parameter.Type.BOOL, False),
                Parameter("start_type_description_service", Parameter.Type.BOOL, False),
            ],
        )
        self.subs = []
        self.subs.append(self.create_subscription(MemoryStatus, "/room_memory/status", self.on_status, latched()))
        self.subs.append(self.create_subscription(ChangeEvent, "/room_memory/events", self.on_event, volatile()))
        self.subs.append(self.create_subscription(RigPose, "/room_memory/rig", self.on_rig, latched()))

    def create_publisher(self, *_a, **_k):
        return Dropped()

    def create_client(self, *_a, **_k):
        raise RuntimeError("bump capture does not create clients")

    def create_service(self, *_a, **_k):
        raise RuntimeError("bump capture does not create services")

    def on_status(self, message):
        emit({"topic": "/room_memory/status", "message": {
            "state": STATE.get(int(message.state), str(int(message.state))),
            "map_epoch": int(message.map_epoch),
        }})

    def on_event(self, message):
        emit({"topic": "/room_memory/events", "message": {
            "state": EVENT.get(int(message.state), str(int(message.state))),
            "event_id": int(message.event_id),
        }})

    def on_rig(self, message):
        emit({"topic": "/room_memory/rig", "message": {
            "imu_ok": bool(message.imu_ok),
            "frame_epoch": int(message.frame_epoch),
            "map_epoch": int(message.map_epoch),
            "outcome": OUTCOME.get(int(message.outcome), str(int(message.outcome))),
        }})

def main():
    rclpy.init()
    node = Reader()
    try:
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline and not all(sub.get_publisher_count() > 0 for sub in node.subs):
            rclpy.spin_once(node, timeout_sec=0.2)
        emit({"op": "ready", "topics": list(TOPICS)})
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

main()
PY
```

That process only subscribes. It turns rosout, the parameter services, and the logger service off, and it does not advertise `/parameter_events`. Every line carries `mono_ns` (`time.monotonic_ns()`, the container clock) and `wall_ns` (`time.time_ns()`, UTC nanoseconds). Before and after in the marks file are still the first and last sample. Use `mono_ns` deltas inside this file for RIG_MOVED timing and relearn delay. Set `wall_ns` next to the marks file's `wall_start_utc` and `wall_end_utc` (those are whole seconds). `mono_ns` here is not the Mac `push_mono_ns`. Stop the capture by Ctrl-C in its own terminal, or `kill` that PID. Do not kill the container. Do not send keystrokes to LabCommand or to any other app. If the guard is off there is no `/room_memory/rig` publisher; the capture still records status and events, and IMU stays `unknown`.

Leave the capture running through the push and the restore. After the leg is back, stop the capture, rewrite `node.log` if you want the log from this visit (`docker logs NAME > node.log`), then press Enter at the script's close prompt. The script reads the files then. It does not read them before the push, and a dry-run read is not a bump observation. `read_after_bump` on the stored summary is true only for that post-close read. `applied_to_bump` is true only for a source that filled an answer.

```text
python3 scripts/room_acceptance_marks.py --live --inspect inspect.json --log node.log --topics bump-topics.jsonl --out DIR
```

`--status` is still a local JSON object, read only. Nothing in this repo writes `rig_guard`, `auto_relearn`, `relearn_after_s`, or `imu_ok` into one, so it is not the visit path. A single object also cannot supply both epochs. `--recording` stores the path, the byte size (a directory sums files one level down, not deeper), and whether `metadata.yaml` is present. That summary is not a bump observation.

The script refuses to overwrite `scenario-marks.json` or `scenario-marks.csv`, refuses an input path that is one of those files, and refuses an `--out` directory that is the recording or sits inside it. Use a new directory for a second run. A missing input path exits 2 and writes no marks. The topics file may be empty when the script starts and grow while the capture runs. The post-close read is the one that is stored. When `--topics` is set, that read writes `read_error` and exits 2 if the file has no lines or no `op: ready` line. The marks files are still written. Exit 2 here means the capture did not start. It is not a harness exit code, and it is not a passed bump, even when the typed answers would have been `observed`. A file that disappears after startup still sets `read_error`, still writes the marks, and keeps the bump's own exit code.

Recovery matches `_track_moved` in `spark/room_memory/room_memory/memory.py`. Agreeing fits inside the align envelope adopt the displaced pose and return the channel to normal (`REALIGNED`, or `IN_PLACE` when the increment is below the move threshold). That is a successful alignment at the displaced pose. It does not by itself change `map_epoch`. Relearning runs only while that recovery is still unresolved and `auto_relearn` is enabled, after `relearn_after_s`. Do not hold the rig out of place to force a relearn.

`bump_evaluation` is `observed` only when the guard was seen `on`, configuration and IMU were answered, both map epochs are integers, both frame epochs are integers or `unshown`, `RIG_MOVED` is `seen` or `not_seen`, recovery is one of the four outcomes, and the candidate note is not empty or `unknown`. Every other case is `not_evaluated`. `observed` means the evidence is complete. It is not a pass.

### Passed bump

A bump passes only when bump_evaluation is observed, RIG_MOVED is seen, recovery is realigned, map_epoch is unchanged, and unexpected candidates are none. `observed` alone is not a pass. Anything else is not a pass. Say the bump separately from `g5_acceptance`, and say which of these failed.

`not_evaluated` cases:

| reason | meaning |
|---|---|
| `guard_disabled` | you saw the guard off |
| `guard_unobserved` | you did not see whether it was on |
| `configuration_unobserved` | `auto_relearn`, `relearn_after_s`, or IMU was `unknown` |
| `evidence_missing` | moved, recovery, epoch, or candidate note was not answered |
| `rehearsal` | dry-run timing sheet |
| `interrupted` / `eof` / `aborted` | the bump watch or its questions did not finish |

Do not point `--topics` at the harness `topics.jsonl`. That log is not the bump, and the script will not treat it as one.

If recovery was `relearning`, do not start the harness on the new epoch until a reference is frozen again and the three objects are checked again. If it realigned, confirm the display is back to `TRACKING` after the leg is home, and check the three objects again. The physical spots moved with the tripod until you put it back.

## 4. Scored session

Stop the marks script. On the machine whose `docker` already sees the container (no `--ssh`, no default container name). The harness runs `docker exec -i NAME` of a process that only subscribes, with the same `bash -lc` setup prefix as the bump capture. The marks script is not part of that, and it never execs. The bump capture is a separate exec and it has already been stopped.

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
| 0 | `g5_acceptance` is `full_candidate`. Reference confirmed, every protocol scenario complete and verifiable, three true detections, both person scenarios showed activity and no candidate, idle candidate alarms in verifiable time are at most `--max-false-alarms-per-hour` (default 1), and `session_dropped_windows` is zero. |
| 1 | The protocol was complete and verifiable, and that candidate bar failed. |
| 2 | Not a pass. A shortened sit or hour, unverifiable coverage, a session that never became protocol-complete, or lost windows (`session_dropped_windows` nonzero, or a scenario in `lost_window_scenarios`). `candidate_result` may say `inconclusive` or `fail`. `g5_acceptance` is `denied` either way. |
| 3 | `score` raised. The traceback is on stderr. A crash is not a scored fail. |

`session_dropped_windows` is the change in `dropped_windows` from ready through `session_end`, per publisher generation. The baseline is the last status at or before ready, or that generation's first status at or after ready when the subscriber records ready before the latched status. A lifetime count already on that baseline is not a drop during the session. The counter is measured from 0 only for a generation that first appears after an earlier generation was already seen, which is a restart during the session. A long-running node's lifetime total does not by itself exit 2.

`full_candidate` does not mean Q3 passed. `confirmed_product_bar` is `not_evaluated` when no `CONFIRMED` message was published, and also when `CONFIRMED` messages exist but the static intervals are not a full verifiable baseline plus hour. Today's node emits `CANDIDATE`. `not_evaluated` means the false-CONFIRMED-per-hour bar was not measured. When the field is `pass` or `fail`, each `CONFIRMED` received during a verifiable static interval counted, including one whose candidate was published before the subscription. The candidate score's idle rate is `false_candidate_alarms_per_hour`. The product metric is `false_confirmed_alarms_per_hour`. Do not report one as the other.

A miss with activity somewhere in the window is `miss_with_activity_observed`. No activity on a person scenario is `person_unverified`. It does not prove the detector discriminated. `bump_evaluation` is not in this report. Say it separately. A bump passes only when bump_evaluation is observed, RIG_MOVED is seen, recovery is realigned, map_epoch is unchanged, and unexpected candidates are none. Say `not_evaluated` when that is the value. `observed` alone is not a pass.

## 7. Stop

Ctrl-C the marks script in its own terminal, or `kill` that PID from another terminal. `kill` sends SIGTERM. Ctrl-C sends SIGINT. Both write the rows already taken and exit 130. The push time stays. The restore time stays if you had already pressed Enter for it. Answers you had already accepted stay, including when the next question is still open. An open watch, with no restore yet, is `completion` `aborted` and `session.end_reason` is `interrupted`. A deadline that passed with no Enter stays `restore_unmarked`, with the restore time empty. Stopping while the handoff text is printing exits 130 and keeps that finished bump. A stop during the post-close read writes the rows already taken and exits 130. Evidence that read had not finished filling stays as the answers already accepted. A stop while the two files are being written is held until both files are in place, then the process exits 130. The stop is delayed, not dropped, and the exit code does not stay 0. If the stop lands after both files are already written, the process still exits 130. It does not report that the output already exists.

`kill -9` stops the process without writing the file. Do not use it.

EOF exits 2. Typing `abort` exits 0 with the bump `not_evaluated`. The answers accepted before `abort` stay in the file.

Stop the harness by its own PID. It already checkpoints. Do not send keystrokes to LabCommand or to any other app. Do not restart the node to clean up.
