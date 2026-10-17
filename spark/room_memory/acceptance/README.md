# Room-memory physical acceptance

Operator script, read-only recording, and an offline score for a live room-memory node.
The score is a candidate score. It is not the CONFIRMED product bar in
`docs/phase3-live-integration.md` §7.5 / Q3 (false CONFIRMED events per hour in a
static room, proposed ≤ 1/h). Today's node emits `CANDIDATE`. A run with no
`CONFIRMED` message leaves `confirmed_product_bar` as `not_evaluated`. When
`CONFIRMED` messages are present on a complete verifiable run, each one received
during a static interval counts toward that rate. The candidate does not have to
be in the log. Events are volatile, so a candidate can be published before the
subscription starts. That confirmation is still a static-room alarm. It is not
excused by the missing line, and it is not a zero-false-alarm pass.

This command does not ssh, publish, or restart anything. It does not dial the
Spark while the tests run. The operator runs it later, on the machine whose
`docker` can already see the room-memory container.

## Still deferred

`evaluate.py`, the `acceptance_replay` ctest, and the `bag_stream.py` flags
(`--rig-move`, `--imu-synth`, `--drop`, `--strip-imu`) are still deferred.
This harness does not add them and does not run them.

## Run

From the repository root. `acceptance/__main__.py` does not edit `sys.path`;
Python has to find the package first.

```text
PYTHONPATH=spark/room_memory python3 -m acceptance score --log DIR/topics.jsonl --marks DIR/marks.json --out DIR
PYTHONPATH=spark/room_memory python3 -m acceptance run --transport ros2 --container NAME --out DIR
```

`run` has no default container and no `--ssh`. The only transport is `ros2`:
`docker exec -i NAME` starts `python3 -u -` inside the container and feeds it
`ros2_subscribe.py` on stdin. That process subscribes to `/room_memory/status`,
`/room_memory/events`, and `/room_memory/activity`. Status and activity are
reliable transient_local depth 1. Events are reliable volatile depth 100.
It does not create a publisher, a client, or a service. Jazzy's `Node`
constructor would otherwise advertise `/parameter_events`, enable rosout, start
the parameter services, and publish parameter events while it declares
`use_sim_time` and `start_type_description_service`. The subscriber turns rosout,
the parameter services, and the logger service off, forces those two parameters
off, and does not advertise `/parameter_events`. Stopping the harness signals
that child PID only.

`--scenarios` is a comma-separated subset. `--watch-s` shortens the baseline and
the three change watches. `--sit-s`, `--walk-s`, and `--hour-s` shorten those
watches only. The protocol durations stay 60 s, 120 s, 105 s, 120 s, and 3600 s.
A shorter plan is a diagnostic. It cannot receive full G5 acceptance.

Tests:

```text
PYTHONPATH=spark/room_memory python3 -m pytest spark/room_memory/acceptance/tests -q
```

## What the operator does

The detector compares each window with the frozen reference, not the previous
frame. The script is the bench arrangement: place a new object, remove a
different object that is already in the reference, and move a third reference
object about 1 m (under 1.5 m). Removing the object you just placed restores
the reference and expires the appearance. It does not produce `DISAPPEARED`.

`TRACKING` does not prove the reference. The operator types `verified` only
after checking the reference itself, then the line

```text
new_object reference_object other_reference_object
```

Anything else blocks the three change scenarios. The tool does not restart the
node and does not change parameters.

Each change has two Enter marks. The first is physical completion (the box is
down, gone, or at the new pose). The second is the clear mark, after stepping
at least 1 m away. A person within 1 m restarts the roughly 40 s settle. Enter
during the watch ends it early and records `shortened`.

After a change watch, label the target in that sensor's frame:

```text
region 1 0 0 0 1 1 1
region 1 5 0 0 6 1 1 from 2.5 0.5 0.5
event 4 7
skip
```

Kind plus time without a label is a temporal match, not a true detection.
Another sensor is corroboration only when it matches that sensor's own target.
That sensor can corroborate a labeled target once. A later candidate from the
same sensor, including the next expiry cycle, is a false alarm.
Separate sensor frames are not compared. `skip` withholds full acceptance.
A line that is not `region`, `event`, or `skip` is asked again. The recording
stays up. Ctrl-C stops the script and still writes `topics.jsonl`, `marks.json`,
and the report. The open scenario is `aborted`, the session is `interrupted`,
and scenarios that had not started are not scored as misses. Those files are
also written as each scenario finishes, so a killed hour still leaves a capture.

`person_sit` is two minutes. `hour_idle` is one hour. `restore_reference` puts
the reference scene back. It is not an expected detection. Type `abort` to stop;
later scenarios are `skipped`.

## Clocks and coverage

Every topic line keeps three times: `receipt_mono_ns` (parent monotonic clock,
when the line was read), `callback_mono_ns` (container monotonic clock), and
`stamp_ns` (publisher header). Marks use the parent clock. Detection delay is
event receipt minus the physical-completion mark. It is labeled
`receipt_mono`. A publisher stamp is not subtracted from that mark. A latched
status is not a sync sample. An offset is reported only from fresh 1 Hz
samples whose stamp delta matches the receipt delta, and it is not applied.

A scenario is verifiable only when the subscription was matched before it
started, fresh status and activity cover the interval with no header-seq gap
and no gap over 2.5 s, expected sensors report nonzero Hz, the state is
`tracking` or `degraded`, `MemoryStatus.dropped_windows` does not increase
across the interval's status samples, and every event seq through
`MemoryStatus.events_seq` on the closing status is in the log, including that
tail. `events_seq` is stored on every status line. Gaps, a missing tail, sensor
dropout, dropped windows, a decode error, and a log that ends before the
interval are unverifiable.

Nonzero Hz does not prove the detector ran: both nodes count every received
frame toward Hz, rejected frames included, and drop a window whose frames were
all rejected. An hour of `degraded` status at 10 Hz can hold no processed window.
The counter is per node process and counts since its start. Each publisher
generation is its own run.

Inside one scenario, the baseline is the last status sample at or before `t0`.
Only the increase after that baseline is new. If the interval has no status
sample at or before `t0`, the first sample inside it is not the baseline: the
count is measured from 0, so a drop already stored on that sample is not missed.
A generation that first appears inside the interval after an earlier one is a
restart, so its count starts at 0. A steady nonzero count is history when a
sample at or before `t0` already shows it. A restart's reset to 0 is not a
decrease. An increase is `dropped_windows`; a decrease within a generation
(`dropped_windows_regressed`) and a missing count (`dropped_windows_unknown`)
are unverifiable too. That time is not exposure and is not a zero-false-alarm
pass. The report lists those windows per scenario and in `dropped_windows`, and
names the scenarios in `lost_window_scenarios`. Any such scenario denies full
acceptance, `restore_reference` included. This is counted even when a restart
or a gap already makes the interval unverifiable.

The same counter is also read across the whole session, from subscription ready
through `session_end`, per publisher generation. A nonzero change denies full
acceptance. That includes a drop in a gap between scenarios and a drop during
setup, before the first watch. Those drops stay out of the neighboring
scenarios, so the scenarios keep their exposure. A steady historical count, and
a restart's reset to 0, still do not count. The session total is
`session_dropped_windows`. `session_end` is in the log so an offline score sees
the same interruption.

The recorder stays up after the last watch until a fresh status sample and a
fresh activity sample have both arrived at or after that watch, and until the
event seq on that status is in the log. It stops waiting 3.5 s after the watch:
the 2.5 s coverage gap, plus 1 s for the event line that follows the status.
Stopping at the watch would leave a normal 1 Hz stream as `missing_tail` and
drop the hour. Samples that arrive during a later label are already in the log,
so this wait is usually the last watch only.

## Score

`score` is a pure function of the log and the marks. Exit 0 means
`g5_acceptance` is `full_candidate`: reference confirmed, every protocol
scenario complete and verifiable, three true detections, both person scenarios
showed activity and no candidate, idle candidate alarms per verifiable
hour are at most `--max-false-alarms-per-hour` (default 1), and
`dropped_windows` did not change from ready through `session_end`. Exit 1 is a
complete session that fails that candidate bar. Exit 2 is inconclusive or
diagnostic, including a shortened sit or hour and a session that lost windows
outside a scenario. Exit 3 means `score` raised. The traceback is on stderr.
Exit 3 is not exit 1: a crash is not a scored fail.

A miss with activity somewhere in the window is `miss_with_activity_observed`.
`change_as_activity` is only activity that overlaps the labeled target after
the clear mark. No activity on a person scenario is `person_unverified`. It
does not prove the detector discriminated. Global activity going quiet is not
the spatial persistence gate.

`report.json` and `report.md` are written to `--out`. The markdown is generated
from the JSON numbers.
