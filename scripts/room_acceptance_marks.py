#!/usr/bin/env python3
"""Rehearse the G5 room visit and record a tripod-bump mark file.

The scored session is PR #36 (``python3 -m acceptance``). This process does not
deploy, restart, publish, or open a connection. It does not ssh. Default mode
is an offline dry-run. ``--live`` only marks the tripod bump.

Imports stay on the stdlib list in ``test_room_acceptance_marks.py``.
"""

import argparse
import csv
import json
import select
import signal
import sys
import time
import weakref
from datetime import datetime, timedelta, timezone
from pathlib import Path

SCHEMA = "scenario-marks-v1"
JSON_NAME = "scenario-marks.json"
CSV_NAME = "scenario-marks.csv"
HARNESS_SCHEMA = "room-memory-acceptance-marks-v1"
SYNTHETIC_START = datetime(2026, 1, 1, tzinfo=timezone.utc)
NS = 1_000_000_000

OBSERVATION_KEYS = (
    "rig_guard",
    "auto_relearn",
    "relearn_after_s",
    "imu",
    "map_epoch_before",
    "map_epoch_after",
    "frame_epoch_before",
    "frame_epoch_after",
    "rig_moved",
    "recovery",
    "unexpected_candidates",
)

STATUS_KEYS = (
    "state",
    "events_seq",
    "map_epoch",
    "frame_epoch",
    "rig_guard",
    "auto_relearn",
    "relearn_after_s",
    "imu_ok",
    "outcome",
    "publisher_generation",
)

# Both launch files declare these. They are an observation of this process only when
# docker inspect (or the command the operator saved) names that file and does not
# override the argument or pass a params file. A remembered default is not enough.
LAUNCH_DEFAULTS = {
    "room_memory.launch.py": {"rig_guard": "off", "auto_relearn": "on", "relearn_after_s": 60},
    "room_memory_cpp.launch.py": {"rig_guard": "off", "auto_relearn": "on", "relearn_after_s": 60},
}
_LAUNCH_FILES = ("room_memory_cpp.launch.py", "room_memory.launch.py")
_HARNESS_TOPICS = {"/room_memory/status", "/room_memory/events", "/room_memory/activity"}
_CONFIG_KEYS = ("rig_guard", "auto_relearn", "relearn_after_s")

CSV_COLUMNS = (
    "id",
    "class",
    "kind",
    "driver",
    "protocol_duration_s",
    "planned_duration_s",
    "actual_duration_s",
    "completion",
    "t0_mono_ns",
    "t1_mono_ns",
    "change_done_mono_ns",
    "clear_mono_ns",
    "push_mono_ns",
    "restore_mono_ns",
    "bump_evaluation",
    "wall_start_utc",
    "wall_end_utc",
)

# Visit order. The bump is finished before PR #36 starts. Harness rows are a
# timing sheet in a dry-run; --live does not conduct them.
PROCEDURE = (
    {"id": "preflight", "class": "setup", "kind": None, "protocol_s": 0, "driver": "room_acceptance_marks"},
    {"id": "tripod_bump", "class": "rig", "kind": None, "protocol_s": 30, "driver": "room_acceptance_marks"},
    {"id": "harness_handoff", "class": "handoff", "kind": None, "protocol_s": 0, "driver": "room_acceptance_marks"},
    {"id": "reference_setup", "class": "setup", "kind": None, "protocol_s": 0, "driver": "harness"},
    {"id": "baseline_idle", "class": "idle", "kind": None, "protocol_s": 60, "driver": "harness"},
    {"id": "box_placed", "class": "change", "kind": "appeared", "protocol_s": 120, "driver": "harness"},
    {"id": "box_removed", "class": "change", "kind": "disappeared", "protocol_s": 120, "driver": "harness"},
    {"id": "box_moved", "class": "change", "kind": "moved", "protocol_s": 120, "driver": "harness"},
    {"id": "person_walk", "class": "activity", "kind": None, "protocol_s": 105, "driver": "harness"},
    {"id": "person_sit", "class": "activity", "kind": None, "protocol_s": 120, "driver": "harness"},
    {"id": "restore_reference", "class": "restoration", "kind": None, "protocol_s": 60, "driver": "harness"},
    {"id": "hour_idle", "class": "idle", "kind": None, "protocol_s": 3600, "driver": "harness"},
)

WORKSHEET = """\
Label worksheet. Resolve it before the scored session. skip does not make a true detection.
Sensor 1 is the Mid-360 (/livox/lidar, mid360_hz). Sensor 2 is the JT128 (/lidar_points, jt128_hz).
A region line is in that sensor's frame. Copy message.sensors from the harness topics.jsonl line.
message.sensors is a bitmask. 3 means both sensors. Do not paste region 3. Paste event GEN ID.
Do not reuse one sensor's box for the other sensor.
  region SENSOR x0 y0 z0 x1 y1 z1 [from x y z]
  region 1 0 0 0 1 1 1
  region 1 5 0 0 6 1 1 from 2.5 0.5 0.5
  event GEN ID
  skip
GEN is publisher_generation on the log line. ID is message.event_id. Several clauses are separated by semicolons.
Copy aabb and from_centroid. The scorer wants IoU at least 0.05 after a 0.10 m pad, and a move's from point within 0.50 m.
When the harness asks, type verified, then exactly:
  new_object reference_object other_reference_object
The third token is the move source. Any other line blocks the three change scenarios.
"""

RECOVERY_TEXT = (
    "A successful alignment recovers at the displaced pose and does not by itself change map_epoch. "
    "Relearning runs only while recovery is still unresolved and auto_relearn is enabled. "
    "Type unknown when you did not see the value. A launch default you remember is not an observation. "
    "A bump passes only when bump_evaluation is observed, RIG_MOVED is seen, recovery is realigned, "
    "map_epoch is unchanged, and unexpected candidates are none. observed alone is not a pass."
)

HANDOFF_TEXT = """\
HANDOFF. Stop this script. Needs Trevor. The room-memory container name is room-memory. Read docker ps --format '{{.Names}} {{.Image}}' first. Never name a lidar or driver container. lidar-center, lidar-dual, and ptp-master must never be named.
On the machine whose docker already sees the container, start PR #36.
Do not pass scenario-marks.json to acceptance score. The clocks are not the same clock.
PYTHONPATH=spark/room_memory python3 -m acceptance run --transport ros2 --container room-memory --out HARNESS_DIR
PYTHONPATH=spark/room_memory python3 -m acceptance score --log HARNESS_DIR/topics.jsonl --marks HARNESS_DIR/marks.json --out HARNESS_DIR
"""

CLOSE_CAPTURE = (
    "Press Enter once the local capture files are closed. "
    "They are read now and count as the bump observation.\n"
)

OBSERVATION_TEXT = (
    "Type unknown unless you are copying the named source. A launch default you remember is unknown. "
    "rig_guard on: a node-log line 'rig guard:' with 'ICP estimates', or the inspect command. "
    "rig_guard off: the inspect command only. "
    "auto_relearn and relearn_after_s: the inspect command, or the launch file it names when that argument is absent. "
    "IMU, both epochs, RIG_MOVED, recovery, and candidates: the topics capture, read after you close it.\n"
)

ROW_PROMPTS = {
    "preflight": "Preflight. Container already up. Do not deploy, restart, or change parameters.\n" + WORKSHEET,
    "tripod_bump": (
        "Tripod bump, before the scored session. Push one leg 3-5 cm, then put it back. "
        "Enter at the push and Enter when the leg is back. That second Enter is complete, "
        "including when it is sooner than 30 s. A deadline with no Enter leaves the restore "
        "empty and records restore_unmarked.\n" + RECOVERY_TEXT
    ),
    "harness_handoff": HANDOFF_TEXT.strip(),
    "reference_setup": (
        "Harness reference setup. Type verified only after checking the frozen reference, then the token line. "
        "TRACKING does not prove the reference."
    ),
    "baseline_idle": "Harness: clear the room to the reference. Protocol watch 60 s. Nobody in view.",
    "box_placed": "Harness: place a NEW object on a spot that is empty in the reference. Two Enters, then the label.",
    "box_removed": "Harness: remove the reference object named for removal. Do not remove the object you just placed.",
    "box_moved": "Harness: move the other reference object about 1 m and stay under 1.5 m. Label from x y z.",
    "person_walk": "Harness: walk through and leave. skip or a person region is allowed here. Protocol 105 s.",
    "person_sit": "Harness: sit still in view for the full two minutes. A shorter watch is not full acceptance.",
    "restore_reference": "Harness: put every reference object back and remove the placed object before the hour.",
    "hour_idle": "Harness: leave the room static for one hour. Enter during the watch withholds full acceptance.",
}


class _Eof(Exception):
    pass


class _Abort(Exception):
    pass


class ManualClock:
    """Monotonic clock that moves only when a test calls ``advance``."""

    def __init__(self, start_ns=0):
        self._ns = int(start_ns)

    def monotonic_ns(self):
        return self._ns

    def advance(self, seconds):
        self._ns += int(round(float(seconds) * NS))


def wall_synthetic(ns):
    seconds, rem = divmod(int(ns), NS)
    moment = SYNTHETIC_START + timedelta(seconds=seconds, microseconds=rem // 1000)
    return moment.strftime("%Y-%m-%dT%H:%M:%SZ")


def wall_utc(_ns):
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def _as_stored(value):
    number = float(value)
    if number.is_integer():
        return int(number)
    return number


def _finite_seconds(text):
    try:
        value = float(text)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("duration must be a number") from exc
    if value < 0 or value != value or value in (float("inf"), float("-inf")):
        raise argparse.ArgumentTypeError("duration must be finite and >= 0")
    return value


def resolve_durations(watch_s, sit_s, walk_s, hour_s, bump_s):
    """Planned seconds. Protocol durations stay on ``PROCEDURE`` and are not changed here."""
    return {
        "baseline_idle": _as_stored(60 if watch_s is None else watch_s),
        "box_placed": _as_stored(120 if watch_s is None else watch_s),
        "box_removed": _as_stored(120 if watch_s is None else watch_s),
        "box_moved": _as_stored(120 if watch_s is None else watch_s),
        "person_walk": _as_stored(105 if walk_s is None else walk_s),
        "person_sit": _as_stored(120 if sit_s is None else sit_s),
        "hour_idle": _as_stored(3600 if hour_s is None else hour_s),
        "restore_reference": 60,
        "tripod_bump": _as_stored(30 if bump_s is None else bump_s),
        "preflight": 0,
        "harness_handoff": 0,
        "reference_setup": 0,
    }


def empty_observations():
    return {key: None for key in OBSERVATION_KEYS}


def _is_number(value):
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and value == value
        and value not in (float("inf"), float("-inf"))
    )


def _is_int(value):
    return isinstance(value, int) and not isinstance(value, bool) and value >= 0


def _frame_ok(value):
    return value == "unshown" or _is_int(value)


def judge_bump(observations):
    """``observed`` only when the operator actually saw a guard-on configuration and the evidence.

    Launch defaults are not filled in. ``off`` and ``unknown`` are not a harness score.
    """
    obs = observations or {}
    guard = obs.get("rig_guard")
    if guard == "off":
        return "not_evaluated", "guard_disabled"
    if guard != "on":
        return "not_evaluated", "guard_unobserved"
    if obs.get("auto_relearn") not in ("on", "off"):
        return "not_evaluated", "configuration_unobserved"
    if not _is_number(obs.get("relearn_after_s")):
        return "not_evaluated", "configuration_unobserved"
    if obs.get("imu") not in ("yes", "no"):
        return "not_evaluated", "configuration_unobserved"
    if obs.get("rig_moved") not in ("seen", "not_seen"):
        return "not_evaluated", "evidence_missing"
    if obs.get("recovery") not in ("realigned", "in_place", "still_moved", "relearning"):
        return "not_evaluated", "evidence_missing"
    if not _is_int(obs.get("map_epoch_before")) or not _is_int(obs.get("map_epoch_after")):
        return "not_evaluated", "evidence_missing"
    if not _frame_ok(obs.get("frame_epoch_before")) or not _frame_ok(obs.get("frame_epoch_after")):
        return "not_evaluated", "evidence_missing"
    note = obs.get("unexpected_candidates")
    if not isinstance(note, str) or not note.strip() or note.strip().lower() == "unknown":
        return "not_evaluated", "evidence_missing"
    return "observed", "observed"


def _base_row(spec, planned, t0, completion, wall):
    span = int(round(float(planned) * NS))
    t1 = int(t0) + span
    return {
        "id": spec["id"],
        "class": spec["class"],
        "kind": spec["kind"],
        "driver": spec["driver"],
        "protocol_duration_s": spec["protocol_s"],
        "planned_duration_s": planned,
        "actual_duration_s": planned,
        "completion": completion,
        "t0_mono_ns": int(t0),
        "t1_mono_ns": t1,
        "change_done_mono_ns": None,
        "clear_mono_ns": None,
        "push_mono_ns": None,
        "restore_mono_ns": None,
        "bump_evaluation": None,
        "wall_start_utc": wall(int(t0)),
        "wall_end_utc": wall(t1),
        "observations": None,
    }


def build_rehearsal(durations, selected=None):
    specs = PROCEDURE if selected is None else selected
    cursor = 0
    rows = []
    for spec in specs:
        planned = durations[spec["id"]]
        completion = "complete" if spec["driver"] == "room_acceptance_marks" else "rehearsal"
        row = _base_row(spec, planned, cursor, completion, wall_synthetic)
        if spec["id"] == "tripod_bump":
            row["push_mono_ns"] = row["t0_mono_ns"]
            row["restore_mono_ns"] = row["t1_mono_ns"]
            row["observations"] = empty_observations()
            row["bump_evaluation"] = "not_evaluated"
        cursor = row["t1_mono_ns"]
        rows.append(row)
    saw_bump = any(row["id"] == "tripod_bump" for row in rows)
    reason = "rehearsal" if saw_bump else "scenario_not_run"
    return _payload(
        "dry-run",
        "synthetic",
        rows,
        _session("dry-run", "", "not_evaluated", reason),
        {"status": None, "topics": None, "recording": None},
    )


def format_rehearsal(payload):
    lines = [
        "G5 room rehearsal (dry-run). This is not the scored session and not a bump evaluation.",
        "Order is the visit: preflight, tripod bump, handoff, then PR #36. The hour is last.",
        WORKSHEET.rstrip(),
        "",
    ]
    for row in payload["scenarios"]:
        offset = row["t0_mono_ns"] / float(NS)
        lines.append(
            "id=%s T+%gs class=%s protocol=%ss planned=%ss driver=%s"
            % (row["id"], offset, row["class"], row["protocol_duration_s"], row["planned_duration_s"], row["driver"])
        )
        lines.append(ROW_PROMPTS[row["id"]])
        lines.append("")
    return "\n".join(lines) + "\n"


def _session(end_reason, detail, evaluation, reason):
    return {
        "end_reason": end_reason,
        "detail": detail,
        "bump_evaluation": evaluation,
        "bump_evaluation_reason": reason,
    }


def _payload(mode, wall_clock, rows, session, summaries):
    return {
        "schema": SCHEMA,
        "decision": "D49",
        "section": "3.13",
        "clock": "monotonic_ns",
        "wall_clock": wall_clock,
        "mode": mode,
        "harness": "PR #36 is the scored session. Do not pass this file to acceptance score.",
        "status": summaries.get("status"),
        "topics": summaries.get("topics"),
        "recording": summaries.get("recording"),
        "inspect": summaries.get("inspect"),
        "log": summaries.get("log"),
        "scenarios": rows,
        "session": session,
    }


def _choice(line, allowed, label):
    value = line.strip().lower()
    if value not in allowed:
        raise ValueError("%s must be one of: %s" % (label, ", ".join(allowed)))
    return value


def _epoch(line, allow_unshown):
    value = line.strip().lower()
    if value == "unknown":
        return "unknown"
    if allow_unshown and value == "unshown":
        return "unshown"
    if value.startswith("-") or not value.isdigit():
        raise ValueError("epoch must be an integer >= 0, or unknown")
    return int(value)


def _seconds_or_unknown(line):
    value = line.strip().lower()
    if value == "unknown":
        return "unknown"
    try:
        number = float(line.strip())
    except ValueError as exc:
        raise ValueError("relearn_after_s must be a finite number >= 0, or unknown") from exc
    if number < 0 or number != number or number in (float("inf"), float("-inf")):
        raise ValueError("relearn_after_s must be a finite number >= 0, or unknown")
    return _as_stored(number)


def _note(line):
    text = line.strip()
    if not text:
        raise ValueError("type none, a short description, or unknown")
    return text


QUESTIONS = (
    ("rig_guard", "Observed rig_guard? on / off / unknown", lambda line: _choice(line, ("on", "off", "unknown"), "rig_guard")),
    ("auto_relearn", "Observed auto_relearn? on / off / unknown", lambda line: _choice(line, ("on", "off", "unknown"), "auto_relearn")),
    ("relearn_after_s", "Observed relearn_after_s? a number, or unknown", _seconds_or_unknown),
    ("imu", "IMU available on the running node? yes / no / unknown", lambda line: _choice(line, ("yes", "no", "unknown"), "imu")),
    ("map_epoch_before", "map_epoch before the push? integer or unknown", lambda line: _epoch(line, False)),
    ("map_epoch_after", "map_epoch after recovery? integer or unknown", lambda line: _epoch(line, False)),
    ("frame_epoch_before", "frame_epoch before the push? integer, unshown, or unknown", lambda line: _epoch(line, True)),
    ("frame_epoch_after", "frame_epoch after recovery? integer, unshown, or unknown", lambda line: _epoch(line, True)),
    ("rig_moved", "RIG_MOVED on the display? seen / not_seen / unknown", lambda line: _choice(line, ("seen", "not_seen", "unknown"), "rig_moved")),
    ("recovery", "Recovery? realigned / in_place / still_moved / relearning / unknown", lambda line: _choice(line, ("realigned", "in_place", "still_moved", "relearning", "unknown"), "recovery")),
    ("unexpected_candidates", "Unexpected candidates? type none, describe them, or unknown", _note),
)


def _ask(read_line, deadline):
    line = read_line(deadline)
    if line is None:
        if deadline is None:
            raise _Eof()
        return None
    if line == "":
        raise _Eof()
    text = line.strip()
    if text.lower() == "abort":
        raise _Abort()
    return text


def _collect_observations(read_line, write, found):
    """Store each accepted answer on ``found`` before asking the next question.

    A stop during a later question keeps the answers already taken. They used to
    live only in this function's local dict and disappeared with the stack.
    """
    for key, prompt, parse in QUESTIONS:
        while True:
            write(prompt + "\n")
            answer = _ask(read_line, None)
            try:
                found[key] = parse(answer)
                break
            except ValueError as exc:
                write(str(exc) + "\n")
    return found


def _blank_bump(planned_s, protocol_s):
    return {
        "id": "tripod_bump",
        "class": "rig",
        "kind": None,
        "driver": "room_acceptance_marks",
        "protocol_duration_s": protocol_s,
        "planned_duration_s": planned_s,
        "actual_duration_s": None,
        "completion": "aborted",
        "t0_mono_ns": None,
        "t1_mono_ns": None,
        "change_done_mono_ns": None,
        "clear_mono_ns": None,
        "push_mono_ns": None,
        "restore_mono_ns": None,
        "bump_evaluation": "not_evaluated",
        "wall_start_utc": None,
        "wall_end_utc": None,
        "observations": empty_observations(),
    }


def _close_bump(row, end_reason, detail, reason):
    """Keep the push, and the restore if it was already taken. Do not invent the missing one."""
    if row.get("restore_mono_ns") is None and row.get("completion") != "restore_unmarked":
        row["completion"] = "aborted"
    row["bump_evaluation"] = "not_evaluated"
    return {
        "row": row,
        "end_reason": end_reason,
        "detail": detail,
        "bump_evaluation": "not_evaluated",
        "bump_evaluation_reason": reason,
    }


def run_bump(clock, read_line, wall, planned_s, protocol_s, write):
    """Mark one bump. ``read_line(deadline_ns)`` returns a line, ``''`` at EOF, or None on timeout.

    A timeout leaves the restore empty even when ``clock`` did not move, so a test can cover the
    deadline without sleeping. Enter before the deadline is the normal restore and keeps the
    original push time. It is ``complete``, including when the leg is back before 30 s.
    """
    planned = _as_stored(planned_s)
    protocol = _as_stored(protocol_s)
    row = _blank_bump(planned, protocol)
    try:
        write(RECOVERY_TEXT + "\n")
        write("Press Enter when you have pushed one tripod leg 3-5 cm. This is the push mark.\n")
        _ask(read_line, None)
        push = int(clock.monotonic_ns())
        row["t0_mono_ns"] = push
        row["push_mono_ns"] = push
        row["wall_start_utc"] = wall(push)
        write(
            "Watching for %g s. Protocol for this bump is %g s. "
            "Press Enter when the leg is back down. That Enter is the restore mark and is complete. "
            "If the deadline passes with no Enter, the restore stays empty and completion is restore_unmarked.\n"
            % (planned, protocol)
        )
        deadline = push + int(round(float(planned) * NS))
        line = _ask(read_line, deadline)
        if line is None:
            now = int(clock.monotonic_ns())
            row["t1_mono_ns"] = deadline if now < deadline else now
            row["restore_mono_ns"] = None
            row["wall_end_utc"] = None
            row["actual_duration_s"] = None
            row["completion"] = "restore_unmarked"
        else:
            end = int(clock.monotonic_ns())
            row["t1_mono_ns"] = end
            row["restore_mono_ns"] = end
            row["wall_end_utc"] = wall(end)
            row["actual_duration_s"] = (end - push) / float(NS)
            row["completion"] = "complete"
        row["observations"] = empty_observations()
        write(OBSERVATION_TEXT)
        _collect_observations(read_line, write, row["observations"])
        evaluation, reason = judge_bump(row["observations"])
        row["bump_evaluation"] = evaluation
        return {
            "row": row,
            "end_reason": "complete",
            "detail": "",
            "bump_evaluation": evaluation,
            "bump_evaluation_reason": reason,
        }
    except _Eof:
        return _close_bump(row, "eof", "stdin closed", "eof")
    except _Abort:
        return _close_bump(row, "aborted", "operator abort", "aborted")
    except KeyboardInterrupt:
        return _close_bump(row, "interrupted", "keyboard interrupt", "interrupted")


# Incomplete line kept when a deadline passes mid-keystroke. Keyed by the stream so a
# later read continues it. A full extra line is never stored here: bytes stay in the
# kernel, where the next select can see them.
_CARRY = weakref.WeakKeyDictionary()


def _read_byte(stdin):
    """One byte from the kernel, not from TextIOWrapper's buffer.

    ``readline`` pulls later typed-ahead lines into a buffer ``select`` cannot see, so the
    next deadline misses a line the operator already typed.
    """
    raw = getattr(getattr(stdin, "buffer", None), "raw", None)
    if raw is not None:
        data = raw.read(1)
        if not data:
            return b""
        if isinstance(data, str):
            return data.encode("utf-8", "surrogateescape")
        return data
    text = stdin.read(1) if hasattr(stdin, "read") else ""
    if not text:
        return b""
    if isinstance(text, bytes):
        return text
    return text.encode("utf-8", "surrogateescape")


def read_line(deadline_ns, clock=None, stdin=None):
    """Blocking read, or ``select`` until ``deadline_ns``. None means the deadline passed."""
    clock = time if clock is None else clock
    stdin = sys.stdin if stdin is None else stdin
    pending = _CARRY.pop(stdin, b"")
    while b"\n" not in pending:
        if deadline_ns is not None:
            timeout = max(0.0, (deadline_ns - clock.monotonic_ns()) / float(NS))
            ready, _, _ = select.select([stdin], [], [], timeout)
            if not ready:
                if pending:
                    _CARRY[stdin] = pending
                return None
        chunk = _read_byte(stdin)
        if chunk == b"":
            if pending:
                return pending.decode("utf-8", "surrogateescape")
            return ""
        pending += chunk
    line, _, rest = pending.partition(b"\n")
    if rest:
        _CARRY[stdin] = rest
    return line.decode("utf-8", "surrogateescape") + "\n"


def _inside(child, parent):
    try:
        child.relative_to(parent)
    except ValueError:
        return False
    return True


def output_error(out, inputs):
    """Refuse before any write. ``inputs`` maps a label to a path or None."""
    out_path = Path(out)
    if out_path.exists() and not out_path.is_dir():
        return "%s is not a directory" % out_path
    out_resolved = out_path.resolve()
    json_path = (out_path / JSON_NAME).resolve()
    csv_path = (out_path / CSV_NAME).resolve()
    if json_path.exists():
        return "%s already exists" % json_path
    if csv_path.exists():
        return "%s already exists" % csv_path
    recording = inputs.get("recording")
    if recording:
        rec = Path(recording).resolve()
        if _inside(out_resolved, rec):
            return "refusing to write marks inside the recording"
    for label, raw in inputs.items():
        if not raw:
            continue
        candidate = Path(raw).resolve()
        if candidate == json_path or candidate == csv_path:
            return "%s aliases an output file" % label
        if candidate == out_resolved:
            return "%s aliases the output directory" % label
    return None


def _norm_state(value):
    names = {0: "building", 1: "tracking", 2: "rig_moved", 3: "degraded", 4: "paused"}
    if isinstance(value, str):
        return value.strip().lower()
    if isinstance(value, int) and not isinstance(value, bool):
        return names.get(value)
    return None


def _norm_outcome(value):
    names = {
        0: "in_place",
        1: "checking",
        2: "moved",
        3: "realigned",
        4: "relearning",
        5: "inconclusive",
    }
    if isinstance(value, str):
        key = value.strip().lower()
        return key if key in names.values() else None
    if isinstance(value, int) and not isinstance(value, bool):
        return names.get(value)
    return None


def _norm_epoch(value):
    if isinstance(value, bool) or isinstance(value, str):
        text = value.strip() if isinstance(value, str) else ""
        if text.isdigit():
            return int(text)
        return None
    if isinstance(value, int) and value >= 0:
        return value
    if isinstance(value, float) and value >= 0 and value.is_integer():
        return int(value)
    return None


def _as_bool(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, int) and value in (0, 1):
        return bool(value)
    if isinstance(value, str):
        text = value.strip().lower()
        if text in ("true", "yes", "1"):
            return True
        if text in ("false", "no", "0"):
            return False
    return None


def _message_of(obj):
    message = obj.get("message")
    if isinstance(message, dict):
        return message
    return {}


def _topic_evidence(status_epochs, rig_epochs, frame_epochs, outcomes, last_imu, status_samples, saw_moved, saw_events, candidates):
    """Fields a bump capture can fill. One sample is not a before and an after."""
    evidence = {}
    if len(status_epochs) >= 2:
        evidence["map_epoch_before"] = status_epochs[0]
        evidence["map_epoch_after"] = status_epochs[-1]
    elif len(rig_epochs) >= 2:
        evidence["map_epoch_before"] = rig_epochs[0]
        evidence["map_epoch_after"] = rig_epochs[-1]
    if len(frame_epochs) >= 2:
        evidence["frame_epoch_before"] = frame_epochs[0]
        evidence["frame_epoch_after"] = frame_epochs[-1]
    if last_imu is not None:
        evidence["imu"] = "yes" if last_imu else "no"
    if saw_moved or "moved" in outcomes:
        evidence["rig_moved"] = "seen"
    elif status_samples >= 2 or len(outcomes) >= 2:
        evidence["rig_moved"] = "not_seen"
    if outcomes:
        recovery = {
            "realigned": "realigned",
            "in_place": "in_place",
            "relearning": "relearning",
            "moved": "still_moved",
        }.get(outcomes[-1])
        if recovery:
            evidence["recovery"] = recovery
    if saw_events:
        if candidates == 0:
            evidence["unexpected_candidates"] = "none"
        else:
            evidence["unexpected_candidates"] = "%d candidate event lines" % candidates
    return evidence


def _command_tokens(obj):
    tokens = []

    def walk(value, allow_command_keys):
        if isinstance(value, str):
            tokens.extend(value.replace("\n", " ").split())
        elif isinstance(value, list):
            for item in value:
                walk(item, True)
        elif isinstance(value, dict) and allow_command_keys:
            for key in ("Cmd", "Entrypoint", "Args", "command", "argv"):
                if key in value:
                    walk(value[key], True)
            config = value.get("Config")
            if isinstance(config, dict):
                for key in ("Cmd", "Entrypoint"):
                    if key in config:
                        walk(config[key], True)

    if isinstance(obj, list):
        for item in obj:
            walk(item, True)
    else:
        walk(obj, True)
    return tokens


def _launch_file(tokens):
    for token in tokens:
        base = token.strip("'\"").rsplit("/", 1)[-1]
        for name in _LAUNCH_FILES:
            if base == name:
                return name
    return None


def _blocks_launch_defaults(tokens):
    for token in tokens:
        if token in ("--params-file",) or token.startswith("--params-file=") or token.startswith("params_file:="):
            return True
    return False


def _ros_switch(value):
    text = value.strip().strip("'\"").lower()
    if text in ("true", "on", "yes", "1"):
        return "on"
    if text in ("false", "off", "no", "0"):
        return "off"
    return None


def _explicit_config(tokens):
    found = {}
    sources = {}
    for token in tokens:
        if ":=" not in token:
            continue
        name, value = token.split(":=", 1)
        name = name.strip("'\"").rsplit("/", 1)[-1]
        if name not in _CONFIG_KEYS:
            continue
        if name == "relearn_after_s":
            try:
                number = float(value.strip().strip("'\""))
            except ValueError:
                continue
            if number < 0 or number != number or number in (float("inf"), float("-inf")):
                continue
            found[name] = _as_stored(number)
        else:
            switch = _ros_switch(value)
            if switch is None:
                continue
            found[name] = switch
        sources[name] = "command"
    return found, sources


def parse_inspect(text):
    """Launch configuration from a saved ``docker inspect`` JSON or a command line.

    The script does not run docker. Explicit ``name:=value`` tokens are source
    ``command``. A named launch file with no params file supplies its defaults
    for arguments that are absent; those are source ``launch_default``.
    """
    try:
        obj = json.loads(text)
    except json.JSONDecodeError:
        obj = {"command": text}
    tokens = _command_tokens(obj)
    launch = _launch_file(tokens)
    values, sources = _explicit_config(tokens)
    defaults_apply = launch is not None and not _blocks_launch_defaults(tokens)
    if defaults_apply:
        for key, value in LAUNCH_DEFAULTS[launch].items():
            if key not in values:
                values[key] = value
                sources[key] = "launch_default"
    summary = {
        "launch_file": launch,
        "defaults_apply": defaults_apply,
        "sources": sources,
    }
    for key in _CONFIG_KEYS:
        if key in values:
            summary[key] = values[key]
    return summary


def _guard_line(text):
    for line in text.splitlines():
        if "rig guard:" in line and "ICP estimates" in line:
            return True
    return False


def _fill(obs, sources, key, value, source, replace=()):
    if value is None:
        return
    current = obs.get(key)
    if current not in (None, "unknown") and current not in replace:
        return
    obs[key] = value
    sources[key] = source


def _status_choice(value, kind):
    if isinstance(value, bool):
        if kind == "imu":
            return "yes" if value else "no"
        return "on" if value else "off"
    if isinstance(value, str):
        text = value.strip().lower()
        if kind == "imu" and text in ("yes", "no"):
            return text
        if text in ("on", "off"):
            return text
    return None


def _apply_evidence(row, summaries):
    """Fill ``unknown`` answers from the post-bump files. Typed answers stay.

    A node-log ``rig guard:`` line is stronger than an inspect value of off:
    that line is printed only while the guard is running. Frame epochs from
    the rig capture replace ``unshown``, because the capture is the source
    for that field. A harness log has no ``bump_evidence`` and fills nothing.
    """
    obs = row.get("observations")
    if not isinstance(obs, dict):
        return {}
    sources = {}
    inspect = summaries.get("inspect") or {}
    inspect_sources = inspect.get("sources") if isinstance(inspect.get("sources"), dict) else {}
    for key in _CONFIG_KEYS:
        if key in inspect and key in inspect_sources:
            _fill(obs, sources, key, inspect.get(key), inspect_sources[key])
    log = summaries.get("log") or {}
    if log.get("rig_guard_line"):
        filled_by_inspect = sources.get("rig_guard") in ("command", "launch_default")
        if obs.get("rig_guard") in (None, "unknown") or (obs.get("rig_guard") == "off" and filled_by_inspect):
            obs["rig_guard"] = "on"
            sources["rig_guard"] = "node_log"
    topics = summaries.get("topics") or {}
    evidence = topics.get("bump_evidence") if isinstance(topics.get("bump_evidence"), dict) else {}
    for key, value in evidence.items():
        replace = ("unshown",) if key in ("frame_epoch_before", "frame_epoch_after") else ()
        _fill(obs, sources, key, value, "topics", replace)
    status = summaries.get("status") or {}
    _fill(obs, sources, "rig_guard", _status_choice(status.get("rig_guard"), "guard"), "status_file")
    _fill(obs, sources, "auto_relearn", _status_choice(status.get("auto_relearn"), "guard"), "status_file")
    if _is_number(status.get("relearn_after_s")):
        _fill(obs, sources, "relearn_after_s", _as_stored(status["relearn_after_s"]), "status_file")
    _fill(obs, sources, "imu", _status_choice(status.get("imu_ok"), "imu"), "status_file")
    if sources:
        row["observation_sources"] = sources
    return sources


def _mark_applied(summaries, sources):
    used = set(sources.values())
    topics = summaries.get("topics")
    if isinstance(topics, dict):
        topics["applied_to_bump"] = "topics" in used
    log = summaries.get("log")
    if isinstance(log, dict):
        log["applied_to_bump"] = "node_log" in used
    status = summaries.get("status")
    if isinstance(status, dict):
        status["applied_to_bump"] = "status_file" in used
    inspect = summaries.get("inspect")
    if isinstance(inspect, dict):
        inspect["applied_to_bump"] = bool({"command", "launch_default"} & used)


def _store_summaries(payload, summaries):
    for key in ("status", "topics", "recording", "inspect", "log"):
        payload[key] = summaries.get(key)


def summarize_status(path):
    text = Path(path).read_text(encoding="utf-8")
    try:
        obj = json.loads(text)
    except json.JSONDecodeError as exc:
        raise ValueError("%s is not a JSON object" % path) from exc
    if not isinstance(obj, dict):
        raise ValueError("%s is not a JSON object" % path)
    summary = {"path": str(path)}
    for key in STATUS_KEYS:
        if key in obj:
            summary[key] = obj[key]
    return summary


def summarize_topics(path):
    lines = Path(path).read_text(encoding="utf-8").splitlines()
    candidate = 0
    rig_moved = 0
    imu = 0
    subscribed = None
    saw_rig = False
    saw_events = False
    saw_ready = False
    status_samples = 0
    status_epochs = []
    rig_epochs = []
    frame_epochs = []
    outcomes = []
    last_imu = None
    imu_topics = ("/room_memory/rig", "/livox/imu")
    for line in lines:
        try:
            obj = json.loads(line)
        except json.JSONDecodeError:
            obj = None
        if isinstance(obj, dict) and obj.get("op") == "ready":
            # Any ready line means the capture started. A topics list, when
            # present, is what it subscribed to. The file may still be empty
            # at startup, before the capture has printed one.
            saw_ready = True
            if isinstance(obj.get("topics"), list):
                subscribed = list(obj["topics"])
                if "/room_memory/events" in subscribed:
                    saw_events = True
        topic = obj.get("topic") if isinstance(obj, dict) else None
        # A ready line names topics. It is not an IMU sample.
        if not (isinstance(obj, dict) and obj.get("op") == "ready"):
            if topic in imu_topics or "imu_ok" in line or "/livox/imu" in line:
                imu += 1
        if not isinstance(obj, dict):
            continue
        message = _message_of(obj)
        state_name = _norm_state(message.get("state"))
        if topic == "/room_memory/events":
            saw_events = True
            if state_name == "candidate":
                candidate += 1
        if topic == "/room_memory/status":
            status_samples += 1
            if state_name == "rig_moved":
                rig_moved += 1
            epoch = _norm_epoch(message.get("map_epoch"))
            if epoch is None:
                epoch = _norm_epoch(obj.get("map_epoch"))
            if epoch is not None:
                status_epochs.append(epoch)
        if topic in imu_topics:
            saw_rig = True
            flag = _as_bool(message.get("imu_ok"))
            if flag is not None:
                last_imu = flag
            frame = _norm_epoch(message.get("frame_epoch"))
            if frame is not None:
                frame_epochs.append(frame)
            epoch = _norm_epoch(message.get("map_epoch"))
            if epoch is not None:
                rig_epochs.append(epoch)
            outcome = _norm_outcome(message.get("outcome"))
            if outcome is not None:
                outcomes.append(outcome)
    imu_subscribed = None
    imu_note = None
    if subscribed is not None:
        imu_subscribed = any(item in imu_topics for item in subscribed)
        if not imu_subscribed:
            imu_note = (
                "This log does not include /room_memory/rig or /livox/imu. "
                "imu_lines 0 is not an observation that the IMU is off."
            )
    harness_log = subscribed is not None and set(subscribed) <= _HARNESS_TOPICS and not saw_rig
    if harness_log:
        evidence = None
        harness_note = (
            "Harness log (status, events, activity only). "
            "It is not a bump observation. imu_lines 0 is not an observation that the IMU is off."
        )
        imu_note = harness_note
    else:
        evidence = _topic_evidence(
            status_epochs, rig_epochs, frame_epochs, outcomes, last_imu,
            status_samples, rig_moved > 0, saw_events, candidate,
        )
    return {
        "path": str(path),
        "lines": len(lines),
        "candidate_lines": candidate,
        "rig_moved_lines": rig_moved,
        "imu_lines": imu,
        "imu_subscribed": imu_subscribed,
        "imu_note": imu_note,
        "saw_ready": saw_ready,
        "bump_evidence": evidence,
    }


def _unstarted_topics(summary):
    """The post-close topics file has to show that the capture started.

    An empty file is what a capture that never imported ROS leaves behind.
    Lines with no ``op: ready`` never became a capture either. ``None`` means
    this input was not a topics file, or the file is usable. The startup read
    does not call this: the file may still be empty when the script starts.
    """
    if not isinstance(summary, dict):
        return None
    path = summary.get("path")
    if summary.get("lines", 0) == 0:
        return "topics file has no lines: %s" % path
    if not summary.get("saw_ready"):
        return "topics file has no op: ready line: %s" % path
    return None


def summarize_recording(path):
    target = Path(path)
    note = "Byte size and metadata only. This is not a bump observation."
    if target.is_file():
        return {
            "path": str(path),
            "bytes": target.stat().st_size,
            "has_metadata": target.name == "metadata.yaml",
            "note": note,
        }
    if not target.is_dir():
        raise ValueError("%s is not a recording file or directory" % path)
    total = 0
    for child in target.iterdir():
        if child.is_file():
            total += child.stat().st_size
    return {
        "path": str(path),
        "bytes": total,
        "has_metadata": (target / "metadata.yaml").is_file(),
        "note": note,
    }


def _tag_read(summary, after_bump):
    if summary is None:
        return None
    summary["read_after_bump"] = bool(after_bump)
    summary["applied_to_bump"] = False
    return summary


def _load_inputs(status, topics, recording, inspect=None, log=None, after_bump=False):
    loaded = {"status": None, "topics": None, "recording": None, "inspect": None, "log": None}
    if status:
        path = Path(status)
        if not path.is_file():
            raise ValueError("status file not found: %s" % path)
        loaded["status"] = _tag_read(summarize_status(path), after_bump)
    if topics:
        path = Path(topics)
        if not path.is_file():
            raise ValueError("topics file not found: %s" % path)
        loaded["topics"] = _tag_read(summarize_topics(path), after_bump)
    if recording:
        path = Path(recording)
        if not path.exists():
            raise ValueError("recording not found: %s" % path)
        loaded["recording"] = _tag_read(summarize_recording(path), after_bump)
    if inspect:
        path = Path(inspect)
        if not path.is_file():
            raise ValueError("inspect file not found: %s" % path)
        summary = parse_inspect(path.read_text(encoding="utf-8"))
        summary["path"] = str(path)
        loaded["inspect"] = _tag_read(summary, after_bump)
    if log:
        path = Path(log)
        if not path.is_file():
            raise ValueError("log file not found: %s" % path)
        text = path.read_text(encoding="utf-8")
        loaded["log"] = _tag_read({
            "path": str(path),
            "lines": len(text.splitlines()),
            "rig_guard_line": _guard_line(text),
        }, after_bump)
    return loaded


def _csv_text(rows):
    class _Buffer:
        def __init__(self):
            self.parts = []

        def write(self, data):
            self.parts.append(data)
            return len(data)

    buffer = _Buffer()
    writer = csv.writer(buffer, lineterminator="\n")
    writer.writerow(CSV_COLUMNS)
    for row in rows:
        writer.writerow(["" if row.get(name) is None else row.get(name) for name in CSV_COLUMNS])
    return "".join(buffer.parts)


def _outputs_match(out, payload):
    destination = Path(out)
    json_path = destination / JSON_NAME
    csv_path = destination / CSV_NAME
    if not json_path.is_file() or not csv_path.is_file():
        return False
    try:
        on_disk = json.loads(json_path.read_text(encoding="utf-8"))
        csv_text = csv_path.read_text(encoding="utf-8")
    except (OSError, json.JSONDecodeError, UnicodeError):
        return False
    return on_disk == payload and csv_text == _csv_text(payload["scenarios"])


def write_outputs(out, payload):
    destination = Path(out)
    destination.mkdir(parents=True, exist_ok=True)
    json_path = destination / JSON_NAME
    csv_path = destination / CSV_NAME
    if json_path.exists() or csv_path.exists():
        # A stop that lands after a finished write retries this function.
        # The files are already this payload. Do not report a false conflict.
        if _outputs_match(destination, payload):
            return
        raise FileExistsError("output already exists")
    json_tmp = destination / ".scenario-marks.json.tmp"
    csv_tmp = destination / ".scenario-marks.csv.tmp"
    try:
        json_tmp.write_text(json.dumps(payload, indent=2, allow_nan=False) + "\n", encoding="utf-8")
        csv_tmp.write_text(_csv_text(payload["scenarios"]), encoding="utf-8")
        json_tmp.replace(json_path)
        csv_tmp.replace(csv_path)
    except BaseException:
        for temporary in (json_tmp, csv_tmp):
            if temporary.exists():
                temporary.unlink()
        # A stop between the two replaces would leave one file and refuse the retry.
        if json_path.exists() and not csv_path.exists():
            json_path.unlink()
        elif csv_path.exists() and not json_path.exists():
            csv_path.unlink()
        raise


def _select_specs(text):
    if not text.strip():
        return list(PROCEDURE)
    wanted = [item.strip() for item in text.split(",") if item.strip()]
    if not wanted:
        raise ValueError("no scenarios selected")
    known = {spec["id"] for spec in PROCEDURE}
    unknown = [item for item in wanted if item not in known]
    if unknown:
        raise ValueError("unknown scenario: %s" % ", ".join(unknown))
    chosen = set(wanted)
    return [spec for spec in PROCEDURE if spec["id"] in chosen]


def _live_scenarios_ok(text):
    if not text.strip():
        return True
    parts = [item.strip() for item in text.split(",") if item.strip()]
    return set(parts) == {"tripod_bump"}


def _say(text):
    sys.stdout.write(text)
    sys.stdout.flush()


def _stdin_line(deadline):
    return read_line(deadline, clock=time, stdin=sys.stdin)


def _install_operator_stop():
    """Turn SIGTERM into KeyboardInterrupt.

    Plain ``kill PID`` sends SIGTERM. The default action exits before
    ``write_outputs``, so the push, the restore, and the answers already taken
    would be lost. Ctrl-C (SIGINT) already raises KeyboardInterrupt.
    """

    def _handler(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, _handler)


def _holding_stop_signals(action):
    """Run ``action``, then surface a stop that arrived while the files were written.

    The handler records the signal and returns, so the write is not torn in half.
    After both files are in place the stop is raised. It is delayed, not discarded.
    """
    pending = []

    def _hold(signum, frame):
        pending.append(signum)

    previous = []
    for name in (signal.SIGINT, signal.SIGTERM):
        try:
            previous.append((name, signal.signal(name, _hold)))
        except (OSError, ValueError):
            pass
    try:
        result = action()
    finally:
        for name, handler in reversed(previous):
            signal.signal(name, handler)
    if pending:
        raise KeyboardInterrupt
    return result


def _conduct_bump(clock, read_line, wall, planned, say, summaries, capture=False):
    """Run the bump and the handoff. A stop still returns the row already taken.

    When ``capture`` is set, a finished bump asks for the local files to be closed
    before the handoff. That Enter is what makes the following read a bump
    observation. A stop or EOF on that prompt leaves ``capture_closed`` false,
    and the caller must not keep a pre-bump snapshot of those files.
    """
    protocol = 30
    result = None
    stopped_after = False
    closed = False
    exit_override = None
    try:
        say(WORKSHEET if WORKSHEET.endswith("\n") else WORKSHEET + "\n")
        result = run_bump(clock, read_line, wall, planned, protocol, say)
        if result["end_reason"] == "complete" and capture:
            say(CLOSE_CAPTURE)
            _ask(read_line, None)
            closed = True
        if result["end_reason"] != "interrupted":
            say(HANDOFF_TEXT if HANDOFF_TEXT.endswith("\n") else HANDOFF_TEXT + "\n")
    except KeyboardInterrupt:
        stopped_after = True
        if result is None:
            result = _close_bump(
                _blank_bump(_as_stored(planned), protocol),
                "interrupted",
                "keyboard interrupt",
                "interrupted",
            )
    except _Eof:
        if result is None:
            result = _close_bump(_blank_bump(_as_stored(planned), protocol), "eof", "stdin closed", "eof")
        elif not closed:
            exit_override = 2
    except _Abort:
        if result is None:
            result = _close_bump(_blank_bump(_as_stored(planned), protocol), "aborted", "operator abort", "aborted")
    session = _session(
        result["end_reason"],
        result["detail"],
        result["bump_evaluation"],
        result["bump_evaluation_reason"],
    )
    payload = _payload("live", "utc", [result["row"]], session, summaries)
    # A stop while the handoff prints still keeps a close the operator already confirmed.
    payload["capture_closed"] = bool(closed)
    code = {"complete": 0, "aborted": 0, "eof": 2, "interrupted": 130}[result["end_reason"]]
    if stopped_after:
        code = 130
    elif exit_override is not None:
        code = exit_override
    return payload, code


def _emit(out, payload):
    """Write both marks files.

    A stop during the write is held until both files exist, then reported to
    the caller as a stop (exit 130). If the files are already this payload, the
    write is not repeated: repeating it would raise ``FileExistsError`` and
    the process would exit 2 even though both files are fine.
    """

    def _write():
        write_outputs(out, payload)

    try:
        _holding_stop_signals(_write)
    except KeyboardInterrupt:
        if not _outputs_match(out, payload):
            try:
                _holding_stop_signals(_write)
            except KeyboardInterrupt:
                if not _outputs_match(out, payload):
                    raise
            except FileExistsError:
                if not _outputs_match(out, payload):
                    raise
        return True
    return False


def _finish_closed_capture(args, payload, code):
    """Fill the bump from the files the operator just closed.

    Returns the exit code. A topics file with no lines, or no ``op: ready``
    line, sets ``read_error`` and turns a success into exit 2. Stray samples
    in that file are not a bump observation. The caller still writes the marks.
    KeyboardInterrupt is not caught here: the caller writes the rows already
    taken and exits 130.
    """
    try:
        summaries = _load_inputs(
            args.status, args.topics, args.recording, args.inspect, args.log, after_bump=True,
        )
    except (OSError, ValueError) as exc:
        sys.stderr.write(str(exc) + "\n")
        payload["read_error"] = str(exc)
        return code
    problem = _unstarted_topics(summaries.get("topics"))
    if problem:
        sys.stderr.write(problem + "\n")
        payload["read_error"] = problem
        if code == 0:
            code = 2
        topics = summaries.get("topics")
        if isinstance(topics, dict):
            topics["bump_evidence"] = None
    row = payload["scenarios"][0]
    sources = _apply_evidence(row, summaries)
    if payload["session"]["end_reason"] == "complete":
        evaluation, reason = judge_bump(row["observations"])
        row["bump_evaluation"] = evaluation
        payload["session"]["bump_evaluation"] = evaluation
        payload["session"]["bump_evaluation_reason"] = reason
    _mark_applied(summaries, sources)
    _store_summaries(payload, summaries)
    return code


def _run(args):
    inputs = {
        "status": args.status,
        "topics": args.topics,
        "recording": args.recording,
        "inspect": args.inspect,
        "log": args.log,
    }
    problem = output_error(args.out, inputs)
    if problem:
        sys.stderr.write(problem + "\n")
        return 2
    try:
        # Fail before the bump if a path is missing or the status file is not an object.
        # Live mode discards this snapshot. The stored summary is the read after the
        # operator closes the captures, so the counts describe the bump and not the room before it.
        preflight = _load_inputs(
            args.status, args.topics, args.recording, args.inspect, args.log, after_bump=False,
        )
    except (OSError, ValueError) as exc:
        sys.stderr.write(str(exc) + "\n")
        return 2
    capture = any(inputs.values())
    if args.live:
        payload, code = _conduct_bump(
            time,
            _stdin_line,
            wall_utc,
            30 if args.bump_s is None else args.bump_s,
            _say,
            {"status": None, "topics": None, "recording": None, "inspect": None, "log": None},
            capture=capture,
        )
        # This read sits outside _conduct_bump. A stop here used to exit 130
        # before the marks were written, which dropped the finished bump.
        if payload.get("capture_closed"):
            try:
                code = _finish_closed_capture(args, payload, code)
            except KeyboardInterrupt:
                code = 130
    else:
        try:
            selected = _select_specs(args.scenarios)
        except ValueError as exc:
            sys.stderr.write(str(exc) + "\n")
            return 2
        durations = resolve_durations(args.watch_s, args.sit_s, args.walk_s, args.hour_s, args.bump_s)
        payload = build_rehearsal(durations, selected)
        _store_summaries(payload, preflight)
        _say(format_rehearsal(payload))
        code = 0
    try:
        if _emit(args.out, payload):
            code = 130
    except OSError as exc:
        sys.stderr.write(str(exc) + "\n")
        return 2
    return code


def main(argv=None):
    parser = argparse.ArgumentParser(
        prog="room_acceptance_marks",
        description=(
            "Offline rehearsal of the G5 room visit, plus a local tripod-bump mark file. "
            "Does not ssh, deploy, restart, or write to the Spark. "
            "The scored session is PR #36."
        ),
    )
    parser.add_argument("--out", required=True, help="directory for scenario-marks.json and scenario-marks.csv")
    parser.add_argument("--dry-run", action="store_true", help="rehearse offline (this is also the default)")
    parser.add_argument("--live", action="store_true", help="mark the tripod bump from the keyboard")
    parser.add_argument("--scenarios", default="", help="dry-run subset, comma-separated, in visit order")
    parser.add_argument("--watch-s", type=_finite_seconds, default=None, help="planned baseline and change watches")
    parser.add_argument("--sit-s", type=_finite_seconds, default=None)
    parser.add_argument("--walk-s", type=_finite_seconds, default=None)
    parser.add_argument("--hour-s", type=_finite_seconds, default=None)
    parser.add_argument("--bump-s", type=_finite_seconds, default=None, help="planned bump watch; protocol stays 30 s")
    parser.add_argument("--status", default=None, help="local status JSON, read only")
    parser.add_argument("--topics", default=None, help="local topics jsonl, read only")
    parser.add_argument("--recording", default=None, help="local recording file or directory, read only")
    parser.add_argument(
        "--inspect",
        default=None,
        help="local docker inspect JSON or the launch command text, read only",
    )
    parser.add_argument("--log", default=None, help="local node log, read only")
    try:
        args = parser.parse_args(argv)
    except SystemExit as exc:
        return int(exc.code) if exc.code is not None else 2
    if args.live and args.dry_run:
        sys.stderr.write("--live and --dry-run cannot be combined\n")
        return 2
    if args.live and not _live_scenarios_ok(args.scenarios):
        sys.stderr.write("live mode only marks tripod_bump; the scored session is PR #36\n")
        return 2
    return _run(args)


if __name__ == "__main__":
    _install_operator_stop()
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.stderr.write("interrupted\n")
        sys.exit(130)
