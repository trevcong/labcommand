"""Operator entry point.

From the repository root:

    PYTHONPATH=spark/room_memory python3 -m acceptance score --log DIR/topics.jsonl --marks DIR/marks.json --out DIR
    PYTHONPATH=spark/room_memory python3 -m acceptance run --transport ros2 --container NAME --out DIR

This module does not put itself on sys.path. Python has to be able to import
``acceptance`` before ``__main__`` runs.
"""

import argparse
import json
import select
import subprocess
import sys
import threading
import time
import traceback
from pathlib import Path

from acceptance.log import TopicLog
from acceptance.log import load_jsonl
from acceptance.log import write_jsonl
from acceptance.report import write_report
from acceptance.scenarios import resolve_durations
from acceptance.score import MAX_GAP_NS
from acceptance.score import interval_tail_ready
from acceptance.score import score
from acceptance.session import empty_marks
from acceptance.session import run_session
from acceptance.transport import ros2_exec_command
from acceptance.transport import script_text
from acceptance.transport import stamp_record

# The scorer accepts a sample at most one coverage gap after the watch. The extra
# second is for the event line that follows the status advertising events_seq.
TAIL_GRACE_NS = 1_000_000_000
_WATCHED = ("complete", "shortened")
# 0 is full_candidate, 1 is a scored fail, 2 is inconclusive. A crash is not 1.
EXIT_SCORE_EXCEPTION = 3


def main(argv=None):
    parser = argparse.ArgumentParser(prog="acceptance")
    sub = parser.add_subparsers(dest="command", required=True)

    score_parser = sub.add_parser("score", help="score a recorded topic log")
    score_parser.add_argument("--log", required=True)
    score_parser.add_argument("--marks", required=True)
    score_parser.add_argument("--out", required=True)
    score_parser.add_argument("--max-false-alarms-per-hour", type=float, default=1.0)

    run_parser = sub.add_parser("run", help="prompt the operator and record read-only")
    run_parser.add_argument("--transport", choices=("ros2",), required=True)
    run_parser.add_argument("--container", required=True)
    run_parser.add_argument("--out", required=True)
    run_parser.add_argument("--scenarios", default="")
    run_parser.add_argument("--watch-s", type=float, default=None)
    run_parser.add_argument("--sit-s", type=float, default=None)
    run_parser.add_argument("--hour-s", type=float, default=None)
    run_parser.add_argument("--walk-s", type=float, default=None)
    run_parser.add_argument("--max-false-alarms-per-hour", type=float, default=1.0)

    args = parser.parse_args(argv)
    if args.command == "score":
        return _score(args)
    return _run(args)


def _score(args):
    try:
        messages = load_jsonl(args.log)
        marks = json.loads(Path(args.marks).read_text(encoding="utf-8"))
        report = score(messages, marks, max_false_alarms_per_hour=args.max_false_alarms_per_hour)
        write_report(report, args.out)
    except Exception:
        traceback.print_exc()
        return EXIT_SCORE_EXCEPTION
    return report["exit_code"]


def _run(args, popen=None, clock=None):
    popen = subprocess.Popen if popen is None else popen
    clock = time if clock is None else clock
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    records = []
    lock = threading.Lock()
    state = {"marks": None, "end_reason": None, "detail": "", "error": None, "exit_code": 2}
    command = ros2_exec_command(args.container)
    proc = None
    writer = None
    thread = None
    try:
        writer = TopicLog(out / "topics.jsonl")
        proc = popen(
            command,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        proc.stdin.write(script_text())
        proc.stdin.close()

        def reader():
            for line in proc.stdout:
                line = line.strip()
                if not line:
                    continue
                try:
                    obj = json.loads(line)
                except json.JSONDecodeError as exc:
                    obj = {"op": "error", "error": str(exc), "topic": "", "mono_ns": 0}
                stamped = stamp_record(obj, clock.monotonic_ns())
                with lock:
                    records.append(stamped)
                # Checkpoint before the session returns. A later Ctrl-C still has the lines.
                writer.append(stamped)

        thread = threading.Thread(target=reader, daemon=True)
        thread.start()

        def read_line(deadline_ns):
            if deadline_ns is None:
                return sys.stdin.readline()
            timeout = max(0.0, (deadline_ns - clock.monotonic_ns()) / 1e9)
            ready, _, _ = select.select([sys.stdin], [], [], timeout)
            if not ready:
                return None
            return sys.stdin.readline()

        def write(text):
            sys.stdout.write(text)
            sys.stdout.flush()

        def on_progress(marks):
            state["marks"] = marks
            _write_marks(out, marks)

        ids = [item for item in args.scenarios.split(",") if item] or None
        try:
            _wait_ready(records, lock, proc, clock)
            marks = run_session(
                clock=clock,
                read_line=read_line,
                write=write,
                scenario_ids=ids,
                durations=resolve_durations(args),
                on_progress=on_progress,
            )
            state["marks"] = marks
            state["end_reason"] = (marks.get("session") or {}).get("end_reason", "complete")
            state["detail"] = (marks.get("session") or {}).get("detail") or ""
            _hold_for_tail(records, lock, marks, clock, write)
        except KeyboardInterrupt:
            state["end_reason"] = "interrupted"
            state["detail"] = state["detail"] or "keyboard interrupt"
            state["marks"] = _coerce_marks(state["marks"], "interrupted", state["detail"])
        except ValueError as exc:
            state["end_reason"] = "error"
            state["detail"] = str(exc)
            state["marks"] = _coerce_marks(state["marks"], "error", state["detail"])
        except Exception as exc:
            state["error"] = exc
            state["end_reason"] = "error"
            state["detail"] = str(exc)
            state["marks"] = _coerce_marks(state["marks"], "error", state["detail"])
    finally:
        if proc is not None:
            _stop(proc)
        if thread is not None:
            thread.join(timeout=2)
        _finalize(out, records, lock, proc, state, clock, args, writer)
    if state["error"] is not None:
        raise state["error"]
    if state["end_reason"] == "error" and state["detail"]:
        sys.stderr.write(state["detail"] + "\n")
    return state["exit_code"]


def _wait_ready(records, lock, proc, clock):
    deadline = clock.monotonic() + 15
    while clock.monotonic() < deadline:
        with lock:
            if any(item.get("op") == "ready" for item in records):
                return
        if proc.poll() is not None:
            return
        clock.sleep(0.05)


def _last_watched(marks):
    """The watched interval whose end is latest. Skipped and blocked rows have no tail."""
    if not marks:
        return None
    rows = []
    for row in marks.get("scenarios") or []:
        if row.get("completion") not in _WATCHED:
            continue
        if row.get("t0_mono_ns") is None or row.get("t1_mono_ns") is None:
            continue
        rows.append(row)
    if not rows:
        return None
    return max(rows, key=lambda row: (row["t1_mono_ns"], row["t0_mono_ns"]))


def wait_for_closing(records, lock, marks, now_ns, sleep):
    """Keep reading until the last watch can be scored, or the bound passes.

    ``now_ns`` and the receipt stamps are the same monotonic clock. A sample read
    more than one coverage gap after the watch cannot close it.
    """
    scenario = _last_watched(marks)
    if scenario is None:
        return
    t0 = scenario["t0_mono_ns"]
    t1 = scenario["t1_mono_ns"]
    deadline = t1 + MAX_GAP_NS + TAIL_GRACE_NS
    while True:
        with lock:
            snapshot = list(records)
        if interval_tail_ready(snapshot, t0, t1):
            return
        now = now_ns()
        if now >= deadline:
            return
        remaining = (deadline - now) / 1e9
        sleep(0.05 if remaining > 0.05 else remaining)


def _hold_for_tail(records, lock, marks, clock, write):
    scenario = _last_watched(marks)
    if scenario is None:
        return
    with lock:
        ready = interval_tail_ready(list(records), scenario["t0_mono_ns"], scenario["t1_mono_ns"])
    if not ready:
        limit_s = (MAX_GAP_NS + TAIL_GRACE_NS) / 1e9
        write("Holding the recorder for the closing status, activity, and event tail (at most %g s).\n" % limit_s)
    wait_for_closing(records, lock, marks, clock.monotonic_ns, clock.sleep)


def _stop(proc):
    if proc.poll() is None:
        proc.terminate()
        try:
            proc.wait(timeout=2)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=2)


def _coerce_marks(marks, reason, detail):
    if not marks:
        return empty_marks(reason, detail)
    updated = dict(marks)
    session = dict(updated.get("session") or {})
    session["end_reason"] = reason
    session["detail"] = detail or session.get("detail") or ""
    updated["session"] = session
    return updated


def _write_marks(out, marks):
    path = out / "marks.json"
    temporary = path.with_name("marks.json.tmp")
    temporary.write_text(json.dumps(marks, indent=2) + "\n", encoding="utf-8")
    temporary.replace(path)


def _finalize(out, records, lock, proc, state, clock, args, writer):
    if writer is not None:
        writer.close()
    marks = state["marks"]
    reason = state["end_reason"]
    detail = state["detail"] or ""
    if marks is None:
        marks = empty_marks(reason or "error", detail)
        reason = marks["session"]["end_reason"]
        detail = marks["session"]["detail"]
    else:
        if reason is None:
            reason = (marks.get("session") or {}).get("end_reason") or "complete"
        if not detail:
            detail = (marks.get("session") or {}).get("detail") or ""
        marks = _coerce_marks(marks, reason, detail)
    if proc is not None and proc.returncode not in (0, None, -15) and reason == "complete":
        reason = "transport_died"
        detail = detail or "subscriber exited"
        marks = _coerce_marks(marks, reason, detail)
    with lock:
        rows = list(records)
    rows.append(stamp_record(
        {"op": "session_end", "reason": reason, "detail": detail, "mono_ns": clock.monotonic_ns()},
        clock.monotonic_ns(),
    ))
    write_jsonl(out / "topics.jsonl", rows)
    _write_marks(out, marks)
    stderr = proc.stderr.read() if proc is not None and proc.stderr else ""
    if stderr:
        (out / "transport.stderr.log").write_text(stderr, encoding="utf-8")
    try:
        report = score(rows, marks, max_false_alarms_per_hour=args.max_false_alarms_per_hour)
        write_report(report, out)
    except Exception:
        traceback.print_exc()
        state["exit_code"] = EXIT_SCORE_EXCEPTION
        state["marks"] = marks
        return
    state["exit_code"] = report["exit_code"]
    state["marks"] = marks


if __name__ == "__main__":
    sys.exit(main())
