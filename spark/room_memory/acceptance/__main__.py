"""Operator entry point.

From the repository root:

    PYTHONPATH=spark/room_memory python3 -m acceptance score --log DIR/topics.jsonl --marks DIR/marks.json --out DIR
    PYTHONPATH=spark/room_memory python3 -m acceptance run --transport ros2 --container NAME --out DIR

This module does not put itself on sys.path. Python has to be able to import
``acceptance`` before ``__main__`` runs.
"""

import argparse
import json
import subprocess
import sys
import threading
import time
from pathlib import Path

from acceptance.log import load_jsonl
from acceptance.log import write_jsonl
from acceptance.report import write_report
from acceptance.scenarios import resolve_durations
from acceptance.score import score
from acceptance.session import run_session
from acceptance.transport import ros2_exec_command
from acceptance.transport import script_text
from acceptance.transport import stamp_record


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
    messages = load_jsonl(args.log)
    marks = json.loads(Path(args.marks).read_text(encoding="utf-8"))
    report = score(messages, marks, max_false_alarms_per_hour=args.max_false_alarms_per_hour)
    write_report(report, args.out)
    return report["exit_code"]


def _run(args):
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    records = []
    lock = threading.Lock()
    command = ros2_exec_command(args.container)
    proc = subprocess.Popen(
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
            stamped = stamp_record(obj, time.monotonic_ns())
            with lock:
                records.append(stamped)

    thread = threading.Thread(target=reader, daemon=True)
    thread.start()
    deadline = time.monotonic() + 15
    while time.monotonic() < deadline:
        with lock:
            if any(item.get("op") == "ready" for item in records):
                break
        if proc.poll() is not None:
            break
        time.sleep(0.05)

    def read_line(deadline_ns):
        if deadline_ns is None:
            return sys.stdin.readline()
        import select
        timeout = max(0.0, (deadline_ns - time.monotonic_ns()) / 1e9)
        ready, _, _ = select.select([sys.stdin], [], [], timeout)
        if not ready:
            return None
        return sys.stdin.readline()

    def write(text):
        sys.stdout.write(text)
        sys.stdout.flush()

    ids = [item for item in args.scenarios.split(",") if item] or None
    try:
        marks = run_session(
            clock=time,
            read_line=read_line,
            write=write,
            scenario_ids=ids,
            durations=resolve_durations(args),
        )
    finally:
        if proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait(timeout=2)
    thread.join(timeout=2)
    stderr = proc.stderr.read() if proc.stderr else ""
    if stderr:
        (out / "transport.stderr.log").write_text(stderr, encoding="utf-8")
    end_reason = marks.get("session", {}).get("end_reason", "complete")
    if proc.returncode not in (0, None, -15) and end_reason == "complete":
        end_reason = "transport_died"
    records.append(stamp_record(
        {"op": "session_end", "reason": end_reason, "detail": "", "mono_ns": time.monotonic_ns()},
        time.monotonic_ns(),
    ))
    write_jsonl(out / "topics.jsonl", records)
    (out / "marks.json").write_text(json.dumps(marks, indent=2) + "\n", encoding="utf-8")
    report = score(records, marks, max_false_alarms_per_hour=args.max_false_alarms_per_hour)
    write_report(report, out)
    return report["exit_code"]


if __name__ == "__main__":
    sys.exit(main())
