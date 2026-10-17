"""Format and dry-run tests for the G5 room-marks rehearsal.

The script under test does not import this package. Nothing here imports
room_memory, rclpy, or the PR #36 acceptance harness.
"""

import ast
import importlib.util
import json
import os
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "room_acceptance_marks.py"
RUNBOOK = ROOT / "docs" / "acceptance-runbook.md"

ORDER = (
    "preflight",
    "tripod_bump",
    "harness_handoff",
    "reference_setup",
    "baseline_idle",
    "box_placed",
    "box_removed",
    "box_moved",
    "person_walk",
    "person_sit",
    "restore_reference",
    "hour_idle",
)
PROTOCOL = {
    "preflight": 0,
    "tripod_bump": 30,
    "harness_handoff": 0,
    "reference_setup": 0,
    "baseline_idle": 60,
    "box_placed": 120,
    "box_removed": 120,
    "box_moved": 120,
    "person_walk": 105,
    "person_sit": 120,
    "restore_reference": 60,
    "hour_idle": 3600,
}
SCRIPT_OWNED = {"preflight", "tripod_bump", "harness_handoff"}
KINDS = {"box_placed": "appeared", "box_removed": "disappeared", "box_moved": "moved"}
CSV_HEADER = (
    "id,class,kind,driver,protocol_duration_s,planned_duration_s,actual_duration_s,"
    "completion,t0_mono_ns,t1_mono_ns,change_done_mono_ns,clear_mono_ns,push_mono_ns,"
    "restore_mono_ns,bump_evaluation,wall_start_utc,wall_end_utc"
)
ALLOWED_IMPORTS = {
    "argparse",
    "csv",
    "json",
    "pathlib",
    "sys",
    "time",
    "typing",
    "datetime",
    "select",
}


def load():
    spec = importlib.util.spec_from_file_location("room_acceptance_marks", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def marks(directory):
    return json.loads((directory / "scenario-marks.json").read_text(encoding="utf-8"))


def test_cli_dry_run_reads_no_stdin(tmp_path):
    out = tmp_path / "out"
    started = time.monotonic()
    proc = subprocess.run(
        [sys.executable, str(SCRIPT), "--out", str(out)],
        stdin=subprocess.DEVNULL,
        capture_output=True,
        text=True,
        timeout=5,
        check=False,
    )
    assert time.monotonic() - started < 2
    assert proc.returncode == 0, proc.stderr
    body = marks(out)
    assert body["mode"] == "dry-run"
    assert body["wall_clock"] == "synthetic"
    assert "id=tripod_bump" in proc.stdout
    assert "id=hour_idle" in proc.stdout


def test_explicit_dry_run_flag_and_closed_stdin(tmp_path):
    module = load()
    out = tmp_path / "out"
    original = sys.stdin
    sys.stdin = _ExplodingStdin()
    try:
        code = module.main(["--dry-run", "--out", str(out)])
    finally:
        sys.stdin = original
    assert code == 0
    body = marks(out)
    assert body["schema"] == "scenario-marks-v1"
    assert body["schema"] != "room-memory-acceptance-marks-v1"
    assert body["decision"] == "D49"
    assert body["section"] == "3.13"
    assert body["clock"] == "monotonic_ns"
    assert body["mode"] == "dry-run"
    assert body["wall_clock"] == "synthetic"
    assert body["status"] is None
    assert body["topics"] is None
    assert body["recording"] is None
    session = body["session"]
    assert session["end_reason"] == "dry-run"
    assert session["bump_evaluation"] == "not_evaluated"
    assert session["bump_evaluation_reason"] == "rehearsal"


def test_procedure_order_and_recovery_wording(tmp_path):
    module = load()
    out = tmp_path / "out"
    original = sys.stdout
    sys.stdout = captured = _Capture()
    try:
        assert module.main(["--out", str(out)]) == 0
    finally:
        sys.stdout = original
    text = captured.text
    positions = [text.index(f"id={name}") for name in ORDER]
    assert positions == sorted(positions)
    assert "successful alignment recovers at the displaced pose" in text.lower()
    assert "leaving the rig displaced" not in text.lower()
    assert "60 s starts a relearn" not in text.lower()
    assert "new_object reference_object other_reference_object" in text
    assert "region SENSOR" in text or "region 1 " in text
    body = marks(out)
    assert [row["id"] for row in body["scenarios"]] == list(ORDER)
    runbook = RUNBOOK.read_text(encoding="utf-8")
    assert "D49" in runbook and "3.13" in runbook
    assert "new_object reference_object other_reference_object" in runbook
    assert "leaving the rig displaced" not in runbook.lower()
    assert "not_evaluated" in runbook
    assert "full_candidate" in runbook


def test_marks_format_and_csv(tmp_path):
    module = load()
    out = tmp_path / "out"
    assert module.main(["--dry-run", "--out", str(out)]) == 0
    body = marks(out)
    rows = body["scenarios"]
    assert rows[0]["t0_mono_ns"] == 0
    previous_end = 0
    for row in rows:
        assert row["protocol_duration_s"] == PROTOCOL[row["id"]]
        assert row["planned_duration_s"] == PROTOCOL[row["id"]]
        assert row["actual_duration_s"] == PROTOCOL[row["id"]]
        assert row["t0_mono_ns"] == previous_end
        assert isinstance(row["t0_mono_ns"], int) and isinstance(row["t1_mono_ns"], int)
        assert row["t0_mono_ns"] <= row["t1_mono_ns"]
        assert row["t1_mono_ns"] - row["t0_mono_ns"] == int(PROTOCOL[row["id"]] * 1_000_000_000)
        assert row["wall_start_utc"].startswith("2026-01-01T00:")
        previous_end = row["t1_mono_ns"]
        if row["id"] in SCRIPT_OWNED:
            assert row["driver"] == "room_acceptance_marks"
            assert row["completion"] == "complete"
        else:
            assert row["driver"] == "harness"
            assert row["completion"] == "rehearsal"
            assert row["change_done_mono_ns"] is None
            assert row["clear_mono_ns"] is None
        if row["id"] in KINDS:
            assert row["kind"] == KINDS[row["id"]]
            assert row["class"] == "change"
        if row["id"] == "tripod_bump":
            assert row["class"] == "rig"
            assert row["push_mono_ns"] == row["t0_mono_ns"]
            assert row["restore_mono_ns"] == row["t1_mono_ns"]
            assert row["bump_evaluation"] == "not_evaluated"
            assert all(value is None for value in row["observations"].values())
    hour = rows[-1]
    assert hour["id"] == "hour_idle"
    assert hour["t1_mono_ns"] == sum(PROTOCOL.values()) * 1_000_000_000
    # 4335 s from 2026-01-01T00:00:00Z is 01:12:15.
    assert hour["wall_end_utc"] == "2026-01-01T01:12:15Z"
    csv_text = (out / "scenario-marks.csv").read_text(encoding="utf-8")
    assert "\r" not in csv_text
    header, *csv_rows = csv_text.strip("\n").split("\n")
    assert header == CSV_HEADER
    assert [line.split(",")[0] for line in csv_rows] == list(ORDER)
    bump_cells = csv_rows[1].split(",")
    assert bump_cells[0] == "tripod_bump"
    assert bump_cells[7] == "complete"
    assert bump_cells[12] == str(rows[1]["push_mono_ns"])
    assert bump_cells[14] == "not_evaluated"


def test_planned_duration_does_not_replace_protocol(tmp_path):
    module = load()
    out = tmp_path / "out"
    assert module.main(["--dry-run", "--hour-s", "1", "--watch-s", "5", "--out", str(out)]) == 0
    by_id = {row["id"]: row for row in marks(out)["scenarios"]}
    assert by_id["hour_idle"]["protocol_duration_s"] == 3600
    assert by_id["hour_idle"]["planned_duration_s"] == 1
    assert by_id["hour_idle"]["actual_duration_s"] == 1
    for name in ("baseline_idle", "box_placed", "box_removed", "box_moved"):
        assert by_id[name]["planned_duration_s"] == 5
        assert by_id[name]["protocol_duration_s"] == PROTOCOL[name]
    assert by_id["person_sit"]["planned_duration_s"] == 120
    assert by_id["person_walk"]["planned_duration_s"] == 105
    assert by_id["restore_reference"]["planned_duration_s"] == 60
    assert by_id["tripod_bump"]["planned_duration_s"] == 30
    assert by_id["tripod_bump"]["protocol_duration_s"] == 30


def test_local_reads_do_not_write_back_or_become_observations(tmp_path):
    module = load()
    status = tmp_path / "status.json"
    status.write_text(
        json.dumps({
            "state": "tracking",
            "events_seq": 9,
            "map_epoch": 4,
            "rig_guard": False,
            "auto_relearn": True,
            "secret": "do-not-copy",
        }),
        encoding="utf-8",
    )
    topics = tmp_path / "topics.jsonl"
    topics.write_text(
        "\n".join([
            json.dumps({"topic": "/room_memory/status", "message": {"state": "rig_moved"}}),
            json.dumps({
                "topic": "/room_memory/events",
                "message": {"state": "candidate", "event_id": 3},
            }),
            json.dumps({"topic": "/room_memory/rig", "message": {"imu_ok": True}}),
            "not json",
        ]) + "\n",
        encoding="utf-8",
    )
    recording = tmp_path / "bag"
    recording.mkdir()
    (recording / "metadata.yaml").write_text("rosbag2\n", encoding="utf-8")
    (recording / "piece.mcap").write_bytes(b"abc")
    nested = recording / "nested"
    nested.mkdir()
    (nested / "ignored.bin").write_bytes(b"zzzz")
    snapshots = {
        path: (path.read_bytes(), path.stat().st_mtime_ns)
        for path in (status, topics, recording / "metadata.yaml", recording / "piece.mcap")
    }
    out = tmp_path / "out"
    assert module.main([
        "--dry-run",
        "--out", str(out),
        "--status", str(status),
        "--topics", str(topics),
        "--recording", str(recording),
    ]) == 0
    for path, (payload, mtime) in snapshots.items():
        assert path.read_bytes() == payload
        assert path.stat().st_mtime_ns == mtime
    body = marks(out)
    assert body["status"]["state"] == "tracking"
    assert body["status"]["events_seq"] == 9
    assert body["status"]["map_epoch"] == 4
    assert body["status"]["rig_guard"] is False
    assert "secret" not in body["status"]
    assert "auto_relearn" in body["status"]
    bump = next(row for row in body["scenarios"] if row["id"] == "tripod_bump")
    assert all(value is None for value in bump["observations"].values())
    assert bump["bump_evaluation"] == "not_evaluated"
    assert body["topics"]["lines"] == 4
    assert body["topics"]["candidate_lines"] == 1
    assert body["topics"]["rig_moved_lines"] == 1
    assert body["topics"]["imu_lines"] == 1
    assert body["recording"]["has_metadata"] is True
    assert body["recording"]["bytes"] == len(b"abc") + len(b"rosbag2\n")


def test_missing_or_unreadable_status_writes_nothing(tmp_path):
    module = load()
    out = tmp_path / "out"
    code = module.main(["--out", str(out), "--status", str(tmp_path / "missing.json")])
    assert code == 2
    assert not out.exists()
    bad = tmp_path / "notes.txt"
    bad.write_text("not json\n", encoding="utf-8")
    assert module.main(["--out", str(out), "--status", str(bad)]) == 2
    assert not out.exists()
    listed = tmp_path / "array.json"
    listed.write_text("[1]\n", encoding="utf-8")
    assert module.main(["--out", str(out), "--status", str(listed)]) == 2
    assert not out.exists()


def test_refuse_existing_outputs_and_aliases(tmp_path):
    module = load()
    out = tmp_path / "out"
    out.mkdir()
    existing = out / "scenario-marks.json"
    existing.write_text("keep\n", encoding="utf-8")
    assert module.main(["--dry-run", "--out", str(out)]) == 2
    assert existing.read_text(encoding="utf-8") == "keep\n"
    assert not (out / "scenario-marks.csv").exists()

    csv_only = tmp_path / "csv-only"
    csv_only.mkdir()
    csv_path = csv_only / "scenario-marks.csv"
    csv_path.write_text("keep-csv\n", encoding="utf-8")
    assert module.main(["--out", str(csv_only)]) == 2
    assert csv_path.read_text(encoding="utf-8") == "keep-csv\n"

    alias = tmp_path / "alias"
    alias.mkdir()
    status = alias / "scenario-marks.json"
    status.write_text('{"state": "tracking"}\n', encoding="utf-8")
    assert module.main(["--out", str(alias), "--status", str(status)]) == 2
    assert status.read_text(encoding="utf-8") == '{"state": "tracking"}\n'

    recording = tmp_path / "recording"
    recording.mkdir()
    sentinel = recording / "metadata.yaml"
    sentinel.write_text("stay\n", encoding="utf-8")
    inside = recording / "nested-out"
    assert module.main(["--out", str(inside), "--recording", str(recording)]) == 2
    assert sentinel.read_text(encoding="utf-8") == "stay\n"
    assert not inside.exists()


def test_live_and_dry_run_together_write_nothing(tmp_path):
    module = load()
    out = tmp_path / "out"
    assert module.main(["--live", "--dry-run", "--out", str(out)]) == 2
    assert not out.exists()


def test_live_refuses_harness_scenarios(tmp_path):
    module = load()
    out = tmp_path / "out"
    assert module.main(["--live", "--scenarios", "hour_idle", "--out", str(out)]) == 2
    assert not out.exists()
    assert module.main(["--live", "--scenarios", "baseline_idle,tripod_bump", "--out", str(out)]) == 2
    assert not out.exists()


def test_judge_bump_does_not_assume_launch_defaults():
    module = load()
    unknown = {key: "unknown" for key in module.OBSERVATION_KEYS}
    evaluation, reason = module.judge_bump(unknown)
    assert evaluation == "not_evaluated"
    assert reason == "guard_unobserved"
    disabled = dict(unknown)
    disabled["rig_guard"] = "off"
    assert module.judge_bump(disabled) == ("not_evaluated", "guard_disabled")
    observed = {
        "rig_guard": "on",
        "auto_relearn": "on",
        "relearn_after_s": 60,
        "imu": "yes",
        "rig_moved": "seen",
        "recovery": "realigned",
        "map_epoch_before": 4,
        "map_epoch_after": 4,
        "frame_epoch_before": 7,
        "frame_epoch_after": 7,
        "unexpected_candidates": "none",
    }
    assert module.judge_bump(observed) == ("observed", "observed")
    missing_epoch = dict(observed)
    missing_epoch["map_epoch_after"] = "unknown"
    assert module.judge_bump(missing_epoch) == ("not_evaluated", "evidence_missing")
    no_imu = dict(observed)
    no_imu["imu"] = "unknown"
    assert module.judge_bump(no_imu) == ("not_evaluated", "configuration_unobserved")


def test_fake_clock_timeout_keeps_the_push_mark():
    module = load()
    clock = module.ManualClock(1_000)
    answers = iter(["\n", "on", "off", "45", "no", "3", "3", "unshown", "unshown", "not_seen", "in_place", "none"])

    def read_line(deadline):
        if deadline is None:
            return next(answers)
        assert deadline == 1_000 + 30 * 1_000_000_000
        return None

    result = module.run_bump(clock, read_line, _wall, 30, 30, _discard)
    row = result["row"]
    assert row["push_mono_ns"] == 1_000
    assert row["t0_mono_ns"] == 1_000
    assert row["restore_mono_ns"] == 1_000 + 30 * 1_000_000_000
    assert row["t1_mono_ns"] == row["restore_mono_ns"]
    assert row["completion"] == "complete"
    assert result["end_reason"] == "complete"
    assert result["bump_evaluation"] == "observed"
    assert clock.monotonic_ns() == 1_000


def test_fake_clock_early_enter_does_not_move_the_start():
    module = load()
    clock = module.ManualClock(5_000)
    phase = {"watch": True}

    def read_line(deadline):
        if deadline is None:
            return "\n" if phase["watch"] else "unknown"
        phase["watch"] = False
        clock.advance(5)
        return "\n"

    result = module.run_bump(clock, read_line, _wall, 30, 30, _discard)
    row = result["row"]
    assert row["t0_mono_ns"] == 5_000
    assert row["push_mono_ns"] == 5_000
    assert row["restore_mono_ns"] == 5_000 + 5 * 1_000_000_000
    assert row["t1_mono_ns"] == row["restore_mono_ns"]
    assert row["completion"] == "shortened"
    assert result["bump_evaluation"] == "not_evaluated"


def test_fake_clock_eof_and_interrupt_keep_completed_marks():
    module = load()
    clock = module.ManualClock(8_000)

    def eof_during_watch(deadline):
        if deadline is None:
            return "\n"
        return ""

    eof = module.run_bump(clock, eof_during_watch, _wall, 30, 30, _discard)
    assert eof["end_reason"] == "eof"
    assert eof["row"]["push_mono_ns"] == 8_000
    assert eof["row"]["t0_mono_ns"] == 8_000
    assert eof["row"]["restore_mono_ns"] is None
    assert eof["row"]["completion"] == "aborted"
    assert eof["bump_evaluation"] == "not_evaluated"

    def interrupt_during_watch(deadline):
        if deadline is None:
            return "\n"
        raise KeyboardInterrupt

    stopped = module.run_bump(clock, interrupt_during_watch, _wall, 30, 30, _discard)
    assert stopped["end_reason"] == "interrupted"
    assert stopped["row"]["push_mono_ns"] == 8_000
    assert stopped["row"]["t0_mono_ns"] == 8_000
    assert stopped["row"]["restore_mono_ns"] is None
    assert stopped["row"]["completion"] == "aborted"

    clock.advance(2)

    def interrupt_after_restore(deadline):
        if deadline is not None:
            clock.advance(4)
            return "\n"
        if interrupt_after_restore.calls == 0:
            interrupt_after_restore.calls += 1
            return "\n"
        raise KeyboardInterrupt

    interrupt_after_restore.calls = 0
    later = module.run_bump(clock, interrupt_after_restore, _wall, 30, 30, _discard)
    start = 8_000 + 2 * 1_000_000_000
    assert later["end_reason"] == "interrupted"
    assert later["row"]["push_mono_ns"] == start
    assert later["row"]["t0_mono_ns"] == start
    assert later["row"]["restore_mono_ns"] == start + 4 * 1_000_000_000
    assert later["row"]["t1_mono_ns"] == later["row"]["restore_mono_ns"]
    assert later["row"]["completion"] == "shortened"
    assert later["bump_evaluation"] == "not_evaluated"
    assert later["bump_evaluation_reason"] == "interrupted"


def test_select_timeout_early_enter_and_eof():
    module = load()
    read_end, write_end = os.pipe()
    reader = os.fdopen(read_end, "r", encoding="utf-8")
    writer = os.fdopen(write_end, "w", encoding="utf-8")
    try:
        deadline = time.monotonic_ns() + 50_000_000
        started = time.monotonic()
        assert module.read_line(deadline, clock=time, stdin=reader) is None
        assert time.monotonic() - started < 2
        writer.write("\n")
        writer.flush()
        far = time.monotonic_ns() + 30 * 1_000_000_000
        assert module.read_line(far, clock=time, stdin=reader) == "\n"
        writer.close()
        assert module.read_line(None, clock=time, stdin=reader) == ""
    finally:
        reader.close()
        if not writer.closed:
            writer.close()


def test_import_allowlist_blocks_remote_execution():
    tree = ast.parse(SCRIPT.read_text(encoding="utf-8"))
    imported = set()
    sleep_calls = []
    banned_calls = []
    select_calls = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for alias in node.names:
                imported.add(alias.name.split(".")[0])
        elif isinstance(node, ast.ImportFrom) and node.module:
            imported.add(node.module.split(".")[0])
        elif isinstance(node, ast.Call):
            func = node.func
            if isinstance(func, ast.Attribute) and func.attr == "sleep":
                sleep_calls.append(node)
            if isinstance(func, ast.Attribute) and isinstance(func.value, ast.Name):
                if func.value.id == "select" and func.attr == "select":
                    select_calls.append(node)
                if func.attr in {"system", "Popen", "popen"} or func.value.id in {"subprocess", "socket", "urllib"}:
                    banned_calls.append(node)
            if isinstance(func, ast.Name) and func.id in {"system", "Popen"}:
                banned_calls.append(node)
    assert imported <= ALLOWED_IMPORTS
    assert "select" in imported
    assert select_calls
    assert not sleep_calls
    assert not banned_calls
    source = SCRIPT.read_text(encoding="utf-8")
    assert "docker exec" not in source
    assert "os.system" not in source


class _ExplodingStdin:
    def readline(self):
        raise AssertionError("dry-run read stdin")

    def fileno(self):
        raise AssertionError("dry-run selected stdin")


class _Capture:
    def __init__(self):
        self.text = ""

    def write(self, data):
        self.text += data
        return len(data)

    def flush(self):
        return None


def _discard(_text):
    return None


def _wall(_ns):
    return "2026-01-01T00:00:00Z"
