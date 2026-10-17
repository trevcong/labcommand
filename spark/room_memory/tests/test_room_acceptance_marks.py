"""Format and dry-run tests for the G5 room-marks rehearsal.

The script under test does not import this package. Nothing here imports
room_memory, rclpy, or the PR #36 acceptance harness.
"""

import ast
import importlib.util
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import pytest

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
    "signal",
    "weakref",
}

# The room-memory image copies only spark/room_memory and runs pytest there.
# scripts/ and docs/ are not in that tree. Skip rather than error on the missing files.
pytestmark = pytest.mark.skipif(
    not SCRIPT.exists(),
    reason="room acceptance marks script is not in this tree",
)


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
    assert body["status"]["read_after_bump"] is False
    assert body["status"]["applied_to_bump"] is False
    assert "secret" not in body["status"]
    assert "auto_relearn" in body["status"]
    bump = next(row for row in body["scenarios"] if row["id"] == "tripod_bump")
    assert all(value is None for value in bump["observations"].values())
    assert bump["bump_evaluation"] == "not_evaluated"
    assert "not a bump observation" in body["recording"]["note"]
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
    assert row["restore_mono_ns"] is None
    assert row["wall_end_utc"] is None
    assert row["wall_start_utc"] == "2026-01-01T00:00:00Z"
    assert row["t1_mono_ns"] == 1_000 + 30 * 1_000_000_000
    assert row["completion"] == "restore_unmarked"
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
    assert row["wall_end_utc"] == "2026-01-01T00:00:00Z"
    assert row["completion"] == "complete"
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
    assert later["row"]["completion"] == "complete"
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
        writer.write("one\ntwo\n")
        writer.flush()
        assert module.read_line(far, clock=time, stdin=reader) == "one\n"
        short = time.monotonic_ns() + 300_000_000
        started = time.monotonic()
        assert module.read_line(short, clock=time, stdin=reader) == "two\n"
        assert time.monotonic() - started < 0.2
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
    assert "SIGTERM" in source


def test_runbook_matches_harness_head_and_the_stop_path():
    text = RUNBOOK.read_text(encoding="utf-8")
    assert "1225db5" in text
    assert "Do not deny acceptance for a lifetime dropped-window count" in text
    assert "Physical acceptance-test harness and scorer" not in text
    assert "5a51554" in text
    assert "docker inspect" in text
    assert "rig guard:" in text
    assert "output already exists" in text
    assert "172518b" not in text
    assert "session_dropped_windows" in text
    assert "dropped_windows did not change from ready" not in text
    assert "restore_unmarked" in text
    assert "records the restore at the deadline" not in text
    assert "SIGTERM" in text
    assert 'print("region", msg.get("sensors"), coords + tail)' in text
    assert 'print("event", row.get("publisher_generation"), msg.get("event_id"))' in text
    assert 'print("event", row.get("publisher_generation"), msg.get("event_id"), msg.get("kind"))' not in text
    assert "not an observation that the IMU is off" in text


def _transport_setup():
    """The harness command string, read as text so this file does not import acceptance."""
    path = ROOT / "spark" / "room_memory" / "acceptance" / "transport.py"
    source = path.read_text(encoding="utf-8")
    found = None
    for node in ast.parse(source).body:
        if isinstance(node, ast.Assign):
            for target in node.targets:
                if isinstance(target, ast.Name) and target.id == "_SETUP" and isinstance(node.value, ast.Constant):
                    found = node.value.value
    assert found == "source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && python3 -u -"
    assert 'return ["docker", "exec", "-i", container, "bash", "-lc", _SETUP]' in source
    return found


def _capture_source(text):
    marker = "<<'PY'\n"
    start = text.index(marker) + len(marker)
    return text[start:text.index("\nPY\n", start)]


def test_runbook_capture_prefix_matches_transport():
    """The bump capture has to source ROS the way acceptance/transport.py does.

    ``docker exec`` does not run the image entrypoint. A bare ``python3`` never
    sees ``rclpy`` or ``labcommand_msgs``.
    """
    setup = _transport_setup()
    text = RUNBOOK.read_text(encoding="utf-8")
    prefix = "docker exec -i NAME bash -lc '%s' > bump-topics.jsonl <<'PY'" % setup
    assert prefix in text
    assert "docker exec -i NAME python3 -u -" not in text


def test_runbook_capture_stamps_each_line():
    source = _capture_source(RUNBOOK.read_text(encoding="utf-8"))
    tree = ast.parse(source)
    emit = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == "emit")
    attrs = [node.attr for node in ast.walk(emit) if isinstance(node, ast.Attribute)]
    assert "monotonic_ns" in attrs
    assert "time_ns" in attrs


def test_runbook_states_the_bump_pass_and_the_both_sensor_label():
    module = load()
    phrase = (
        "A bump passes only when bump_evaluation is observed, RIG_MOVED is seen, "
        "recovery is realigned, map_epoch is unchanged, and unexpected candidates are none."
    )
    both = (
        "message.sensors is a bitmask. 3 means both sensors. Do not paste region 3. "
        "Split it into region 1 and region 2, or use event GEN ID."
    )
    runbook = RUNBOOK.read_text(encoding="utf-8").replace("`", "")
    assert phrase in runbook
    assert "observed alone is not a pass" in runbook.lower()
    assert both in runbook
    assert phrase in module.RECOVERY_TEXT
    assert "observed alone is not a pass" in module.RECOVERY_TEXT
    assert both in module.WORKSHEET
    assert "A stop during the post-close read writes the rows already taken and exits 130." in runbook


def test_empty_scenario_list_is_refused(tmp_path):
    module = load()
    out = tmp_path / "out"
    assert module.main(["--scenarios", ",", "--out", str(out)]) == 2
    assert not out.exists()
    assert module.main(["--scenarios", ", ,", "--out", str(out)]) == 2
    assert not out.exists()
    assert module.main(["--live", "--scenarios", ",", "--out", str(out)]) == 2
    assert not out.exists()


def test_harness_topics_log_does_not_count_as_an_imu_observation(tmp_path):
    module = load()
    topics = tmp_path / "topics.jsonl"
    topics.write_text(
        "\n".join([
            json.dumps({
                "op": "ready",
                "topics": ["/room_memory/status", "/room_memory/events", "/room_memory/activity"],
            }),
            json.dumps({
                "topic": "/room_memory/status",
                "message": {"state": "tracking", "dropped_windows": 5},
            }),
            json.dumps({
                "topic": "/room_memory/events",
                "publisher_generation": 4,
                "message": {"state": "candidate", "event_id": 7, "kind": "appeared"},
            }),
        ]) + "\n",
        encoding="utf-8",
    )
    summary = module.summarize_topics(topics)
    assert summary["lines"] == 3
    assert summary["candidate_lines"] == 1
    assert summary["imu_lines"] == 0
    assert summary["imu_subscribed"] is False
    assert "not an observation" in summary["imu_note"]
    assert summary["bump_evidence"] is None


def test_interrupt_during_questions_keeps_answers_already_taken():
    module = load()
    clock = module.ManualClock(2_000)
    answers = iter(["\n", "on", "off"])

    def read_line(deadline):
        if deadline is not None:
            clock.advance(3)
            return "\n"
        try:
            return next(answers)
        except StopIteration:
            raise KeyboardInterrupt

    result = module.run_bump(clock, read_line, _wall, 30, 30, _discard)
    row = result["row"]
    assert result["end_reason"] == "interrupted"
    assert result["bump_evaluation_reason"] == "interrupted"
    assert row["push_mono_ns"] == 2_000
    assert row["restore_mono_ns"] == 2_000 + 3 * 1_000_000_000
    assert row["completion"] == "complete"
    assert row["observations"]["rig_guard"] == "on"
    assert row["observations"]["auto_relearn"] == "off"
    assert row["observations"]["relearn_after_s"] is None
    assert row["bump_evaluation"] == "not_evaluated"


def test_interrupt_during_handoff_keeps_the_finished_bump():
    module = load()
    clock = module.ManualClock(9)

    def read_line(deadline):
        if deadline is not None:
            clock.advance(2)
            return "\n"
        return "unknown"

    def say(text):
        if text.startswith("HANDOFF"):
            raise KeyboardInterrupt

    payload, code = module._conduct_bump(
        clock,
        read_line,
        _wall,
        30,
        say,
        {"status": None, "topics": None, "recording": None},
    )
    assert code == 130
    row = payload["scenarios"][0]
    assert row["push_mono_ns"] == 9
    assert row["restore_mono_ns"] == 9 + 2 * 1_000_000_000
    assert row["completion"] == "complete"
    assert row["observations"]["rig_guard"] == "unknown"
    assert payload["session"]["end_reason"] == "complete"
    assert payload["session"]["bump_evaluation"] == "not_evaluated"


def test_interrupt_before_the_push_still_returns_a_row():
    module = load()

    def read_line(_deadline):
        raise AssertionError("should not read")

    def say(_text):
        raise KeyboardInterrupt

    payload, code = module._conduct_bump(
        module.ManualClock(0),
        read_line,
        _wall,
        30,
        say,
        {"status": None, "topics": None, "recording": None},
    )
    assert code == 130
    row = payload["scenarios"][0]
    assert row["push_mono_ns"] is None
    assert row["restore_mono_ns"] is None
    assert row["completion"] == "aborted"
    assert payload["session"]["end_reason"] == "interrupted"


def test_sigterm_during_the_watch_writes_the_push_mark(tmp_path):
    proc, out, stdout_path, stderr_path = _spawn_live(tmp_path)
    try:
        _wait_live(stdout_path, "push mark", proc)
        proc.stdin.write("\n")
        proc.stdin.flush()
        _wait_live(stdout_path, "Watching for", proc)
        proc.send_signal(signal.SIGTERM)
        code = proc.wait(timeout=10)
    finally:
        _stop_live(proc)
    assert code == 130
    stderr = stderr_path.read_text(encoding="utf-8")
    assert "Traceback" not in stderr
    body = marks(out)
    bump = body["scenarios"][0]
    assert bump["id"] == "tripod_bump"
    assert bump["push_mono_ns"] is not None
    assert bump["restore_mono_ns"] is None
    assert bump["completion"] == "aborted"
    assert body["session"]["end_reason"] == "interrupted"
    assert (out / "scenario-marks.csv").is_file()
    assert (out / "scenario-marks.json").is_file()


def test_sigterm_during_questions_keeps_answers_and_the_restore(tmp_path):
    proc, out, stdout_path, stderr_path = _spawn_live(tmp_path)
    try:
        _wait_live(stdout_path, "push mark", proc)
        proc.stdin.write("\n")
        proc.stdin.flush()
        _wait_live(stdout_path, "Watching for", proc)
        proc.stdin.write("\n")
        proc.stdin.flush()
        _wait_live(stdout_path, "Observed rig_guard?", proc)
        proc.stdin.write("on\n")
        proc.stdin.flush()
        _wait_live(stdout_path, "Observed auto_relearn?", proc)
        proc.stdin.write("off\n")
        proc.stdin.flush()
        _wait_live(stdout_path, "Observed relearn_after_s?", proc)
        proc.send_signal(signal.SIGTERM)
        code = proc.wait(timeout=10)
    finally:
        _stop_live(proc)
    assert code == 130
    assert "Traceback" not in stderr_path.read_text(encoding="utf-8")
    body = marks(out)
    bump = body["scenarios"][0]
    assert bump["push_mono_ns"] is not None
    assert bump["restore_mono_ns"] is not None
    assert bump["wall_end_utc"]
    assert bump["completion"] == "complete"
    assert bump["observations"]["rig_guard"] == "on"
    assert bump["observations"]["auto_relearn"] == "off"
    assert bump["observations"]["relearn_after_s"] is None
    assert body["session"]["end_reason"] == "interrupted"
    assert (out / "scenario-marks.csv").is_file()


def _rehearsal_payload(module):
    return module.build_rehearsal(module.resolve_durations(None, None, None, None, None))


def test_launch_defaults_match_the_launch_files():
    module = load()
    for relative in (
        "spark/room_memory/launch/room_memory.launch.py",
        "spark/room_memory_cpp/launch/room_memory_cpp.launch.py",
    ):
        text = (ROOT / relative).read_text(encoding="utf-8")
        assert '"rig_guard": ("false", bool)' in text
        assert '"auto_relearn": ("true", bool)' in text
        assert '"relearn_after_s": ("60.0", float)' in text
        assert module.LAUNCH_DEFAULTS[Path(relative).name] == {
            "rig_guard": "off",
            "auto_relearn": "on",
            "relearn_after_s": 60,
        }


def test_inspect_counts_a_named_launch_file_and_not_a_remembered_default():
    module = load()
    named = module.parse_inspect(json.dumps([{
        "Config": {
            "Cmd": ["ros2", "launch", "room_memory", "room_memory.launch.py", "rig_guard:=true"],
        }
    }]))
    assert named["launch_file"] == "room_memory.launch.py"
    assert named["rig_guard"] == "on"
    assert named["sources"]["rig_guard"] == "command"
    assert named["auto_relearn"] == "on"
    assert named["sources"]["auto_relearn"] == "launch_default"
    assert named["relearn_after_s"] == 60
    assert named["sources"]["relearn_after_s"] == "launch_default"
    args_only = module.parse_inspect(json.dumps([{
        "Args": ["ros2", "launch", "room_memory_cpp", "room_memory_cpp.launch.py"],
    }]))
    assert args_only["launch_file"] == "room_memory_cpp.launch.py"
    assert args_only["rig_guard"] == "off"
    assert args_only["sources"]["rig_guard"] == "launch_default"
    bare = module.parse_inspect("ros2 run room_memory room_memory_node")
    assert bare["defaults_apply"] is False
    assert "auto_relearn" not in bare
    assert "relearn_after_s" not in bare
    blocked = module.parse_inspect(
        "ros2 launch room_memory room_memory.launch.py rig_guard:=true --params-file /tmp/p.yaml"
    )
    assert blocked["defaults_apply"] is False
    assert blocked["rig_guard"] == "on"
    assert blocked["sources"]["rig_guard"] == "command"
    assert "auto_relearn" not in blocked


def test_named_sources_make_an_untyped_bump_observed():
    """A visit that types unknown can still be observed from the files it can actually collect."""
    module = load()
    row = {"observations": {key: "unknown" for key in module.OBSERVATION_KEYS}}
    evidence = {
        "imu": "yes",
        "map_epoch_before": 4,
        "map_epoch_after": 4,
        "frame_epoch_before": 9,
        "frame_epoch_after": 9,
        "rig_moved": "seen",
        "recovery": "realigned",
        "unexpected_candidates": "none",
    }
    summaries = {
        "inspect": module.parse_inspect("ros2 launch room_memory room_memory.launch.py"),
        "log": {"rig_guard_line": True},
        "topics": {"bump_evidence": evidence},
        "status": {},
    }
    sources = module._apply_evidence(row, summaries)
    assert row["observations"]["rig_guard"] == "on"
    assert sources["rig_guard"] == "node_log"
    assert row["observations"]["auto_relearn"] == "on"
    assert sources["auto_relearn"] == "launch_default"
    assert row["observations"]["relearn_after_s"] == 60
    assert sources["imu"] == "topics"
    assert module.judge_bump(row["observations"]) == ("observed", "observed")
    # A typed answer is not replaced by the file.
    row["observations"]["auto_relearn"] = "off"
    module._apply_evidence(row, summaries)
    assert row["observations"]["auto_relearn"] == "off"


def test_one_status_sample_does_not_fill_both_epochs(tmp_path):
    module = load()
    topics = tmp_path / "topics.jsonl"
    topics.write_text(
        json.dumps({"topic": "/room_memory/status", "message": {"state": "tracking", "map_epoch": 4}}) + "\n",
        encoding="utf-8",
    )
    summary = module.summarize_topics(topics)
    assert "map_epoch_before" not in summary["bump_evidence"]
    assert "map_epoch_after" not in summary["bump_evidence"]
    topics.write_text(
        "\n".join([
            json.dumps({"topic": "/room_memory/status", "message": {"state": "tracking", "map_epoch": 4}}),
            json.dumps({"topic": "/room_memory/status", "message": {"state": "rig_moved", "map_epoch": 4}}),
            json.dumps({
                "topic": "/room_memory/rig",
                "message": {"imu_ok": False, "frame_epoch": 4, "map_epoch": 4, "outcome": "moved"},
            }),
            json.dumps({
                "topic": "/room_memory/rig",
                "message": {"imu_ok": False, "frame_epoch": 4, "map_epoch": 4, "outcome": "realigned"},
            }),
        ]) + "\n",
        encoding="utf-8",
    )
    summary = module.summarize_topics(topics)
    evidence = summary["bump_evidence"]
    assert evidence["map_epoch_before"] == 4
    assert evidence["map_epoch_after"] == 4
    assert evidence["frame_epoch_before"] == 4
    assert evidence["imu"] == "no"
    assert evidence["rig_moved"] == "seen"
    assert evidence["recovery"] == "realigned"


def test_live_csv_leaves_restore_cells_empty_when_unmarked():
    module = load()
    clock = module.ManualClock(1_000)
    answers = iter(["\n", "on", "off", "45", "no", "3", "3", "unshown", "unshown", "not_seen", "in_place", "none"])

    def read_line(deadline):
        if deadline is None:
            return next(answers)
        return None

    result = module.run_bump(clock, read_line, _wall, 30, 30, _discard)
    text = module._csv_text([result["row"]])
    assert text.endswith("\n")
    assert "\r" not in text
    header, row = text.strip("\n").split("\n")
    assert header == CSV_HEADER
    cells = dict(zip(module.CSV_COLUMNS, row.split(",")))
    assert cells["id"] == "tripod_bump"
    assert cells["completion"] == "restore_unmarked"
    assert cells["push_mono_ns"] == "1000"
    assert cells["restore_mono_ns"] == ""
    assert cells["wall_end_utc"] == ""
    assert cells["bump_evaluation"] == "observed"
    assert cells["actual_duration_s"] == ""


def test_stop_during_the_write_is_delayed_and_exits_130(tmp_path):
    module = load()
    out = tmp_path / "out"
    payload = _rehearsal_payload(module)
    previous = signal.getsignal(signal.SIGINT)
    real = module.write_outputs

    def write_and_signal(path, body):
        real(path, body)
        os.kill(os.getpid(), signal.SIGINT)

    module.write_outputs = write_and_signal
    try:
        stopped = module._emit(out, payload)
    finally:
        module.write_outputs = real
        signal.signal(signal.SIGINT, previous)
    assert stopped is True
    assert module._outputs_match(out, payload)
    code = 130 if stopped else 0
    assert code == 130


def test_stop_after_both_files_exist_does_not_report_a_conflict(tmp_path):
    module = load()
    out = tmp_path / "out"
    payload = _rehearsal_payload(module)
    module.write_outputs(out, payload)
    real_hold = module._holding_stop_signals

    def finished_then_stop(action):
        action()
        raise KeyboardInterrupt

    module._holding_stop_signals = finished_then_stop
    try:
        assert module._emit(out, payload) is True
    finally:
        module._holding_stop_signals = real_hold
    assert module._outputs_match(out, payload)
    # A second write of the same payload is the retry the race used to take.
    module.write_outputs(out, payload)
    assert module._outputs_match(out, payload)


def test_missing_inspect_writes_nothing(tmp_path):
    module = load()
    out = tmp_path / "out"
    assert module.main(["--out", str(out), "--inspect", str(tmp_path / "missing.json")]) == 2
    assert not out.exists()


def test_interrupt_during_capture_close_does_not_mark_the_files_read():
    module = load()
    clock = module.ManualClock(4)

    def read_line(deadline):
        if deadline is not None:
            clock.advance(1)
            return "\n"
        return "unknown"

    def say(text):
        if "capture files are closed" in text:
            raise KeyboardInterrupt

    payload, code = module._conduct_bump(clock, read_line, _wall, 30, say, {
        "status": None, "topics": None, "recording": None, "inspect": None, "log": None,
    }, capture=True)
    assert code == 130
    assert payload["capture_closed"] is False
    assert payload["scenarios"][0]["push_mono_ns"] == 4
    assert payload["scenarios"][0]["completion"] == "complete"


def test_live_capture_is_read_after_the_bump_and_can_be_observed(tmp_path):
    module = load()
    inspect = tmp_path / "inspect.json"
    inspect.write_text(json.dumps([{
        "Config": {
            "Cmd": ["ros2", "launch", "room_memory", "room_memory.launch.py", "rig_guard:=true"],
        }
    }]), encoding="utf-8")
    log = tmp_path / "node.log"
    log.write_text("[INFO] rig guard: 3 ICP estimates, 1.2 ms of the window\n", encoding="utf-8")
    topics = tmp_path / "topics.jsonl"
    topics.write_text(json.dumps({
        "op": "ready",
        "topics": ["/room_memory/status", "/room_memory/events", "/room_memory/rig"],
    }) + "\n", encoding="utf-8")
    proc, out, stdout_path, stderr_path = _spawn_live(
        tmp_path, "--inspect", str(inspect), "--log", str(log), "--topics", str(topics),
    )
    try:
        _wait_live(stdout_path, "push mark", proc)
        unknowns = "".join("unknown\n" for _ in module.QUESTIONS)
        proc.stdin.write("\n\n" + unknowns)
        proc.stdin.flush()
        _wait_live(stdout_path, "capture files are closed", proc)
        with topics.open("a", encoding="utf-8") as handle:
            for row in (
                {"topic": "/room_memory/status", "message": {"state": "tracking", "map_epoch": 4}},
                {"topic": "/room_memory/status", "message": {"state": "rig_moved", "map_epoch": 4}},
                {"topic": "/room_memory/status", "message": {"state": "tracking", "map_epoch": 4}},
                {"topic": "/room_memory/rig", "message": {
                    "imu_ok": True, "frame_epoch": 9, "map_epoch": 4, "outcome": "moved",
                }},
                {"topic": "/room_memory/rig", "message": {
                    "imu_ok": True, "frame_epoch": 9, "map_epoch": 4, "outcome": "realigned",
                }},
            ):
                handle.write(json.dumps(row) + "\n")
        proc.stdin.write("\n")
        proc.stdin.flush()
        code = proc.wait(timeout=10)
    finally:
        _stop_live(proc)
    assert code == 0, stderr_path.read_text(encoding="utf-8")
    body = marks(out)
    bump = body["scenarios"][0]
    assert bump["bump_evaluation"] == "observed"
    assert body["session"]["bump_evaluation"] == "observed"
    assert body["session"]["bump_evaluation_reason"] == "observed"
    assert bump["observations"]["auto_relearn"] == "on"
    assert bump["observation_sources"]["auto_relearn"] == "launch_default"
    assert bump["observations"]["relearn_after_s"] == 60
    assert bump["observations"]["imu"] == "yes"
    assert bump["observations"]["map_epoch_before"] == 4
    assert bump["observations"]["frame_epoch_after"] == 9
    assert bump["observations"]["recovery"] == "realigned"
    assert bump["observations"]["unexpected_candidates"] == "none"
    assert body["topics"]["read_after_bump"] is True
    assert body["topics"]["applied_to_bump"] is True
    assert body["topics"]["lines"] == 6
    assert body["topics"]["lines"] == len(topics.read_text(encoding="utf-8").splitlines())
    assert body["inspect"]["applied_to_bump"] is True
    assert body["capture_closed"] is True


def _finished_live_payload(module):
    row = module._blank_bump(30, 30)
    row["push_mono_ns"] = 10
    row["t0_mono_ns"] = 10
    row["restore_mono_ns"] = 40
    row["t1_mono_ns"] = 40
    row["completion"] = "complete"
    row["actual_duration_s"] = 30
    row["wall_start_utc"] = "2026-01-01T00:00:00Z"
    row["wall_end_utc"] = "2026-01-01T00:00:30Z"
    row["observations"] = {key: "unknown" for key in module.OBSERVATION_KEYS}
    row["observations"]["frame_epoch_before"] = "unshown"
    row["observations"]["frame_epoch_after"] = "unshown"
    session = module._session("complete", "", "not_evaluated", "guard_unobserved")
    payload = module._payload(
        "live",
        "utc",
        [row],
        session,
        {"status": None, "topics": None, "recording": None, "inspect": None, "log": None},
    )
    payload["capture_closed"] = True
    return payload


def _run_closed_live(module, out, topics, payload):
    real = module._conduct_bump
    module._conduct_bump = lambda *_args, **_kwargs: (payload, 0)
    stderr = _Capture()
    original = sys.stderr
    sys.stderr = stderr
    try:
        code = module.main(["--live", "--topics", str(topics), "--out", str(out)])
    finally:
        sys.stderr = original
        module._conduct_bump = real
    return code, stderr.text


def test_unstarted_topics_names_empty_and_missing_ready(tmp_path):
    module = load()
    empty = tmp_path / "empty.jsonl"
    empty.write_text("", encoding="utf-8")
    empty_summary = module.summarize_topics(empty)
    assert empty_summary["lines"] == 0
    assert empty_summary["saw_ready"] is False
    assert "no lines" in module._unstarted_topics(empty_summary)
    bare = tmp_path / "bare.jsonl"
    bare.write_text(
        json.dumps({"topic": "/room_memory/status", "message": {"state": "tracking", "map_epoch": 1}}) + "\n",
        encoding="utf-8",
    )
    bare_summary = module.summarize_topics(bare)
    assert bare_summary["lines"] == 1
    assert bare_summary["saw_ready"] is False
    assert "no op: ready line" in module._unstarted_topics(bare_summary)
    ready = tmp_path / "ready.jsonl"
    ready.write_text(json.dumps({"op": "ready"}) + "\n", encoding="utf-8")
    ready_summary = module.summarize_topics(ready)
    assert ready_summary["saw_ready"] is True
    assert module._unstarted_topics(ready_summary) is None
    assert module._unstarted_topics(None) is None


def test_post_close_unstarted_topics_sets_read_error_and_still_writes(tmp_path):
    module = load()
    empty_topics = tmp_path / "empty.jsonl"
    empty_topics.write_text("", encoding="utf-8")
    dry = tmp_path / "dry"
    assert module.main(["--out", str(dry), "--topics", str(empty_topics)]) == 0
    dry_body = marks(dry)
    assert dry_body["topics"]["lines"] == 0
    assert dry_body["topics"]["read_after_bump"] is False
    assert "read_error" not in dry_body

    code, stderr = _run_closed_live(module, tmp_path / "empty-out", empty_topics, _finished_live_payload(module))
    assert code == 2
    assert "topics file has no lines" in stderr
    empty_body = marks(tmp_path / "empty-out")
    assert "topics file has no lines" in empty_body["read_error"]
    assert empty_body["scenarios"][0]["push_mono_ns"] == 10
    assert empty_body["scenarios"][0]["restore_mono_ns"] == 40
    assert empty_body["topics"]["bump_evidence"] is None
    assert (tmp_path / "empty-out" / "scenario-marks.csv").is_file()

    bare = tmp_path / "bare.jsonl"
    bare.write_text(
        json.dumps({"topic": "/room_memory/status", "message": {"state": "rig_moved", "map_epoch": 4}}) + "\n",
        encoding="utf-8",
    )
    code, stderr = _run_closed_live(module, tmp_path / "bare-out", bare, _finished_live_payload(module))
    assert code == 2
    assert "topics file has no op: ready line" in stderr
    bare_body = marks(tmp_path / "bare-out")
    assert "no op: ready line" in bare_body["read_error"]
    assert bare_body["topics"]["bump_evidence"] is None
    assert bare_body["scenarios"][0]["observations"]["rig_moved"] == "unknown"
    assert (tmp_path / "bare-out" / "scenario-marks.json").is_file()

    ready = tmp_path / "ready.jsonl"
    ready.write_text(
        json.dumps({"op": "ready", "topics": ["/room_memory/status", "/room_memory/events", "/room_memory/rig"]}) + "\n",
        encoding="utf-8",
    )
    code, stderr = _run_closed_live(module, tmp_path / "ready-out", ready, _finished_live_payload(module))
    assert code == 0, stderr
    ready_body = marks(tmp_path / "ready-out")
    assert "read_error" not in ready_body
    assert ready_body["topics"]["saw_ready"] is True
    assert ready_body["topics"]["read_after_bump"] is True


def test_stop_during_the_post_close_read_writes_the_finished_bump(tmp_path):
    module = load()
    out = tmp_path / "out"
    topics = tmp_path / "bump-topics.jsonl"
    topics.write_text(
        json.dumps({"op": "ready", "topics": ["/room_memory/status", "/room_memory/rig"]}) + "\n",
        encoding="utf-8",
    )
    payload = _finished_live_payload(module)
    real_load = module._load_inputs
    real_conduct = module._conduct_bump

    def load_inputs(*args, **kwargs):
        if kwargs.get("after_bump"):
            raise KeyboardInterrupt
        return real_load(*args, **kwargs)

    module._load_inputs = load_inputs
    module._conduct_bump = lambda *_args, **_kwargs: (payload, 0)
    try:
        code = module.main(["--live", "--topics", str(topics), "--out", str(out)])
    finally:
        module._load_inputs = real_load
        module._conduct_bump = real_conduct
    assert code == 130
    body = marks(out)
    bump = body["scenarios"][0]
    assert bump["push_mono_ns"] == 10
    assert bump["restore_mono_ns"] == 40
    assert bump["completion"] == "complete"
    assert bump["observations"]["rig_guard"] == "unknown"
    assert body["session"]["end_reason"] == "complete"
    assert "read_error" not in body
    assert (out / "scenario-marks.csv").is_file()
    assert (out / "scenario-marks.json").is_file()


def test_stop_during_post_close_judge_writes_the_finished_bump(tmp_path):
    module = load()
    out = tmp_path / "out"
    topics = tmp_path / "bump-topics.jsonl"
    topics.write_text(
        json.dumps({"op": "ready", "topics": ["/room_memory/status", "/room_memory/rig"]}) + "\n",
        encoding="utf-8",
    )
    payload = _finished_live_payload(module)
    real_judge = module.judge_bump
    real_conduct = module._conduct_bump

    def judge(_observations):
        raise KeyboardInterrupt

    module.judge_bump = judge
    module._conduct_bump = lambda *_args, **_kwargs: (payload, 0)
    try:
        code = module.main(["--live", "--topics", str(topics), "--out", str(out)])
    finally:
        module.judge_bump = real_judge
        module._conduct_bump = real_conduct
    assert code == 130
    body = marks(out)
    bump = body["scenarios"][0]
    assert bump["push_mono_ns"] == 10
    assert bump["restore_mono_ns"] == 40
    assert bump["completion"] == "complete"
    assert (out / "scenario-marks.csv").is_file()


def _spawn_live(tmp_path, *extra):
    out = tmp_path / "marks"
    stdout_path = tmp_path / "stdout.txt"
    stderr_path = tmp_path / "stderr.txt"
    stdout_f = stdout_path.open("w", encoding="utf-8")
    stderr_f = stderr_path.open("w", encoding="utf-8")
    proc = subprocess.Popen(
        [sys.executable, str(SCRIPT), "--live", "--bump-s", "30", "--out", str(out), *extra],
        stdin=subprocess.PIPE,
        stdout=stdout_f,
        stderr=stderr_f,
        text=True,
        env={**os.environ, "PYTHONUNBUFFERED": "1"},
    )
    stdout_f.close()
    stderr_f.close()
    return proc, out, stdout_path, stderr_path


def _wait_live(path, needle, proc, timeout_s=10):
    deadline = time.monotonic() + timeout_s
    last = ""
    while time.monotonic() < deadline:
        last = path.read_text(encoding="utf-8") if path.exists() else ""
        if needle in last:
            return last
        if proc.poll() is not None:
            break
        time.sleep(0.02)
    raise AssertionError("missing %r (exit %s): %r" % (needle, proc.poll(), last))


def _stop_live(proc):
    if proc.poll() is None:
        proc.send_signal(signal.SIGKILL)
        proc.wait(timeout=5)
    if proc.stdin and not proc.stdin.closed:
        proc.stdin.close()


class _ExplodingStdin:
    """A blocking read has to fail the test. End-of-input would hide it."""

    def read(self, _n=-1):
        raise AssertionError("dry-run read stdin")

    def readline(self, _n=-1):
        raise AssertionError("dry-run read stdin")

    def fileno(self):
        raise AssertionError("dry-run selected stdin")

    @property
    def buffer(self):
        raise AssertionError("dry-run read stdin")


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
