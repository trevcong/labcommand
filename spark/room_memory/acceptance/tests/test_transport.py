"""Read-only transport boundaries. These tests do not start docker or open a socket."""

import ast
import json
from pathlib import Path

from acceptance.log import load_jsonl, write_jsonl
from acceptance.score import score
from acceptance.transport import ros2_exec_command, stamp_record

ROOT = Path(__file__).resolve().parents[1]
REPO = ROOT.parents[2]


def test_ros2_source_has_subscriptions_and_no_writers():
    source = (ROOT / "ros2_subscribe.py").read_text()
    tree = ast.parse(source)
    calls = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
            calls.append(node.func.attr)
    assert "create_subscription" in calls
    for banned in ("create_publisher", "create_client", "create_service"):
        assert banned not in calls
        assert banned not in source
    assert "TRANSIENT_LOCAL" in source
    assert "VOLATILE" in source
    assert "ssh" not in source
    assert "events_seq" in source


def test_main_does_not_insert_sys_path():
    source = (ROOT / "__main__.py").read_text()
    tree = ast.parse(source)
    for node in ast.walk(tree):
        if isinstance(node, ast.Attribute) and isinstance(node.value, ast.Name):
            assert not (node.value.id == "sys" and node.attr == "path")
    readme = (ROOT / "README.md").read_text()
    assert "PYTHONPATH=spark/room_memory python3 -m acceptance" in readme
    assert "does not ssh" in readme.lower() or "no ssh" in readme.lower()
    assert "CONFIRMED" in readme


def test_docker_exec_is_read_only_and_requires_a_container():
    command = ros2_exec_command("room-memory")
    assert command[:4] == ["docker", "exec", "-i", "room-memory"]
    joined = " ".join(command)
    for banned in ("ssh", "docker stop", "docker restart", "docker kill", "publish"):
        assert banned not in joined
    for bad in ("", "-it", "name;rm", "a b"):
        try:
            ros2_exec_command(bad)
        except ValueError:
            continue
        raise AssertionError(bad)


def test_parent_stamps_receipt_and_keeps_callback_time():
    stamped = stamp_record({"op": "ready", "mono_ns": 10, "matched": True, "topics": []}, 90)
    assert stamped["receipt_mono_ns"] == 90
    assert stamped["callback_mono_ns"] == 10
    assert stamped["schema"] == "room-memory-acceptance-log-v1"


def test_score_cli_writes_json_and_markdown(tmp_path):
    from acceptance.__main__ import main
    from acceptance.tests.support import event, marks, scenario, stream, target, PLACE

    messages = stream(9, 40, [event(20, "appeared", 1, PLACE)])
    mark = marks([scenario(
        "box_placed", "change", 10, 30, kind="appeared", protocol=120, actual=20, planned=120,
        completion="shortened", change_done=10, clear=12, targets=[target(1, PLACE)],
    )])
    log_path = tmp_path / "topics.jsonl"
    marks_path = tmp_path / "marks.json"
    write_jsonl(log_path, messages)
    marks_path.write_text(json.dumps(mark))
    code = main(["score", "--log", str(log_path), "--marks", str(marks_path), "--out", str(tmp_path)])
    assert code == 2
    report = json.loads((tmp_path / "report.json").read_text())
    markdown = (tmp_path / "report.md").read_text()
    assert report["true_detections"] == 1
    assert "markdown" not in report
    assert "true_detections: 1" in markdown
    assert load_jsonl(log_path)[0]["schema"] == "room-memory-acceptance-log-v1"
    assert (tmp_path / "report.md").is_file()


def test_score_matches_loaded_fixture(tmp_path):
    from acceptance.tests.support import dump_jsonl, load_jsonl as load_rows, roundtrip, stream

    messages = stream(0, 5)
    path = tmp_path / "topics.jsonl"
    dump_jsonl(path, messages)
    assert load_rows(path) == roundtrip(messages)
    loaded = load_jsonl(path)
    assert loaded[0]["schema"] == "room-memory-acceptance-log-v1"
    report = score(loaded, {"schema": "room-memory-acceptance-marks-v1", "clock": "receipt_mono",
                            "expected_sensors": [1, 2], "reference_setup": {}, "scenarios": []})
    assert report["exit_code"] == 2
