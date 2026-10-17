"""Read-only transport boundaries. These tests do not start docker or open a socket."""

import ast
import json
import subprocess
import sys
from pathlib import Path

from acceptance.log import load_jsonl, write_jsonl
from acceptance.score import score
from acceptance.transport import ros2_exec_command, stamp_record

ROOT = Path(__file__).resolve().parents[1]
REPO = ROOT.parents[2]
TOPICS = ["/room_memory/status", "/room_memory/activity", "/room_memory/events"]


def _call_attrs(tree):
    names = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
            names.append(node.func.attr)
    return names


def test_ros2_source_has_subscriptions_and_no_writers():
    source = (ROOT / "ros2_subscribe.py").read_text()
    tree = ast.parse(source)
    calls = _call_attrs(tree)
    assert "create_subscription" in calls
    for banned in ("create_publisher", "create_client", "create_service"):
        assert banned not in calls
    defs = [node.name for node in ast.walk(tree) if isinstance(node, ast.FunctionDef)]
    assert "create_publisher" in defs
    assert "enable_rosout=False" in source
    assert "start_parameter_services=False" in source
    assert "enable_logger_service=False" in source
    assert "start_type_description_service" in source
    assert "use_sim_time" in source
    assert "TRANSIENT_LOCAL" in source
    assert "VOLATILE" in source
    assert "ssh" not in source
    assert "events_seq" in source


# Jazzy Node.__init__ creates /parameter_events, enables rosout, starts parameter
# services, and TimeSource plus TypeDescriptionService declare parameters (which
# publishes). The double below does those steps. The subscriber has to stay quiet
# against that constructor, not only against an AST scan of this file.
_JAZZY_PROBE = r"""
import importlib.util
import json
import sys
import types

class _Param:
    def __init__(self, name, value):
        self.name = name
        self.value = value

class _Entity:
    def __init__(self, kind, name):
        self.kind = kind
        self.name = name

class _Publisher:
    def __init__(self, node, topic):
        self.node = node
        self.topic = topic
    def publish(self, payload):
        self.node.publications.append({"topic": self.topic, "payload": payload})

class Parameter:
    class Type:
        BOOL = "bool"
    def __init__(self, name, type_=None, value=None):
        self.name = name
        self.type_ = type_
        self.value = value

class Node:
    def __init__(self, node_name, *, context=None, cli_args=None, namespace=None,
                 use_global_arguments=True, enable_rosout=True,
                 start_parameter_services=True, parameter_overrides=None,
                 allow_undeclared_parameters=False,
                 automatically_declare_parameters_from_overrides=False,
                 enable_logger_service=False):
        self._name = node_name
        self._publishers = []
        self._subscriptions = []
        self._services = []
        self._clients = []
        self._parameters = {}
        self._overrides = {}
        self.publications = []
        self.rosout_enabled = bool(enable_rosout)
        self.parameter_services_started = bool(start_parameter_services)
        self.logger_service_enabled = bool(enable_logger_service)
        for item in parameter_overrides or []:
            self._overrides[item.name] = item
        if enable_rosout:
            self.create_publisher("Log", "/rosout", 10)
        self._parameter_event_publisher = self.create_publisher(
            "ParameterEvent", "/parameter_events", 10)
        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", False)
        if self.get_parameter("use_sim_time").value is True:
            self.create_subscription("Clock", "/clock", lambda _message: None, 1)
        if start_parameter_services:
            for service in (
                "describe_parameters", "get_parameters", "get_parameter_types",
                "list_parameters", "set_parameters", "set_parameters_atomically",
            ):
                self.create_service("srv", "/" + node_name + "/" + service)
        if enable_logger_service:
            self.create_service("srv", "/" + node_name + "/get_logger_levels")
        if not self.has_parameter("start_type_description_service"):
            self.declare_parameter("start_type_description_service", True)
        if self.get_parameter("start_type_description_service").value:
            self._services.append(_Entity("service", node_name + "/get_type_description"))

    def has_parameter(self, name):
        return name in self._parameters

    def get_parameter(self, name):
        return self._parameters[name]

    def declare_parameter(self, name, value=None, descriptor=None):
        if name in self._overrides:
            value = self._overrides[name].value
        self._parameters[name] = _Param(name, value)
        self._parameter_event_publisher.publish({"name": name, "value": value})
        return self._parameters[name]

    def create_publisher(self, msg_type, topic, qos_profile, **kwargs):
        publisher = _Publisher(self, topic)
        self._publishers.append(publisher)
        return publisher

    def create_subscription(self, msg_type, topic, callback, qos_profile, **kwargs):
        entity = _Entity("subscription", topic)
        self._subscriptions.append(entity)
        return entity

    def create_service(self, srv_type, name, *args, **kwargs):
        entity = _Entity("service", name)
        self._services.append(entity)
        return entity

    def create_client(self, srv_type, name, *args, **kwargs):
        entity = _Entity("client", name)
        self._clients.append(entity)
        return entity

def _install(name, module):
    sys.modules[name] = module
    return module

rclpy = _install("rclpy", types.ModuleType("rclpy"))
rclpy.__path__ = []
node_mod = _install("rclpy.node", types.ModuleType("rclpy.node"))
node_mod.Node = Node
param_mod = _install("rclpy.parameter", types.ModuleType("rclpy.parameter"))
param_mod.Parameter = Parameter
qos_mod = _install("rclpy.qos", types.ModuleType("rclpy.qos"))
qos_mod.DurabilityPolicy = type("DurabilityPolicy", (), {"TRANSIENT_LOCAL": "transient_local", "VOLATILE": "volatile"})
qos_mod.ReliabilityPolicy = type("ReliabilityPolicy", (), {"RELIABLE": "reliable"})
class QoSProfile:
    def __init__(self, depth=10, reliability=None, durability=None, **kwargs):
        self.depth = depth
qos_mod.QoSProfile = QoSProfile
msgs = _install("labcommand_msgs", types.ModuleType("labcommand_msgs"))
msgs.__path__ = []
msg_mod = _install("labcommand_msgs.msg", types.ModuleType("labcommand_msgs.msg"))
msg_mod.Activity = type("Activity", (), {})
msg_mod.ChangeEvent = type("ChangeEvent", (), {})
msg_mod.MemoryStatus = type("MemoryStatus", (), {})

spec = importlib.util.spec_from_file_location("ros2_subscribe_under_test", sys.argv[1])
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)

def dump(node):
    return {
        "publishers": [item.topic for item in node._publishers],
        "subscriptions": [item.name for item in node._subscriptions],
        "services": [item.name for item in node._services],
        "clients": [item.name for item in node._clients],
        "publications": list(node.publications),
        "rosout_enabled": node.rosout_enabled,
        "parameter_services_started": node.parameter_services_started,
        "logger_service_enabled": node.logger_service_enabled,
    }

bare = Node("bare_jazzy_node")
reader = module.Reader()
sys.stdout.write(json.dumps({"bare": dump(bare), "reader": dump(reader)}))
"""


def test_subscriber_constructor_does_not_publish(tmp_path):
    probe = tmp_path / "jazzy_probe.py"
    probe.write_text(_JAZZY_PROBE)
    result = subprocess.run(
        [sys.executable, str(probe), str(ROOT / "ros2_subscribe.py")],
        check=False,
        capture_output=True,
        text=True,
        timeout=30,
    )
    assert result.returncode == 0, result.stderr
    report = json.loads(result.stdout)
    bare = report["bare"]
    assert "/parameter_events" in bare["publishers"]
    assert "/rosout" in bare["publishers"]
    assert {item["payload"]["name"] for item in bare["publications"]} >= {
        "use_sim_time", "start_type_description_service",
    }
    assert bare["parameter_services_started"] is True
    assert any(name.endswith("/get_type_description") for name in bare["services"])
    for service in (
        "describe_parameters", "get_parameters", "get_parameter_types",
        "list_parameters", "set_parameters", "set_parameters_atomically",
    ):
        assert any(name.endswith("/" + service) for name in bare["services"])
    reader = report["reader"]
    assert reader["publishers"] == []
    assert reader["publications"] == []
    assert reader["services"] == []
    assert reader["clients"] == []
    assert reader["subscriptions"] == TOPICS
    assert "/clock" not in reader["subscriptions"]
    assert reader["rosout_enabled"] is False
    assert reader["parameter_services_started"] is False
    assert reader["logger_service_enabled"] is False


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
