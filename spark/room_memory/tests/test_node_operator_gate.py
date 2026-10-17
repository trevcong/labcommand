"""``operator_actions`` gates both operator paths of the real ``RoomMemoryNode`` (D46).

Runs without ROS: ``room_memory/node.py`` is loaded against a minimal in-process stand-in for
rclpy and the message packages, whose ``deliver`` hands bytes to every subscription on a topic,
as DDS does for any peer on the domain. The module is loaded under its own name and never
registered, so ``test_node.py``'s ``importorskip("rclpy")`` still sees the real rclpy or none.
The rclpy twin of these tests is in ``test_node.py``.
"""

import importlib.util
import pathlib
import sys
import types

import pytest

from room_memory.memory import RoomMemory
from room_memory.messages import ChangeEvent, EventState, OperatorActionKind, OperatorOutcome
from test_operator import START, act, candidate, config

NODE_PY = pathlib.Path(__file__).resolve().parents[1] / "room_memory" / "node.py"


class _Value:
    def __init__(self, value):
        self.value = value


class _Logger:
    def __init__(self):
        self.lines = []

    def info(self, line, **_):
        self.lines.append(line)

    warning = fatal = info


class _Clock:
    def now(self):
        return types.SimpleNamespace(seconds_nanoseconds=lambda: (START.sec + 5, 0))


class _Publisher:
    def __init__(self, topic):
        self.topic_name = topic
        self.sent = []

    def publish(self, data):
        self.sent.append(data)


class _Node:
    """What ``RoomMemoryNode`` calls on ``rclpy.node.Node``, recorded."""

    def __init__(self, name, parameter_overrides=None):
        self.name = name
        self._overrides = dict(parameter_overrides or {})
        self.publishers, self.subscriptions, self.services = [], [], []
        self._logger = _Logger()

    def declare_parameter(self, name, default):
        return _Value(self._overrides.get(name, default))

    def create_publisher(self, _type, topic, _qos):
        self.publishers.append(_Publisher(topic))
        return self.publishers[-1]

    def create_subscription(self, _type, topic, callback, _qos, raw=False):
        self.subscriptions.append((topic, callback, raw))

    def create_service(self, _type, name, callback):
        self.services.append((name, callback))

    def create_timer(self, _period, _callback):
        pass

    def get_logger(self):
        return self._logger

    def get_clock(self):
        return _Clock()

    def deliver(self, topic, data):
        """A peer publishes ``data`` on ``topic``: every subscription to it gets the bytes."""
        for name, callback, raw in self.subscriptions:
            if name == topic:
                assert raw
                callback(data)


def _module(monkeypatch, name, **attributes):
    module = types.ModuleType(name)
    module.__dict__.update(attributes)
    monkeypatch.setitem(sys.modules, name, module)
    return module


@pytest.fixture
def node_module(monkeypatch):
    """``room_memory/node.py`` over the stand-in; ``sys.modules`` is restored afterwards."""
    stub = type("Stub", (), {})
    _module(monkeypatch, "rclpy")
    _module(monkeypatch, "rclpy.node", Node=_Node)
    _module(monkeypatch, "rclpy.qos", DurabilityPolicy=types.SimpleNamespace(VOLATILE=0, TRANSIENT_LOCAL=1),
            QoSProfile=lambda **fields: fields, ReliabilityPolicy=types.SimpleNamespace(RELIABLE=0),
            qos_profile_sensor_data=None)
    _module(monkeypatch, "rclpy.serialization", serialize_message=lambda message: message)
    _module(monkeypatch, "sensor_msgs")
    _module(monkeypatch, "sensor_msgs.msg", PointCloud2=stub)
    _module(monkeypatch, "labcommand_msgs")
    _module(monkeypatch, "labcommand_msgs.msg", **{name: stub for name in (
        "Activity", "ChangeEvent", "MapBricks", "MemoryStatus", "OperatorAction", "RequestSync", "SyncEnd")})
    _module(monkeypatch, "labcommand_msgs.srv", OperatorAction=stub, RequestSync=stub)
    spec = importlib.util.spec_from_file_location("room_memory._node_under_stub", NODE_PY)
    module = importlib.util.module_from_spec(spec)
    module.__package__ = "room_memory"
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def make_node(node_module, tmp_path):
    made = []

    def make(**overrides):
        # Its own state_dir: the generation allocation holds publisher.lock until release.
        state_dir = tmp_path / f"node{len(made)}"
        state_dir.mkdir()
        node = node_module.RoomMemoryNode(parameter_overrides={
            "state_dir": str(state_dir), "sensors": ["mid360"], "max_range": 0.0, **overrides})
        made.append(node)
        node.suppressed = state_dir / "lab" / "suppressed-mid360"
        # A memory with one candidate open (event id below), on the node's own generation.
        node.memory = RoomMemory(config(state_dir), START, node.generation.generation)
        node.event_id = candidate(node.memory)
        return node

    yield make
    for node in made:
        node.generation.release()


def _dismiss(node):
    return act(OperatorActionKind.DISMISS, node.event_id, EventState.CANDIDATE, n=1,
               generation=node.generation.generation)


def _events(node, module):
    return next(p for p in node.publishers if p.topic_name == module.EVENTS_TOPIC).sent


def test_without_the_flag_an_action_on_the_topic_changes_nothing(make_node, node_module):
    node = make_node()
    assert node_module.OPERATOR_TOPIC not in [topic for topic, _, _ in node.subscriptions]
    assert node_module.OPERATOR_SERVICE not in [name for name, _ in node.services]
    action = _dismiss(node)
    node.deliver(node_module.OPERATOR_TOPIC, action.to_cdr())
    assert _events(node, node_module) == []
    assert node.memory.suppressions("mid360") == []
    assert not node.suppressed.exists()
    assert not any("OperatorAction" in line for line in node.get_logger().lines)
    # The event is still a candidate and the action id is unused: the same action applies now,
    # once, and is not a repeat.
    outcome, events, repeat, _ = node.memory.operate(action, START)
    assert outcome == OperatorOutcome.APPLIED and not repeat
    assert [(e.event_id, e.state) for e in events] == [(node.event_id, EventState.REVERTED)]


def test_with_the_flag_the_same_action_on_the_topic_applies(make_node, node_module):
    # The control: the stand-in delivers, and the topic path is the node's own handler.
    node = make_node(operator_actions=True)
    assert node_module.OPERATOR_SERVICE in [name for name, _ in node.services]
    node.deliver(node_module.OPERATOR_TOPIC, _dismiss(node).to_cdr())
    assert [(e.event_id, e.state) for e in map(ChangeEvent.from_cdr, _events(node, node_module))] == [
        (node.event_id, EventState.REVERTED)]
    assert len(node.memory.suppressions("mid360")) == 1
    assert node.suppressed.exists()


def test_the_flag_defaults_off_and_is_read_once(make_node, node_module):
    assert [topic for topic, _, _ in make_node(operator_actions=False).subscriptions] == \
        [topic for topic, _, _ in make_node().subscriptions]
    on = [topic for topic, _, _ in make_node(operator_actions=True).subscriptions]
    assert on.count(node_module.OPERATOR_TOPIC) == 1
