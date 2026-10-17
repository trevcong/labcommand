"""rclpy wiring. Runs only where ROS 2 Jazzy and a built labcommand_msgs are on the path."""

from dataclasses import replace

import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("labcommand_msgs.msg")

from labcommand_msgs.msg import Activity as ActivityMsg  # noqa: E402
from labcommand_msgs.msg import MemoryStatus as MemoryStatusMsg  # noqa: E402
from labcommand_msgs.msg import RequestSync as RequestSyncMsg  # noqa: E402
from labcommand_msgs.srv import RequestSync as RequestSyncSrv  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from rclpy.serialization import deserialize_message, serialize_message  # noqa: E402

from room_memory.generation import GenerationError, generation_path  # noqa: E402
from room_memory.memory import SENSORS  # noqa: E402
from room_memory.messages import (  # noqa: E402
    Activity, Header, MemoryState, MemoryStatus, RequestSync, SyncEnd, SyncTopic,
)
from room_memory.node import (  # noqa: E402
    ACTIVITY_TOPIC, EVENTS_TOPIC, OPERATOR_SERVICE, OPERATOR_TOPIC, REQUEST_SERVICE, STATUS_TOPIC, SYNC_END_TOPIC,
    RoomMemoryNode,
)
from test_messages import T10, TOKEN_A, appeared_event  # noqa: E402


def _make_node(state_dir, *extra):
    return RoomMemoryNode(parameter_overrides=[
        Parameter("state_dir", value=str(state_dir)), Parameter("sensors", value=["mid360"]),
        Parameter("max_range", value=0.0), *extra,
    ])


@pytest.fixture
def node(tmp_path):
    rclpy.init()
    node = _make_node(tmp_path)
    yield node
    node.destroy_node()
    node.generation.release()
    rclpy.shutdown()


def test_node_wires_topics_and_parameters(node, tmp_path):
    assert node.config.state_dir == str(tmp_path) and node.config.max_range is None
    assert node.config.sensors == ("mid360",)
    assert {publisher.topic_name for publisher in node.publishers} >= {
        EVENTS_TOPIC, STATUS_TOPIC, ACTIVITY_TOPIC, SYNC_END_TOPIC}
    subscribed = {subscription.topic_name for subscription in node.subscriptions}
    assert SENSORS["mid360"].topic in subscribed and SENSORS["jt128"].topic not in subscribed


def test_status_bytes_reach_a_raw_subscriber(node):
    received = []
    node.create_subscription(
        MemoryStatusMsg, STATUS_TOPIC, received.append, QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE),
        raw=True,
    )
    for _ in range(50):
        node._publish_status()
        rclpy.spin_once(node, timeout_sec=0.1)
        if received:
            break
    status = MemoryStatus.from_cdr(received[0])
    assert status.header.room_id == "lab" and status.state == MemoryState.DEGRADED and status.jt128_hz is None
    assert status.events_seq is None


def test_activity_goes_out_with_each_status(node, monkeypatch):
    sent = []
    monkeypatch.setattr(node.activity, "publish", sent.append)
    node._publish_status()
    node._publish_status()
    messages = [Activity.from_cdr(data) for data in sent]
    assert [m.header.seq for m in messages] == [0, 1] and not any(m.active for m in messages)
    assert all(m.since_s is None and m.header.room_id == "lab" for m in messages)
    assert all(m.publisher_generation == node.generation.generation for m in messages)


def test_activity_reaches_a_late_transient_local_subscriber(node):
    node._publish_status()
    received = []
    node.create_subscription(
        ActivityMsg, ACTIVITY_TOPIC, received.append,
        QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        raw=True,
    )
    for _ in range(50):
        rclpy.spin_once(node, timeout_sec=0.1)
        if received:
            break
    assert [Activity.from_cdr(data).header.seq for data in received] == [0]


def test_status_reaches_a_late_transient_local_subscriber(node):
    node._publish_status()
    received = []
    node.create_subscription(
        MemoryStatusMsg, STATUS_TOPIC, received.append,
        QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        raw=True,
    )
    for _ in range(50):
        rclpy.spin_once(node, timeout_sec=0.1)
        if received:
            break
    statuses = [MemoryStatus.from_cdr(data) for data in received]
    assert [(s.header.seq, s.publisher_generation) for s in statuses] == [(0, node.generation.generation)]


def test_publishers_declare_durability(node):
    transient = {STATUS_TOPIC, ACTIVITY_TOPIC}
    for publisher in node.publishers:
        if not publisher.topic_name.startswith("/room_memory/"):
            continue
        want = DurabilityPolicy.TRANSIENT_LOCAL if publisher.topic_name in transient else DurabilityPolicy.VOLATILE
        assert publisher.qos_profile.durability == want, publisher.topic_name


def test_status_reports_the_last_published_event_seq(node, monkeypatch):
    events = [replace(appeared_event(), header=Header("lab", 0, seq, T10)) for seq in (0, 1)]

    def close_window(stamp):
        node.memory.event_seq = len(events)  # the engine numbers the events it returns
        return events

    monkeypatch.setattr(node.memory, "close_window", close_window)
    published = []
    monkeypatch.setattr(node.events, "publish", published.append)
    statuses = []
    monkeypatch.setattr(node.status, "publish", statuses.append)
    node._publish_status()
    node._close_window()
    node._publish_status()
    assert len(published) == 2
    assert [MemoryStatus.from_cdr(data).events_seq for data in statuses] == [None, 1]


def test_generation_is_allocated_before_the_first_publish(node, tmp_path, monkeypatch):
    stored = int(generation_path(tmp_path, "lab").read_bytes())
    assert stored == node.generation.generation == node.memory.publisher_generation
    statuses = []
    monkeypatch.setattr(node.status, "publish", statuses.append)
    node._publish_status()
    assert MemoryStatus.from_cdr(statuses[0]).publisher_generation == stored


def test_node_refuses_to_start_without_state_dir(tmp_path):
    rclpy.init()
    try:
        with pytest.raises(GenerationError):
            _make_node("")
        with pytest.raises(GenerationError):
            _make_node(tmp_path / "missing")
    finally:
        rclpy.shutdown()


def test_answer_publishes_messages_then_sync_end(node, monkeypatch):
    order = []
    monkeypatch.setattr(node.sync_events, "publish", lambda data: order.append(("events", data)))
    monkeypatch.setattr(node.sync_end, "publish", lambda data: order.append(("end", data)))
    request = RequestSync(Header("lab", (1 << 64) - 1, 0, T10), None, SyncTopic.EVENTS, 0, TOKEN_A)
    node._answer(request.to_cdr())
    assert [topic for topic, _ in order] == ["end"]
    end = SyncEnd.from_cdr(order[0][1])
    assert end.token == TOKEN_A and end.bound is None and end.publisher_generation == node.generation.generation


def _service_request(request):
    return RequestSyncSrv.Request(request=deserialize_message(request.to_cdr(), RequestSyncMsg))


def test_node_offers_the_request_service(node):
    services = dict(node.get_service_names_and_types_by_node("room_memory", "/"))
    assert services[REQUEST_SERVICE] == ["labcommand_msgs/srv/RequestSync"]


def test_service_request_has_the_message_layout():
    # The Mac sends its RequestSync CDR as the service-call payload (D39): the request has the
    # message's layout. rmw leaves alignment padding uninitialized, so compare length and value.
    request = RequestSync(Header("lab", 7, 41, T10), (7, 41), SyncTopic.EVENTS, 1234, TOKEN_A)
    data = serialize_message(_service_request(request))
    assert len(data) == len(request.to_cdr())
    assert RequestSync.from_cdr(data) == request
    assert deserialize_message(request.to_cdr(), RequestSyncSrv.Request).request == _service_request(request).request


def test_service_answers_like_the_topic(node, monkeypatch):
    order = []
    monkeypatch.setattr(node.sync_events, "publish", lambda data: order.append(("events", data)))
    monkeypatch.setattr(node.sync_end, "publish", lambda data: order.append(("end", data)))
    request = RequestSync(Header("lab", (1 << 64) - 1, 0, T10), None, SyncTopic.EVENTS, 0, TOKEN_A)
    response = node._serve_request(_service_request(request), RequestSyncSrv.Response())
    assert response.accepted
    assert [topic for topic, _ in order] == ["end"]
    end = SyncEnd.from_cdr(order[0][1])
    assert end.token == TOKEN_A and end.publisher_generation == node.generation.generation


def test_service_refuses_another_room(node, monkeypatch):
    order = []
    monkeypatch.setattr(node.sync_end, "publish", lambda data: order.append(data))
    request = RequestSync(Header("elsewhere", (1 << 64) - 1, 0, T10), None, SyncTopic.EVENTS, 0, TOKEN_A)
    assert not node._serve_request(_service_request(request), RequestSyncSrv.Response()).accepted
    assert order == []


def test_the_operator_service_is_off_by_default(node):
    services = dict(node.get_service_names_and_types_by_node("room_memory", "/"))
    assert OPERATOR_SERVICE not in services
    # The topic too (review of #31): any peer on the ROS domain could publish on it.
    assert OPERATOR_TOPIC not in {subscription.topic_name for subscription in node.subscriptions}


def test_operator_actions_true_offers_the_topic_and_the_service(tmp_path):
    rclpy.init()
    node = _make_node(tmp_path, Parameter("operator_actions", value=True))
    try:
        assert OPERATOR_TOPIC in {subscription.topic_name for subscription in node.subscriptions}
        assert OPERATOR_SERVICE in {service.srv_name for service in node.services}
    finally:
        node.destroy_node()
        node.generation.release()
        rclpy.shutdown()


def test_operator_actions_answer_with_an_outcome(node, monkeypatch):
    from room_memory.messages import EventState, OperatorAction, OperatorActionKind, OperatorOutcome
    published = []
    monkeypatch.setattr(node.events, "publish", lambda data: published.append(data))
    assert node._act(b"\x00\x01\x00\x00") == OperatorOutcome.INVALID
    action = OperatorAction(Header("lab", 0, 0, T10), OperatorActionKind.CONFIRM, 5, EventState.CANDIDATE,
                            node.generation.generation, "", "trevor", "00000000-0000-4000-8000-000000000001")
    assert node._act(action.to_cdr()) == OperatorOutcome.NOT_TRACKING
    assert published == []

