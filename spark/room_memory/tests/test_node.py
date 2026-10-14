"""rclpy wiring. Runs only where ROS 2 Jazzy and a built labcommand_msgs are on the path."""

import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("labcommand_msgs.msg")

from labcommand_msgs.msg import MemoryStatus as MemoryStatusMsg  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.qos import QoSProfile, ReliabilityPolicy  # noqa: E402

from room_memory.memory import SENSORS  # noqa: E402
from room_memory.messages import MemoryState, MemoryStatus  # noqa: E402
from room_memory.node import EVENTS_TOPIC, STATUS_TOPIC, RoomMemoryNode  # noqa: E402


@pytest.fixture
def node():
    rclpy.init()
    node = RoomMemoryNode(parameter_overrides=[
        Parameter("state_dir", value=""), Parameter("sensors", value=["mid360"]), Parameter("max_range", value=0.0),
    ])
    yield node
    node.destroy_node()
    rclpy.shutdown()


def test_node_wires_topics_and_parameters(node):
    assert node.config.state_dir is None and node.config.max_range is None
    assert node.config.sensors == ("mid360",)
    assert {publisher.topic_name for publisher in node.publishers} >= {EVENTS_TOPIC, STATUS_TOPIC}
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
