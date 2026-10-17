"""Read-only subscriptions for the three room-memory topics.

Run inside the room-memory container, on stdin. This process does not advertise,
and it does not create a publisher, client, or service. QoS matches the node:
status and activity are reliable transient_local depth 1, events are reliable
volatile depth 100.

Jazzy's ``Node`` constructor advertises ``/parameter_events``, turns on rosout,
starts the parameter services, and publishes parameter events while ``TimeSource``
and ``TypeDescriptionService`` declare parameters. Those side effects are suppressed
from the constructor call onward. Subscriptions are created only after that.
"""

import json
import math
import sys
import time

import rclpy
from labcommand_msgs.msg import Activity
from labcommand_msgs.msg import ChangeEvent
from labcommand_msgs.msg import MemoryStatus
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy

EVENTS_SEQ_NONE = (1 << 64) - 1
HZ_UNAVAILABLE = 65535
KIND = {0: "appeared", 1: "disappeared", 2: "moved"}
STATE = {0: "candidate", 1: "confirmed", 2: "accepted", 3: "reverted", 4: "expired"}
MEMORY = {0: "building", 1: "tracking", 2: "rig_moved", 3: "degraded", 4: "paused"}
TOPICS = ["/room_memory/status", "/room_memory/events", "/room_memory/activity"]


def latched_qos():
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def volatile_qos():
    return QoSProfile(
        depth=100,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


def emit(payload):
    sys.stdout.write(json.dumps(payload, allow_nan=False, separators=(",", ":")) + "\n")
    sys.stdout.flush()


def num(value):
    if value is None:
        return None
    number = float(value)
    if math.isnan(number):
        return None
    return number


def stamp_ns(stamp):
    return int(stamp.sec) * 1000000000 + int(stamp.nanosec)


def hz(value):
    if value is None or int(value) == HZ_UNAVAILABLE:
        return None
    return int(value)


def events_seq(value):
    if value is None or int(value) == EVENTS_SEQ_NONE:
        return None
    return int(value)


def xyz(values):
    numbers = [num(item) for item in values]
    if all(item is None for item in numbers):
        return None
    return numbers


class _DroppedPublisher:
    """Replaces the parameter-event publisher ``Node`` builds during ``__init__``.

    ``publish`` discards the message. The object is not a middleware publisher,
    so declaring ``use_sim_time`` and ``start_type_description_service`` does not
    advertise ``/parameter_events``.
    """

    def publish(self, *_args, **_kwargs):
        return None


class Reader(Node):
    def __init__(self):
        super().__init__(
            "room_memory_acceptance_subscribe",
            enable_rosout=False,
            start_parameter_services=False,
            enable_logger_service=False,
            parameter_overrides=[
                Parameter("use_sim_time", Parameter.Type.BOOL, False),
                Parameter("start_type_description_service", Parameter.Type.BOOL, False),
            ],
        )
        self.saw_status = False
        self.saw_activity = False
        self.subs = []
        self.subs.append(self.create_subscription(MemoryStatus, "/room_memory/status", self.on_status, latched_qos()))
        self.subs.append(self.create_subscription(Activity, "/room_memory/activity", self.on_activity, latched_qos()))
        self.subs.append(self.create_subscription(ChangeEvent, "/room_memory/events", self.on_event, volatile_qos()))

    def create_publisher(self, *_args, **_kwargs):
        return _DroppedPublisher()

    def create_client(self, *_args, **_kwargs):
        raise RuntimeError("room-memory acceptance subscriber does not create clients")

    def create_service(self, *_args, **_kwargs):
        raise RuntimeError("room-memory acceptance subscriber does not create services")

    def matched(self):
        return all(sub.get_publisher_count() > 0 for sub in self.subs)

    def on_status(self, message):
        initial = not self.saw_status
        self.saw_status = True
        emit(self.record("/room_memory/status", message, initial, {
            "state": MEMORY.get(int(message.state), str(int(message.state))),
            "events_seq": events_seq(message.events_seq),
            "mid360_hz": hz(message.per_sensor_hz[0]),
            "jt128_hz": hz(message.per_sensor_hz[1]),
            "dropped_windows": int(message.dropped_windows),
            "n_open_events": int(message.n_open_events),
            "pose_residual_m": num(message.pose_residual_m),
        }))

    def on_activity(self, message):
        initial = not self.saw_activity
        self.saw_activity = True
        aabb = None
        centroid = xyz(message.centroid)
        flat = [num(item) for item in message.aabb]
        if centroid is not None and not any(item is None for item in flat):
            aabb = [flat[0:3], flat[3:6]]
        emit(self.record("/room_memory/activity", message, initial, {
            "active": bool(message.active),
            "since_s": num(message.since_s),
            "sensors": int(message.sensors),
            "largest_sensor": int(message.largest_sensor),
            "n_regions": int(message.n_regions),
            "voxel_count": int(message.voxel_count),
            "centroid": centroid,
            "aabb": aabb,
        }))

    def on_event(self, message):
        aabb = [num(item) for item in message.aabb]
        emit(self.record("/room_memory/events", message, False, {
            "state": STATE.get(int(message.state), str(int(message.state))),
            "kind": KIND.get(int(message.kind), str(int(message.kind))),
            "event_id": int(message.event_id),
            "sensors": int(message.sensors),
            "centroid": xyz(message.centroid),
            "aabb": [aabb[0:3], aabb[3:6]],
            "from_centroid": xyz(message.from_centroid),
            "voxel_count": int(message.voxel_count),
        }))

    def record(self, topic, message, initial, body):
        mono = time.monotonic_ns()
        return {
            "schema": "room-memory-acceptance-log-v1",
            "topic": topic,
            "mono_ns": mono,
            "stamp_ns": stamp_ns(message.stamp),
            "seq": int(message.seq),
            "publisher_generation": int(message.publisher_generation),
            "initial": bool(initial),
            "message": body,
        }


def main():
    rclpy.init()
    node = Reader()
    try:
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline and not node.matched():
            rclpy.spin_once(node, timeout_sec=0.2)
        emit({
            "schema": "room-memory-acceptance-log-v1",
            "op": "ready",
            "mono_ns": time.monotonic_ns(),
            "topics": list(TOPICS),
            "matched": bool(node.matched()),
        })
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
