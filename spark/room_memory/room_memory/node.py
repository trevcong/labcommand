"""Thin rclpy node around ``RoomMemory``: raw CDR in, codec bytes out.

Subscribes to each configured lidar with the sensor-data QoS and ``raw=True``, so frames go
through the tested ``decode_pointcloud2`` rather than rclpy's deserializer. Publishes the
codec's CDR bytes on ``/room_memory/status`` and ``/room_memory/activity`` (1 Hz), ``/room_memory/events`` and
``/room_memory/map_bricks`` (after each window, when ``map_update`` has a message), and answers
``/room_memory/request_sync`` on the two sync topics, each answer followed by its ``SyncEnd`` on
``/room_memory/sync/end`` (protocol §2, §5); the ``labcommand_msgs`` classes only provide the type
support. A ``RequestSync`` also arrives as the ``/room_memory/sync/request`` service, which the :8767
bridge exposes to the Mac (D39); its request is serialized back to the message's bytes and answered
the same way. An ``OperatorAction`` (D46) arrives on ``/room_memory/operator`` from a relay, or, with
``operator_actions:=true``, as the ``/room_memory/operator/action`` service; its events go out on
``/room_memory/events``, and an accept's ACCEPTED right after the map message that carries it. The publisher generation is allocated (durably, under ``publisher.lock``) before any
publisher exists; without it the node refuses to start. A rejected frame logs one warning per
sensor per 10 s.
"""

import time

import rclpy
from labcommand_msgs.msg import Activity as ActivityMsg
from labcommand_msgs.msg import ChangeEvent as ChangeEventMsg
from labcommand_msgs.msg import MapBricks as MapBricksMsg
from labcommand_msgs.msg import MemoryStatus as MemoryStatusMsg
from labcommand_msgs.msg import OperatorAction as OperatorActionMsg
from labcommand_msgs.msg import RequestSync as RequestSyncMsg
from labcommand_msgs.msg import SyncEnd as SyncEndMsg
from labcommand_msgs.srv import OperatorAction as OperatorActionSrv
from labcommand_msgs.srv import RequestSync as RequestSyncSrv
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.serialization import serialize_message
from sensor_msgs.msg import PointCloud2

from .generation import GenerationError, allocate_generation
from .memory import SENSORS, MemoryConfig, RoomMemory
from .messages import CDRError, OperatorAction, OperatorOutcome, RequestSync, SyncTopic, Time
from .rejectlog import RejectionLog

STATUS_TOPIC = "/room_memory/status"
ACTIVITY_TOPIC = "/room_memory/activity"
EVENTS_TOPIC = "/room_memory/events"
MAP_TOPIC = "/room_memory/map_bricks"
REQUEST_TOPIC = "/room_memory/request_sync"
OPERATOR_TOPIC = "/room_memory/operator"
OPERATOR_SERVICE = "/room_memory/operator/action"
REQUEST_SERVICE = "/room_memory/sync/request"
SYNC_MAP_TOPIC = "/room_memory/sync/map_bricks"
SYNC_EVENTS_TOPIC = "/room_memory/sync/events"
SYNC_END_TOPIC = "/room_memory/sync/end"


def _time(node):
    sec, nanosec = node.get_clock().now().seconds_nanoseconds()
    return Time(int(sec), int(nanosec))


class RoomMemoryNode(Node):
    """ROS 2 wrapper: parameters → ``MemoryConfig``, timers → windows and status."""

    def __init__(self, **kwargs):
        super().__init__("room_memory", **kwargs)
        defaults = MemoryConfig()
        params = {
            "room_id": defaults.room_id,
            "sensors": list(defaults.sensors),
            "voxel_size": defaults.voxel_size,
            "max_range": defaults.max_range,
            "tolerance": defaults.tolerance,
            "min_voxels": defaults.min_voxels,
            "min_extent": defaults.min_extent,
            "window_s": defaults.window_s,
            "reference_windows": defaults.reference_windows,
            "recent_windows": defaults.recent_windows,
            "state_dir": defaults.state_dir,
            "occupied_min_hits": defaults.occupied_min_hits,
            "occupied_min_ratio": defaults.occupied_min_ratio,
            "free_min_seen": defaults.free_min_seen,
            "free_max_ratio": defaults.free_max_ratio,
            "candidate_windows": defaults.candidate_windows,
            "expire_windows": defaults.expire_windows,
            "fresh_min_voxels": defaults.fresh_min_voxels,
            "fresh_min_fraction": defaults.fresh_min_fraction,
            "stable_min_fraction": defaults.stable_min_fraction,
            "activity_radius_m": defaults.activity_radius_m,
            "learn_max_offset_m": defaults.learn_max_offset_m,
            "learn_windows": defaults.learn_windows,
            "map_sensor": defaults.map_sensor,
            "delta_period_s": defaults.delta_period_s,
            "keyframe_period_s": defaults.keyframe_period_s,
            "keyframe_delta_ratio": defaults.keyframe_delta_ratio,
            "suppress_clear_windows": defaults.suppress_clear_windows,
        }
        values = {name: self.declare_parameter(name, value).value for name, value in params.items()}
        # The operator service changes durable state through an unauthenticated port: off unless asked (D46).
        operator_actions = self.declare_parameter("operator_actions", False).value
        values["sensors"] = tuple(values["sensors"])
        values["max_range"] = values["max_range"] if values["max_range"] > 0 else None
        values["state_dir"] = values["state_dir"] or None
        self.config = MemoryConfig(**values)
        # Before any publisher: every message this process sends carries the generation. The
        # allocation holds publisher.lock until the process exits.
        try:
            self.generation = allocate_generation(self.config.state_dir, self.config.room_id, int(time.time() * 1000))
        except GenerationError as error:
            self.get_logger().fatal(f"refusing to start: {error}")
            raise
        self.memory = RoomMemory(self.config, _time(self), self.generation.generation)

        # Durability is declared on every topic (protocol §2, D24). Status and activity are full state at
        # 1 Hz, so a late subscriber (the bridge, when the first Mac subscribes) gets the last one at once;
        # a copy delivered again repeats its seq in its generation. Everything else is volatile: the Mac
        # gets the map and recent events by RequestSync, and a restarted node never receives an old
        # request. Same QoS as the C++ node.
        def volatile(depth):
            return QoSProfile(depth=depth, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)

        def latched():
            return QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.events = self.create_publisher(ChangeEventMsg, EVENTS_TOPIC, volatile(100))
        self.status = self.create_publisher(MemoryStatusMsg, STATUS_TOPIC, latched())
        self.activity = self.create_publisher(ActivityMsg, ACTIVITY_TOPIC, latched())
        self._rejections = RejectionLog()
        self.map = self.create_publisher(MapBricksMsg, MAP_TOPIC, volatile(10))
        self.sync_map = self.create_publisher(MapBricksMsg, SYNC_MAP_TOPIC, volatile(10))
        self.sync_events = self.create_publisher(ChangeEventMsg, SYNC_EVENTS_TOPIC, volatile(200))
        self.sync_end = self.create_publisher(SyncEndMsg, SYNC_END_TOPIC, volatile(10))
        self.create_subscription(RequestSyncMsg, REQUEST_TOPIC, self._answer, volatile(10), raw=True)
        self.create_service(RequestSyncSrv, REQUEST_SERVICE, self._serve_request)
        # Operator actions (D46). rclpy.spin runs every callback on one thread, so RoomMemory needs no lock.
        self.create_subscription(OperatorActionMsg, OPERATOR_TOPIC, self._act, volatile(10), raw=True)
        if operator_actions:
            self.create_service(OperatorActionSrv, OPERATOR_SERVICE, self._serve_action)
        for name in self.config.sensors:
            sensor = SENSORS[name]
            self.create_subscription(
                PointCloud2, sensor.topic, lambda data, name=name: self._on_cloud(name, data),
                qos_profile_sensor_data, raw=True,
            )
        self._last_status = time.monotonic()
        self.create_timer(self.config.window_s, self._close_window)
        self.create_timer(1.0, self._publish_status)
        self.get_logger().info(
            f"room '{self.config.room_id}' sensors {list(self.config.sensors)} state {self.memory.state.name} "
            f"map_epoch {self.memory.map_epoch} publisher_generation {self.generation.generation}"
        )

    def _on_cloud(self, name, data):
        if self.memory.add_cdr(name, data):
            return
        type_name, message = self.memory.channels[name].last_error
        line = self._rejections.observe(name, type_name, message)
        if line is not None:
            self.get_logger().warning(line)

    def _close_window(self):
        start = time.perf_counter()
        cpu = time.process_time()
        stamp = _time(self)
        events = self.memory.close_window(stamp)
        for event in events:
            self.events.publish(event.to_cdr())
        message = self.memory.map_update(stamp)
        if message is not None:
            self.map.publish(message)
            # An accept's ACCEPTED goes out right after the map message that carries it (protocol §4.2).
            for event in self.memory.accepted_after_map(stamp):
                self.events.publish(event.to_cdr())
                self.get_logger().info(f"event {event.event_id} ACCEPTED with map_epoch {event.header.map_epoch}")
        if self.memory.storage_error:
            self.get_logger().warning(f"storage: {self.memory.storage_error}", throttle_duration_sec=10.0)
        elapsed_ms = (time.perf_counter() - start) * 1000
        cpu_ms = (time.process_time() - cpu) * 1000
        self.get_logger().info(
            f"window {elapsed_ms:.1f} ms wall, {cpu_ms:.1f} ms cpu, {len(events)} event messages, "
            f"state {self.memory.state.name}"
        )
        if elapsed_ms > self.config.window_s * 1000:
            self.get_logger().warning(f"window took {elapsed_ms:.0f} ms, longer than window_s")

    def _serve_request(self, request, response):
        response.accepted = self._answer(serialize_message(request.request))
        return response

    def _serve_action(self, request, response):
        response.outcome = int(self._act(serialize_message(request.action)))
        return response

    def _act(self, data):
        """Decodes and applies one operator action, publishes its events and logs it with its outcome."""
        try:
            action = OperatorAction.from_cdr(data)
        except CDRError as error:
            self.get_logger().warning(f"OperatorAction dropped (INVALID): {error}")
            return OperatorOutcome.INVALID
        outcome, events, repeat, differs = self.memory.operate(action, _time(self))
        for event in events:
            self.events.publish(event.to_cdr())
        note = f" note '{action.text}'" if action.text else ""
        again = (" (repeat, payload differs)" if differs else " (repeat)") if repeat else ""
        self.get_logger().info(
            f"OperatorAction {action.action.name} action_id {action.action_id} operator '{action.operator_id}'{note} "
            f"event {'NONE' if action.event_id is None else action.event_id}: {outcome.name}{again}, "
            f"{len(events)} event messages")
        if self.memory.storage_error:
            self.get_logger().warning(f"storage: {self.memory.storage_error}")
        return outcome

    def _answer(self, data):
        """Publishes the answer and its ``SyncEnd``; False when the request was dropped."""
        try:
            request = RequestSync.from_cdr(data)
        except CDRError as error:
            self.get_logger().warning(f"RequestSync dropped: {error}")
            return False
        publisher = self.sync_map if request.topic == SyncTopic.MAP_BRICKS else self.sync_events
        reply, end = self.memory.sync(request, _time(self))
        if end is None:
            self.get_logger().warning(f"RequestSync for room '{request.header.room_id}' dropped: not this room")
            return False
        # The answer first, then its manifest (which may still arrive first: it has its own topic).
        for message in reply:
            publisher.publish(message)
        self.sync_end.publish(end.to_cdr())
        self.get_logger().info(
            f"RequestSync {request.topic.name} expected_generation {request.expected_generation}: {len(reply)} messages, "
            f"bound {end.bound}, delivered {list(end.delivered)}, unavailable {list(end.unavailable)}")
        return True

    def _publish_status(self):
        now = time.monotonic()
        stamp = _time(self)
        message = self.memory.status(stamp, now - self._last_status)
        self._last_status = now
        self.status.publish(message.to_cdr())
        self.activity.publish(self.memory.activity(stamp).to_cdr())


def main(args=None):
    """Entry point for ``ros2 run room_memory room_memory_node``."""
    rclpy.init(args=args)
    node = RoomMemoryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
