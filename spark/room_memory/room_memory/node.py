"""Thin rclpy node around ``RoomMemory``: raw CDR in, codec bytes out.

Subscribes to each configured lidar with the sensor-data QoS and ``raw=True``, so frames go
through the tested ``decode_pointcloud2`` rather than rclpy's deserializer. Publishes the
codec's CDR bytes on ``/room_memory/status`` and ``/room_memory/activity`` (1 Hz), ``/room_memory/events`` and
``/room_memory/map_bricks`` (after each window, when ``map_update`` has a message), and answers
``/room_memory/request_sync`` on the two sync topics, each answer followed by its ``SyncEnd`` on
``/room_memory/sync/end`` (protocol §2, §5); the ``labcommand_msgs`` classes only provide the type
support. A ``RequestSync`` also arrives as the ``/room_memory/sync/request`` service, which the :8767
bridge exposes to the Mac (D39); its request is serialized back to the message's bytes and answered
the same way. The publisher generation is allocated (durably, under ``publisher.lock``) before any
publisher exists; without it the node refuses to start. A rejected frame logs one warning per
sensor per 10 s.

With ``rig_guard`` it also feeds ``imu_topic`` (raw ``sensor_msgs/Imu``, sensor-data QoS; "" for none)
to the guard and publishes ``RigPose`` on ``/room_memory/rig`` after each window, when ``rig_pose``
has one (reliable, transient_local, keep-last 1).

With ``rhythm_publish`` it publishes ``RhythmMinute`` on ``/room_memory/rhythm`` after each window
(reliable, transient_local, keep-last equal to the sensor count, D45). The logger runs only when
``rhythm_enable`` is on. Both default off.
"""

import dataclasses
import time

import rclpy
from labcommand_msgs.msg import Activity as ActivityMsg
from labcommand_msgs.msg import ChangeEvent as ChangeEventMsg
from labcommand_msgs.msg import MapBricks as MapBricksMsg
from labcommand_msgs.msg import MemoryStatus as MemoryStatusMsg
from labcommand_msgs.msg import RequestSync as RequestSyncMsg
from labcommand_msgs.msg import SyncEnd as SyncEndMsg
from labcommand_msgs.msg import RigPose as RigPoseMsg
from labcommand_msgs.srv import RequestSync as RequestSyncSrv
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.serialization import serialize_message
from sensor_msgs.msg import Imu, PointCloud2

from .generation import GenerationError, allocate_generation
from .memory import SENSORS, MemoryConfig, RoomMemory
from .messages import CDRError, RequestSync, SyncTopic, Time
from .rejectlog import RejectionLog
from .rhythm import history_depth

STATUS_TOPIC = "/room_memory/status"
ACTIVITY_TOPIC = "/room_memory/activity"
EVENTS_TOPIC = "/room_memory/events"
MAP_TOPIC = "/room_memory/map_bricks"
REQUEST_TOPIC = "/room_memory/request_sync"
REQUEST_SERVICE = "/room_memory/sync/request"
SYNC_MAP_TOPIC = "/room_memory/sync/map_bricks"
SYNC_EVENTS_TOPIC = "/room_memory/sync/events"
SYNC_END_TOPIC = "/room_memory/sync/end"
RIG_TOPIC = "/room_memory/rig"


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
            "rhythm_enable": defaults.rhythm_enable,
            "rhythm_cell_m": defaults.rhythm_cell_m,
            "rhythm_dims": defaults.rhythm_dims,
            "rhythm_period_s": defaults.rhythm_period_s,
            "rhythm_publish": defaults.rhythm_publish,
            "rhythm_dir": defaults.rhythm_dir,
            "rhythm_keep_days": defaults.rhythm_keep_days,
            "rhythm_max_bytes": defaults.rhythm_max_bytes,
            "rhythm_max_file_bytes": defaults.rhythm_max_file_bytes,
            "rhythm_fsync_s": defaults.rhythm_fsync_s,
            "rig_guard": defaults.rig_guard,
            "imu_topic": defaults.imu_topic,
            "imu_shock_frac": defaults.imu_shock_frac,
            "imu_gyro_dps": defaults.imu_gyro_dps,
            "imu_quiet_s": defaults.imu_quiet_s,
            "tilt_trigger_deg": defaults.tilt_trigger_deg,
            "pose_trigger_m": defaults.pose_trigger_m,
            "guard_check_period_s": defaults.guard_check_period_s,
            "check_max_s": defaults.check_max_s,
            "move_min_m": defaults.move_min_m,
            "move_min_deg": defaults.move_min_deg,
            "align_max_m": defaults.align_max_m,
            "align_max_deg": defaults.align_max_deg,
            "align_inlier_ratio": defaults.align_inlier_ratio,
            "align_max_rms_m": defaults.align_max_rms_m,
            "align_stable_windows": defaults.align_stable_windows,
            "observe_min_eig": defaults.observe_min_eig,
            "gross_inlier_ratio": defaults.gross_inlier_ratio,
            "relearn_after_s": defaults.relearn_after_s,
            "auto_relearn": defaults.auto_relearn,
            "reg_voxel_m": defaults.reg_voxel_m,
            "reg_max_points": defaults.reg_max_points,
            "src_voxel_m": defaults.src_voxel_m,
            "src_max_points": defaults.src_max_points,
            "icp_max_range_m": defaults.icp_max_range_m,
        }
        values = {name: self.declare_parameter(name, value).value for name, value in params.items()}
        values["sensors"] = tuple(values["sensors"])
        values["max_range"] = values["max_range"] if values["max_range"] > 0 else None
        values["state_dir"] = values["state_dir"] or None
        values["rhythm_dir"] = values["rhythm_dir"] or ""
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
        # request. Rhythm, when published, is transient_local with one stored minute per configured
        # sensor (D45). Depth 1 would leave a late subscriber the newest sensor only. Same QoS as the C++ node.
        def volatile(depth):
            return QoSProfile(depth=depth, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)

        def latched(depth=1):
            return QoSProfile(depth=depth, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.events = self.create_publisher(ChangeEventMsg, EVENTS_TOPIC, volatile(100))
        self.status = self.create_publisher(MemoryStatusMsg, STATUS_TOPIC, latched())
        self.activity = self.create_publisher(ActivityMsg, ACTIVITY_TOPIC, latched())
        self._rejections = RejectionLog()
        self.map = self.create_publisher(MapBricksMsg, MAP_TOPIC, volatile(10))
        self.sync_map = self.create_publisher(MapBricksMsg, SYNC_MAP_TOPIC, volatile(10))
        self.sync_events = self.create_publisher(ChangeEventMsg, SYNC_EVENTS_TOPIC, volatile(200))
        self.sync_end = self.create_publisher(SyncEndMsg, SYNC_END_TOPIC, volatile(10))
        if self.config.rhythm_publish:
            from labcommand_msgs.msg import RhythmMinute as RhythmMinuteMsg
            self.rhythm = self.create_publisher(
                RhythmMinuteMsg, "/room_memory/rhythm", latched(history_depth(len(self.config.sensors))),
            )
            self.memory.set_rhythm_publish(lambda data: self.rhythm.publish(data))
        self.create_subscription(RequestSyncMsg, REQUEST_TOPIC, self._answer, volatile(10), raw=True)
        self.create_service(RequestSyncSrv, REQUEST_SERVICE, self._serve_request)
        for name in self.config.sensors:
            sensor = SENSORS[name]
            self.create_subscription(
                PointCloud2, sensor.topic, lambda data, name=name: self._on_cloud(name, data),
                qos_profile_sensor_data, raw=True,
            )
        self.rig = None
        if self.config.rig_guard:
            # RigPose is full state too (on change and every 10 s): latched like status (protocol D42).
            self.rig = self.create_publisher(RigPoseMsg, RIG_TOPIC, latched())
            if self.config.imu_topic:
                self.create_subscription(Imu, self.config.imu_topic, self._on_imu, qos_profile_sensor_data, raw=True)
        self._last_status = time.monotonic()
        self._rig_save_failed = 0  # memory.rig_save_failed at the last window
        self._rig_save_failed_at = 0  # its value before the current run of failures
        self._rig_save_lagging = False
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

    def _on_imu(self, data):
        self.memory.add_imu(data, _time(self))

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
        self.memory.enqueue_rhythm()
        if self.rig is not None:  # created with rig_guard only; rig_pose is None without it
            rig = self.memory.rig_pose(stamp)
            if rig is not None:
                self.rig.publish(rig.to_cdr())
            self._report_rig_save()
        elapsed_ms = (time.perf_counter() - start) * 1000
        cpu_ms = (time.process_time() - cpu) * 1000
        self.get_logger().info(
            f"window {elapsed_ms:.1f} ms wall, {cpu_ms:.1f} ms cpu, {len(events)} event messages, "
            f"state {self.memory.state.name}"
        )
        if self.memory.window_estimates:
            self.get_logger().info(f"rig guard: {self.memory.window_estimates} ICP estimates, "
                                   f"{self.memory.window_estimate_ms:.1f} ms of the window")
        if elapsed_ms > self.config.window_s * 1000:
            self.get_logger().warning(f"window took {elapsed_ms:.0f} ms, longer than window_s")

    def _serve_request(self, request, response):
        response.accepted = self._answer(serialize_message(request.request))
        return response

    def _report_rig_save(self):
        """One warning when the rig files start lagging the guard (or a relearn archive fails, or the
        epoch file cannot take a new epoch), one line when a retry catches up; not one per window while
        the disk stays full. A held epoch write is not a pending save, so both count."""
        failed = self.memory.rig_save_failed
        lagging = self.memory.rig_save_pending or self.memory.epoch_held
        if lagging and not self._rig_save_lagging:
            self.get_logger().warning(f"rig state not saved: {self.memory.rig_save_error}; "
                                      "the guard runs on and retries every window")
            self._rig_save_failed_at = self._rig_save_failed
        elif self._rig_save_lagging and not lagging:
            self.get_logger().info(f"rig state saved after {failed - self._rig_save_failed_at} failed attempts")
        self._rig_save_failed, self._rig_save_lagging = failed, lagging

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

    def destroy_node(self):
        memory = getattr(self, "memory", None)
        if memory is not None:
            memory.close()
        super().destroy_node()


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
