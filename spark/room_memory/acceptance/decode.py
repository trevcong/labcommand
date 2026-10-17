"""CDR bytes to one acceptance log record. Imported by tests and by a future capture path.

The scorer does not import this module. The live ros2 transport uses typed messages
inside the container and does not import it either.
"""

from room_memory.messages import Activity, ChangeEvent, MemoryStatus

TOPIC_TYPE = {
    "/room_memory/status": MemoryStatus,
    "/room_memory/events": ChangeEvent,
    "/room_memory/activity": Activity,
}


def _stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _xyz(value):
    if value is None:
        return None
    return [float(item) for item in value]


def _message_dict(message):
    if isinstance(message, ChangeEvent):
        return {
            "state": message.state.name.lower(),
            "kind": message.kind.name.lower(),
            "event_id": int(message.event_id),
            "sensors": int(message.sensors),
            "centroid": _xyz(message.centroid),
            "aabb": [_xyz(message.aabb_min), _xyz(message.aabb_max)],
            "from_centroid": _xyz(message.from_centroid),
            "voxel_count": int(message.voxel_count),
        }
    if isinstance(message, MemoryStatus):
        return {
            "state": message.state.name.lower(),
            "events_seq": None if message.events_seq is None else int(message.events_seq),
            "mid360_hz": None if message.mid360_hz is None else int(message.mid360_hz),
            "jt128_hz": None if message.jt128_hz is None else int(message.jt128_hz),
            "dropped_windows": int(message.dropped_windows),
            "n_open_events": int(message.n_open_events),
            "pose_residual_m": None if message.pose_residual_m is None else float(message.pose_residual_m),
        }
    if isinstance(message, Activity):
        aabb = None
        if message.aabb_min is not None and message.aabb_max is not None:
            aabb = [_xyz(message.aabb_min), _xyz(message.aabb_max)]
        return {
            "active": bool(message.active),
            "since_s": None if message.since_s is None else float(message.since_s),
            "sensors": int(message.sensors),
            "largest_sensor": int(message.largest_sensor),
            "n_regions": int(message.n_regions),
            "voxel_count": int(message.voxel_count),
            "centroid": _xyz(message.centroid),
            "aabb": aabb,
        }
    raise ValueError(f"unsupported message {type(message).__name__}")


def cdr_to_record(topic, data, receipt_mono_ns, callback_mono_ns=None, initial=False):
    """Decode ``data`` and return a log record. Raises the codec's errors on a bad frame."""
    kind = TOPIC_TYPE.get(topic)
    if kind is None:
        raise ValueError(f"topic {topic} is not scored")
    message = kind.from_cdr(data)
    return {
        "schema": "room-memory-acceptance-log-v1",
        "topic": topic,
        "receipt_mono_ns": int(receipt_mono_ns),
        "callback_mono_ns": int(receipt_mono_ns if callback_mono_ns is None else callback_mono_ns),
        "stamp_ns": _stamp_ns(message.header.stamp),
        "seq": int(message.header.seq),
        "publisher_generation": message.publisher_generation,
        "initial": bool(initial),
        "message": _message_dict(message),
    }
