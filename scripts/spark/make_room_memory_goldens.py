#!/usr/bin/env python3
"""Serialize the hand-golden room-memory values with rclpy, so the Mac can compare real ROS 2 bytes.

Status: written for PR 0.2, never run. It needs ROS 2 Jazzy and a built labcommand_msgs, so it
runs on the Spark (inside a Jazzy container), by Trevor or an agent he authorizes:

    cd <labcommand checkout>/ros && colcon build --packages-select labcommand_msgs
    source install/setup.bash
    python3 ../scripts/spark/make_room_memory_goldens.py <out_dir>

It writes <out_dir>/<name>.bin for each little-endian golden in Tests/Fixtures/room/golden/hand/
(ROS 2 serializes in host byte order, so there are no big-endian Spark goldens). Copy them to
Tests/Fixtures/room/golden/spark/ on the Mac. The follow-up test decodes each one with the Swift
codec and requires the value of the matching hand golden ("decode identically"). The bytes should
also match the .hex files, except for up to 3 bytes of trailing padding the rmw layer may add.
"""

import pathlib
import sys

from builtin_interfaces.msg import Time
from labcommand_msgs.msg import ChangeEvent, Epoch, MapBricks, MemoryStatus
from rclpy.serialization import serialize_message

NAN = float("nan")
T10 = (1760400010, 250000000)


def stamp(sec, nanosec):
    return Time(sec=sec, nanosec=nanosec)


def common(msg, room_id, map_epoch, seq, sec, nanosec):
    msg.room_id = room_id
    msg.map_epoch = map_epoch
    msg.seq = seq
    msg.stamp = stamp(sec, nanosec)
    return msg


def mask(first, last=0):
    return [first] + [0] * 62 + [last]


def memory_status_le():
    msg = common(MemoryStatus(), "lab-a", 42, 7, 1760400000, 500000000)
    msg.state = MemoryStatus.STATE_TRACKING
    msg.n_bricks = 3200
    msg.n_open_events = 2
    msg.coverage_known_frac = 0.75
    msg.pose_residual_m = NAN
    msg.window_s = 1.0
    msg.per_sensor_hz = [10, MemoryStatus.HZ_UNAVAILABLE]
    msg.dropped_windows = 3
    return msg


def epoch_le():
    msg = common(Epoch(), "lab", 4294967298, 1000, 1760400005, 0)
    msg.n_appeared = 5
    msg.n_disappeared = 1
    msg.n_moved = 16909060
    msg.coverage_known_frac = NAN
    msg.state = MemoryStatus.STATE_RIG_MOVED
    return msg


def map_bricks_delta_le():
    msg = common(MapBricks(), "lab", 43, 12, *T10)
    msg.kind = MapBricks.KIND_DELTA
    msg.base_epoch = 42
    msg.voxel_size = 0.05
    msg.brick_dim = 8
    msg.brick_ijk = [-1, 2, -3]
    msg.occ_bits = mask(0x01, 0x80)
    msg.known_bits = mask(0x03, 0x80)
    msg.removed_bricks_idx = [0, 5]
    msg.compression = MapBricks.COMPRESSION_NONE
    return msg


def change_event_moved_mask_le():
    region = common(MapBricks(), "lab", 43, 0, *T10)
    region.kind = MapBricks.KIND_SNAPSHOT
    region.base_epoch = 0
    region.voxel_size = 0.05
    region.brick_dim = 8
    region.brick_ijk = [3, -1, 0]
    region.occ_bits = mask(0x01)
    region.known_bits = mask(0x01)
    region.removed_bricks_idx = []
    region.compression = MapBricks.COMPRESSION_NONE

    msg = common(ChangeEvent(), "lab", 43, 3, *T10)
    msg.event_id = 9
    msg.kind = ChangeEvent.KIND_MOVED
    msg.state = ChangeEvent.STATE_CONFIRMED
    msg.centroid = [1.5, -0.25, 0.125]
    msg.aabb = [1.25, -0.5, 0.0, 1.75, 0.0, 0.25]
    msg.from_centroid = [0.5, -0.25, 0.125]
    msg.voxel_count = 125
    msg.volume_m3 = 0.015625
    msg.evidence = 0.875
    msg.alpha = NAN
    msg.sensors = ChangeEvent.SENSOR_MID360 | ChangeEvent.SENSOR_JT128
    msg.first_seen = stamp(1760400000, 0)
    msg.last_seen = stamp(*T10)
    msg.mask = [region]
    return msg


def change_event_appeared_le():
    msg = common(ChangeEvent(), "lab", 43, 4, *T10)
    msg.event_id = 10
    msg.kind = ChangeEvent.KIND_APPEARED
    msg.state = ChangeEvent.STATE_CANDIDATE
    msg.centroid = [2.0, 1.0, 0.5]
    msg.aabb = [1.875, 0.875, 0.375, 2.125, 1.125, 0.625]
    msg.from_centroid = [NAN, NAN, NAN]
    msg.voxel_count = 8
    msg.volume_m3 = 0.001
    msg.evidence = NAN
    msg.alpha = 0.05
    msg.sensors = ChangeEvent.SENSOR_MID360
    msg.first_seen = stamp(1760400010, 0)
    msg.last_seen = stamp(*T10)
    msg.mask = []
    return msg


GOLDENS = [
    memory_status_le,
    epoch_le,
    map_bricks_delta_le,
    change_event_moved_mask_le,
    change_event_appeared_le,
]


def main():
    if len(sys.argv) != 2:
        sys.exit("usage: make_room_memory_goldens.py <out_dir>")
    out = pathlib.Path(sys.argv[1])
    out.mkdir(parents=True, exist_ok=True)
    for build in GOLDENS:
        data = serialize_message(build())
        (out / f"{build.__name__}.bin").write_bytes(data)
        print(f"wrote {build.__name__}.bin ({len(data)} bytes, header {data[:4].hex(' ')})")


if __name__ == "__main__":
    main()
