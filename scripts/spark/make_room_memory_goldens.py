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
codec and requires the value of the matching hand golden ("decode identically").

The bytes should also match the .hex files, except for up to 3 bytes of trailing padding the rmw
layer may add. That holds for the compressed goldens too, because their payloads are built by
hand here exactly as in the .hex files: a raw DEFLATE stored block (checked with
zlib.decompress(payload, -15)) and an LZFSE uncompressed block. Python's stdlib has no LZFSE, so
the Spark cannot produce a compressed LZFSE stream; it sends COMPRESSION_ZLIB, made with
zlib.compressobj(6, zlib.DEFLATED, -15). Such a stream is not byte-comparable with Apple's, which
is why compressed messages are compared by decoded value.
"""

import pathlib
import struct
import sys
import zlib

from builtin_interfaces.msg import Time
from labcommand_msgs.msg import ChangeEvent, Epoch, MapBricks, MemoryStatus, RequestSync
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


def packed_payload(bricks, removed):
    """The uncompressed MapBricks payload: all brick_ijk, all occ_bits, all known_bits, then
    removed_bricks_idx, packed little endian with no padding. bricks: [((i, j, k), occ, known)]."""
    ijk = b"".join(struct.pack("<3h", *coordinate) for coordinate, _, _ in bricks)
    occupied = b"".join(bytes(occ) for _, occ, _ in bricks)
    known = b"".join(bytes(kn) for _, _, kn in bricks)
    return ijk + occupied + known + struct.pack(f"<{len(removed)}I", *removed)


def deflate_stored(raw):
    """One final raw DEFLATE stored block (RFC 1951 3.2.4), as in map_bricks_snapshot_zlib_le.hex."""
    block = b"\x01" + struct.pack("<HH", len(raw), len(raw) ^ 0xFFFF) + raw
    assert zlib.decompress(block, -15) == raw
    return block


def lzfse_uncompressed(raw):
    """An LZFSE stream of one uncompressed block, as in map_bricks_delta_lzfse_le.hex."""
    return b"bvx-" + struct.pack("<I", len(raw)) + raw + b"bvx$"


def map_bricks(msg, compression, kind, base_epoch, bricks, removed, encode):
    msg.compression = compression
    msg.kind = kind
    msg.base_epoch = base_epoch
    msg.voxel_size = 0.05
    msg.brick_dim = 8
    msg.n_bricks = len(bricks)
    msg.n_removed = len(removed)
    msg.payload = list(encode(packed_payload(bricks, removed)))
    return msg


DELTA_BRICKS = [((-1, 2, -3), mask(0x01, 0x80), mask(0x03, 0x80))]
DELTA_REMOVED = [0, 5]


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
    return map_bricks(common(MapBricks(), "lab", 43, 12, *T10), MapBricks.COMPRESSION_NONE,
                      MapBricks.KIND_DELTA, 42, DELTA_BRICKS, DELTA_REMOVED, bytes)


def map_bricks_delta_lzfse_le():
    return map_bricks(common(MapBricks(), "lab", 43, 12, *T10), MapBricks.COMPRESSION_LZFSE,
                      MapBricks.KIND_DELTA, 42, DELTA_BRICKS, DELTA_REMOVED, lzfse_uncompressed)


def map_bricks_snapshot_zlib_le():
    bricks = DELTA_BRICKS + [((0, 0, 1), [0] * 64, [0xFF] * 64)]
    return map_bricks(common(MapBricks(), "lab", 43, 13, *T10), MapBricks.COMPRESSION_ZLIB,
                      MapBricks.KIND_SNAPSHOT, 0, bricks, [], deflate_stored)


def change_event_moved_mask_le():
    region = map_bricks(common(MapBricks(), "lab", 43, 0, *T10), MapBricks.COMPRESSION_NONE,
                        MapBricks.KIND_SNAPSHOT, 0, [((3, -1, 0), mask(0x01), mask(0x01))], [], bytes)

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


def request_sync_map_le():
    msg = common(RequestSync(), "lab", 43, 2, *T10)
    msg.since_epoch = 43
    msg.since_seq = 12
    msg.topic = RequestSync.TOPIC_MAP_BRICKS
    return msg


def request_sync_events_none_le():
    msg = common(RequestSync(), "lab-a", RequestSync.SINCE_NONE, 0, 1760400000, 500000000)
    msg.since_epoch = RequestSync.SINCE_NONE
    msg.since_seq = RequestSync.SINCE_NONE
    msg.topic = RequestSync.TOPIC_EVENTS
    return msg


GOLDENS = [
    memory_status_le,
    epoch_le,
    map_bricks_delta_le,
    map_bricks_delta_lzfse_le,
    map_bricks_snapshot_zlib_le,
    change_event_moved_mask_le,
    change_event_appeared_le,
    request_sync_map_le,
    request_sync_events_none_le,
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
