#!/usr/bin/env python3
"""Write a synthetic marked-loop recording as a ROS 2 MCAP bag.

Runs inside the spatio/replay image (rclpy, rosbag2_py, livox_ros_driver2).
Started by make_recording.sh; see there for the folder layout.

    write_bag.py OUT_DIR --params params.json

Livox CustomMsg is serialised directly with numpy (one message holds ~12k
points), and the first message is round-tripped through rclpy's
deserialiser so a layout mistake fails here, not in FAST-LIO.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import struct
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from scene import Path, SceneParams, imu_samples, scan, scan_count  # noqa: E402

POINT = np.dtype([("offset_time", "<u4"), ("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                  ("reflectivity", "u1"), ("tag", "u1"), ("line", "u1"), ("pad", "u1")])


def _align(buf: bytearray, n: int) -> None:
    # CDR alignment is relative to the payload after the 4-byte encapsulation header.
    while (len(buf) - 4) % n:
        buf.append(0)


def custom_msg_cdr(stamp_ns: int, frame_id: str, off_ns, xyz, refl, line) -> bytes:
    b = bytearray(b"\x00\x01\x00\x00")                   # CDR little-endian
    b += struct.pack("<iI", stamp_ns // 1_000_000_000, stamp_ns % 1_000_000_000)
    fid = frame_id.encode() + b"\x00"
    b += struct.pack("<I", len(fid)) + fid
    _align(b, 8)
    b += struct.pack("<Q", stamp_ns)                     # timebase = first point time
    _align(b, 4)
    n = len(off_ns)
    b += struct.pack("<I", n)                            # point_num
    b += bytes([0, 0, 0, 0])                             # lidar_id, rsvd[3]
    _align(b, 4)
    b += struct.pack("<I", n)                            # sequence length
    pts = np.zeros(n, dtype=POINT)
    pts["offset_time"] = off_ns
    pts["x"], pts["y"], pts["z"] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    pts["reflectivity"] = refl
    pts["tag"] = 0x10                                    # "normal" return in Livox tag bits
    pts["line"] = line
    raw = pts.tobytes()
    return bytes(b + raw[:-1])                           # no padding after the last point


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--params", required=True)
    ap.add_argument("--lidar-topic", required=True)
    ap.add_argument("--imu-topic", required=True)
    ap.add_argument("--lidar-frame", required=True)
    ap.add_argument("--imu-frame", required=True)
    ap.add_argument("--scan-lines", type=int, default=4)
    args = ap.parse_args()

    import rosbag2_py
    from rclpy.serialization import deserialize_message, serialize_message
    from livox_ros_driver2.msg import CustomMsg
    from sensor_msgs.msg import Imu

    with open(args.params) as f:
        raw = json.load(f)
    raw["boxes"] = [(tuple(a), tuple(b)) for a, b in raw["boxes"]]
    for k in ("centre_xy", "semi_axes", "end_offset_m", "room_min", "room_max", "elevation_deg"):
        raw[k] = tuple(raw[k])
    p = SceneParams(**raw)
    path = Path(p)
    rng = np.random.default_rng(p.seed)

    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=args.out, storage_id="mcap"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    for name, typ in ((args.lidar_topic, "livox_ros_driver2/msg/CustomMsg"),
                      (args.imu_topic, "sensor_msgs/msg/Imu")):
        writer.create_topic(rosbag2_py.TopicMetadata(0, name, typ, "cdr") if _new_api(rosbag2_py)
                            else rosbag2_py.TopicMetadata(name=name, type=typ, serialization_format="cdr"))

    # Interleave by time: IMU samples and scans (stamped at their first point).
    ti, acc_g, gyro = imu_samples(path, rng)
    n_scans = scan_count(path)
    scan_times = p.t0 + np.arange(n_scans) / p.lidar_rate_hz
    events = [(int(round(t * 1e9)), 0, i) for i, t in enumerate(ti)]
    # A scan is published when it ends (receipt time = end of the 0.1 s sweep).
    events += [(int(round((t + 1.0 / p.lidar_rate_hz) * 1e9)), 1, k) for k, t in enumerate(scan_times)]
    events.sort()
    checked = False
    for recv_ns, kind, idx in events:
        if kind == 0:
            m = Imu()
            stamp_ns = int(round(ti[idx] * 1e9))
            m.header.stamp.sec, m.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
            m.header.frame_id = args.imu_frame
            m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = map(float, acc_g[idx])
            m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = map(float, gyro[idx])
            m.orientation.w = 1.0
            m.orientation_covariance[0] = -1.0           # orientation not provided
            writer.write(args.imu_topic, serialize_message(m), recv_ns)
        else:
            t_scan, off, xyz = scan(path, idx, rng)
            stamp_ns = int(round(t_scan * 1e9))
            refl = rng.integers(10, 200, len(off), dtype=np.uint8)
            line = (np.arange(len(off)) % args.scan_lines).astype(np.uint8)
            data = custom_msg_cdr(stamp_ns, args.lidar_frame, off, xyz, refl, line)
            if not checked:
                m = deserialize_message(data, CustomMsg)
                assert m.point_num == len(off) and len(m.points) == len(off), "point count"
                assert m.timebase == stamp_ns and m.header.frame_id == args.lidar_frame
                q = m.points[-1]
                assert q.offset_time == int(off[-1]) and q.line == int(line[-1]) and q.tag == 0x10
                assert abs(q.x - float(xyz[-1, 0])) < 1e-6 and abs(q.z - float(xyz[-1, 2])) < 1e-6
                checked = True
            writer.write(args.lidar_topic, data, recv_ns)
    del writer
    print(f"wrote {len(ti)} IMU samples and {n_scans} scans to {args.out}")
    return 0


def _new_api(mod) -> bool:
    # Jazzy's TopicMetadata takes (id, name, type, serialization_format, ...).
    try:
        mod.TopicMetadata(0, "/x", "std_msgs/msg/String", "cdr")
        return True
    except TypeError:
        return False


if __name__ == "__main__":
    sys.exit(main())
