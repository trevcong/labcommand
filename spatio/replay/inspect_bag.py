#!/usr/bin/env python3
"""Brief timestamp inspection of a recording bag. Runs inside the container.

Per topic: header stamp vs bag receipt time, rate, gaps. For the lidar topic:
per-scan point timing span and whether IMU samples bracket each scan. For the
IMU topic: acceleration magnitude (unit guess). Facts only; nothing is fixed.

    inspect_bag.py BAG --lidar /livox/lidar --imu /livox/imu --out timing.json
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
from collections import defaultdict

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import timing  # noqa: E402

# PointCloud2 'time' field scale per FAST-LIO's preprocess.timestamp_unit.
TIME_UNIT_SCALE = {0: 1.0, 1: 1e-3, 2: 1e-6, 3: 1e-9}


def stamp(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def lidar_scan_window(msg, type_name, time_unit):
    """(start, end, info) for one scan, using FAST-LIO's own conventions.

    Livox CustomMsg: start = header stamp; per-point offset_time is ns.
    PointCloud2 (lidar_type 2): start = header stamp; 'time' field scaled by
    timestamp_unit. If the field is absent or all zero, FAST-LIO's Velodyne
    handler synthesises timing from yaw -- reported as such.
    """
    start = stamp(msg.header)
    if type_name == "livox_ros_driver2/msg/CustomMsg":
        offs = [p.offset_time for p in msg.points]
        span = (max(offs) * 1e-9) if offs else 0.0
        info = {"points": msg.point_num,
                "timebase_minus_header_s": msg.timebase * 1e-9 - start}
        return start, start + span, info
    if type_name == "sensor_msgs/msg/PointCloud2":
        from sensor_msgs_py import point_cloud2 as pc2
        names = [f.name for f in msg.fields]
        info = {"points": msg.width * msg.height, "fields": names}
        if "time" not in names:
            info["time_field"] = "absent"
            return start, start, info
        ts = [float(r[0]) for r in pc2.read_points(msg, field_names=("time",), skip_nans=False)]
        tmin, tmax = (min(ts), max(ts)) if ts else (0.0, 0.0)
        scale = TIME_UNIT_SCALE[time_unit]
        info.update({"time_field_min": tmin, "time_field_max": tmax,
                     "time_field": "all zero" if tmin == 0.0 and tmax == 0.0 else "present"})
        return start + tmin * scale, start + tmax * scale, info
    raise SystemExit(f"unsupported lidar type {type_name}")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("bag")
    ap.add_argument("--lidar", required=True)
    ap.add_argument("--imu", required=True)
    ap.add_argument("--timestamp-unit", type=int, default=3,
                    help="FAST-LIO preprocess.timestamp_unit for PointCloud2 'time' (default 3 = ns)")
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=args.bag, storage_id=""),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    for t in (args.lidar, args.imu):
        if t not in types:
            raise SystemExit(f"{t} not in bag; topics: {sorted(types)}")

    classes = {}
    header_t, receipt_t = defaultdict(list), defaultdict(list)
    scans, scan_infos, imu_t, acc_norms, gyro_norms = [], [], [], [], []
    frame_ids = defaultdict(set)
    while reader.has_next():
        topic, data, recv_ns = reader.read_next()
        typ = types[topic]
        if typ.startswith("hesai_ros_driver/"):
            continue  # driver diagnostics; message package not built here
        if topic not in classes:
            try:
                classes[topic] = get_message(typ)
            except (AttributeError, ModuleNotFoundError, ValueError):
                classes[topic] = None
        cls = classes[topic]
        if cls is None:
            continue
        m = deserialize_message(data, cls)
        h = getattr(m, "header", None)
        if h is None:
            continue
        header_t[topic].append(stamp(h))
        receipt_t[topic].append(recv_ns * 1e-9)
        frame_ids[topic].add(h.frame_id)
        if topic == args.lidar:
            s, e, info = lidar_scan_window(m, typ, args.timestamp_unit)
            scans.append((s, e))
            scan_infos.append(info)
        elif topic == args.imu:
            imu_t.append(stamp(h))
            a, w = m.linear_acceleration, m.angular_velocity
            acc_norms.append(math.sqrt(a.x * a.x + a.y * a.y + a.z * a.z))
            gyro_norms.append(math.sqrt(w.x * w.x + w.y * w.y + w.z * w.z))

    report = {"bag": args.bag, "types": types, "topics": {}}
    for topic in sorted(header_t):
        report["topics"][topic] = {
            "type": types[topic], "frame_ids": sorted(frame_ids[topic]),
            **timing.stream_stats(header_t[topic], receipt_t[topic]),
        }

    imu_sorted = sorted(imu_t)
    report["imu_header_not_monotonic"] = imu_sorted != imu_t
    report["imu_coverage_of_lidar_scans"] = timing.imu_coverage(scans, imu_sorted)
    # First 200 IMU samples (~1 s at 200 Hz): rig assumed still at start.
    report["imu_accel_units"] = timing.accel_units_guess(acc_norms[:200])
    report["imu_accel_norm_all"] = {"min": min(acc_norms), "max": max(acc_norms)} if acc_norms else None
    report["imu_gyro_norm_max"] = max(gyro_norms) if gyro_norms else None
    if scan_infos:
        pts = [i["points"] for i in scan_infos]
        report["lidar_scan_info"] = {"points_min": min(pts), "points_max": max(pts),
                                     "first_scan": scan_infos[0]}
        zero = sum(1 for i in scan_infos if i.get("time_field") in ("all zero", "absent"))
        report["lidar_scans_without_point_timing"] = zero
        if "timebase_minus_header_s" in scan_infos[0]:
            d = [i["timebase_minus_header_s"] for i in scan_infos]
            report["livox_timebase_minus_header_s"] = {"min": min(d), "max": max(d)}

    with open(args.out, "w") as f:
        json.dump(report, f, indent=2, default=str)
    cov = report["imu_coverage_of_lidar_scans"]
    print(f"inspected {len(scans)} scans, {len(imu_t)} imu; covered {cov.get('covered')}; -> {args.out}")


if __name__ == "__main__":
    main()
