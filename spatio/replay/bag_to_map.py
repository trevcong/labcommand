#!/usr/bin/env python3
"""Build map.pcd from recorded /cloud_registered using the copied map_saver.py.

Runs inside the replay container. Uses the historical MapSaver class
unchanged (same voxel logic as the Spark maps), but feeds it from a bag
instead of a live subscription, so a slow Python callback cannot drop scans.

    bag_to_map.py BAG OUT.pcd [--voxel 0.03] [--topic /cloud_registered]
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import sys

MAP_SAVER = "/opt/spatio/map_saver.py"


def load_map_saver(path):
    spec = importlib.util.spec_from_file_location("map_saver", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("bag")
    ap.add_argument("out")
    ap.add_argument("--voxel", type=float, default=0.03)
    ap.add_argument("--topic", default="/cloud_registered")
    ap.add_argument("--map-saver", default=MAP_SAVER)
    ap.add_argument("--summary", default=None)
    args = ap.parse_args()

    import rclpy
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2

    ms = load_map_saver(args.map_saver)
    rclpy.init()
    node = ms.MapSaver(args.out, args.voxel)

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=args.bag, storage_id=""),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if types.get(args.topic) != "sensor_msgs/msg/PointCloud2":
        raise SystemExit(f"{args.topic} missing or wrong type in bag: {types}")
    reader.set_filter(rosbag2_py.StorageFilter(topics=[args.topic]))
    clouds, frames = 0, set()
    while reader.has_next():
        _, data, _ = reader.read_next()
        msg = deserialize_message(data, PointCloud2)
        frames.add(msg.header.frame_id)
        node.on_cloud(msg)
        clouds += 1

    ok = node.save()
    n = 0 if node.pts is None else len(node.pts)
    node.destroy_node()
    rclpy.shutdown()
    if args.summary:
        with open(args.summary, "w") as f:
            json.dump({"topic": args.topic, "clouds_in_bag": clouds,
                       "clouds_accumulated": node.msgs, "frames": sorted(frames),
                       "voxel_m": args.voxel, "points_written": n}, f, indent=2)
    print(f"{clouds} clouds -> {n} points ({args.voxel} m voxel) -> {args.out}")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
