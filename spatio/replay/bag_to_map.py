#!/usr/bin/env python3
"""Export registered points for scoring and a separate voxel map for display.

Runs inside the replay container. Uses the historical MapSaver class
unchanged for display (same voxel logic as the Spark maps), but feeds it from a bag
instead of a live subscription, so a slow Python callback cannot drop scans.
OUT.pcd retains every finite registered XYZ point, including repeated points.

    bag_to_map.py BAG OUT.pcd [--voxel 0.03] [--topic /cloud_registered]
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import math
from pathlib import Path
import sys

MAP_SAVER = "/opt/spatio/map_saver.py"


def load_map_saver(path):
    spec = importlib.util.spec_from_file_location("map_saver", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def write_header(stream, count):
    # Fixed-width counts allow streaming all points without keeping them in RAM.
    stream.write((
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
        f"WIDTH {count:020d}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {count:020d}\nDATA binary\n"
    ).encode("ascii"))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("bag")
    ap.add_argument("out", help="unfiltered registered XYZ PCD for scoring")
    ap.add_argument("--display-out", help="voxel PCD (default: OUT stem + _display.pcd)")
    ap.add_argument("--voxel", type=float, default=0.03)
    ap.add_argument("--topic", default="/cloud_registered")
    ap.add_argument("--map-saver", default=MAP_SAVER)
    ap.add_argument("--summary", default=None)
    args = ap.parse_args()
    display_out = args.display_out or str(
        Path(args.out).with_name(Path(args.out).stem + "_display.pcd"))
    if Path(display_out).resolve() == Path(args.out).resolve():
        ap.error("scoring and display paths must differ")
    if not math.isfinite(args.voxel) or args.voxel <= 0:
        ap.error("--voxel must be finite and positive (display only)")

    import numpy as np
    import rclpy
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2

    ms = load_map_saver(args.map_saver)
    rclpy.init()
    node = ms.MapSaver(display_out, args.voxel)

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=args.bag, storage_id=""),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if types.get(args.topic) != "sensor_msgs/msg/PointCloud2":
        raise SystemExit(f"{args.topic} missing or wrong type in bag: {types}")
    reader.set_filter(rosbag2_py.StorageFilter(topics=[args.topic]))
    clouds, frames = 0, set()
    points = 0
    with open(args.out, "wb") as raw:
        write_header(raw, 0)
        while reader.has_next():
            _, data, _ = reader.read_next()
            msg = deserialize_message(data, PointCloud2)
            frames.add(msg.header.frame_id)
            arr = point_cloud2.read_points_numpy(
                msg, field_names=("x", "y", "z"), skip_nans=True)
            if arr is not None:
                xyz = np.asarray(arr, dtype="<f4").reshape(-1, 3)
                xyz = xyz[np.isfinite(xyz).all(axis=1)]
                raw.write(xyz.tobytes())
                points += len(xyz)
            node.on_cloud(msg)
            clouds += 1
        raw.seek(0)
        write_header(raw, points)

    ok = node.save()
    n = 0 if node.pts is None else len(node.pts)
    node.destroy_node()
    rclpy.shutdown()
    if args.summary:
        with open(args.summary, "w") as f:
            json.dump({"topic": args.topic, "clouds_in_bag": clouds,
                       "clouds_accumulated": node.msgs, "frames": sorted(frames),
                       "scoring_map": args.out, "points_written": points,
                       "scoring_voxel_m": None, "display_map": display_out,
                       "display_voxel_m": args.voxel,
                       "display_points_written": n}, f, indent=2)
    print(f"{clouds} clouds -> {points} registered points for scoring -> {args.out}")
    print(f"{n} display points ({args.voxel} m voxel) -> {display_out}")
    sys.exit(0 if ok and points > 0 else 1)


if __name__ == "__main__":
    main()
