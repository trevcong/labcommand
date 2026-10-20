#!/usr/bin/env python3
"""Export nav_msgs/Odometry (or the last nav_msgs/Path) from a ROS 2 bag to TUM.

Runs inside the replay container (needs rosbag2_py). Pose time is the message
header stamp, i.e. FAST-LIO's scan time, not bag receipt time.

    bag_to_tum.py BAG OUT.tum [--topic /Odometry] [--path-topic /path]
"""

from __future__ import annotations

import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import tum  # noqa: E402


def open_reader(uri):
    import rosbag2_py

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=uri, storage_id=""),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    return reader


def topic_types(reader):
    return {t.name: t.type for t in reader.get_all_topics_and_types()}


def pose_from_stamped(stamp, pose):
    return tum.make_pose(
        tum.stamp_to_seconds(stamp.sec, stamp.nanosec),
        (pose.position.x, pose.position.y, pose.position.z),
        (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
    )


def read_odometry(uri, topic):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = open_reader(uri)
    types = topic_types(reader)
    if topic not in types:
        raise SystemExit(f"{topic} not in bag; topics: {sorted(types)}")
    if types[topic] != "nav_msgs/msg/Odometry":
        raise SystemExit(f"{topic} is {types[topic]}, expected nav_msgs/msg/Odometry")
    cls = get_message(types[topic])
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    poses, frames = [], set()
    while reader.has_next():
        _, data, _ = reader.read_next()
        m = deserialize_message(data, cls)
        frames.add((m.header.frame_id, m.child_frame_id))
        poses.append(pose_from_stamped(m.header.stamp, m.pose.pose))
    return poses, sorted(frames)


def read_last_path(uri, topic):
    """Poses of the final Path message (FAST-LIO republishes the whole path)."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = open_reader(uri)
    types = topic_types(reader)
    if topic not in types:
        return None, 0
    cls = get_message(types[topic])
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    last, count = None, 0
    while reader.has_next():
        _, data, _ = reader.read_next()
        last = data
        count += 1
    if last is None:
        return [], 0
    m = deserialize_message(last, cls)
    return [pose_from_stamped(ps.header.stamp, ps.pose) for ps in m.poses], count


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("bag")
    ap.add_argument("out")
    ap.add_argument("--topic", default="/Odometry")
    ap.add_argument("--path-topic", default=None,
                    help="also compare against the last Path message on this topic")
    ap.add_argument("--summary", default=None, help="write a JSON summary here")
    args = ap.parse_args()

    poses, frames = read_odometry(args.bag, args.topic)
    if not poses:
        raise SystemExit(f"no messages on {args.topic}")
    comments = [
        "t x y z qx qy qz qw  (t = header stamp seconds; map frame; metres)",
        f"source: {args.topic} in {args.bag}",
        f"frames (frame_id, child_frame_id): {frames}",
    ]
    n = tum.write_tum(args.out, poses, comments)
    summary = {"topic": args.topic, "frames": frames, **tum.summarize(poses)}

    if args.path_topic:
        path_poses, path_msgs = read_last_path(args.bag, args.path_topic)
        if path_poses is None:
            summary["path_check"] = f"{args.path_topic} not in bag"
        else:
            summary["path_check"] = {
                "topic": args.path_topic,
                "path_messages": path_msgs,
                "poses_in_last_path": len(path_poses),
                "last_path_summary": tum.summarize(path_poses) if path_poses else None,
            }

    if args.summary:
        with open(args.summary, "w") as f:
            json.dump(summary, f, indent=2)
    print(f"wrote {n} poses to {args.out}; start-end 3D {summary['start_end_3d_m']:.3f} m")


if __name__ == "__main__":
    main()
