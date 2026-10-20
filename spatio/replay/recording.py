#!/usr/bin/env python3
"""Describe a copied recording from its own files. Runs on the Mac.

Nothing here knows any particular recording. Topics come from the FAST-LIO
config, message types and counts from the bag's metadata.yaml, and the copy
check compares metadata.yaml with the MCAP files' own summaries.

    recording.py complete REC            exit 0 = every bag file present and
                                         its MCAP summary matches metadata.yaml
    recording.py profile REC --config CFG [--lidar-topic T] [--imu-topic T]
                                         prints shell assignments (LIDAR, IMU,
                                         LIDAR_MSG_TYPE, TS_UNIT, SENSOR_KIND)
    recording.py find-config REC         prints the one FAST-LIO config (*.yaml
                                         with lid_topic) in REC, else exit 2

Standard library only, except `complete`, which needs the `mcap` package.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import shlex
import sys

LIVOX_CUSTOM = "livox_ros_driver2/msg/CustomMsg"
POINTCLOUD2 = "sensor_msgs/msg/PointCloud2"
IMU = "sensor_msgs/msg/Imu"
# FAST-LIO preprocess.lidar_type: 1 Livox CustomMsg, 2 Velodyne, 3 Ouster,
# 4 generic PointCloud2 (upstream preprocess.h). Only the message type matters here.
LIDAR_TYPE_MSG = {1: LIVOX_CUSTOM, 2: POINTCLOUD2, 3: POINTCLOUD2, 4: POINTCLOUD2}


class RecordingError(Exception):
    pass


def _strip(value: str) -> str:
    value = value.split(" #", 1)[0].strip()
    if len(value) >= 2 and value[0] == value[-1] and value[0] in "'\"":
        value = value[1:-1]
    return value


def parse_metadata(text: str) -> dict:
    """Topics and bag files from a rosbag2 metadata.yaml (no PyYAML needed).

    Returns {"topics": {name: {"type": str, "count": int}}, "files": [str]}.
    """
    topics: dict = {}
    files: list = []
    section = None
    current = None
    for raw in text.splitlines():
        line = raw.rstrip()
        key = line.strip()
        if key.startswith("topics_with_message_count:"):
            section = "topics"
            continue
        if key.startswith("relative_file_paths:"):
            section = "paths"
            continue
        if re.match(r"^\s{2}[a-z_]+:", line) and not line.startswith("    "):
            # Another top-level key under rosbag2_bagfile_information.
            section = None
        if section == "topics":
            m = re.match(r"^\s*(?:- )?(name|type|message_count):\s*(.*)$", line)
            if not m:
                continue
            field, value = m.group(1), _strip(m.group(2))
            if field == "name":
                current = {"name": value, "type": None, "count": None}
            elif field == "type" and current is not None:
                current["type"] = value
            elif field == "message_count" and current is not None:
                current["count"] = int(value)
                topics[current["name"]] = {"type": current["type"], "count": current["count"]}
                current = None
        elif section == "paths":
            m = re.match(r"^\s*-\s+(.+)$", line)
            if m:
                files.append(_strip(m.group(1)))
    return {"topics": topics, "files": files}


def read_metadata(rec: str) -> dict:
    path = os.path.join(rec, "bag", "metadata.yaml")
    try:
        with open(path, encoding="utf-8") as f:
            meta = parse_metadata(f.read())
    except OSError as exc:
        raise RecordingError(f"cannot read {path}: {exc}")
    if not meta["topics"]:
        raise RecordingError(f"{path} lists no topics")
    if not meta["files"]:
        raise RecordingError(f"{path} lists no bag files")
    return meta


def parse_fastlio_config(text: str) -> dict:
    """The few FAST-LIO keys the pipeline needs. Keys are unique in the file."""
    out = {}
    for key in ("lid_topic", "imu_topic", "lidar_type", "timestamp_unit", "scan_rate"):
        m = re.search(r"^\s*%s:\s*(.+)$" % key, text, re.M)
        if m:
            out[key] = _strip(m.group(1))
    for key in ("lidar_type", "timestamp_unit"):
        if key in out:
            try:
                out[key] = int(out[key])
            except ValueError:
                raise RecordingError(f"config {key} is not an integer: {out[key]!r}")
    return out


def find_config(rec: str) -> str:
    hits = []
    for name in sorted(os.listdir(rec)):
        path = os.path.join(rec, name)
        if name.endswith((".yaml", ".yml")) and os.path.isfile(path):
            with open(path, encoding="utf-8", errors="replace") as f:
                if re.search(r"^\s*lid_topic:", f.read(), re.M):
                    hits.append(path)
    if len(hits) != 1:
        raise RecordingError(
            f"expected exactly one FAST-LIO config (*.yaml with lid_topic) in {rec}, "
            f"found {len(hits)}; pass --config")
    return hits[0]


def profile(rec: str, config: str, lidar_topic=None, imu_topic=None) -> dict:
    meta = read_metadata(rec)
    with open(config, encoding="utf-8") as f:
        cfg = parse_fastlio_config(f.read())
    lidar = lidar_topic or cfg.get("lid_topic")
    imu = imu_topic or cfg.get("imu_topic")
    if not lidar or not imu:
        raise RecordingError(f"{config} has no common.lid_topic / common.imu_topic")
    if lidar_topic and cfg.get("lid_topic") and lidar_topic != cfg["lid_topic"]:
        raise RecordingError(
            f"--lidar-topic {lidar_topic} differs from the config's lid_topic {cfg['lid_topic']}; "
            "FAST-LIO would subscribe to the config's topic")
    if imu_topic and cfg.get("imu_topic") and imu_topic != cfg["imu_topic"]:
        raise RecordingError(
            f"--imu-topic {imu_topic} differs from the config's imu_topic {cfg['imu_topic']}")
    for name in (lidar, imu):
        if name not in meta["topics"]:
            raise RecordingError(
                f"topic {name} (from the config) is not in the bag; bag has: "
                + ", ".join(sorted(meta["topics"])))
        if not meta["topics"][name]["count"]:
            raise RecordingError(f"topic {name} has 0 messages in the bag")
    lidar_type = meta["topics"][lidar]["type"]
    if meta["topics"][imu]["type"] != IMU:
        raise RecordingError(f"{imu} is {meta['topics'][imu]['type']}, expected {IMU}")
    want = LIDAR_TYPE_MSG.get(cfg.get("lidar_type"))
    if want is None:
        raise RecordingError(f"config preprocess.lidar_type {cfg.get('lidar_type')!r} is not 1-4")
    if lidar_type != want:
        raise RecordingError(
            f"config lidar_type {cfg['lidar_type']} expects {want}, but {lidar} is {lidar_type}")
    if lidar_type == POINTCLOUD2 and "timestamp_unit" not in cfg:
        raise RecordingError("PointCloud2 input needs preprocess.timestamp_unit in the config")
    kind = "Livox CustomMsg" if lidar_type == LIVOX_CUSTOM else "PointCloud2"
    return {
        "LIDAR": lidar,
        "IMU": imu,
        "LIDAR_MSG_TYPE": lidar_type,
        # Unused by FAST-LIO's Livox handler (offset_time is always ns); 3 keeps
        # inspect_bag's default for CustomMsg.
        "TS_UNIT": str(cfg.get("timestamp_unit", 3)),
        "SENSOR_KIND": f"{kind} lidar on {lidar} + IMU on {imu}",
    }


def check_complete(rec: str) -> dict:
    """Every bag file exists and its MCAP summary matches metadata.yaml.

    An interrupted copy leaves a truncated MCAP without a readable summary, or
    different counts. Returns per-topic counts; raises RecordingError otherwise.
    """
    meta = read_metadata(rec)
    try:
        from mcap.reader import make_reader
    except ImportError:
        raise RecordingError("python package 'mcap' is needed for the copy check")
    counts: dict = {}
    for name in meta["files"]:
        path = os.path.join(rec, "bag", name)
        if not path.endswith(".mcap"):
            raise RecordingError(f"{name}: only MCAP bags are supported")
        if not os.path.isfile(path):
            raise RecordingError(f"bag file missing: {path}")
        try:
            with open(path, "rb") as f:
                summary = make_reader(f).get_summary()
        except Exception as exc:  # truncated or corrupt file
            raise RecordingError(f"{path}: cannot read MCAP summary ({exc})")
        if summary is None or summary.statistics is None:
            raise RecordingError(f"{path}: no MCAP summary/statistics (incomplete copy?)")
        for cid, n in summary.statistics.channel_message_counts.items():
            topic = summary.channels[cid].topic
            counts[topic] = counts.get(topic, 0) + n
    for topic, info in meta["topics"].items():
        if counts.get(topic, 0) != info["count"]:
            raise RecordingError(
                f"{topic}: metadata.yaml says {info['count']} messages, MCAP summary has "
                f"{counts.get(topic, 0)}")
    return counts


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("complete")
    p.add_argument("rec")
    p = sub.add_parser("profile")
    p.add_argument("rec")
    p.add_argument("--config", required=True)
    p.add_argument("--lidar-topic")
    p.add_argument("--imu-topic")
    p = sub.add_parser("find-config")
    p.add_argument("rec")
    args = ap.parse_args(argv)
    try:
        if args.cmd == "complete":
            print(json.dumps(check_complete(args.rec), sort_keys=True))
        elif args.cmd == "profile":
            for k, v in profile(args.rec, args.config, args.lidar_topic, args.imu_topic).items():
                print(f"{k}={shlex.quote(v)}")
        else:
            print(find_config(args.rec))
    except RecordingError as exc:
        print(f"recording.py: {exc}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
