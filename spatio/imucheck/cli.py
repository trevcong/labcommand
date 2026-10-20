"""Command line for the IMU characterizer."""

from __future__ import annotations

import argparse
import sys

from spatio.imucheck.analyze import characterize
from spatio.imucheck.fastlio import load_quotes
from spatio.imucheck.reader import read_imu_and_odometry
from spatio.imucheck.report import write_reports


def build_parser():
    parser = argparse.ArgumentParser(
        prog="python3 -m spatio.imucheck",
        description=(
            "Report IMU acceleration and gyro units, gravity axis, bias, "
            "rate, and FAST-LIO's normalization from one sensor_msgs/Imu topic."
        ),
    )
    parser.add_argument("bag", help="Recording directory or a .mcap file")
    parser.add_argument("--topic", required=True, help="IMU topic, for example /livox/imu")
    parser.add_argument(
        "--odometry",
        default="/Odometry",
        help="Odometry topic used only for the gyro-unit ratio. Default: /Odometry",
    )
    parser.add_argument(
        "--max-seconds",
        type=float,
        default=None,
        help="Stop after this many header seconds from the first IMU sample. Development slices.",
    )
    parser.add_argument("--out", required=True, help="Directory for the JSON and markdown")
    return parser


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.max_seconds is not None and args.max_seconds <= 0:
        print("imucheck: --max-seconds must be positive", file=sys.stderr)
        return 2
    odom_topic = args.odometry or None
    try:
        loaded = read_imu_and_odometry(
            args.bag,
            args.topic,
            odom_topic=odom_topic,
            max_seconds=args.max_seconds,
        )
    except FileNotFoundError as exc:
        print("imucheck: {0}".format(exc), file=sys.stderr)
        return 2
    if loaded["t_s"].size == 0:
        print(
            "imucheck: no messages on {0} in {1}".format(args.topic, args.bag),
            file=sys.stderr,
        )
        return 2
    odom_t = loaded["odom_t"] if loaded["odom_t"].size else None
    odom_q = loaded["odom_quat"] if loaded["odom_quat"].size else None
    report = characterize(
        loaded["t_s"],
        loaded["accel"],
        loaded["gyro"],
        frame_ids=loaded["frame_ids"],
        orientation_covariance=loaded["orientation_covariance"],
        angular_velocity_covariance=loaded["angular_velocity_covariance"],
        linear_acceleration_covariance=loaded["linear_acceleration_covariance"],
        odom_t=odom_t,
        odom_quat=odom_q,
        topic=args.topic,
    )
    quotes = load_quotes()
    json_path, md_path = write_reports(
        report,
        args.out,
        bag_path=loaded["paths"][0] if len(loaded["paths"]) == 1 else loaded["paths"],
        library_versions=loaded["library_versions"],
        quotes=quotes,
    )
    print(json_path)
    print(md_path)
    return 0
