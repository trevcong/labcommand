#!/usr/bin/env python3
"""Count /Odometry messages and record when the last one arrived.

Runs inside the replay container so the runner can tell when FAST-LIO has
drained its backlog after playback ends. Writes "<count> <monotonic_s>" to
STATE_FILE after each message.

    odom_watch.py STATE_FILE [--topic /Odometry]
"""

import argparse
import signal
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("state_file")
    ap.add_argument("--topic", default="/Odometry")
    args = ap.parse_args()

    rclpy.init()
    node = Node("spatio_odom_watch")
    count = [0]

    def write():
        with open(args.state_file, "w") as f:
            f.write(f"{count[0]} {time.monotonic():.3f}\n")

    def cb(_msg):
        count[0] += 1
        write()

    node.create_subscription(Odometry, args.topic, cb, 1000)
    write()
    stop = [False]
    signal.signal(signal.SIGINT, lambda *_: stop.__setitem__(0, True))
    signal.signal(signal.SIGTERM, lambda *_: stop.__setitem__(0, True))
    while rclpy.ok() and not stop[0]:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
