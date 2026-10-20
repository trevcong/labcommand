"""Stream sensor_msgs/Imu and nav_msgs/Odometry out of an mcap bag.

This module is the only place that imports mcap. It does not resample or
rescale the vectors. Header stamp is sec + nanosec * 1e-9. Log time is
not used; comparing log time with header time belongs to another task.
"""

from __future__ import annotations

from pathlib import Path
from typing import List, Optional, Sequence

import numpy as np

import mcap
import mcap_ros2
from mcap_ros2.reader import read_ros2_messages

from spatio.imucheck.analyze import parse_positive_seconds


def library_versions():
    return "mcap {0}, mcap_ros2 {1}".format(
        getattr(mcap, "__version__", "unknown"),
        getattr(mcap_ros2, "__version__", "unknown"),
    )


def resolve_mcap_paths(path) -> List[Path]:
    """A .mcap file, or the bag/*.mcap files inside a recording directory."""
    candidate = Path(path)
    if candidate.is_file():
        return [candidate]
    if not candidate.is_dir():
        raise FileNotFoundError("bag path does not exist: {0}".format(candidate))
    bag_dir = candidate / "bag"
    found = sorted(bag_dir.glob("*.mcap")) if bag_dir.is_dir() else []
    if not found:
        found = sorted(candidate.glob("*.mcap"))
    if not found:
        raise FileNotFoundError("no .mcap under {0}".format(candidate))
    return found


def read_imu_and_odometry(path, imu_topic, odom_topic=None, max_seconds=None):
    """Return numpy arrays for one IMU topic and an optional odometry topic.

    max_seconds stops the scan once a header stamp is past the first IMU
    header plus that many seconds. The stop follows file order, so it is a
    development slice, not a clock analysis.
    """
    if max_seconds is not None:
        max_seconds = parse_positive_seconds(max_seconds, "max_seconds")
    paths = resolve_mcap_paths(path)
    topics = [imu_topic]
    if odom_topic:
        topics.append(odom_topic)

    t_s: List[float] = []
    accel: List[Sequence[float]] = []
    gyro: List[Sequence[float]] = []
    frames: List[str] = []
    frame_seen = set()
    ori_cov: List[Sequence[float]] = []
    gyro_cov: List[Sequence[float]] = []
    acc_cov: List[Sequence[float]] = []
    odom_t: List[float] = []
    odom_q: List[Sequence[float]] = []

    t_limit = None
    for mcap_path in paths:
        for item in read_ros2_messages(str(mcap_path), topics=topics, log_time_order=False):
            topic = item.channel.topic
            msg = item.ros_msg
            stamp = _header_seconds(msg)
            if topic == imu_topic:
                if t_limit is None and max_seconds is not None and stamp is not None:
                    t_limit = stamp + float(max_seconds)
                if t_limit is not None and stamp is not None and stamp > t_limit:
                    break
                if stamp is None:
                    continue
                acc, gyr = _vector3(msg.linear_acceleration), _vector3(msg.angular_velocity)
                t_s.append(stamp)
                accel.append(acc)
                gyro.append(gyr)
                frame = _frame_id(msg)
                if frame not in frame_seen:
                    frame_seen.add(frame)
                    frames.append(frame)
                ori_cov.append(_cov9(getattr(msg, "orientation_covariance", None)))
                gyro_cov.append(_cov9(getattr(msg, "angular_velocity_covariance", None)))
                acc_cov.append(_cov9(getattr(msg, "linear_acceleration_covariance", None)))
            elif odom_topic and topic == odom_topic:
                if stamp is None:
                    continue
                if t_limit is not None and stamp > t_limit:
                    continue
                quat = _odom_quat(msg)
                if quat is None:
                    continue
                odom_t.append(stamp)
                odom_q.append(quat)
        else:
            continue
        break

    if max_seconds is not None and t_s:
        # File order can deliver a late odometry message before the IMU
        # sample that trips the stop. Keep odometry inside the same slice.
        limit = t_s[0] + float(max_seconds)
        kept_t = []
        kept_q = []
        for stamp, quat in zip(odom_t, odom_q):
            if stamp <= limit:
                kept_t.append(stamp)
                kept_q.append(quat)
        odom_t = kept_t
        odom_q = kept_q

    return {
        "paths": [str(p) for p in paths],
        "t_s": np.asarray(t_s, dtype=np.float64),
        "accel": np.asarray(accel, dtype=np.float64).reshape((-1, 3)),
        "gyro": np.asarray(gyro, dtype=np.float64).reshape((-1, 3)),
        "frame_ids": frames,
        "orientation_covariance": np.asarray(ori_cov, dtype=np.float64).reshape((-1, 9)),
        "angular_velocity_covariance": np.asarray(gyro_cov, dtype=np.float64).reshape((-1, 9)),
        "linear_acceleration_covariance": np.asarray(acc_cov, dtype=np.float64).reshape((-1, 9)),
        "odom_t": np.asarray(odom_t, dtype=np.float64),
        "odom_quat": np.asarray(odom_q, dtype=np.float64).reshape((-1, 4)),
        "library_versions": library_versions(),
    }


def _header_seconds(msg) -> Optional[float]:
    header = getattr(msg, "header", None)
    if header is None:
        return None
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return None
    sec = getattr(stamp, "sec", None)
    nanosec = getattr(stamp, "nanosec", None)
    if sec is None or nanosec is None:
        return None
    return float(sec) + float(nanosec) * 1e-9


def _frame_id(msg) -> str:
    header = getattr(msg, "header", None)
    if header is None:
        return ""
    frame = getattr(header, "frame_id", "")
    if frame is None:
        return ""
    return str(frame)


def _vector3(value):
    return (
        float(getattr(value, "x")),
        float(getattr(value, "y")),
        float(getattr(value, "z")),
    )


def _cov9(value):
    if value is None:
        return (0.0,) * 9
    nums = [float(v) for v in list(value)]
    if len(nums) < 9:
        nums.extend([0.0] * (9 - len(nums)))
    return tuple(nums[:9])


def _odom_quat(msg):
    pose = getattr(msg, "pose", None)
    if pose is None:
        return None
    inner = getattr(pose, "pose", None)
    if inner is None:
        return None
    quat = getattr(inner, "orientation", None)
    if quat is None:
        return None
    return (
        float(getattr(quat, "x")),
        float(getattr(quat, "y")),
        float(getattr(quat, "z")),
        float(getattr(quat, "w")),
    )
