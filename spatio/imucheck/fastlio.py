"""Static quotes from the copied FAST-LIO and Livox sources.

Line numbers are whatever the file on disk contains when this runs.
A missing file or a missing line is reported as not found. Nothing is
invented to fill the gap.
"""

from __future__ import annotations

from pathlib import Path
from typing import List, Optional

DEFAULT_SPARK_SRC = Path.home() / "Developer" / "spatio-data" / "spark-src"

# (path relative to spark-src, unique substring, why it is quoted)
QUOTE_REQUESTS = (
    (
        "FAST_LIO_ROS2/src/IMU_Processing.hpp",
        "normalize the acceleration measurenments to unit gravity",
        "Init comment: acceleration is normalized to unit gravity.",
    ),
    (
        "FAST_LIO_ROS2/src/IMU_Processing.hpp",
        "init_state.grav = S2(- mean_acc / mean_acc.norm() * G_m_s2)",
        "Gravity is the opposite of the measured mean acceleration, scaled to G_m_s2.",
    ),
    (
        "FAST_LIO_ROS2/src/IMU_Processing.hpp",
        "init_state.bg  = mean_gyr",
        "Gyro bias is the mean angular velocity during init, with no unit conversion.",
    ),
    (
        "FAST_LIO_ROS2/src/IMU_Processing.hpp",
        "acc_avr * G_m_s2 / mean_acc.norm()",
        "Every later acceleration sample is rescaled by 9.81 / |mean_acc|.",
    ),
    (
        "FAST_LIO_ROS2/src/IMU_Processing.hpp",
        "in.gyro = angvel_avr",
        "Angular velocity is passed on with no deg-to-rad factor.",
    ),
    (
        "FAST_LIO_ROS2/include/common_lib.h",
        "#define G_m_s2",
        "G_m_s2 is the constant used in the rescale.",
    ),
    (
        "FAST_LIO_ROS2/include/use-ikfom.hpp",
        "res(i + 3) =  omega[i]",
        "The filter treats that angular velocity as the SO3 rate (radians).",
    ),
    (
        "livox_ros_driver2/src/comm/lidar_imu_data_queue.h",
        "Gyroscope X axis, Unit:rad/s",
        "Livox driver comment on the SDK gyro field. A comment, not a measurement.",
    ),
    (
        "livox_ros_driver2/src/comm/lidar_imu_data_queue.h",
        "Accelerometer X axis, Unit:g",
        "Livox driver comment on the SDK accel field. A comment, not a measurement.",
    ),
    (
        "livox_ros_driver2/src/lddc.cpp",
        "imu_msg.linear_acceleration.x = imu_data.acc_x",
        "The Livox driver copies accel into sensor_msgs/Imu with no scale factor.",
    ),
    (
        "livox_ros_driver2/src/lddc.cpp",
        "imu_msg.angular_velocity.x = imu_data.gyro_x",
        "The Livox driver copies gyro into sensor_msgs/Imu with no scale factor.",
    ),
)


def load_quotes(spark_src: Optional[Path] = None) -> List[dict]:
    root = Path(spark_src) if spark_src is not None else DEFAULT_SPARK_SRC
    quotes = []
    for relative, needle, why in QUOTE_REQUESTS:
        path = root / relative
        quotes.append(_one(path, relative, needle, why))
    adapter = root / "launchers" / "jt128_slam_adapter.py"
    quotes.append(_adapter_note(adapter))
    return quotes


def _one(path: Path, relative: str, needle: str, why: str) -> dict:
    if not path.is_file():
        return {
            "file": relative,
            "line": None,
            "text": None,
            "why": why,
            "status": "file not found: {0}".format(path),
        }
    # The copied launcher files can be empty. Source files we quote should not be.
    found = None
    with path.open("r", encoding="utf-8", errors="replace") as handle:
        for number, line in enumerate(handle, start=1):
            if needle in line:
                found = (number, line.strip())
                break
    if found is None:
        return {
            "file": relative,
            "line": None,
            "text": None,
            "why": why,
            "status": "line not found in {0}".format(path),
        }
    return {
        "file": relative,
        "line": found[0],
        "text": found[1],
        "why": why,
        "status": "ok",
    }


def _adapter_note(path: Path) -> dict:
    if not path.is_file():
        status = "file not found: {0}".format(path)
        size = None
    else:
        size = path.stat().st_size
        if size == 0:
            status = "file is 0 bytes in this copy; no adapter behavior was read"
        else:
            status = "file is {0} bytes; this tool does not interpret it".format(size)
    return {
        "file": "launchers/jt128_slam_adapter.py",
        "line": None,
        "text": None,
        "why": "JT128 adapter that publishes /slam/imu. Empty in this copy, so no claim is made.",
        "status": status,
        "size_bytes": size,
    }


def quotes_as_text(quotes=None) -> str:
    if quotes is None:
        quotes = load_quotes()
    rows = []
    for item in quotes:
        if item.get("line") and item.get("text"):
            rows.append(
                "{0}:{1}: {2}".format(item["file"], item["line"], item["text"])
            )
        else:
            rows.append("{0}: {1}".format(item["file"], item.get("status")))
    return "\n".join(rows)
