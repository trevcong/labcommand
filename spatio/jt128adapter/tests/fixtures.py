"""Small synthetic ROS2 MCAPs; no ROS installation or real-data extracts."""
from __future__ import annotations

import math
import struct
from mcap_ros2.writer import Writer

SEP = "\n" + "=" * 80 + "\nMSG: "
HEADER = (SEP + "std_msgs/Header\nbuiltin_interfaces/Time stamp\nstring frame_id\n" +
          SEP + "builtin_interfaces/Time\nint32 sec\nuint32 nanosec\n")
IMU_SCHEMA = ("std_msgs/Header header\ngeometry_msgs/Quaternion orientation\nfloat64[9] orientation_covariance\n"
              "geometry_msgs/Vector3 angular_velocity\nfloat64[9] angular_velocity_covariance\n"
              "geometry_msgs/Vector3 linear_acceleration\nfloat64[9] linear_acceleration_covariance\n" + HEADER +
              SEP + "geometry_msgs/Quaternion\nfloat64 x\nfloat64 y\nfloat64 z\nfloat64 w\n" +
              SEP + "geometry_msgs/Vector3\nfloat64 x\nfloat64 y\nfloat64 z\n")
CLOUD_SCHEMA = ("std_msgs/Header header\nuint32 height\nuint32 width\nsensor_msgs/PointField[] fields\n"
                "bool is_bigendian\nuint32 point_step\nuint32 row_step\nuint8[] data\nbool is_dense\n" + HEADER +
                SEP + "sensor_msgs/PointField\nstring name\nuint32 offset\nuint8 datatype\nuint32 count\n")


def header(ns, frame="hesai_lidar"):
    return {"stamp": {"sec": ns // 10**9, "nanosec": ns % 10**9}, "frame_id": frame}


def imu(ns, gyro, accel):
    return {"header": header(ns), "orientation": dict(x=0., y=0., z=0., w=1.),
            "orientation_covariance": [0.] * 9,
            "angular_velocity": dict(zip("xyz", gyro)), "angular_velocity_covariance": [0.] * 9,
            "linear_acceleration": dict(zip("xyz", accel)), "linear_acceleration_covariance": [0.] * 9}


def make_mcap(path, fields, gyro_scale=math.pi / 180, accel_scale=1., first_time=0., stamp_offset=0):
    epoch = 1_787_700_000_000_000_000
    receipt = epoch + 1000 * 10**9
    data = b"".join(struct.pack("<fffffHxx", float(i), 2., 3., 10., t, ring)
                    for i, (t, ring) in enumerate([(first_time, 0), (.05, 4), (.1, 127)]))
    cloud = dict(header=header(epoch), height=1, width=3, fields=fields, is_bigendian=False,
                 point_step=24, row_step=72, data=data, is_dense=True)
    with path.open("wb") as f:
        with Writer(f) as writer:
            im = writer.register_msgdef("sensor_msgs/msg/Imu", IMU_SCHEMA)
            pc = writer.register_msgdef("sensor_msgs/msg/PointCloud2", CLOUD_SCHEMA)
            writer.write_message("/slam/points", pc, cloud, log_time=receipt)
            for k in range(4):
                ns = epoch + k * 5_000_000
                gyro = [1. + k, -2. - 2 * k, 3. + k * k]
                accel = [.1 + k * .01, -.3 + k * .02, .95 + k * .001]
                writer.write_message("/lidar_imu", im, imu(ns, gyro, accel), log_time=receipt + k * 5_000_000 + 1)
                writer.write_message("/slam/imu", im, imu(ns + stamp_offset, [x * gyro_scale for x in gyro],
                                                         [x * accel_scale for x in accel]), log_time=receipt + k * 5_000_000 + 2)
    return path
