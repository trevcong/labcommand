#!/usr/bin/env python3
"""Adapt JT128 Hesai ROS topics to what FAST-LIO's Velodyne handler expects.

Hesai publishes:
  /lidar_points  PointCloud2 (x y z intensity ring timestamp[float64])
  /lidar_imu     sensor_msgs/Imu with accel in g and gyro in deg/s

FAST-LIO VELO16 (lidar_type 2) wants:
  PointCloud2 with a float32 ``time`` field (seconds from scan start)
  IMU gyro in rad/s (accel may stay in g; FAST-LIO rescales by ||mean_acc||)

Pure conversion helpers are imported by tests; ROS is only used in main().
"""

from __future__ import annotations

import math
import struct
from typing import Iterable

DEG_TO_RAD = math.pi / 180.0
VELO_STEP = 24  # x y z intensity time ring + pad


def gyro_deg_s_to_rad_s(
    x: float, y: float, z: float
) -> tuple[float, float, float]:
    """JT128 tail stores angular velocity in deg/s (30.49 mdps LSB)."""
    return (x * DEG_TO_RAD, y * DEG_TO_RAD, z * DEG_TO_RAD)


def field_offsets(fields: Iterable) -> dict[str, tuple[int, int]]:
    """Map field name -> (offset, datatype). Accepts ROS Field objects or dicts."""
    out: dict[str, tuple[int, int]] = {}
    for f in fields:
        if isinstance(f, dict):
            out[f["name"]] = (int(f["offset"]), int(f["datatype"]))
        else:
            out[f.name] = (int(f.offset), int(f.datatype))
    return out


def hesai_cloud_to_velodyne(
    data: bytes,
    width: int,
    height: int,
    point_step: int,
    offsets: dict[str, tuple[int, int]],
    stride: int = 1,
) -> tuple[bytes, int, float]:
    """Rewrite one Hesai PointCloud2 payload as Velodyne-layout bytes.

    Returns (payload, n_kept, dt_s) where dt_s is last-first timestamp in seconds.
    ``time`` is seconds from the first kept point (FAST-LIO timestamp_unit SEC).
    """
    needed = ("x", "y", "z", "intensity", "timestamp")
    for name in needed:
        if name not in offsets:
            raise ValueError(f"missing PointCloud2 field {name}")
    x_off = offsets["x"][0]
    y_off = offsets["y"][0]
    z_off = offsets["z"][0]
    i_off = offsets["intensity"][0]
    t_off = offsets["timestamp"][0]
    r_off = offsets["ring"][0] if "ring" in offsets else None
    n = width * height
    if n <= 0:
        return b"", 0, 0.0
    try:
        import numpy as np
    except ImportError:
        np = None
    if np is not None and stride >= 1:
        raw = np.frombuffer(data, dtype=np.uint8)
        if raw.size < n * point_step:
            n = raw.size // point_step
        view = raw[: n * point_step].reshape(n, point_step)
        sel = view[::stride]
        x = sel[:, x_off : x_off + 4].copy().view(np.float32).reshape(-1)
        y = sel[:, y_off : y_off + 4].copy().view(np.float32).reshape(-1)
        z = sel[:, z_off : z_off + 4].copy().view(np.float32).reshape(-1)
        inten = sel[:, i_off : i_off + 4].copy().view(np.float32).reshape(-1)
        t = sel[:, t_off : t_off + 8].copy().view(np.float64).reshape(-1)
        ring = (
            sel[:, r_off : r_off + 2].copy().view(np.uint16).reshape(-1)
            if r_off is not None
            else np.zeros(sel.shape[0], dtype=np.uint16)
        )
        t0 = float(t[0])
        rel = np.maximum(t - t0, 0.0).astype(np.float32)
        kept = int(sel.shape[0])
        out = np.zeros((kept, VELO_STEP), dtype=np.uint8)
        out[:, 0:4] = np.ascontiguousarray(x).view(np.uint8).reshape(-1, 4)
        out[:, 4:8] = np.ascontiguousarray(y).view(np.uint8).reshape(-1, 4)
        out[:, 8:12] = np.ascontiguousarray(z).view(np.uint8).reshape(-1, 4)
        out[:, 12:16] = np.ascontiguousarray(inten).view(np.uint8).reshape(-1, 4)
        out[:, 16:20] = np.ascontiguousarray(rel).view(np.uint8).reshape(-1, 4)
        out[:, 20:22] = np.ascontiguousarray(ring).view(np.uint8).reshape(-1, 2)
        return out.reshape(-1).tobytes(), kept, float(t[-1] - t0)
    t0 = None
    t_last = 0.0
    kept = 0
    for i in range(0, n, stride):
        t = struct.unpack_from("<d", data, i * point_step + t_off)[0]
        if t0 is None:
            t0 = t
        t_last = t
        kept += 1
    if t0 is None or kept == 0:
        return b"", 0, 0.0
    out = bytearray(kept * VELO_STEP)
    idx = 0
    for i in range(0, n, stride):
        base = i * point_step
        x, y, z, inten = struct.unpack_from("<ffff", data, base + x_off) if x_off == 0 else (
            struct.unpack_from("<f", data, base + x_off)[0],
            struct.unpack_from("<f", data, base + y_off)[0],
            struct.unpack_from("<f", data, base + z_off)[0],
            struct.unpack_from("<f", data, base + i_off)[0],
        )
        if x_off != 0:
            pass
        else:
            # x_off==0 but y/z/i may not be 4/8/12; unpack individually if needed
            if (y_off, z_off, i_off) != (4, 8, 12):
                x = struct.unpack_from("<f", data, base + x_off)[0]
                y = struct.unpack_from("<f", data, base + y_off)[0]
                z = struct.unpack_from("<f", data, base + z_off)[0]
                inten = struct.unpack_from("<f", data, base + i_off)[0]
        t = struct.unpack_from("<d", data, base + t_off)[0]
        ring = struct.unpack_from("<H", data, base + r_off)[0] if r_off is not None else 0
        rel = float(t - t0)
        if rel < 0.0:
            rel = 0.0
        dst = idx * VELO_STEP
        struct.pack_into("<ffff", out, dst, x, y, z, inten)
        struct.pack_into("<f", out, dst + 16, rel)
        struct.pack_into("<H", out, dst + 20, ring & 0xFFFF)
        idx += 1
    return bytes(out), kept, float(t_last - t0)


def main() -> int:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Imu, PointCloud2, PointField

    class Adapter(Node):
        def __init__(self) -> None:
            super().__init__("jt128_slam_adapter")
            self.stride = 4
            # FAST-LIO subscribes RELIABLE; sensor_data (best-effort) is dropped.
            qos = QoSProfile(
                depth=50,
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
            )
            self.pub_cloud = self.create_publisher(PointCloud2, "/slam/points", qos)
            self.pub_imu = self.create_publisher(Imu, "/slam/imu", qos)
            self.create_subscription(PointCloud2, "/lidar_points", self._on_cloud, qos)
            self.create_subscription(Imu, "/lidar_imu", self._on_imu, qos)
            self._n_cloud = 0
            self.get_logger().info(
                "JT128 SLAM adapter: /lidar_points -> /slam/points (VELO time), "
                "/lidar_imu gyro deg/s -> /slam/imu rad/s"
            )

        def _on_imu(self, msg: Imu) -> None:
            out = Imu()
            out.header = msg.header
            out.orientation = msg.orientation
            out.orientation_covariance = msg.orientation_covariance
            gx, gy, gz = gyro_deg_s_to_rad_s(
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z,
            )
            out.angular_velocity.x = gx
            out.angular_velocity.y = gy
            out.angular_velocity.z = gz
            out.angular_velocity_covariance = msg.angular_velocity_covariance
            out.linear_acceleration = msg.linear_acceleration
            out.linear_acceleration_covariance = msg.linear_acceleration_covariance
            self.pub_imu.publish(out)

        def _on_cloud(self, msg: PointCloud2) -> None:
            offs = field_offsets(msg.fields)
            payload, n, dt = hesai_cloud_to_velodyne(
                bytes(msg.data),
                int(msg.width),
                int(msg.height),
                int(msg.point_step),
                offs,
                stride=self.stride,
            )
            if n == 0:
                return
            out = PointCloud2()
            out.header = msg.header
            out.height = 1
            out.width = n
            out.is_bigendian = False
            out.is_dense = True
            out.point_step = VELO_STEP
            out.row_step = VELO_STEP * n
            out.fields = [
                PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(
                    name="intensity", offset=12, datatype=PointField.FLOAT32, count=1
                ),
                PointField(name="time", offset=16, datatype=PointField.FLOAT32, count=1),
                PointField(name="ring", offset=20, datatype=PointField.UINT16, count=1),
            ]
            out.data = payload
            self.pub_cloud.publish(out)
            self._n_cloud += 1
            if self._n_cloud <= 3 or self._n_cloud % 50 == 0:
                self.get_logger().info(
                    f"slam cloud n={n} dt={dt:.4f}s frame={msg.header.frame_id}"
                )

    rclpy.init()
    node = Adapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
