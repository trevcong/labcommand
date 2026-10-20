"""Synthetic tests for the IMU characterizer. No real bags."""

from __future__ import annotations

import json
import math
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

from spatio.imucheck.analyze import DEG_PER_RAD, characterize

REPO = Path(__file__).resolve().parents[3]

IMU_SCHEMA = """
std_msgs/Header header
geometry_msgs/Quaternion orientation
float64[9] orientation_covariance
geometry_msgs/Vector3 angular_velocity
float64[9] angular_velocity_covariance
geometry_msgs/Vector3 linear_acceleration
float64[9] linear_acceleration_covariance
================================================================================
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id
================================================================================
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec
================================================================================
MSG: geometry_msgs/Quaternion
float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Vector3
float64 x
float64 y
float64 z
"""

ODOM_SCHEMA = """
std_msgs/Header header
string child_frame_id
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist
================================================================================
MSG: geometry_msgs/PoseWithCovariance
geometry_msgs/Pose pose
float64[36] covariance
================================================================================
MSG: geometry_msgs/Pose
geometry_msgs/Point position
geometry_msgs/Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/TwistWithCovariance
geometry_msgs/Twist twist
float64[36] covariance
================================================================================
MSG: geometry_msgs/Twist
geometry_msgs/Vector3 linear
geometry_msgs/Vector3 angular
================================================================================
MSG: geometry_msgs/Vector3
float64 x
float64 y
float64 z
================================================================================
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id
================================================================================
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec
"""


def _quiet(seconds, accel_xyz, hz=100.0):
    n = int(round(seconds * hz))
    t = np.arange(n, dtype=np.float64) / hz
    accel = np.tile(np.asarray(accel_xyz, dtype=np.float64), (n, 1))
    gyro = np.zeros((n, 3), dtype=np.float64)
    gyro[:, 0] = 0.01
    return t, accel, gyro


class ImuCheckTests(unittest.TestCase):
    def test_stationary_window_detection(self):
        hz = 100.0
        t = np.arange(int(4 * hz), dtype=np.float64) / hz
        accel = np.zeros((len(t), 3), dtype=np.float64)
        gyro = np.zeros((len(t), 3), dtype=np.float64)
        accel[:, 2] = 1.0
        gyro[:, 0] = 0.01
        wild_gyro = (t >= 1.0) & (t < 2.0)
        gyro[wild_gyro, 2] = np.sin(2.0 * np.pi * 4.0 * (t[wild_gyro] - 1.0))
        wild_accel = (t >= 2.0) & (t < 3.0)
        accel[wild_accel, 2] = 1.0 + 0.6 * np.sin(2.0 * np.pi * 5.0 * (t[wild_accel] - 2.0))

        report = characterize(t, accel, gyro, topic="/test")
        windows = report["stationary_windows"]
        self.assertTrue(any(_is_window(w, 0.0, 1.0) for w in windows))
        self.assertTrue(any(_is_window(w, 3.0, 4.0) for w in windows))
        self.assertFalse(any(_overlaps(w, 1.0, 2.0) for w in windows))
        self.assertFalse(any(_overlaps(w, 2.0, 3.0) for w in windows))

    def test_accel_unit_g(self):
        t, accel, gyro = _quiet(2.0, (0.0, 0.0, 1.0))
        report = characterize(t, accel, gyro, topic="/test")
        self.assertEqual(report["accel_unit"]["verdict"], "g")
        self.assertAlmostEqual(report["accel_norm_stationary"]["median"], 1.0, places=6)

    def test_accel_unit_m_s2(self):
        t, accel, gyro = _quiet(2.0, (0.0, 0.0, 9.81))
        report = characterize(t, accel, gyro, topic="/test")
        self.assertEqual(report["accel_unit"]["verdict"], "m/s^2")
        self.assertAlmostEqual(report["accel_norm_stationary"]["median"], 9.81, places=6)

    def test_accel_unit_undetermined_when_never_stationary(self):
        t = np.arange(0.0, 5.0, 0.01)
        accel = np.zeros((len(t), 3), dtype=np.float64)
        gyro = np.zeros((len(t), 3), dtype=np.float64)
        accel[:, 0] = 5.0 * np.sin(2.0 * np.pi * 3.0 * t)
        accel[:, 2] = 8.0 + 4.0 * np.sin(2.0 * np.pi * 1.3 * t)
        gyro[:, 2] = 2.0 * np.sin(2.0 * np.pi * 0.7 * t)
        report = characterize(t, accel, gyro, topic="/test")
        self.assertEqual(report["stationary_windows"], [])
        self.assertEqual(report["accel_unit"]["verdict"], "undetermined")
        self.assertIsNone(report["accel_norm_stationary"]["median"])
        self.assertIn("not guessed", report["accel_unit"]["evidence"].lower())

    def test_gravity_axis_upright(self):
        t, accel, gyro = _quiet(2.0, (0.0, 0.0, 1.0))
        report = characterize(t, accel, gyro, topic="/test")
        gravity = report["gravity"]
        self.assertEqual(gravity["dominant_axis"], "+z")
        self.assertAlmostEqual(gravity["angle_from_axis_deg"], 0.0, places=6)
        self.assertEqual(gravity["mount_sentence"], "sensor appears upright")
        direction = gravity["mean_dir_sensor"]
        self.assertAlmostEqual(direction[2], 1.0, places=6)

    def test_gravity_axis_inverted(self):
        t, accel, gyro = _quiet(2.0, (0.0, 0.0, -1.0))
        report = characterize(t, accel, gyro, topic="/test")
        gravity = report["gravity"]
        self.assertEqual(gravity["dominant_axis"], "-z")
        self.assertAlmostEqual(gravity["angle_from_axis_deg"], 0.0, places=6)
        self.assertEqual(gravity["mount_sentence"], "sensor appears inverted")
        self.assertAlmostEqual(gravity["mean_dir_sensor"][2], -1.0, places=6)

    def test_gravity_tilt_angle_reported(self):
        tilt = math.radians(30.0)
        t, accel, gyro = _quiet(2.0, (math.sin(tilt), 0.0, math.cos(tilt)))
        report = characterize(t, accel, gyro, topic="/test")
        gravity = report["gravity"]
        self.assertEqual(gravity["dominant_axis"], "+z")
        self.assertAlmostEqual(gravity["angle_from_axis_deg"], 30.0, places=5)
        self.assertIn("tilted", gravity["mount_sentence"])
        self.assertIn("30.0", gravity["mount_sentence"])

    def test_gyro_unit_rad_s_from_odometry_ratio(self):
        t, accel, gyro, odom_t, odom_q = _yaw_stream(rate_rad_s=0.5, gyro_scale=1.0)
        report = characterize(
            t, accel, gyro, odom_t=odom_t, odom_quat=odom_q, topic="/test"
        )
        gyro_unit = report["gyro_unit"]
        self.assertEqual(gyro_unit["verdict"], "rad/s")
        self.assertGreater(gyro_unit["windows_used"], 0)
        self.assertAlmostEqual(gyro_unit["ratio"], 1.0, places=4)
        self.assertIn("not proof", gyro_unit["caveat"])

    def test_gyro_unit_deg_s_from_odometry_ratio(self):
        t, accel, gyro, odom_t, odom_q = _yaw_stream(rate_rad_s=0.5, gyro_scale=DEG_PER_RAD)
        report = characterize(
            t, accel, gyro, odom_t=odom_t, odom_quat=odom_q, topic="/test"
        )
        gyro_unit = report["gyro_unit"]
        self.assertEqual(gyro_unit["verdict"], "deg/s")
        self.assertGreater(gyro_unit["windows_used"], 0)
        self.assertAlmostEqual(gyro_unit["ratio"], DEG_PER_RAD, places=4)

    def test_deg_s_stationary_gate_uses_detected_unit(self):
        # A resting bias of about 1.7 deg/s is still, but it is above the
        # unscaled 0.20 gate. Odometry shows the message is deg/s, so the
        # same physical gate (0.20 rad/s) must accept the still section.
        hz = 100.0
        t = np.arange(int(4 * hz), dtype=np.float64) / hz
        accel = np.zeros((len(t), 3), dtype=np.float64)
        gyro = np.zeros((len(t), 3), dtype=np.float64)
        accel[:, 2] = 1.0
        gyro[:, 0] = 1.5
        gyro[:, 1] = -0.8
        gyro[:, 2] = 0.4
        moving = t >= 2.0
        gyro[moving, :] = 0.0
        gyro[moving, 2] = 40.0
        odom_t = t[::5].copy()
        yaw = np.zeros(len(odom_t), dtype=np.float64)
        moved = odom_t >= 2.0
        yaw[moved] = (40.0 / DEG_PER_RAD) * (odom_t[moved] - 2.0)
        odom_q = np.zeros((len(odom_t), 4), dtype=np.float64)
        odom_q[:, 2] = np.sin(yaw / 2.0)
        odom_q[:, 3] = np.cos(yaw / 2.0)
        report = characterize(
            t, accel, gyro, odom_t=odom_t, odom_quat=odom_q, topic="/test"
        )
        self.assertEqual(report["gyro_unit"]["verdict"], "deg/s")
        self.assertAlmostEqual(report["gyro_unit"]["ratio"], DEG_PER_RAD, places=3)
        self.assertTrue(any(_overlaps(w, 0.0, 1.0) for w in report["stationary_windows"]))
        self.assertFalse(any(_overlaps(w, 2.0, 4.0) for w in report["stationary_windows"]))
        self.assertEqual(report["accel_unit"]["verdict"], "g")

    def test_gyro_unit_undetermined_without_odometry(self):
        t, accel, gyro, _odom_t, _odom_q = _yaw_stream(rate_rad_s=0.5, gyro_scale=1.0)
        report = characterize(t, accel, gyro, topic="/test")
        gyro_unit = report["gyro_unit"]
        self.assertEqual(gyro_unit["verdict"], "undetermined")
        self.assertIsNone(gyro_unit["ratio"])
        self.assertEqual(gyro_unit["windows_used"], 0)
        self.assertIn("not proof", gyro_unit["caveat"])
        self.assertIn("not guessed", gyro_unit["evidence"].lower())

    def test_gyro_bias_and_noise(self):
        n = 200
        t = np.arange(n, dtype=np.float64) / float(n)
        sign = np.where(np.arange(n) % 2 == 0, 1.0, -1.0)
        accel = np.zeros((n, 3), dtype=np.float64)
        gyro = np.zeros((n, 3), dtype=np.float64)
        accel[:, 2] = 1.0 + 0.004 * sign
        gyro[:, 0] = 0.01 + 0.002 * sign
        gyro[:, 1] = -0.02
        gyro[:, 2] = 0.005
        report = characterize(t, accel, gyro, topic="/test")
        scale = math.sqrt(n / (n - 1))
        self.assertAlmostEqual(report["gyro_bias"][0], 0.01, places=10)
        self.assertAlmostEqual(report["gyro_bias"][1], -0.02, places=10)
        self.assertAlmostEqual(report["gyro_bias"][2], 0.005, places=10)
        self.assertAlmostEqual(report["gyro_noise_std"][0], 0.002 * scale, places=8)
        self.assertAlmostEqual(report["gyro_noise_std"][1], 0.0, places=12)
        self.assertAlmostEqual(report["gyro_noise_std"][2], 0.0, places=12)
        self.assertAlmostEqual(report["accel_noise_std"][2], 0.004 * scale, places=8)
        self.assertAlmostEqual(report["accel_noise_std"][0], 0.0, places=12)
        self.assertAlmostEqual(report["accel_noise_std"][1], 0.0, places=12)

    def test_rate_and_interval_jitter(self):
        dts = np.array([0.004] + [0.005] * 18 + [0.007], dtype=np.float64)
        t = np.concatenate([np.array([0.0]), np.cumsum(dts)])
        accel = np.zeros((len(t), 3), dtype=np.float64)
        accel[:, 2] = 1.0
        gyro = np.zeros((len(t), 3), dtype=np.float64)
        report = characterize(t, accel, gyro, topic="/test")
        self.assertAlmostEqual(report["rate_hz"], 20.0 / 0.101, places=6)
        interval = report["interval_s"]
        self.assertAlmostEqual(interval["p5"], 0.00495, places=6)
        self.assertAlmostEqual(interval["p50"], 0.005, places=8)
        self.assertAlmostEqual(interval["p95"], 0.0051, places=6)
        self.assertAlmostEqual(interval["max"], 0.007, places=8)

    def test_input_values_never_rescaled(self):
        t = np.arange(200, dtype=np.float64) / 100.0
        accel = np.zeros((200, 3), dtype=np.float64)
        accel[:, 2] = 9.81
        gyro = np.zeros((200, 3), dtype=np.float64)
        gyro[:, 0] = 0.02
        odom_t = t[::10].copy()
        odom_q = np.tile(np.array([0.0, 0.0, 0.0, 1.0]), (len(odom_t), 1))
        t_before = t.copy()
        accel_before = accel.copy()
        gyro_before = gyro.copy()
        odom_t_before = odom_t.copy()
        odom_q_before = odom_q.copy()
        report = characterize(
            t, accel, gyro, odom_t=odom_t, odom_quat=odom_q, topic="/test"
        )
        np.testing.assert_array_equal(t, t_before)
        np.testing.assert_array_equal(accel, accel_before)
        np.testing.assert_array_equal(gyro, gyro_before)
        np.testing.assert_array_equal(odom_t, odom_t_before)
        np.testing.assert_array_equal(odom_q, odom_q_before)
        self.assertEqual(report["accel_unit"]["verdict"], "m/s^2")
        self.assertAlmostEqual(report["accel_norm_stationary"]["median"], 9.81, places=9)

    def test_synthetic_mcap_roundtrip_cli(self):
        from mcap.writer import CompressionType
        from mcap_ros2.writer import Writer

        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            bag = root / "synthetic.mcap"
            out = root / "out"
            _write_synthetic_mcap(bag, Writer, CompressionType)
            proc = subprocess.run(
                [
                    sys.executable,
                    "-m",
                    "spatio.imucheck",
                    str(bag),
                    "--topic",
                    "/imu",
                    "--odometry",
                    "/odom",
                    "--out",
                    str(out),
                ],
                cwd=str(REPO),
                capture_output=True,
                text=True,
                check=False,
            )
            self.assertEqual(proc.returncode, 0, proc.stderr + proc.stdout)
            report_path = out / "imucheck_imu.json"
            page_path = out / "imucheck_imu.md"
            self.assertTrue(report_path.is_file())
            self.assertTrue(page_path.is_file())
            report = json.loads(report_path.read_text(encoding="utf-8"))
            self.assertEqual(report["schema"], "spatio.imucheck/1")
            self.assertEqual(report["topic"], "/imu")
            self.assertEqual(report["count"], 400)
            self.assertEqual(report["frame_ids"], ["livox_frame"])
            self.assertEqual(report["accel_unit"]["verdict"], "g")
            self.assertEqual(report["gravity"]["dominant_axis"], "+z")
            self.assertEqual(report["gravity"]["mount_sentence"], "sensor appears upright")
            self.assertEqual(report["gyro_unit"]["verdict"], "rad/s")
            self.assertAlmostEqual(report["gyro_unit"]["ratio"], 1.0, places=3)
            self.assertEqual(
                report["covariance_published"]["linear_acceleration"]["pattern"],
                "all_zero",
            )
            page = page_path.read_text(encoding="utf-8")
            self.assertIn("/imu", page)
            if "mean_acc.norm" not in page:
                self.assertIn("not found", page.lower())


def _is_window(window, t0, t1):
    return abs(window["t0_s"] - t0) < 1e-9 and abs(window["t1_s"] - t1) < 1e-9


def _overlaps(window, t0, t1):
    return window["t0_s"] < t1 and window["t1_s"] > t0


def _yaw_stream(rate_rad_s, gyro_scale, seconds=4.0, hz=100.0):
    n = int(round(seconds * hz))
    t = np.arange(n, dtype=np.float64) / hz
    accel = np.zeros((n, 3), dtype=np.float64)
    accel[:, 2] = 1.0
    gyro = np.zeros((n, 3), dtype=np.float64)
    gyro[:, 2] = rate_rad_s * gyro_scale
    odom_t = t[::5].copy()
    yaw = rate_rad_s * odom_t
    odom_q = np.zeros((len(odom_t), 4), dtype=np.float64)
    odom_q[:, 2] = np.sin(yaw / 2.0)
    odom_q[:, 3] = np.cos(yaw / 2.0)
    return t, accel, gyro, odom_t, odom_q


def _split_stamp(t):
    sec = int(math.floor(t))
    nanosec = int(round((t - sec) * 1e9))
    if nanosec >= 1000000000:
        sec += 1
        nanosec -= 1000000000
    if nanosec < 0:
        nanosec = 0
    return sec, nanosec


def _write_synthetic_mcap(path, writer_cls, compression_type):
    hz = 100.0
    n = 400
    t = np.arange(n, dtype=np.float64) / hz
    split = 2.0
    with open(path, "wb") as handle:
        writer = writer_cls(handle, compression=compression_type.NONE)
        imu_schema = writer.register_msgdef("sensor_msgs/msg/Imu", IMU_SCHEMA)
        odom_schema = writer.register_msgdef("nav_msgs/msg/Odometry", ODOM_SCHEMA)
        zeros9 = [0.0] * 9
        zeros36 = [0.0] * 36
        for index, stamp in enumerate(t):
            sec, nanosec = _split_stamp(float(stamp))
            log_ns = sec * 1000000000 + nanosec
            moving = float(stamp) >= split
            gyro_z = 0.5 if moving else 0.0
            gyro_x = 0.0 if moving else 0.01
            writer.write_message(
                topic="/imu",
                schema=imu_schema,
                message={
                    "header": {
                        "stamp": {"sec": sec, "nanosec": nanosec},
                        "frame_id": "livox_frame",
                    },
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                    "orientation_covariance": zeros9,
                    "angular_velocity": {"x": gyro_x, "y": 0.0, "z": gyro_z},
                    "angular_velocity_covariance": list(zeros9),
                    "linear_acceleration": {"x": 0.0, "y": 0.0, "z": 1.0},
                    "linear_acceleration_covariance": list(zeros9),
                },
                log_time=log_ns,
                publish_time=log_ns,
                sequence=index,
            )
            if index % 5 == 0:
                yaw = 0.5 * (float(stamp) - split) if moving else 0.0
                writer.write_message(
                    topic="/odom",
                    schema=odom_schema,
                    message={
                        "header": {
                            "stamp": {"sec": sec, "nanosec": nanosec},
                            "frame_id": "map",
                        },
                        "child_frame_id": "base",
                        "pose": {
                            "pose": {
                                "position": {"x": 0.0, "y": 0.0, "z": 0.0},
                                "orientation": {
                                    "x": 0.0,
                                    "y": 0.0,
                                    "z": math.sin(yaw / 2.0),
                                    "w": math.cos(yaw / 2.0),
                                },
                            },
                            "covariance": list(zeros36),
                        },
                        "twist": {
                            "twist": {
                                "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
                                "angular": {"x": 0.0, "y": 0.0, "z": gyro_z},
                            },
                            "covariance": list(zeros36),
                        },
                    },
                    log_time=log_ns,
                    publish_time=log_ns,
                    sequence=index,
                )
        writer.finish()


if __name__ == "__main__":
    unittest.main()
