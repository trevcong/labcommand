"""spatio/replay/recording.py: topics from the config, types from the bag,
copy completeness from the MCAP summary. Uses made-up topic names so nothing
can depend on the Mid-360 or JT128 recordings."""

import os
import shutil
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "replay"))
import recording  # noqa: E402

META = """rosbag2_bagfile_information:
  version: 9
  storage_identifier: mcap
  duration:
    nanoseconds: 1000
  message_count: {total}
  topics_with_message_count:
    - topic_metadata:
        name: {lidar}
        type: {lidar_type}
        serialization_format: cdr
        offered_qos_profiles:
          - history: unknown
            depth: 0
      message_count: {n_lidar}
    - topic_metadata:
        name: {imu}
        type: sensor_msgs/msg/Imu
        serialization_format: cdr
        offered_qos_profiles:
          - history: unknown
      message_count: {n_imu}
  compression_format: ""
  relative_file_paths:
    - bag_0.mcap
  files:
    - path: bag_0.mcap
      message_count: {total}
"""

CONFIG = """/**:
    ros__parameters:
        common:
            lid_topic:  "{lidar}"
            imu_topic:  "{imu}"
        preprocess:
            lidar_type: {lidar_type}       # comment
            timestamp_unit: 2
"""


def write_rec(root, lidar="/rig7/points", imu="/rig7/inertial", lidar_type="livox_ros_driver2/msg/CustomMsg",
              n_lidar=3, n_imu=5, cfg_type=1, mcap_counts=None):
    os.makedirs(os.path.join(root, "bag"))
    with open(os.path.join(root, "bag", "metadata.yaml"), "w") as f:
        f.write(META.format(lidar=lidar, imu=imu, lidar_type=lidar_type, n_lidar=n_lidar, n_imu=n_imu,
                            total=n_lidar + n_imu))
    with open(os.path.join(root, "rig7_fastlio.yaml"), "w") as f:
        f.write(CONFIG.format(lidar=lidar, imu=imu, lidar_type=cfg_type))
    if mcap_counts is not None:
        from mcap.writer import Writer
        with open(os.path.join(root, "bag", "bag_0.mcap"), "wb") as fh:
            w = Writer(fh)
            w.start()
            sid = w.register_schema(name="x", encoding="ros2msg", data=b"")
            t = 1
            for topic, n in mcap_counts.items():
                cid = w.register_channel(topic=topic, message_encoding="cdr", schema_id=sid)
                for _ in range(n):
                    w.add_message(channel_id=cid, log_time=t, data=b"\x00\x01\x00\x00", publish_time=t)
                    t += 1
            w.finish()


class RecordingTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.rec = os.path.join(self.tmp, "rec")

    def tearDown(self):
        shutil.rmtree(self.tmp)

    def test_parse_metadata_any_topic_names(self):
        write_rec(self.rec)
        meta = recording.read_metadata(self.rec)
        self.assertEqual(meta["files"], ["bag_0.mcap"])
        self.assertEqual(meta["topics"]["/rig7/points"], {"type": "livox_ros_driver2/msg/CustomMsg", "count": 3})
        self.assertEqual(meta["topics"]["/rig7/inertial"]["count"], 5)

    def test_profile_takes_topics_from_config(self):
        write_rec(self.rec)
        cfg = recording.find_config(self.rec)
        prof = recording.profile(self.rec, cfg)
        self.assertEqual(prof["LIDAR"], "/rig7/points")
        self.assertEqual(prof["IMU"], "/rig7/inertial")
        self.assertIn("Livox CustomMsg", prof["SENSOR_KIND"])

    def test_profile_pointcloud2_uses_config_timestamp_unit(self):
        write_rec(self.rec, lidar_type="sensor_msgs/msg/PointCloud2", cfg_type=4)
        prof = recording.profile(self.rec, recording.find_config(self.rec))
        self.assertEqual(prof["TS_UNIT"], "2")

    def test_profile_rejects_type_mismatch(self):
        write_rec(self.rec, lidar_type="sensor_msgs/msg/PointCloud2", cfg_type=1)
        with self.assertRaisesRegex(recording.RecordingError, "expects livox"):
            recording.profile(self.rec, recording.find_config(self.rec))

    def test_profile_rejects_topic_not_in_bag(self):
        write_rec(self.rec)
        cfg = os.path.join(self.tmp, "other.yaml")
        with open(cfg, "w") as f:
            f.write(CONFIG.format(lidar="/livox/lidar", imu="/rig7/inertial", lidar_type=1))
        with self.assertRaisesRegex(recording.RecordingError, "/livox/lidar .* not in the bag"):
            recording.profile(self.rec, cfg)

    def test_profile_rejects_empty_topic(self):
        write_rec(self.rec, n_imu=0)
        with self.assertRaisesRegex(recording.RecordingError, "0 messages"):
            recording.profile(self.rec, recording.find_config(self.rec))

    def test_override_topic_must_match_config(self):
        write_rec(self.rec)
        with self.assertRaisesRegex(recording.RecordingError, "differs"):
            recording.profile(self.rec, recording.find_config(self.rec), lidar_topic="/other")

    def test_find_config_needs_exactly_one(self):
        write_rec(self.rec)
        with open(os.path.join(self.rec, "second.yaml"), "w") as f:
            f.write("lid_topic: /a\n")
        with self.assertRaisesRegex(recording.RecordingError, "found 2"):
            recording.find_config(self.rec)

    def test_complete_matches_mcap_summary(self):
        write_rec(self.rec, mcap_counts={"/rig7/points": 3, "/rig7/inertial": 5})
        self.assertEqual(recording.check_complete(self.rec), {"/rig7/points": 3, "/rig7/inertial": 5})

    def test_complete_rejects_count_mismatch(self):
        write_rec(self.rec, mcap_counts={"/rig7/points": 2, "/rig7/inertial": 5})
        with self.assertRaisesRegex(recording.RecordingError, "metadata.yaml says 3"):
            recording.check_complete(self.rec)

    def test_complete_rejects_truncated_copy(self):
        write_rec(self.rec, mcap_counts={"/rig7/points": 3, "/rig7/inertial": 5})
        path = os.path.join(self.rec, "bag", "bag_0.mcap")
        size = os.path.getsize(path)
        with open(path, "r+b") as f:
            f.truncate(size // 2)
        with self.assertRaises(recording.RecordingError):
            recording.check_complete(self.rec)

    def test_complete_rejects_missing_file(self):
        write_rec(self.rec)
        with self.assertRaisesRegex(recording.RecordingError, "bag file missing"):
            recording.check_complete(self.rec)


if __name__ == "__main__":
    unittest.main()
