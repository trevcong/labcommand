"""Synthetic bag/ROS stand-ins; real exporter and binary PCD output."""
import contextlib
import io
import json
from pathlib import Path
import struct
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import bag_to_map


def read_pcd(path):
    header, data = Path(path).read_bytes().split(b"DATA binary\n", 1)
    count = int(next(line.split()[1] for line in header.splitlines()
                     if line.startswith(b"POINTS ")))
    width = int(next(line.split()[1] for line in header.splitlines()
                     if line.startswith(b"WIDTH ")))
    assert count == width
    assert len(data) == 12 * count
    return list(struct.iter_unpack("<fff", data))


class BagToMapTests(unittest.TestCase):
    def export(self, directory, clouds, topic, frame, voxel, display_ok=True,
               display_out=None):
        messages = [NS(header=NS(frame_id=frame), xyz=np.array(p, dtype="float32"))
                    for p in clouds]
        pending = iter(messages)

        class Reader:
            def open(self, storage, converter):
                self.uri = storage.uri

            def get_all_topics_and_types(self):
                return [NS(name=topic, type="sensor_msgs/msg/PointCloud2")]

            def set_filter(self, filt):
                assert filt.topics == [topic]

            def has_next(self):
                self.msg = next(pending, None)
                return self.msg is not None

            def read_next(self):
                return topic, self.msg, 0

        class DisplaySaver:
            # Stand-in for the external voxel saver, deliberately deduplicating.
            def __init__(self, out, voxel):
                self.out, self.voxel, self.msgs = out, voxel, 0
                self.pts = np.empty((0, 3), dtype="float32")

            def on_cloud(self, msg):
                xyz = msg.xyz.reshape(-1, 3)
                xyz = xyz[np.isfinite(xyz).all(axis=1)]
                if len(xyz):
                    combined = np.concatenate([self.pts, xyz])
                    _, indices = np.unique(
                        np.floor(combined / self.voxel), axis=0, return_index=True)
                    self.pts = combined[indices]
                    self.msgs += 1

            def save(self):
                if not display_ok or not len(self.pts):
                    return False
                with open(self.out, "wb") as stream:
                    stream.write((
                        "VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\n"
                        "TYPE F F F\nCOUNT 1 1 1\n"
                        f"WIDTH {len(self.pts)}\nHEIGHT 1\n"
                        f"POINTS {len(self.pts)}\nDATA binary\n"
                    ).encode("ascii"))
                    stream.write(self.pts.astype("<f4").tobytes())
                return True

            def destroy_node(self):
                pass

        modules = {
            "rclpy": NS(init=lambda: None, shutdown=lambda: None),
            "rclpy.serialization": NS(deserialize_message=lambda data, _: data),
            "rosbag2_py": NS(SequentialReader=Reader, StorageOptions=NS,
                            ConverterOptions=lambda *a: a, StorageFilter=NS),
            "sensor_msgs": NS(), "sensor_msgs.msg": NS(PointCloud2=object),
            "sensor_msgs_py": NS(point_cloud2=NS(
                read_points_numpy=lambda msg, **kw: msg.xyz)),
        }
        out = Path(directory) / "map.pcd"
        summary = Path(directory) / "summary.json"
        argv = ["bag_to_map.py", str(Path(directory) / "fresh-bag"),
                str(out), "--topic", topic, "--voxel", str(voxel),
                "--summary", str(summary)]
        if display_out:
            argv += ["--display-out", str(display_out)]
        with patch.dict(sys.modules, modules), \
                patch.object(bag_to_map, "load_map_saver",
                             return_value=NS(MapSaver=DisplaySaver)), \
                patch.object(sys, "argv", argv), contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaises(SystemExit) as result:
                bag_to_map.main()
        return result.exception.code, json.loads(summary.read_text()), read_pcd(out)

    def test_registered_points_and_repeats_survive_display_deduplication(self):
        clouds = [[[1, 2, 3], [1.01, 2, 3], [float("nan"), 0, 0]],
                  [[1, 2, 3], [2, 2, 3], [float("inf"), 0, 0]]]
        with tempfile.TemporaryDirectory() as directory:
            rc, summary, points = self.export(
                directory, clouds, "/new_walk/registered", "new_map", 0.1)
            self.assertEqual(rc, 0)
            self.assertEqual(len(points), 4)
            self.assertEqual(points[0], points[2])
            self.assertAlmostEqual(points[1][0], 1.01, places=6)
            self.assertEqual(len(read_pcd(summary["display_map"])), 2)
            self.assertEqual(summary["points_written"], 4)
            self.assertEqual(summary["display_points_written"], 2)
            self.assertEqual(summary["scoring_map"], str(Path(directory) / "map.pcd"))
            self.assertIsNone(summary["scoring_voxel_m"])
            self.assertEqual(summary["frames"], ["new_map"])
            self.assertEqual(summary["topic"], "/new_walk/registered")

    def test_second_recording_voxel_changes_display_only(self):
        clouds = [[[-3, 0, 0], [-2.94, 0, 0]], [[-3, 0, 0], [8, 1, 2]]]
        raw = []
        display_counts = []
        for voxel in (0.01, 0.5):
            with tempfile.TemporaryDirectory() as directory:
                rc, summary, points = self.export(
                    directory, clouds, "/rig_b/world_points", "world_b", voxel)
                self.assertEqual(rc, 0)
                raw.append(points)
                display_counts.append(summary["display_points_written"])
        self.assertEqual(raw[0], raw[1])
        self.assertEqual(len(raw[0]), 4)
        self.assertEqual(display_counts, [3, 2])

    def test_empty_registered_map_fails(self):
        with tempfile.TemporaryDirectory() as directory:
            rc, summary, points = self.export(directory, [[]], "/empty", "map", 0.1)
            self.assertEqual(rc, 1)
            self.assertEqual(points, [])
            self.assertEqual(summary["points_written"], 0)

    def test_display_save_failure_fails_export(self):
        with tempfile.TemporaryDirectory() as directory:
            rc, _, points = self.export(
                directory, [[[1, 2, 3]]], "/cloud", "map", 0.1, display_ok=False)
            self.assertEqual(rc, 1)
            self.assertEqual(len(points), 1)

    def test_output_paths_must_differ(self):
        with patch.object(sys, "argv", [
            "bag_to_map.py", "bag", "map.pcd", "--display-out", "./map.pcd"
        ]), contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit) as result:
                bag_to_map.main()
        self.assertEqual(result.exception.code, 2)

    def test_explicit_display_path_is_used_and_reported(self):
        with tempfile.TemporaryDirectory() as directory:
            display = Path(directory) / "preview.pcd"
            rc, summary, _ = self.export(
                directory, [[[1, 2, 3]]], "/custom", "map", 0.1,
                display_out=display)
            self.assertEqual(rc, 0)
            self.assertEqual(summary["display_map"], str(display))
            self.assertEqual(read_pcd(display), [(1.0, 2.0, 3.0)])

    def test_invalid_display_voxel_rejected(self):
        for voxel in ("0", "-1", "nan", "inf"):
            with self.subTest(voxel=voxel), patch.object(sys, "argv", [
                "bag_to_map.py", "bag", "map.pcd", "--voxel", voxel
            ]), contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit) as result:
                    bag_to_map.main()
                self.assertEqual(result.exception.code, 2)


if __name__ == "__main__":
    unittest.main()
