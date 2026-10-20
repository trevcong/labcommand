from __future__ import annotations

import ast
import builtins
import hashlib
import importlib.util
import io
import json
import math
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

import numpy as np
from mcap.records import Chunk
from spatio.jt128adapter import bagcheck
from spatio.jt128adapter.tests.fixtures import make_mcap

BASE = Path(__file__).resolve().parents[1]
HASH = "1eee0c0173e0fe622f2964e378426de0be9fec29678041564171a492e73c3df8"
UPSTREAM_HASH = "6ae8bf50835379384cbd22b01b9c921e6e3e03c832dd4cda87b964e833be6311"


def load_file(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


adapter = load_file("jt128_slam_adapter", BASE / "vendor/jt128_slam_adapter.py")
TREE = ast.parse((BASE / "vendor/jt128_slam_adapter.py").read_text())
OFFSETS = dict(x=(0, 7), y=(4, 7), z=(8, 7), intensity=(12, 7), ring=(16, 4), timestamp=(18, 8))


def pack(times, rings=None):
    rings = list(rings) if rings is not None else [i % 128 for i in range(len(times))]
    return b"".join(struct.pack("<ffffHd", float(i), 2., 3., 10., rings[i], t) for i, t in enumerate(times))


def convert(times, stride=1, rings=None):
    payload, n, dt = adapter.hesai_cloud_to_velodyne(pack(times, rings), len(times), 1, 26, OFFSETS, stride)
    rows = [struct.unpack_from("<fffffH", payload, i * 24) for i in range(n)]
    return payload, rows, dt


def load_upstream():
    # Decoder is used only by the unavailable PCAP test. Never fake its behavior.
    with patch.dict(sys.modules, {"jt128_decode": types.ModuleType("jt128_decode"), "jt128_slam_adapter": adapter}):
        original = sys.path[:]
        try:
            return load_file("vendored_upstream", BASE / "vendor/test_jt128_slam_adapter.py")
        finally:
            sys.path[:] = original


UPSTREAM = load_upstream()


class UpstreamWithMissingInput(UPSTREAM.TestJt128SlamAdapter):
    @unittest.skip("jt128_one_frame.pcap was not copied to this Mac; upstream real-PCAP input unavailable")
    def test_adapter_on_real_jt128_pcap_points(self):
        super().test_adapter_on_real_jt128_pcap_points()


class Characterization(unittest.TestCase):
    def test_vendor_sha256_matches_provenance(self):
        provenance = (BASE / "vendor/PROVENANCE.md").read_text()
        for name, expected in [("jt128_slam_adapter.py", HASH), ("test_jt128_slam_adapter.py", UPSTREAM_HASH)]:
            self.assertEqual(hashlib.sha256((BASE / "vendor" / name).read_bytes()).hexdigest(), expected)
            self.assertIn(expected, provenance)
        self.assertIn("not verified to be the version that produced the 20260826T221936Z bag", provenance)

    def test_import_never_imports_rclpy(self):
        original = builtins.__import__
        def guarded(name, *args, **kwargs):
            if name.split(".")[0] in ("rclpy", "sensor_msgs"):
                self.fail("pure helper import attempted ROS: " + name)
            return original(name, *args, **kwargs)
        with patch("builtins.__import__", guarded):
            load_file("standalone_adapter", BASE / "vendor/jt128_slam_adapter.py")

    def test_missing_required_field_raises(self):
        for name in ("x", "y", "z", "intensity", "timestamp"):
            with self.subTest(name=name), self.assertRaisesRegex(ValueError, "missing PointCloud2 field " + name):
                adapter.hesai_cloud_to_velodyne(pack([1.]), 1, 1, 26, {k: v for k, v in OFFSETS.items() if k != name})

    def test_output_point_step_and_field_layout(self):
        declared = []
        for node in ast.walk(TREE):
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and node.func.id == "PointField":
                kw = {k.arg: k.value for k in node.keywords}
                declared.append(dict(name=ast.literal_eval(kw["name"]), offset=ast.literal_eval(kw["offset"]),
                                     datatype={"FLOAT32": 7, "UINT16": 4}[kw["datatype"].attr], count=ast.literal_eval(kw["count"])))
        self.assertEqual(declared, bagcheck.EXPECTED_FIELDS)
        payload, rows, _ = convert([1000., 1000.1])
        self.assertEqual(adapter.VELO_STEP, 24)
        self.assertEqual(len(payload), 48)
        self.assertEqual(rows[1][:4], (1., 2., 3., 10.))
        self.assertEqual(payload[22:24], b"\0\0")

    def test_time_is_seconds_from_first_kept_point(self):
        times = [1787700000. + x for x in (.03, .04, .08, .10)]
        _, rows, _ = convert(times)
        np.testing.assert_array_equal([r[4] for r in rows], (np.array(times) - times[0]).astype(np.float32))
        self.assertEqual(rows[0][4], 0.)
        self.assertAlmostEqual(rows[-1][4], .07, places=6)

    def test_float32_time_precision_from_float64_absolute_epoch(self):
        ideal = np.linspace(0., .1, 12801)
        times = 1787700000. + ideal
        _, rows, _ = convert(times)
        output = np.array([r[4] for r in rows])
        relative = times - times[0]
        cast_us = float(np.max(np.abs(output - relative)) * 1e6)
        epoch_us = float(np.max(np.abs(output - ideal)) * 1e6)
        self.assertEqual(cast_us, 0.)  # epoch ULP grid is exactly representable as float32 over 0.1 s
        self.assertAlmostEqual(epoch_us, .11825561523765016, places=8)
        self.assertLessEqual(epoch_us, np.spacing(1787700000.) * 1e6 / 2)
        print("\nPrecision: float32 relative cast max {:.9f} us; epoch+cast vs ideal max {:.9f} us".format(cast_us, epoch_us))

    def test_ring_values_preserved_through_conversion(self):
        rings = list(range(128)) + [65535]
        _, rows, _ = convert([1000. + i * .0001 for i in range(len(rings))], rings=rings)
        self.assertEqual([r[5] for r in rows], rings)

    def test_stride_4_keeps_which_indices(self):
        _, rows, _ = convert([1000. + i * .001 for i in range(19)], stride=4)
        self.assertEqual([int(r[0]) for r in rows], [0, 4, 8, 12, 16])

    def test_stride_4_ring_coverage_with_interleaved_firing(self):
        # 10 azimuth blocks, each firing rings 0..127 in order.
        _, rows, _ = convert([1787700000. + i * .1 / 1280 for i in range(1280)], stride=4)
        self.assertEqual(sorted({r[5] for r in rows}), list(range(0, 128, 4)))
        self.assertEqual(len(rows), 320)

    def test_scan_dt_is_last_minus_first_kept(self):
        times = [1787700000. + i * .01 for i in range(10)]
        _, rows, dt = convert(times, stride=4)
        self.assertEqual(dt, times[8] - times[0])
        self.assertNotEqual(dt, times[9] - times[0])
        self.assertEqual(rows[-1][4], dt)

    def test_nonmonotonic_input_timestamps_behaviour(self):
        self._nonmonotonic()

    def test_observed_negative_times_clamped_unsorted_positive_times_and_negative_dt(self):
        self._nonmonotonic()

    def _nonmonotonic(self):
        _, rows, dt = convert([1000., 1000.2, 1000.1, 999.9])
        np.testing.assert_array_equal([r[4] for r in rows], np.array([0., .2, .1, 0.], dtype=np.float32))
        self.assertAlmostEqual(dt, -.1)

    def test_nan_or_zero_points_behaviour(self):
        self._nan_zero()

    def test_observed_nan_and_zero_xyz_are_kept_despite_dense_declaration(self):
        self._nan_zero()
        self.assertIn("out.is_dense = True", ast.unparse(TREE))  # static main path, not a ROS execution

    def _nan_zero(self):
        raw = bytearray(pack([1000., 1000.1]))
        struct.pack_into("<fff", raw, 0, 0., 0., 0.)
        struct.pack_into("<f", raw, 26, math.nan)
        payload, n, _ = adapter.hesai_cloud_to_velodyne(raw, 2, 1, 26, OFFSETS)
        self.assertEqual(n, 2)
        self.assertEqual(struct.unpack_from("<fff", payload, 0), (0., 0., 0.))
        self.assertTrue(math.isnan(struct.unpack_from("<f", payload, 24)[0]))

    def test_gyro_deg_s_to_rad_s_exact(self):
        values = (-180., .15245, 90.)
        self.assertEqual(adapter.gyro_deg_s_to_rad_s(*values), tuple(v * (math.pi / 180) for v in values))

    def test_accel_passthrough_units(self):
        # Static evidence only: do not extract/execute a counterfeit ROS callback.
        fn = next(n for n in ast.walk(TREE) if isinstance(n, ast.FunctionDef) and n.name == "_on_imu")
        assignments = {ast.unparse(n.targets[0]): ast.unparse(n.value) for n in fn.body if isinstance(n, ast.Assign)}
        self.assertEqual(assignments["out.linear_acceleration"], "msg.linear_acceleration")
        self.assertEqual(assignments["out.linear_acceleration_covariance"], "msg.linear_acceleration_covariance")
        self.assertEqual(assignments["out.header"], "msg.header")

    def test_upstream_tests_run_as_vendored(self):
        # Actual three outcomes are top-level unittest cases from this loaded class.
        self.assertEqual(Path(UPSTREAM.__file__), BASE / "vendor/test_jt128_slam_adapter.py")
        names = unittest.defaultTestLoader.getTestCaseNames(UpstreamWithMissingInput)
        self.assertEqual(names, ["test_adapter_on_real_jt128_pcap_points", "test_cloud_time_is_seconds_from_scan_start", "test_gyro_manual_example_is_deg_s_not_rad_s"])
        self.assertIs(UpstreamWithMissingInput.test_cloud_time_is_seconds_from_scan_start,
                      UPSTREAM.TestJt128SlamAdapter.test_cloud_time_is_seconds_from_scan_start)
        self.assertIs(UpstreamWithMissingInput.test_gyro_manual_example_is_deg_s_not_rad_s,
                      UPSTREAM.TestJt128SlamAdapter.test_gyro_manual_example_is_deg_s_not_rad_s)
        self.assertFalse(UPSTREAM.PCAP.exists())

    def test_numpy_and_no_numpy_paths_match(self):
        data = pack([1787700000. + i * .001 for i in range(128)])
        expected = adapter.hesai_cloud_to_velodyne(data, 128, 1, 26, OFFSETS, stride=4)
        original = builtins.__import__
        def no_numpy(name, *args, **kwargs):
            if name == "numpy":
                raise ImportError("exercise pure-Python fallback")
            return original(name, *args, **kwargs)
        with patch("builtins.__import__", no_numpy):
            actual = adapter.hesai_cloud_to_velodyne(data, 128, 1, 26, OFFSETS, stride=4)
        self.assertEqual(actual, expected)

    def test_observed_missing_ring_defaults_to_zero(self):
        payload, n, _ = adapter.hesai_cloud_to_velodyne(pack([1., 2.]), 2, 1, 26, {k: v for k, v in OFFSETS.items() if k != "ring"})
        self.assertEqual([struct.unpack_from("<H", payload, i * 24 + 20)[0] for i in range(n)], [0, 0])

    def test_observed_each_input_message_is_a_scan(self):
        for times in ([1000., 1000.1], [1000.2, 1000.3]):
            _, rows, _ = convert(times)
            self.assertEqual(rows[0][4], 0.)
        fn = next(n for n in ast.walk(TREE) if isinstance(n, ast.FunctionDef) and n.name == "_on_cloud")
        self.assertIn("out.header = msg.header", ast.unparse(fn))
        self.assertIn("self.pub_cloud.publish(out)", ast.unparse(fn))


class Bagcheck(unittest.TestCase):
    def setUp(self):
        (BASE / "_out").mkdir(exist_ok=True)
        self.temp = tempfile.TemporaryDirectory(dir=BASE / "_out")
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)

    def fixture(self, **kwargs):
        return make_mcap(self.root / "tiny.mcap", bagcheck.EXPECTED_FIELDS, **kwargs)

    def test_bagcheck_on_synthetic_mcap(self):
        report = bagcheck.inspect_bag(self.fixture())
        self.assertEqual(report["vendor_sha256"], HASH)
        self.assertEqual(report["imu"]["matched"], 4)
        self.assertEqual(report["imu"]["exact"], 4)
        for k, expected in [("gyro", math.pi / 180), ("accel", 1.)]:
            for axis in "xyz":
                self.assertAlmostEqual(report["imu"][k]["axes"][axis]["ratio_slam_over_raw"]["median"], expected, places=15)
        self.assertEqual(report["points"]["ring_histogram"], {"0": 1, "4": 1, "127": 1})
        self.assertEqual(report["points"]["header_minus_receipt_s"]["min"], -1000.)
        verdicts = {v["property"]: v["verdict"] for v in report["verdicts"]}
        self.assertEqual(verdicts["gyro conversion"], "consistent with vendored source")
        self.assertEqual(verdicts["accel handling"], "consistent with vendored source")
        self.assertEqual(verdicts["field layout"], "consistent with vendored source")
        self.assertEqual(verdicts["time origin"], "not determinable from this bag")
        self.assertEqual(verdicts["stride"], "not determinable from this bag")

    def test_bagcheck_cli_outputs(self):
        p = self.fixture()
        result = subprocess.run([sys.executable, "-m", "spatio.jt128adapter.bagcheck", str(p), "--out", str(self.root / "report")], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        report = json.loads((self.root / "report/adaptercheck.json").read_text())
        self.assertEqual(report["imu"]["raw_count"], 4)
        md = (self.root / "report/adaptercheck.md").read_text()
        self.assertIn("not verified to be the version", md)
        self.assertIn("Receipt timestamps", md)

    def test_multichunk_reader_consumes_cloud_before_reading_later_chunks(self):
        path = self.fixture(chunk_size=1, cloud_count=6)
        with path.open("rb") as stream:
            chunk_count = len(bagcheck.make_reader(stream).get_summary().chunk_indexes)
        self.assertGreaterEqual(chunk_count, 6)
        reads_at_cloud = []
        original = bagcheck.cloud_arrays
        with patch.object(Chunk, "read", wraps=Chunk.read) as read_chunk:
            def observe(msg):
                reads_at_cloud.append(read_chunk.call_count)
                return original(msg)
            with patch.object(bagcheck, "cloud_arrays", side_effect=observe):
                report = bagcheck.inspect_bag(path)
            self.assertEqual(read_chunk.call_count, chunk_count)
        self.assertEqual(reads_at_cloud[0], 1)
        self.assertEqual(len(reads_at_cloud), 6)
        self.assertEqual(report["points"]["messages"], 6)
        self.assertEqual(report["imu"]["matched"], 4)

    def test_multichunk_window_filters_receipt_time_without_assuming_order(self):
        # Later physical chunks contain clouds earlier than the last IMU stamp.
        report = bagcheck.inspect_bag(self.fixture(chunk_size=1, cloud_count=6), .01)
        self.assertEqual(report["points"]["messages"], 2)
        self.assertEqual(report["imu"]["matched"], 2)
        self.assertEqual(report["inventory_whole_bag"]["/slam/points"], 6)

    def test_window_cannot_establish_whole_bag_consistency(self):
        path = self.fixture(chunk_size=1, cloud_count=6)
        for seconds, clouds, pairs in ((.006, 2, 2), (1., 6, 4)):
            with self.subTest(seconds=seconds):
                report = bagcheck.inspect_bag(path, seconds)
                self.assertFalse(report["scope"]["full_pass"])
                self.assertEqual(report["points"]["messages"], clouds)
                self.assertEqual(report["imu"]["matched"], pairs)
                self.assertEqual(report["inventory_whole_bag"],
                                 {"/slam/points": 6, "/lidar_imu": 4, "/slam/imu": 4})
                for row in report["verdicts"]:
                    self.assertEqual(row["verdict"], "not determinable from this bag")

    def test_full_pass_requires_decoded_counts_to_match_inventory(self):
        path = self.fixture(chunk_size=1, cloud_count=6)
        original = bagcheck.stream_messages
        # Simulate an incomplete decode despite an unrestricted requested pass.
        # Dropping both IMU samples preserves complete pairing of the subset.
        for omitted in (("/slam/points",), ("/lidar_imu", "/slam/imu")):
            def incomplete(path, start, end):
                remaining = set(omitted)
                for record in original(path, start, end):
                    if record.channel.topic in remaining:
                        remaining.remove(record.channel.topic)
                    else:
                        yield record
            with self.subTest(omitted=omitted), patch.object(
                    bagcheck, "stream_messages", side_effect=incomplete):
                report = bagcheck.inspect_bag(path)
                self.assertTrue(report["scope"]["full_pass"])
                self.assertNotEqual(report["decoded_counts"], report["inventory_whole_bag"])
                for row in report["verdicts"]:
                    self.assertEqual(row["verdict"], "not determinable from this bag")

    def test_window_preserves_observed_contradictions(self):
        fields = [dict(field) for field in bagcheck.EXPECTED_FIELDS]
        fields[3]["offset"] = 13  # A valid buffer with a non-vendor intensity layout.
        path = make_mcap(self.root / "contradiction.mcap", fields,
                         gyro_scale=1., accel_scale=9.81, first_time=.02,
                         chunk_size=1, cloud_count=6)
        report = bagcheck.inspect_bag(path, .006)
        self.assertFalse(report["scope"]["full_pass"])
        self.assertEqual(report["imu"]["matched"], 2)
        verdicts = {v["property"]: v["verdict"] for v in report["verdicts"]}
        for prop in ("field layout", "gyro conversion", "accel handling", "time origin"):
            self.assertEqual(verdicts[prop], "inconsistent (evidence)")

    def imu_verdicts(self, raw, adapted):
        points = bagcheck.inspect_bag(self.fixture())["points"]
        return {v["property"]: v["verdict"]
                for v in bagcheck.verdicts(points, bagcheck.imu_report(raw, adapted),
                                          whole_bag_covered=True)}

    def test_zero_imu_pairs_do_not_establish_conversion(self):
        zero = [(0, [0., 0., 0.], [0., 0., 0.], "hesai_lidar")]
        result = self.imu_verdicts(zero, zero)
        for prop in ("gyro conversion", "accel handling"):
            self.assertEqual(result[prop], "not determinable from this bag")

    def test_unexcited_axis_does_not_establish_conversion(self):
        raw = [(0, [1., 2., 0.], [1., 2., 0.], "hesai_lidar")]
        adapted = [(0, [math.pi / 180, math.pi / 90, 0.], [1., 2., 0.], "hesai_lidar")]
        result = self.imu_verdicts(raw, adapted)
        for prop in ("gyro conversion", "accel handling"):
            self.assertEqual(result[prop], "not determinable from this bag")

    def test_partial_pairing_does_not_establish_bag_conversion(self):
        raw = [(k * 10_000_000, [1., 2., 3.], [.1, .2, .3], "hesai_lidar") for k in range(4)]
        adapted = [(0, [v * math.pi / 180 for v in raw[0][1]], raw[0][2], "hesai_lidar")]
        # Wrongly scaled samples sit outside the 1 ms matching tolerance.
        adapted += [(r[0] + 2_000_000, r[1], [v * 9.81 for v in r[2]], r[3]) for r in raw[1:]]
        for inputs in ((raw, adapted), (raw, adapted[:1]), (raw[:1], adapted)):
            with self.subTest(counts=tuple(map(len, inputs))):
                result = self.imu_verdicts(*inputs)
                for prop in ("gyro conversion", "accel handling"):
                    self.assertEqual(result[prop], "not determinable from this bag")

    def test_matched_contradiction_wins_over_incomplete_coverage(self):
        raw = [(k * 10_000_000, [1., 2., 3.], [.1, .2, .3], "hesai_lidar") for k in range(4)]
        adapted = [(0, [1., 2., 3.], [1., 2., 3.], "hesai_lidar")]
        result = self.imu_verdicts(raw, adapted)
        for prop in ("gyro conversion", "accel handling"):
            self.assertEqual(result[prop], "inconsistent (evidence)")

    def test_inconsistent_values_are_reported(self):
        report = bagcheck.inspect_bag(self.fixture(gyro_scale=1., accel_scale=9.81, first_time=.02))
        verdicts = {v["property"]: v["verdict"] for v in report["verdicts"]}
        for prop in ("gyro conversion", "accel handling", "time origin"):
            self.assertEqual(verdicts[prop], "inconsistent (evidence)")

    def test_nearest_pairing_and_exact_match_priority(self):
        report = bagcheck.inspect_bag(self.fixture(stamp_offset=500_000))
        self.assertEqual(report["imu"]["nearest_within_1ms"], 4)
        self.assertEqual(report["imu"]["stamp_delta_slam_minus_raw_ns"]["max"], 500_000)
        self.assertEqual(bagcheck.pair_imu([(0,), (900_000,)], [(500_000,), (900_000,)]), [(1, 1, "exact"), (0, 0, "nearest")])
        self.assertEqual(bagcheck.pair_imu([(0,)], [(0,), (0,)]), [(0, 0, "exact")])
        self.assertEqual(bagcheck.pair_imu([(0,)], [(1_000_001,)]), [])

    def test_max_seconds_and_empty_inputs(self):
        report = bagcheck.inspect_bag(self.fixture(), .006)
        self.assertEqual(report["imu"]["matched"], 2)
        self.assertFalse(report["scope"]["full_pass"])
        with self.assertRaisesRegex(ValueError, "finite and positive"):
            bagcheck.inspect_bag(self.root, -1)
        empty = bagcheck.imu_report([], [])
        self.assertEqual(empty["matched"], 0)
        self.assertIsNone(empty["raw_match_rate"])

    def test_axis_permutation_and_sign_detection(self):
        raw = [[1., 2., 3.], [4., 7., 8.]]
        adapted = [[-2., 3., 1.], [-7., 8., 4.]]
        result = bagcheck.axis_analysis(raw, adapted, 1.)
        self.assertEqual(result["axis_mapping"]["raw_axis_for_slam_xyz"], ["y", "z", "x"])
        self.assertEqual(result["axis_mapping"]["signs"], [-1, 1, 1])
        self.assertTrue(result["axis_mapping"]["unique_best"])


if __name__ == "__main__":
    unittest.main()
