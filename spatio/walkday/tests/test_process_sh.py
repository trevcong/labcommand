"""process.sh refusal paths that need no Docker: every one must fail loudly,
leave a NOT-EVALUABLE report (or a usage error), and never a walk_result."""

import json
import os
import shutil
import subprocess
import tempfile
import unittest

from spatio.walkday.tests.test_recording import write_rec

HERE = os.path.dirname(os.path.abspath(__file__))
PROCESS = os.path.join(HERE, "..", "process.sh")
SERIES = os.path.join(HERE, "..", "series.sh")


class ProcessShTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.root = os.path.join(self.tmp, "sessions")
        self.rec = os.path.join(self.tmp, "rec")

    def tearDown(self):
        shutil.rmtree(self.tmp)

    def sh(self, *args, script=PROCESS):
        env = dict(os.environ, SPATIO_WALKDAY_ROOT=self.root)
        env.pop("SPATIO_WALK_SPEC", None)
        return subprocess.run(["bash", script, *args], capture_output=True, text=True, env=env, timeout=120)

    def report(self, run_id):
        with open(os.path.join(self.root, run_id, "walkday.json")) as f:
            return json.load(f)

    def assert_stopped(self, proc, run_id, stage):
        self.assertEqual(proc.returncode, 2, proc.stdout + proc.stderr)
        self.assertIn("NOT-EVALUABLE", proc.stdout)
        rep = self.report(run_id)
        self.assertEqual(rep["status"], "NOT_EVALUABLE")
        self.assertEqual(rep["stopped_at"], stage)
        self.assertFalse(os.path.exists(os.path.join(self.root, run_id, "walk_result.json")))

    def test_usage_errors(self):
        self.assertEqual(self.sh().returncode, 3)
        self.assertEqual(self.sh("/nonexistent", "r1").returncode, 3)
        write_rec(self.rec)
        self.assertEqual(self.sh(self.rec, "bad id").returncode, 3)
        self.assertEqual(self.sh(self.rec, "r1", "--placement-cm", "0.5").returncode, 3)   # no spec
        self.assertEqual(self.sh(self.rec, "r1", "--heading", "maybe").returncode, 3)
        self.assertEqual(self.sh(self.rec, "r1", "--start-placement-cm", "0.2").returncode, 3)
        self.assertFalse(os.path.exists(os.path.join(self.root, "r1")))

    def test_incomplete_copy_stops(self):
        write_rec(self.rec)                       # metadata.yaml but no MCAP file
        self.assert_stopped(self.sh(self.rec, "r2"), "r2", "copy-check")

    def test_count_mismatch_stops(self):
        write_rec(self.rec, mcap_counts={"/rig7/points": 2, "/rig7/inertial": 5})
        self.assert_stopped(self.sh(self.rec, "r3"), "r3", "copy-check")

    def test_config_not_matching_bag_stops(self):
        write_rec(self.rec, mcap_counts={"/rig7/points": 3, "/rig7/inertial": 5})
        cfg = os.path.join(self.tmp, "mid360_like.yaml")
        with open(cfg, "w") as f:
            f.write('common:\n  lid_topic: "/livox/lidar"\n  imu_topic: "/livox/imu"\npreprocess:\n  lidar_type: 1\n')
        self.assert_stopped(self.sh(self.rec, "r4", "--config", cfg), "r4", "config")

    def test_config_not_matching_frozen_spec_stops(self):
        write_rec(self.rec, mcap_counts={"/rig7/points": 3, "/rig7/inertial": 5})
        spec = os.path.join(self.tmp, "spec.json")
        with open(spec, "w") as f:
            json.dump({"spec_id": "x", "config_sha256": "0" * 64}, f)
        self.assert_stopped(self.sh(self.rec, "r5", "--spec", spec), "r5", "spec")

    def test_existing_session_is_never_overwritten(self):
        write_rec(self.rec)
        os.makedirs(os.path.join(self.root, "r6"))
        self.assertEqual(self.sh(self.rec, "r6").returncode, 3)

    def test_series_refuses_unscored_run(self):
        write_rec(self.rec)
        self.sh(self.rec, "r7")                   # stops at copy-check: no walk_result.json
        proc = self.sh("r7", "r-missing", script=SERIES)
        self.assertEqual(proc.returncode, 2)
        self.assertIn("NOT-EVALUABLE series", proc.stderr)


if __name__ == "__main__":
    unittest.main()
