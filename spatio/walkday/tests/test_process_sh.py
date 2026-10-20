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

    def test_missing_recording_start_time_stops_before_replay(self):
        # Complete copy and a matching config, but no bag-level starting_time:
        # the series could not order this run, so stop before the long stages.
        write_rec(self.rec, mcap_counts={"/rig7/points": 3, "/rig7/inertial": 5}, starting_ns=None)
        proc = self.sh(self.rec, "r8")
        self.assert_stopped(proc, "r8", "recording-time")
        self.assertNotIn("[2/9]", proc.stdout)

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

    def driver(self, text, name="rig7_driver.yaml"):
        path = os.path.join(self.tmp, name)
        with open(path, "w") as f:
            f.write(text)
        return path

    def bad_spec(self):
        spec = os.path.join(self.tmp, "spec.json")
        with open(spec, "w") as f:
            json.dump({"spec_id": "x", "config_sha256": "0" * 64}, f)
        return spec

    def test_driver_config_is_hashed_checked_and_kept(self):
        # Review P2 (process.sh:59): a two-config run (JT128: FAST-LIO config +
        # the driver's jt128_post.yaml). Synthetic rig7 topics, not JT128's. The
        # bad spec stops the run right after stage 1, as B5 stops the JT128 sample.
        write_rec(self.rec, mcap_counts={"/rig7/points": 3, "/rig7/inertial": 5})
        drv = self.driver("lidar:\n  - driver:\n      source_type: 2\n    ros:\n"
                          "      ros_send_point_cloud_topic: /rig7/raw_points\n"
                          '      ros_send_imu_topic: "/rig7/inertial"\n')
        self.assert_stopped(self.sh(self.rec, "d1", "--driver-config", drv, "--spec", self.bad_spec()), "d1", "spec")
        logs = os.path.join(self.root, "d1", "logs")
        with open(os.path.join(logs, "provenance.json")) as f:
            prov = json.load(f)
        import hashlib
        with open(drv, "rb") as f:
            want = hashlib.sha256(f.read()).hexdigest()
        with open(os.path.join(self.rec, "rig7_fastlio.yaml"), "rb") as f:
            want_cfg = hashlib.sha256(f.read()).hexdigest()
        self.assertEqual(prov["driver_config"]["sha256"], want)
        self.assertEqual(prov["driver_config"]["topics_in_bag"], ["/rig7/inertial"])
        self.assertEqual(prov["fastlio_config"]["sha256"], want_cfg)
        self.assertTrue(os.path.isfile(os.path.join(logs, "driver_config.rig7_driver.yaml")))
        rep = self.report("d1")
        self.assertEqual(rep["provenance"]["driver_config"]["sha256"], want)
        self.assertEqual(rep["provenance"]["fastlio_config"]["sha256"], want_cfg)
        md = open(os.path.join(self.root, "d1", "walkday.md")).read()
        self.assertIn(want, md)
        self.assertIn(want_cfg, md)

    def test_driver_config_that_cannot_belong_to_the_recording_stops(self):
        write_rec(self.rec, mcap_counts={"/rig7/points": 3, "/rig7/inertial": 5})
        cases = {
            "d2": os.path.join(self.rec, "rig7_fastlio.yaml"),                             # the FAST-LIO config itself
            "d3": self.driver('common:\n  lid_topic: "/rig7/points"\n', "other_fastlio.yaml"),  # a FAST-LIO config
            "d4": self.driver("ros:\n  ros_send_imu_topic: /elsewhere/imu\n", "d4.yaml"),       # topics not in bag
            "d5": self.driver("lidar:\n  - driver:\n      source_type: 2\n", "d5.yaml"),          # declares no topic
            "d6": self.driver("", "d6.yaml"),                                                  # empty
        }
        for run_id, drv in cases.items():
            proc = self.sh(self.rec, run_id, "--driver-config", drv)
            self.assert_stopped(proc, run_id, "config")
            self.assertNotIn("[2/9]", proc.stdout)
        self.assertEqual(self.sh(self.rec, "d7", "--driver-config", "/nonexistent.yaml").returncode, 3)
        self.assertFalse(os.path.exists(os.path.join(self.root, "d7")))

    def test_settings_identity_recorded_and_bad_settings_refused(self):
        write_rec(self.rec, mcap_counts={"/rig7/points": 3, "/rig7/inertial": 5})
        conf = os.path.join(HERE, "..", "walkday.conf")
        with open(conf) as f:
            text = f.read()
        self.assertIn("MAP_VOXEL_M=0.03", text)          # the edits below must really apply
        self.assertIn("MAX_HOLD_SPEED_M_S=0.10", text)
        # A setting left empty: a usage/setup error before any session exists.
        empty = os.path.join(self.tmp, "empty.conf")
        with open(empty, "w") as f:
            f.write(text.replace("MAP_VOXEL_M=0.03", "MAP_VOXEL_M="))
        self.assertEqual(self.sh(self.rec, "s1", "--settings", empty).returncode, 3)
        self.assertFalse(os.path.exists(os.path.join(self.root, "s1")))
        # A comment edit keeps the identity; a value edit changes it.
        shas = []
        for run_id, body in (("s2", text), ("s3", "# reworded comment\n" + text),
                             ("s4", text.replace("MAX_HOLD_SPEED_M_S=0.10", "MAX_HOLD_SPEED_M_S=0.12"))):
            path = os.path.join(self.tmp, run_id + ".conf")
            with open(path, "w") as f:
                f.write(body)
            self.assert_stopped(self.sh(self.rec, run_id, "--settings", path, "--spec", self.bad_spec()), run_id, "spec")
            shas.append(self.report(run_id)["provenance"]["settings_sha256"])
        self.assertEqual(shas[0], shas[1])
        self.assertNotEqual(shas[0], shas[2])

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

    def test_series_usage_error_is_not_fail(self):
        # spatio.series uses 3 for FAIL, so series.sh usage errors are 2.
        self.assertEqual(self.sh(script=SERIES).returncode, 2)
        self.assertEqual(self.sh("r1", "--bogus", script=SERIES).returncode, 2)
        self.assertEqual(self.sh("r1", "--series-id", "bad id", script=SERIES).returncode, 2)


if __name__ == "__main__":
    unittest.main()
