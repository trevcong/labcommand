"""Builder and validator tests. Fixtures are temporary and synthetic."""

from __future__ import annotations

import hashlib
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import spatio.manifest.hashing as hashing
from spatio.manifest import (
    REPLAY_LABEL,
    ManifestBuildError,
    build_manifest,
    placement_within_tolerance,
    sha256_file,
    validate_physical_reference,
    validate_session,
)
from spatio.manifest.checks import parse_iso8601_with_offset
from spatio.manifest.findings import REQUIRED_KEYS


REPO_ROOT = Path(__file__).resolve().parents[3]
EXACT_LABEL = "replay demonstrated; physical accuracy not evaluated"


def _errors(findings):
    return [item for item in findings if item.severity == "ERROR"]


def _warnings(findings):
    return [item for item in findings if item.severity == "WARNING"]


def _rules(findings):
    return [item.rule for item in findings]


class ManifestTests(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.root = Path(self._tmp.name)

    def tearDown(self):
        self._tmp.cleanup()

    def _write(self, path, text):
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text, encoding="utf-8")
        return path

    def _make_source(self, name="recording"):
        source = self.root / name
        self._write(source / "bag" / "metadata.yaml", "rosbag2: fixture\n")
        (source / "bag" / "bag_0.mcap").write_bytes(b"mcap-bytes")
        self._write(source / "trajectory.tum", "0 0 0 0 0 0 0 1\n")
        self._write(
            source / "map.pcd",
            "# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\n"
            "TYPE F F F\nCOUNT 1 1 1\nWIDTH 1\nHEIGHT 1\n"
            "VIEWPOINT 0 0 0 1 0 0 0\nPOINTS 1\nDATA ascii\n0 0 0\n",
        )
        return source

    def _make_config(self, name="sensor.yaml", text="lidar_type: 1\n"):
        return self._write(self.root / name, text)

    def _build(self, run_id="run-a", source=None, config=None, **kwargs):
        if source is None:
            source = self._make_source()
        if config is None:
            config = self._make_config()
        session = self.root / "sessions" / run_id
        notes = kwargs.pop("notes", "unit test")
        manifest = build_manifest(
            session,
            run_id,
            str(source),
            kwargs.pop("sensor", "mid360"),
            kwargs.pop("fastlio_revision", "abc1234"),
            str(config),
            kwargs.pop("image", "none"),
            notes,
            **kwargs,
        )
        return session, manifest

    def _load(self, session):
        return json.loads((session / "manifest.json").read_text(encoding="utf-8"))

    def _save(self, session, data):
        (session / "manifest.json").write_text(
            json.dumps(data, indent=2, allow_nan=False) + "\n",
            encoding="utf-8",
        )

    def _reference(self, run_id, **overrides):
        data = {
            "schema": "spatio.physical_reference/1",
            "run_id": run_id,
            "spec_id": "m1-test",
            "spec_sha256": "ab" * 32,
            "operator": "tester",
            "measured_at": "2026-10-19T21:30:00-05:00",
            "start_placement_residual_m": 0.004,
            "end_placement_residual_m": 0.006,
            "heading_check": "ok",
            "placement_tolerance_m": 0.01,
            "method": "fixture tape from the cart mark",
            "notes": "synthetic",
        }
        data.update(overrides)
        return data

    def _write_reference(self, run_id, **overrides):
        path = self.root / ("reference-%s.json" % run_id)
        path.write_text(
            json.dumps(self._reference(run_id, **overrides), indent=2) + "\n",
            encoding="utf-8",
        )
        return path

    def test_build_minimal_valid_manifest(self):
        config = self._make_config()
        session, manifest = self._build(config=config)
        self.assertEqual(manifest["run_id"], session.name)
        self.assertEqual(manifest["sensor"], "mid360")
        self.assertEqual(manifest["image"], "none")
        self.assertIs(manifest["physical_accuracy_evaluated"], False)
        self.assertEqual(manifest["accuracy_label"], EXACT_LABEL)
        self.assertEqual(manifest["config_sha256"], sha256_file(config))
        self.assertEqual(len(manifest["config_sha256"]), 64)
        self.assertIsNotNone(parse_iso8601_with_offset(manifest["created_at"]))
        self.assertIn("sensor.yaml", manifest["files"])
        self.assertIn("trajectory.tum", manifest["files"])
        self.assertIn("map.pcd", manifest["files"])
        self.assertIsNone(manifest["source_fingerprint"]["bag_files"][0]["sha256"])
        self.assertEqual(manifest["source_fingerprint"]["bag_files"][0]["bytes"], len(b"mcap-bytes"))
        findings = validate_session(session)
        self.assertEqual(_errors(findings), [])
        self.assertEqual(_warnings(findings), [])

    def test_build_refuses_overwrite_without_force(self):
        session, _manifest = self._build()
        before = (session / "manifest.json").read_bytes()
        with self.assertRaises(ManifestBuildError) as caught:
            self._build()
        self.assertIn("force", str(caught.exception))
        self.assertEqual((session / "manifest.json").read_bytes(), before)
        again = build_manifest(
            session,
            session.name,
            self._load(session)["source_recording"],
            "mid360",
            "abc1234",
            self._load(session)["config_file"],
            "none",
            "rewritten note",
            force=True,
        )
        self.assertEqual(again["notes"], "rewritten note")
        self.assertEqual(_errors(validate_session(session)), [])

    def test_build_historical_sets_false_and_exact_label(self):
        self.assertEqual(REPLAY_LABEL, EXACT_LABEL)
        _session, manifest = self._build(notes="copied recording, no floor marks")
        self.assertIs(manifest["physical_accuracy_evaluated"], False)
        self.assertEqual(manifest["accuracy_label"], EXACT_LABEL)
        self.assertNotIn(EXACT_LABEL, manifest["notes"])
        findings = validate_session(_session)
        self.assertFalse(any(item.rule == "M3" for item in findings))
        self.assertEqual(_errors(findings), [])

    def test_build_with_physical_reference_sets_true_and_copies_file(self):
        run_id = "run-measured"
        source_ref = self._write_reference(run_id)
        original = source_ref.read_bytes()
        session, manifest = self._build(
            run_id=run_id,
            physical_reference=str(source_ref),
            notes="operator measured the tape marks",
        )
        self.assertIs(manifest["physical_accuracy_evaluated"], True)
        self.assertNotIn("accuracy_label", manifest)
        copied = session / "physical_reference.json"
        self.assertEqual(copied.read_bytes(), original)
        self.assertTrue(placement_within_tolerance(self._load_json(copied)))
        findings = validate_session(session)
        self.assertEqual(_errors(findings), [])
        self.assertEqual(_warnings(findings), [])

    def _load_json(self, path):
        return json.loads(Path(path).read_text(encoding="utf-8"))

    def test_validate_each_missing_required_key_is_error(self):
        session, _manifest = self._build()
        for key in REQUIRED_KEYS:
            with self.subTest(key=key):
                data = self._load(session)
                del data[key]
                self._save(session, data)
                findings = validate_session(session)
                matching = [
                    item
                    for item in _errors(findings)
                    if item.rule == "M1" and key in item.message
                ]
                self.assertTrue(matching, findings)

    def test_validate_wrong_types_are_errors(self):
        session, _manifest = self._build()
        original = self._load(session)
        cases = {
            "run_id": 1,
            "source_recording": ["recording"],
            "sensor": None,
            "fastlio_revision": 7,
            "config_file": {"path": "x"},
            "config_sha256": 1,
            "image": False,
            "created_at": 0,
            "notes": ["note"],
            "physical_accuracy_evaluated": "false",
        }
        for key, bad in cases.items():
            with self.subTest(key=key):
                data = json.loads(json.dumps(original))
                data[key] = bad
                self._save(session, data)
                findings = validate_session(session)
                matching = [
                    item
                    for item in _errors(findings)
                    if item.rule == "M1" and key in item.message
                ]
                self.assertTrue(matching, findings)
        with self.subTest(key="config_sha256_uppercase"):
            data = json.loads(json.dumps(original))
            data["config_sha256"] = original["config_sha256"].upper()
            self._save(session, data)
            findings = validate_session(session)
            self.assertTrue(
                any(item.rule == "M1" and item.severity == "ERROR" for item in findings)
            )
        with self.subTest(key="physical_accuracy_evaluated_zero"):
            data = json.loads(json.dumps(original))
            data["physical_accuracy_evaluated"] = 0
            self._save(session, data)
            findings = validate_session(session)
            self.assertTrue(
                any(
                    item.rule == "M1" and "physical_accuracy_evaluated" in item.message
                    for item in _errors(findings)
                )
            )

    def test_validate_unknown_extra_key_is_warning_only(self):
        session, _manifest = self._build()
        data = self._load(session)
        data["replay_tool"] = "task-a"
        self._save(session, data)
        findings = validate_session(session)
        self.assertEqual(_errors(findings), [])
        self.assertEqual(len(_warnings(findings)), 1)
        self.assertEqual(_warnings(findings)[0].rule, "M2")
        self.assertIn("replay_tool", _warnings(findings)[0].message)

    def test_validate_false_without_label_is_error(self):
        session, _manifest = self._build()
        data = self._load(session)
        del data["accuracy_label"]
        data["notes"] = "historical replay"
        data["physical_accuracy_evaluated"] = False
        self._save(session, data)
        findings = validate_session(session)
        self.assertTrue(any(item.rule == "M3" for item in _errors(findings)))

    def test_validate_label_in_notes_accepted(self):
        session, _manifest = self._build()
        data = self._load(session)
        del data["accuracy_label"]
        data["notes"] = "prefix %s suffix" % EXACT_LABEL
        data["physical_accuracy_evaluated"] = False
        self._save(session, data)
        findings = validate_session(session)
        self.assertFalse(any(item.rule == "M3" for item in findings))
        self.assertEqual(_errors(findings), [])

    def test_validate_label_must_be_exact_string(self):
        self.assertEqual(REPLAY_LABEL, EXACT_LABEL)
        near_misses = (
            "Replay demonstrated; physical accuracy not evaluated",
            "replay demonstrated, physical accuracy not evaluated",
            "replay demonstrated;  physical accuracy not evaluated",
            "replay demonstrated; physical accuracy NOT evaluated",
            "replay  demonstrated; physical accuracy not evaluated",
            "replay demonstrated; physical accuracy was not evaluated",
            EXACT_LABEL.upper(),
            EXACT_LABEL[:-1],
        )
        session, _manifest = self._build()
        for text in near_misses:
            with self.subTest(text=text):
                self.assertNotIn(EXACT_LABEL, text)
                data = self._load(session)
                data["accuracy_label"] = text
                data["notes"] = text
                data["physical_accuracy_evaluated"] = False
                self._save(session, data)
                findings = validate_session(session)
                self.assertTrue(
                    any(item.rule == "M3" for item in _errors(findings)),
                    findings,
                )
        # The sentence has to appear. A longer field that still contains it
        # is valid; a one-character edit that breaks the sentence is not.
        for text in ("note: %s" % EXACT_LABEL, EXACT_LABEL + "."):
            with self.subTest(accepted=text):
                data = self._load(session)
                data["accuracy_label"] = text
                data["notes"] = "unit test"
                data["physical_accuracy_evaluated"] = False
                self._save(session, data)
                findings = validate_session(session)
                self.assertFalse(any(item.rule == "M3" for item in findings), findings)

    def test_validate_true_without_physical_reference_is_error(self):
        session, _manifest = self._build()
        data = self._load(session)
        data["physical_accuracy_evaluated"] = True
        self._save(session, data)
        findings = validate_session(session)
        matching = [item for item in _errors(findings) if item.rule == "M4"]
        self.assertTrue(matching, findings)
        self.assertIn("missing", matching[0].message)

    def test_validate_physical_reference_run_id_mismatch_is_error(self):
        run_id = "run-measured"
        session, _manifest = self._build(run_id=run_id)
        bad = self._reference(run_id)
        bad["run_id"] = "other-run"
        (session / "physical_reference.json").write_text(
            json.dumps(bad, indent=2) + "\n", encoding="utf-8"
        )
        data = self._load(session)
        data["physical_accuracy_evaluated"] = True
        del data["accuracy_label"]
        self._save(session, data)
        findings = validate_session(session)
        matching = [
            item
            for item in _errors(findings)
            if item.rule == "M4" and "run_id" in item.message
        ]
        self.assertTrue(matching, findings)

    def test_validate_config_hash_mismatch_is_error(self):
        session, manifest = self._build()
        data = self._load(session)
        real_hash = manifest["config_sha256"]
        data["config_sha256"] = "f" * 64 if real_hash != "f" * 64 else "0" * 64
        self._save(session, data)
        findings = validate_session(session)
        error_rules = {item.rule for item in _errors(findings)}
        self.assertIn("M5", error_rules)
        self.assertNotIn("M6", error_rules)

    def test_validate_config_unreadable_is_warning(self):
        session, manifest = self._build()
        data = self._load(session)
        config_name = Path(manifest["config_file"]).name
        del data["files"][config_name]
        data["config_file"] = str(self.root / "no-such-config.yaml")
        self._save(session, data)
        findings = validate_session(session)
        warnings = [item for item in _warnings(findings) if item.rule == "M5"]
        self.assertEqual(len(warnings), 1)
        self.assertIn("config not verifiable here", warnings[0].message)
        self.assertEqual(_errors(findings), [])

    def test_validate_files_hash_mismatch_is_error(self):
        session, _manifest = self._build()
        data = self._load(session)
        entry = data["files"]["trajectory.tum"]
        entry["sha256"] = "a" * 64 if not entry["sha256"].startswith("a") else "b" * 64
        self._save(session, data)
        findings = validate_session(session)
        hash_errors = [
            item
            for item in _errors(findings)
            if item.rule == "M6" and "trajectory.tum" in item.message
        ]
        self.assertTrue(hash_errors, findings)
        self.assertFalse(any(item.rule == "M5" for item in _errors(findings)))

        data = self._load(session)
        # Reload the honest manifest, then break only the size.
        fresh = build_manifest(
            session,
            session.name,
            data["source_recording"],
            data["sensor"],
            data["fastlio_revision"],
            data["config_file"],
            data["image"],
            data["notes"],
            force=True,
        )
        broken = self._load(session)
        self.assertEqual(broken["files"]["map.pcd"]["sha256"], fresh["files"]["map.pcd"]["sha256"])
        broken["files"]["map.pcd"]["bytes"] = broken["files"]["map.pcd"]["bytes"] + 1
        self._save(session, broken)
        findings = validate_session(session)
        size_errors = [
            item
            for item in _errors(findings)
            if item.rule == "M6" and "map.pcd" in item.message
        ]
        self.assertTrue(size_errors, findings)

    def test_source_fingerprint_null_hash_ok(self):
        session, manifest = self._build(hash_bag=False)
        bag = manifest["source_fingerprint"]["bag_files"][0]
        self.assertIsNone(bag["sha256"])
        self.assertEqual(bag["bytes"], len(b"mcap-bytes"))
        findings = validate_session(session)
        self.assertEqual(_errors(findings), [])

        with self.subTest(case="file_removed"):
            data = self._load(session)
            bag_path = Path(data["source_recording"]) / data["source_fingerprint"]["bag_files"][0]["name"]
            bag_path.unlink()
            findings = validate_session(session)
            self.assertFalse(any(item.rule == "M7" for item in _errors(findings)))
            self.assertEqual(_errors(findings), [])
            bag_path.write_bytes(b"mcap-bytes")

        with self.subTest(case="wrong_hash"):
            data = self._load(session)
            data["source_fingerprint"]["bag_files"][0]["sha256"] = "c" * 64
            self._save(session, data)
            findings = validate_session(session)
            self.assertTrue(any(item.rule == "M7" for item in _errors(findings)), findings)

        with self.subTest(case="metadata_mismatch"):
            data = self._load(session)
            data["source_fingerprint"]["bag_files"][0]["sha256"] = None
            data["source_fingerprint"]["metadata_yaml_sha256"] = "d" * 64
            self._save(session, data)
            findings = validate_session(session)
            self.assertTrue(
                any(
                    item.rule == "M7" and "metadata.yaml" in item.message
                    for item in _errors(findings)
                ),
                findings,
            )

    def test_run_id_must_equal_folder_name(self):
        session, _manifest = self._build(run_id="folder-name")
        data = self._load(session)
        data["run_id"] = "other-name"
        self._save(session, data)
        findings = validate_session(session)
        matching = [
            item
            for item in _errors(findings)
            if item.rule == "M1" and "other-name" in item.message and "folder-name" in item.message
        ]
        self.assertTrue(matching, findings)

    def test_created_at_requires_offset(self):
        session, manifest = self._build()
        rejected = (
            "2026-10-19T21:30:00",
            "2026-10-19 21:30:00-05:00",
            "2026-10-19T21:30:00-0500",
            "2026-10-19",
            "2026-13-01T00:00:00Z",
        )
        for text in rejected:
            with self.subTest(created_at=text):
                data = self._load(session)
                data["created_at"] = text
                self._save(session, data)
                findings = validate_session(session)
                self.assertTrue(
                    any(item.rule == "M1" and "created_at" in item.message for item in _errors(findings)),
                    findings,
                )
        accepted = (
            manifest["created_at"],
            "2026-10-19T21:30:00Z",
            "2026-10-19T21:30:00-05:00",
            "2026-10-19T21:30:00+00:00",
            "2026-10-19T21:30:00.870055+00:00",
        )
        for text in accepted:
            with self.subTest(created_at=text):
                data = self._load(session)
                data["created_at"] = text
                self._save(session, data)
                findings = validate_session(session)
                self.assertFalse(
                    any(item.rule == "M1" and "created_at" in item.message for item in findings),
                    findings,
                )
                self.assertEqual(_errors(findings), [])

    def test_physical_reference_null_residual_not_within_tolerance(self):
        base = self._reference("run-a")
        self.assertTrue(placement_within_tolerance(base))
        cases = {
            "start_null": {"start_placement_residual_m": None},
            "end_null": {"end_placement_residual_m": None},
            "both_null": {
                "start_placement_residual_m": None,
                "end_placement_residual_m": None,
            },
            "heading_off": {"heading_check": "off"},
            "heading_not_checked": {"heading_check": "not_checked"},
            "above_tolerance": {"end_placement_residual_m": 0.02},
        }
        for name, overrides in cases.items():
            with self.subTest(case=name):
                data = self._reference("run-a", **overrides)
                self.assertFalse(placement_within_tolerance(data))
                findings = validate_physical_reference(data, expected_run_id="run-a")
                self.assertFalse(any(item.rule == "P1" for item in findings), findings)
                self.assertTrue(any(item.rule == "P2" and item.severity == "WARNING" for item in findings))
        on_tolerance = self._reference(
            "run-a",
            start_placement_residual_m=0.01,
            end_placement_residual_m=0.0,
        )
        self.assertTrue(placement_within_tolerance(on_tolerance))

    def test_physical_reference_negative_or_nan_residual_rejected(self):
        bad_values = (-0.001, float("nan"), float("inf"), float("-inf"), True, "0.1")
        for value in bad_values:
            for field in ("start_placement_residual_m", "end_placement_residual_m"):
                with self.subTest(field=field, value=value):
                    data = self._reference("run-a", **{field: value})
                    self.assertFalse(placement_within_tolerance(data))
                    findings = validate_physical_reference(data, expected_run_id="run-a")
                    self.assertTrue(
                        any(item.rule == "P1" and item.severity == "ERROR" and field in item.message for item in findings),
                        findings,
                    )

        session, _manifest = self._build(run_id="run-nan")
        reference = self._reference("run-nan")
        reference["start_placement_residual_m"] = float("nan")
        (session / "physical_reference.json").write_text(
            json.dumps(reference),
            encoding="utf-8",
        )
        data = self._load(session)
        data["physical_accuracy_evaluated"] = True
        self._save(session, data)
        # Rewrite the reference after _save so the NaN token survives.
        (session / "physical_reference.json").write_text(
            json.dumps(reference),
            encoding="utf-8",
        )
        findings = validate_session(session)
        self.assertTrue(any(item.rule == "P1" for item in _errors(findings)), findings)

    def test_sha256_streams_large_file(self):
        size = 50 * 1024 * 1024
        path = self.root / "large.bin"
        block = b"a" * hashing.CHUNK_BYTES
        expected = hashlib.sha256()
        remaining = size
        with open(path, "wb") as handle:
            while remaining:
                chunk = block if remaining >= len(block) else block[:remaining]
                handle.write(chunk)
                expected.update(chunk)
                remaining -= len(chunk)
        requested = []
        returned = []
        real_open_rb = hashing._open_rb

        def wrapped(file):
            handle = real_open_rb(file)
            original = handle.read

            def read(n=-1):
                data = original(n)
                requested.append(n)
                returned.append(len(data))
                return data

            handle.read = read
            return handle

        hashing._open_rb = wrapped
        try:
            digest = sha256_file(path)
        finally:
            hashing._open_rb = real_open_rb
        self.assertEqual(digest, expected.hexdigest())
        self.assertTrue(requested)
        self.assertTrue(all(n == hashing.CHUNK_BYTES for n in requested))
        self.assertEqual(sum(returned), size)
        self.assertLessEqual(max(returned), hashing.CHUNK_BYTES)
        self.assertGreaterEqual(len([count for count in returned if count]), 50)

    def test_build_refuses_protected_data_dirs(self):
        # A missing config means a broken guard still fails before mkdir.
        for root_name in ("sessions", "recordings"):
            protected = (
                Path.home()
                / "Developer"
                / "spatio-data"
                / root_name
                / "g2-manifest-guard-test"
            )
            self.assertFalse(protected.exists())
            try:
                with self.assertRaises(ManifestBuildError) as caught:
                    build_manifest(
                        protected,
                        protected.name,
                        str(self.root / "missing-recording"),
                        "mid360",
                        "unknown",
                        str(self.root / "missing-config.yaml"),
                        "none",
                        "must not be written",
                    )
                self.assertIn("refusing to write", str(caught.exception))
                self.assertFalse(protected.exists())
            finally:
                if protected.is_dir():
                    for child in list(protected.iterdir()):
                        if child.is_file():
                            child.unlink()
                    if not any(protected.iterdir()):
                        protected.rmdir()

    def test_cli_exit_codes(self):
        source = self._make_source(name="cli-recording")
        config = self._make_config(name="cli-sensor.yaml", text="lidar_type: 9\n")
        session = self.root / "cli-run"
        base = [
            "build",
            str(session),
            "--run-id",
            "cli-run",
            "--source-recording",
            str(source),
            "--sensor",
            "jt128",
            "--fastlio-revision",
            "unknown",
            "--config-file",
            str(config),
            "--image",
            "none",
            "--notes",
            "cli fixture",
        ]
        built = self._run(base)
        self.assertEqual(built.returncode, 0, built.stderr)
        self.assertIn("config_sha256", built.stdout)
        again = self._run(base)
        self.assertEqual(again.returncode, 1, again.stdout + again.stderr)
        self.assertIn("force", again.stderr)

        good = self._run(["validate", str(session)])
        self.assertEqual(good.returncode, 0, good.stderr)
        self.assertEqual(good.stdout, "")

        data = self._load(session)
        data["extra_from_replay"] = {"ok": True}
        self._save(session, data)
        warned = self._run(["validate", str(session)])
        self.assertEqual(warned.returncode, 0, warned.stderr)
        self.assertIn("WARNING M2", warned.stdout)

        data = self._load(session)
        del data["sensor"]
        self._save(session, data)
        bad = self._run(["validate", str(session)])
        self.assertEqual(bad.returncode, 1, bad.stdout)
        self.assertRegex(bad.stdout.splitlines()[0], r"^ERROR M1 ")

        missing = self._run(["validate", str(self.root / "does-not-exist")])
        self.assertEqual(missing.returncode, 2, missing.stdout)
        self.assertRegex(missing.stdout.splitlines()[0], r"^ERROR IO ")

        empty = self.root / "empty-session"
        empty.mkdir()
        no_manifest = self._run(["validate", str(empty)])
        self.assertEqual(no_manifest.returncode, 2)

        broken = self.root / "broken-json"
        broken.mkdir()
        (broken / "manifest.json").write_text("{", encoding="utf-8")
        invalid = self._run(["validate", str(broken)])
        self.assertEqual(invalid.returncode, 2)
        self.assertIn("ERROR IO", invalid.stdout)

        listed = self._run(["validate", "--json", str(self.root / "does-not-exist")])
        self.assertEqual(listed.returncode, 2)
        payload = json.loads(listed.stdout)
        self.assertIsInstance(payload, list)
        self.assertEqual(payload[0]["rule"], "IO")
        self.assertEqual(payload[0]["severity"], "ERROR")

        mixed = self._run(
            ["validate", str(session), str(self.root / "does-not-exist")]
        )
        self.assertEqual(mixed.returncode, 2)

    def _run(self, args):
        return subprocess.run(
            [sys.executable, "-m", "spatio.manifest"] + list(args),
            cwd=str(REPO_ROOT),
            capture_output=True,
            text=True,
            timeout=60,
            check=False,
        )


if __name__ == "__main__":
    unittest.main()
