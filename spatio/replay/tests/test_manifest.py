import json
import os
import subprocess
import sys
import tempfile
import unittest

HERE = os.path.join(os.path.dirname(__file__), "..")
sys.path.insert(0, HERE)

import manifest  # noqa: E402


class ManifestTests(unittest.TestCase):
    def build(self, **kw):
        args = dict(run_id="r1", source_recording="/rec", sensor="Mid-360",
                    fastlio_revision="abc", config_file="/c.yaml",
                    config_sha256="00", image="sha256:1", notes="n")
        args.update(kw)
        return manifest.build_manifest(**args)

    def test_exact_contract_keys_and_never_claims_accuracy(self):
        m = self.build()
        self.assertEqual(tuple(m), manifest.CONTRACT_KEYS)
        self.assertIs(m["physical_accuracy_evaluated"], False)

    def test_historical_label_always_present_once(self):
        self.assertTrue(self.build(notes="x")["notes"].startswith(manifest.HISTORICAL_LABEL))
        n = self.build(notes=manifest.HISTORICAL_LABEL + "; already")["notes"]
        self.assertEqual(n.count(manifest.HISTORICAL_LABEL), 1)

    def test_rejects_path_like_run_id(self):
        with self.assertRaises(ValueError):
            self.build(run_id="../x")

    def test_cli_hashes_config(self):
        with tempfile.TemporaryDirectory() as d:
            cfg = os.path.join(d, "c.yaml")
            with open(cfg, "w") as f:
                f.write("a: 1\n")
            sess = os.path.join(d, "s")
            os.mkdir(sess)
            subprocess.run([sys.executable, os.path.join(HERE, "manifest.py"), sess,
                            "--run-id", "r", "--source-recording", "/rec", "--sensor", "s",
                            "--fastlio-revision", "abc", "--config-file", cfg,
                            "--image", "sha256:1"],
                           check=True, capture_output=True)
            with open(os.path.join(sess, "manifest.json")) as f:
                m = json.load(f)
            self.assertEqual(m["config_sha256"], manifest.sha256_file(cfg))
            self.assertEqual(len(m["config_sha256"]), 64)


if __name__ == "__main__":
    unittest.main()
