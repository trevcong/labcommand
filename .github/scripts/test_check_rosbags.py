"""scripts/check-rosbags.sh fails, not skips, when rosbags cannot be imported (issue #73)."""
import importlib.util
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

SCRIPT = Path(__file__).resolve().parents[2] / 'scripts' / 'check-rosbags.sh'
HAS_PYTEST = all(importlib.util.find_spec(name) for name in ('pytest', 'numpy'))


def run(pythonpath=None):
    env = dict(os.environ, PY=sys.executable)
    if pythonpath:
        env['PYTHONPATH'] = os.pathsep.join(filter(None, (pythonpath, env.get('PYTHONPATH'))))
    return subprocess.run(['bash', str(SCRIPT)], capture_output=True, text=True, env=env,
                          stdin=subprocess.DEVNULL, timeout=600)


@unittest.skipUnless(HAS_PYTEST, 'the script runs pytest; needs pytest and numpy')
class CheckRosbagsTests(unittest.TestCase):
    def test_without_rosbags_the_script_fails_and_says_skipped(self):
        with tempfile.TemporaryDirectory() as tmp:
            shadow = Path(tmp) / 'rosbags'
            shadow.mkdir()
            (shadow / '__init__.py').write_text('raise ImportError("rosbags hidden by test_check_rosbags")\n')
            result = run(pythonpath=tmp)
        self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
        self.assertIn('rosbags independent CDR cross-check: **SKIPPED**', result.stdout)
        self.assertTrue(result.stdout.splitlines()[-1].startswith('rosbags SKIPPED (rosbags not installed'),
                        result.stdout)

    @unittest.skipUnless(importlib.util.find_spec('rosbags'), 'rosbags is not installed')
    def test_with_rosbags_the_script_passes(self):
        result = run()
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn('rosbags independent CDR cross-check: **PASS**', result.stdout)
        self.assertTrue(result.stdout.splitlines()[-1].startswith('rosbags PASS'), result.stdout)


if __name__ == '__main__':
    unittest.main()
