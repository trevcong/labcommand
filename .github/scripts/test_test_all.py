"""scripts/test-all.sh: the rosbags stage fails a run without rosbags (issue #73), and SKIP_CPP=1 is
reported as PARTIAL with its own nonzero exit, never as a pass. Every stage except the rosbags
cross-check and the shared-golden comparison is stubbed, so this needs no C++, Swift or venv."""
import importlib.util
import os
import stat
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

SCRIPT = Path(__file__).resolve().parents[2] / 'scripts' / 'test-all.sh'
HAS_PYTEST = all(importlib.util.find_spec(name) for name in ('pytest', 'numpy'))
PARTIAL_EXIT = 10
CPP_STAGES = ('C++ configure', 'C++ build', 'C++ ctest')

# Stands in for PYTHON and SPATIO_PYTHON. With real_rosbags, check-rosbags.sh's calls reach the real
# interpreter; otherwise its summary reports a canned PASS.
PYTHON_STUB = """#!/bin/bash
if [[ -n "{real}" ]]; then
  case "$*" in
    *test_rosbags.py*|*"ci_checks.py summary"*|"-c "*) exec "{real}" "$@" ;;
  esac
fi
echo "python $*" >> "{log}"
case "$*" in
  *"ci_checks.py summary"*) echo "rosbags independent CDR cross-check: **PASS** (stubbed)" ;;
esac
"""
TOOL_STUB = """#!/bin/bash
echo "$(basename "$0") $*" >> "{log}"
exit {code}
"""


def write_executable(path, text):
    path.write_text(text)
    path.chmod(path.stat().st_mode | stat.S_IXUSR)


@unittest.skipUnless(HAS_PYTEST, 'the rosbags stage runs pytest; needs pytest and numpy')
class TestAllTests(unittest.TestCase):
    def run_script(self, skip_cpp=False, real_rosbags=False, hide_rosbags=False, swift_exit=0):
        with tempfile.TemporaryDirectory() as tmp:
            tmp = Path(tmp)
            log = tmp / 'calls.log'
            log.touch()
            bin_dir = tmp / 'bin'
            bin_dir.mkdir()
            python = tmp / 'python'
            write_executable(python, PYTHON_STUB.format(real=sys.executable if real_rosbags else '', log=log))
            for tool, code in (('swift', swift_exit), ('cmake', 0), ('ctest', 0)):
                write_executable(bin_dir / tool, TOOL_STUB.format(log=log, code=code))
            env = {key: value for key, value in os.environ.items()
                   if key not in ('SKIP_CPP', 'PY', 'PYTHONPATH') and not key.startswith('ROOM_MEMORY_')}
            env.update(PYTHON=str(python), SPATIO_PYTHON=str(python),
                       PATH=os.pathsep.join((str(bin_dir), env.get('PATH', ''))))
            if skip_cpp:
                env['SKIP_CPP'] = '1'
            if hide_rosbags:
                shadow = tmp / 'shadow' / 'rosbags'
                shadow.mkdir(parents=True)
                (shadow / '__init__.py').write_text('raise ImportError("rosbags hidden by test_test_all")\n')
                env['PYTHONPATH'] = str(shadow.parent)
            result = subprocess.run(['bash', str(SCRIPT)], capture_output=True, text=True, env=env,
                                    stdin=subprocess.DEVNULL, timeout=600)
            return result, log.read_text()

    def summary(self, result):
        lines = result.stdout.splitlines()
        return lines[lines.index('Local test summary (exit %d)' % result.returncode) + 1:]

    def test_missing_rosbags_fails_the_run(self):
        result, calls = self.run_script(real_rosbags=True, hide_rosbags=True)
        output = result.stdout + result.stderr
        self.assertEqual(result.returncode, 1, output)
        summary = self.summary(result)
        self.assertIn('PASS: Spark pytest', summary, output)
        self.assertIn('FAIL: rosbags cross-check', summary, output)
        self.assertIn('SKIPPED (not reached): C++ configure', summary, output)
        self.assertIn('SKIPPED (not reached): Spatio fixture regeneration check', summary, output)
        self.assertEqual(summary[-1], 'Overall: FAIL', output)
        self.assertIn('rosbags SKIPPED (rosbags not installed', result.stdout)
        self.assertNotIn('swift', calls)

    @unittest.skipUnless(importlib.util.find_spec('rosbags'), 'rosbags is not installed')
    def test_with_rosbags_a_full_run_passes(self):
        result, calls = self.run_script(real_rosbags=True)
        output = result.stdout + result.stderr
        self.assertEqual(result.returncode, 0, output)
        summary = self.summary(result)
        self.assertIn('PASS: rosbags cross-check', summary, output)
        for stage in CPP_STAGES:
            self.assertIn('PASS: ' + stage, summary, output)
        self.assertEqual(summary[-1], 'Overall: PASS', output)
        self.assertTrue(any(line.startswith('rosbags PASS') for line in result.stdout.splitlines()), output)
        self.assertIn('ctest --test-dir', calls)

    def test_skip_cpp_is_partial_not_pass(self):
        result, calls = self.run_script(skip_cpp=True)
        output = result.stdout + result.stderr
        self.assertEqual(result.returncode, PARTIAL_EXIT, output)
        summary = self.summary(result)
        for stage in CPP_STAGES:
            self.assertIn('SKIPPED (SKIP_CPP=1): ' + stage, summary, output)
        self.assertIn('PASS: rosbags cross-check', summary, output)
        self.assertIn('PASS: Spatio fixture regeneration check', summary, output)
        self.assertIn('room_memory/tests/test_rosbags.py', calls)
        self.assertNotIn('FAIL: runner setup', summary, output)
        self.assertEqual(summary[-1], 'Overall: PARTIAL (SKIP_CPP=1: 3 C++ stages skipped)', output)
        self.assertNotIn('cmake', calls)
        self.assertNotIn('ctest', calls)

    def test_skip_cpp_with_a_later_failure_is_fail(self):
        result, _ = self.run_script(skip_cpp=True, swift_exit=4)
        output = result.stdout + result.stderr
        self.assertEqual(result.returncode, 4, output)
        summary = self.summary(result)
        self.assertIn('FAIL: Swift core tests', summary, output)
        self.assertEqual(summary[-1], 'Overall: FAIL', output)

    def test_a_stage_failure_with_the_partial_code_is_not_partial(self):
        result, _ = self.run_script(skip_cpp=True, swift_exit=PARTIAL_EXIT)
        output = result.stdout + result.stderr
        self.assertEqual(result.returncode, 1, output)
        summary = self.summary(result)
        self.assertIn('FAIL: Swift core tests', summary, output)
        self.assertEqual(summary[-1], 'Overall: FAIL', output)


if __name__ == '__main__':
    unittest.main()
