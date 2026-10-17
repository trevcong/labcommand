"""Focused regressions for runner paths; full YAML validation is separate."""
import os
from pathlib import Path
import re
import subprocess
import tempfile
import textwrap
import unittest


WORKFLOW = Path(__file__).resolve().parents[1] / 'workflows' / 'ci.yml'
PATHS = {
    'REPORTS': 'ci-reports',
    'CPP_BUILD': 'room-memory-build',
    'ROOM_MEMORY_RECORDING': 'absent-recordings/static5m',
    'ROOM_MEMORY_RECORDING_B': 'absent-recordings/static5m-b',
}


class WorkflowEnvironmentTests(unittest.TestCase):
    def test_ubuntu_job_environment_uses_only_plain_values(self):
        workflow = WORKFLOW.read_text()
        block = re.search(r'^  ubuntu:\n.*?^    env:\n(.*?)^    steps:\n',
                          workflow, re.MULTILINE | re.DOTALL)
        self.assertIsNotNone(block, 'Ubuntu job env must be checked')
        # runner is unavailable at jobs.<job_id>.env. This job needs no
        # expressions there; path expansion belongs in a runner shell step.
        self.assertNotIn('${{', block[1])
        for name in PATHS:
            self.assertNotRegex(block[1], rf'(?m)^\s*{name}:')
        self.assertIn('PYTHONDONTWRITEBYTECODE:', block[1])
        self.assertIn('PYTEST_ADDOPTS:', block[1])

    def test_first_step_exports_all_paths_for_later_steps(self):
        workflow = WORKFLOW.read_text()
        block = re.search(
            r'^  ubuntu:\n.*?^    steps:\n'
            r'      - name: Initialize runner paths\n'
            r'        run: \|\n((?:          [^\n]*\n)+)',
            workflow, re.MULTILINE | re.DOTALL)
        self.assertIsNotNone(block, 'Initialize paths before any consumer step')
        script = textwrap.dedent(block[1])
        with tempfile.TemporaryDirectory(prefix='ci runner paths ') as tmp:
            env_file = Path(tmp) / 'github env'
            env_file.write_text('EXISTING=preserved\n')
            subprocess.run(['bash', '-euo', 'pipefail', '-c', script], check=True,
                           stdin=subprocess.DEVNULL, capture_output=True, text=True,
                           env={**os.environ, 'RUNNER_TEMP': tmp, 'GITHUB_ENV': str(env_file)})
            expected = {'EXISTING': 'preserved',
                        **{name: f'{tmp}/{suffix}' for name, suffix in PATHS.items()}}
            self.assertCountEqual(env_file.read_text().splitlines(),
                                  [f'{name}={value}' for name, value in expected.items()])


if __name__ == '__main__':
    unittest.main()
