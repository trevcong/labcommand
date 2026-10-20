"""Regression tests for CI's fixture guard and report/skip accounting."""
import tempfile
import subprocess
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

import ci_checks


class GoldenTests(unittest.TestCase):
    def test_committed_fixture_deleted_or_emptied_fails_preflight(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            with patch('ci_checks.subprocess.check_output', return_value=b'fixture.bin\0'):
                with self.assertRaisesRegex(ValueError, 'missing or empty committed fixture'):
                    ci_checks.check_fixtures(root)
                (root / 'fixture.bin').touch()
                with self.assertRaisesRegex(ValueError, 'missing or empty committed fixture'):
                    ci_checks.check_fixtures(root)

    def test_compare_complete_nonempty_byte_sets(self):
        with tempfile.TemporaryDirectory() as tmp:
            left, right = Path(tmp) / 'left', Path(tmp) / 'right'
            left.mkdir()
            right.mkdir()
            with self.assertRaises(ValueError):
                ci_checks.compare_goldens(left, right)
            for root in (left, right):
                (root / 'a.hex').write_bytes(b'00\n')
            self.assertEqual(ci_checks.compare_goldens(left, right), 1)
            for bad in (b'01\n', b'00', b''):
                (right / 'a.hex').write_bytes(bad)
                with self.assertRaises(ValueError):
                    ci_checks.compare_goldens(left, right)
            (right / 'a.hex').unlink()
            (right / 'extra.hex').write_bytes(b'00\n')
            with self.assertRaises(ValueError):
                ci_checks.compare_goldens(left, right)
            (left / 'a.hex').write_bytes(b'')
            (right / 'extra.hex').unlink()
            (right / 'a.hex').write_bytes(b'')
            with self.assertRaises(ValueError):
                ci_checks.compare_goldens(left, right)


class ReportTests(unittest.TestCase):
    def report(self, xml, kind='pytest', require_rosbags=False):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'report.xml'
            if xml is not None:
                path.write_text(xml)
            return ci_checks.summarize_report(path, kind, require_rosbags)

    def test_rosbags_missing_or_skipped_is_explicit_and_fails_when_required(self):
        for cases in (
            '<testcase name="other"/>',
            '<testcase name="test_rosbags"><skipped message="No module named rosbags"/></testcase>',
            '<testcase classname="tests.test_rosbags" name="one"/>'
            '<testcase classname="tests.test_rosbags" name="two"><skipped/></testcase>',
        ):
            xml = '<testsuite>' + cases + '</testsuite>'
            with self.subTest(cases=cases):
                summary, ok = self.report(xml)
                self.assertTrue(ok)
                self.assertIn('rosbags independent CDR cross-check: **SKIPPED**', summary)
                summary, ok = self.report(xml, require_rosbags=True)
                self.assertFalse(ok)
                self.assertIn('Required rosbags cross-check', summary)

    def test_rosbags_pass_and_failure_are_reported(self):
        for outcome, status in (('', 'PASS'), ('<failure/>', 'FAIL'), ('<error/>', 'FAIL')):
            xml = ('<testsuite><testcase classname="room_memory.tests.test_rosbags" '
                   f'name="roundtrip">{outcome}</testcase></testsuite>')
            summary, ok = self.report(xml, require_rosbags=True)
            self.assertEqual(ok, status == 'PASS')
            self.assertIn(f'rosbags independent CDR cross-check: **{status}**', summary)

    def test_cli_required_rosbags_skip_returns_failure(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root / 'pytest.xml').write_text(
                '<testsuite><testcase name="test_rosbags"><skipped '
                'message="No module named rosbags"/></testcase></testsuite>')
            (root / 'ctest.xml').write_text(self.ctest_xml())
            command = [sys.executable, str(Path(ci_checks.__file__)), 'summary', str(root)]
            optional = subprocess.run(command, capture_output=True, text=True)
            self.assertEqual(optional.returncode, 0, optional.stdout + optional.stderr)
            self.assertIn('**SKIPPED**', optional.stdout)
            required = subprocess.run(command + ['--require-rosbags'], capture_output=True, text=True)
            self.assertEqual(required.returncode, 1, required.stdout + required.stderr)
            self.assertIn('Required rosbags cross-check', required.stdout)

    def test_cli_kinds_pytest_reads_no_ctest_report(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            command = [sys.executable, str(Path(ci_checks.__file__)), 'summary', str(root), '--require-rosbags']
            (root / 'pytest.xml').write_text(
                '<testsuite><testcase classname="room_memory.tests.test_rosbags" name="one"/></testsuite>')
            both = subprocess.run(command, capture_output=True, text=True)
            self.assertEqual(both.returncode, 1, both.stdout + both.stderr)
            self.assertIn('### CTest entries', both.stdout)
            alone = subprocess.run(command + ['--kinds', 'pytest'], capture_output=True, text=True)
            self.assertEqual(alone.returncode, 0, alone.stdout + alone.stderr)
            self.assertIn('rosbags independent CDR cross-check: **PASS**', alone.stdout)
            self.assertNotIn('### CTest entries', alone.stdout)
            (root / 'pytest.xml').write_text(
                '<testsuite><testcase name="test_rosbags"><skipped/></testcase></testsuite>')
            skipped = subprocess.run(command + ['--kinds', 'pytest'], capture_output=True, text=True)
            self.assertEqual(skipped.returncode, 1, skipped.stdout + skipped.stderr)
            self.assertIn('**SKIPPED**', skipped.stdout)

    def test_nested_suites_failures_errors_and_skip_reasons(self):
        summary, ok = self.report('''<testsuites><testsuite tests="4">
          <testcase name="pass"/><testcase name="fail"><failure/></testcase>
          <testcase name="error"><error/></testcase>
          <testcase name="skip"><skipped message="no rclpy"/></testcase>
        </testsuite></testsuites>''')
        self.assertFalse(ok)
        self.assertIn('1 passed, 2 failed, 1 skipped', summary)
        self.assertIn('no rclpy', summary)

    def test_missing_malformed_empty_and_incomplete_reports_fail(self):
        for xml in (None, '<testsuite', '<testsuite tests="0"/>',
                    '<testsuite tests="2"><testcase name="one"/></testsuite>'):
            with self.subTest(xml=xml):
                summary, ok = self.report(xml)
                self.assertFalse(ok)
                self.assertNotIn('0 failed', summary)

    def ctest_xml(self, reason='static recording not found at /absent', extra=''):
        cases = ''.join(f'''<testcase name="{name}" status="notrun">
          <skipped message="SKIP_RETURN_CODE=77"/><system-out>{reason}</system-out>
        </testcase>''' for name in ci_checks.DATASET_TESTS)
        return f'<testsuite>{cases}<testcase name="core" status="run"/>{extra}</testsuite>'

    def test_dataset_skips_are_explicit_and_counted(self):
        summary, ok = self.report(self.ctest_xml(), 'ctest')
        self.assertTrue(ok)
        self.assertIn('1 passed, 0 failed, 3 skipped', summary)
        self.assertIn('missing dataset', summary)

    def test_missing_dependency_is_not_a_dataset_skip(self):
        summary, ok = self.report(self.ctest_xml('built without libzstd'), 'ctest')
        self.assertFalse(ok)
        self.assertIn('unexpected skip', summary)

    def test_missing_dataset_entry_or_unexpected_skip_fails(self):
        for xml in ('<testsuite><testcase name="core"/></testsuite>',
                    self.ctest_xml(extra='<testcase name="differential"><skipped/></testcase>')):
            self.assertFalse(self.report(xml, 'ctest')[1])

    def test_nested_skips_in_passing_ctest_entry_are_not_hidden(self):
        for output in ('4 passed, 0 failed, 1 skipped', '=== 4 passed, 1 skipped in 0.2s ==='):
            summary, ok = self.report(self.ctest_xml(extra=f'''
              <testcase name="goldens"><system-out>{output}</system-out></testcase>'''), 'ctest')
            self.assertFalse(ok)
            self.assertIn('nested skipped cases: 1', summary)


if __name__ == '__main__':
    unittest.main()
