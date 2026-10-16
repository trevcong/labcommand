"""A run in which every test skipped exits 77, CTest's SKIP_RETURN_CODE, instead of pytest's 0 or 5.

Without the recording (or ROOM_MEMORY_BAG) every test in test_static_recording.py skips, and
CTest would otherwise report static_recording as passed. Without the mcap package (the image has
none) the module itself skips at collection, nothing is collected, and pytest exits 5
(NO_TESTS_COLLECTED), which CTest would report as failed. A collection error, a failure, or a run
that collected nothing and skipped nothing keeps its own exit code.
"""

import pytest

SKIPPED_EVERYTHING = 77
RAN = ("passed", "failed", "error", "xfailed", "xpassed")


def pytest_sessionfinish(session, exitstatus):
    reporter = session.config.pluginmanager.get_plugin("terminalreporter")
    if reporter is None or exitstatus not in (pytest.ExitCode.OK, pytest.ExitCode.NO_TESTS_COLLECTED):
        return
    if reporter.stats.get("skipped") and not any(reporter.stats.get(outcome) for outcome in RAN):
        session.exitstatus = SKIPPED_EVERYTHING
