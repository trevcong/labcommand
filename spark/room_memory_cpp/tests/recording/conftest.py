"""A run in which every test skipped exits 77, CTest's SKIP_RETURN_CODE, instead of pytest's 0.

Without the recording (or the mcap package, or ROOM_MEMORY_BAG) every test in
test_static_recording.py skips, and CTest would otherwise report static_recording as passed.
"""

SKIPPED_EVERYTHING = 77
RAN = ("passed", "failed", "error", "xfailed", "xpassed")


def pytest_sessionfinish(session, exitstatus):
    reporter = session.config.pluginmanager.get_plugin("terminalreporter")
    if reporter is None or exitstatus != 0:
        return
    if reporter.stats.get("skipped") and not any(reporter.stats.get(outcome) for outcome in RAN):
        session.exitstatus = SKIPPED_EVERYTHING
