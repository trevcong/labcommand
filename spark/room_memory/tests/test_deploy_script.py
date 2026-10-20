"""scripts/spark/deploy-room-memory.sh --self-test. Docker and ssh are stubs on PATH and the host is
a .invalid name: nothing leaves this machine. It deploys a room-memory that runs with
rig_guard:=true and checks that the replacement and every rollback keep that argument.

A failed stop is rolled back like every failure after it. A rollback whose rm, rename, or start
fails exits 3 and prints what is left to do from what `docker ps` shows running. With no
room-memory container running, it is one command joined by &&, whose start checks `docker ps`
again and refuses if one runs: the self-test runs it with the failed step failing again, beside
another running room-memory container, and against a working docker. With one running (the
replacement, or the original under another name), no start is printed. When rm of a replacement
that is still running fails, or that replacement cannot be inspected, the original is not
started. A rollback that brings the original back prints 'rolled back'. A deploy that is already
up, whose `docker images` then fails while pruning, still exits 0.

ROOM_MEMORY_SELF_TEST_CASE runs one rollback or recovery case, so each reports on its own."""

import os
import pathlib
import shutil
import subprocess

import pytest

SCRIPT = pathlib.Path(__file__).resolve().parents[3] / "scripts" / "spark" / "deploy-room-memory.sh"

needs_tools = pytest.mark.skipif(not all(shutil.which(tool) for tool in ("bash", "git", "python3")),
                                 reason="the self-test needs bash, git and python3")

# Rolled back: the original is room-memory again, running.
ROLLED_BACK = ["stop-fail", "stop-fail-stopped", "start-fail", "rename-fail", "cleanup-perm", "health-exit",
               "health-restart", "health-unhealthy", "health-never-ready", "health-late-exit", "health-node-died"]
# The rollback failed and no room-memory container runs: a checked start, behind the rm and rename.
NONE_RUNNING = ["rollback-start-fail", "rollback-rm-start-fail"]
# The rollback failed and one runs: the replacement, or the original as room-memory-previous or
# room-memory. No start is printed.
ONE_RUNNING = ["rollback-rm-fail-running", "rollback-rm-fail-blind", "rollback-rename-fail", "rollback-rm-fail",
               "rollback-rm-before-rename"]
OTHER = ["rollback-ps-fail", "fresh-rm-fail"]


def self_test(case=None):
    env = dict(os.environ)
    env.pop("ROOM_MEMORY_SELF_TEST_CASE", None)
    if case is not None:
        env["ROOM_MEMORY_SELF_TEST_CASE"] = case
    return subprocess.run([str(SCRIPT), "--self-test"], stdin=subprocess.DEVNULL, capture_output=True, text=True,
                          timeout=300, env=env)


@needs_tools
def test_deploy_script_self_test():
    result = self_test()
    assert result.returncode == 0, result.stdout + result.stderr
    assert "self-test ok" in result.stdout


@needs_tools
@pytest.mark.parametrize("case", ROLLED_BACK + NONE_RUNNING + ONE_RUNNING + OTHER)
def test_deploy_script_rollback_case(case):
    result = self_test(case)
    assert result.returncode == 0, result.stdout + result.stderr
    assert f"self-test case {case} ok" in result.stdout


@needs_tools
def test_deploy_script_unknown_case_fails():
    result = self_test("no-such-case")
    assert result.returncode != 0
    assert "no self-test case named no-such-case" in result.stderr
