"""scripts/spark/deploy-room-memory.sh --self-test. Docker and ssh are stubs on PATH and the host is
a .invalid name: nothing leaves this machine. It deploys a room-memory that runs with
rig_guard:=true and checks that the replacement and every rollback keep that argument."""

import pathlib
import shutil
import subprocess

import pytest

SCRIPT = pathlib.Path(__file__).resolve().parents[3] / "scripts" / "spark" / "deploy-room-memory.sh"


@pytest.mark.skipif(not all(shutil.which(tool) for tool in ("bash", "git", "python3")),
                    reason="the self-test needs bash, git and python3")
def test_deploy_script_self_test():
    result = subprocess.run([str(SCRIPT), "--self-test"], stdin=subprocess.DEVNULL, capture_output=True, text=True,
                            timeout=300)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "self-test ok" in result.stdout
