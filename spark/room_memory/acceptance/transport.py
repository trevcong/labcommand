"""Read-only ros2 transport. The parent stamps receipt time. It does not ssh or publish.

``docker exec`` runs only on the machine that can already see the container. Stopping
the harness signals that child PID and no other process.
"""

import pathlib
import re

SCHEMA = "room-memory-acceptance-log-v1"
_CONTAINER = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,127}\Z")
_SETUP = "source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && python3 -u -"


def ros2_exec_command(container):
    if not isinstance(container, str) or _CONTAINER.fullmatch(container) is None:
        raise ValueError("container name is required")
    return ["docker", "exec", "-i", container, "bash", "-lc", _SETUP]


def script_text():
    return pathlib.Path(__file__).with_name("ros2_subscribe.py").read_text(encoding="utf-8")


def stamp_record(obj, receipt_mono_ns):
    """Parent clock wins for ``receipt_mono_ns``. The container monotonic time is kept beside it."""
    out = dict(obj)
    if "callback_mono_ns" not in out and "mono_ns" in out:
        out["callback_mono_ns"] = out["mono_ns"]
    out["receipt_mono_ns"] = int(receipt_mono_ns)
    out.setdefault("schema", SCHEMA)
    return out
