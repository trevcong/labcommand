"""The :8767 bridge sidecar must give the Mac a request path (issue #21), read from the launch file as
text: nothing here needs ROS.

Live on c5518eb the bridge said ``capabilities: ["connectionGraph"]`` and nothing else, so the Mac had
no way to send ``RequestSync`` and showed "history incomplete: 20 missing · no request path".
"""

import ast
import pathlib

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[3]
CPP_LAUNCH = ROOT / "spark/room_memory_cpp/launch/room_memory_cpp.launch.py"
REQUEST_SERVICE = "/room_memory/sync/request"


def _bridge_parameters():
    """The literal parameter dict of the launch file's foxglove_bridge Node, as source text per key."""
    tree = ast.parse(CPP_LAUNCH.read_text())
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and any(
                k.arg == "package" and isinstance(k.value, ast.Constant) and k.value.value == "foxglove_bridge"
                for k in node.keywords):
            parameters = next(k.value for k in node.keywords if k.arg == "parameters")
            (table,) = parameters.elts
            return {ast.literal_eval(key): ast.unparse(value) for key, value in zip(table.keys, table.values)}
    raise AssertionError("no foxglove_bridge Node in the launch file")


@pytest.mark.xfail(strict=True, reason="issue #21: the bridge offers no request path")
def test_bridge_offers_the_request_service():
    parameters = _bridge_parameters()
    assert "service_whitelist" in parameters
    assert "services" in parameters["capabilities"]
