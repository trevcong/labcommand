"""The :8767 bridge sidecar must give the Mac a request path (issue #21), read from the sources as
text: nothing here needs ROS.

Live on c5518eb the bridge said ``capabilities: ["connectionGraph"]`` and nothing else, so the Mac had
no way to send ``RequestSync`` and showed "history incomplete: 20 missing · no request path". The
path is now the node's ``/room_memory/sync/request`` service through the bridge's ``services``
capability (protocol D39). Client publishing stays off: foxglove_bridge does not enforce
``client_topic_whitelist``.
"""

import ast
import pathlib
import re

ROOT = pathlib.Path(__file__).resolve().parents[3]
CPP_LAUNCH = ROOT / "spark/room_memory_cpp/launch/room_memory_cpp.launch.py"
REQUEST_SERVICE = "/room_memory/sync/request"
OPERATOR_SERVICE = "/room_memory/operator/action"


def _module_constants(path):
    """Top-level assignments of a module that evaluate as literals, without importing it."""
    constants = {}
    tree = ast.parse(path.read_text())
    for node in tree.body:
        if isinstance(node, ast.Assign) and isinstance(node.targets[0], ast.Name):
            try:
                constants[node.targets[0].id] = ast.literal_eval(node.value)
            except ValueError:
                pass
    return constants


def _bridge_parameters():
    """The parameter dict of the launch file's foxglove_bridge Node, as source text per key."""
    tree = ast.parse(CPP_LAUNCH.read_text())
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and any(
                k.arg == "package" and isinstance(k.value, ast.Constant) and k.value.value == "foxglove_bridge"
                for k in node.keywords):
            parameters = next(k.value for k in node.keywords if k.arg == "parameters")
            (table,) = parameters.elts
            return {ast.literal_eval(key): ast.unparse(value) for key, value in zip(table.keys, table.values)}
    raise AssertionError("no foxglove_bridge Node in the launch file")


def test_bridge_offers_the_request_service():
    constants = _module_constants(CPP_LAUNCH)
    parameters = _bridge_parameters()
    assert constants["REQUEST_SERVICE"] == REQUEST_SERVICE
    assert constants["BRIDGE_CAPABILITIES"] == ["connectionGraph", "services"]
    assert parameters["service_whitelist"] == "ParameterValue(service_whitelist, value_type=List[str])"
    assert parameters["capabilities"] == "ParameterValue(capabilities, value_type=List[str])"


def test_service_whitelist_admits_only_the_request_service():
    # The bridge's default is ".*": every node's services, parameter services included.
    (pattern,) = _module_constants(CPP_LAUNCH)["BRIDGE_SERVICE_WHITELIST"]
    assert re.fullmatch(pattern, REQUEST_SERVICE)
    for other in ("/room_memory/sync/request_x", "/x/room_memory/sync/request", "/room_memory/request_sync",
                  "/room_memory/set_parameters", "/hesai_ros_driver_node/set_parameters", "/lidar/sync/request"):
        assert not re.search(pattern, other), other


def test_client_publish_stays_opt_in():
    source = CPP_LAUNCH.read_text()
    assert _module_constants(CPP_LAUNCH)["BRIDGE_ARGUMENTS"]["foxglove_client_publish"] == "false"
    assert "clientPublish" not in _module_constants(CPP_LAUNCH)["BRIDGE_CAPABILITIES"]
    # The capabilities expression: BRIDGE_CAPABILITIES, plus clientPublish only when asked for.
    assert 'repr(["clientPublish"] + BRIDGE_CAPABILITIES)' in source
    assert '"\'.lower() == \'true\' else ", repr(BRIDGE_CAPABILITIES)' in source


def test_every_side_names_the_same_service():
    python_node = _module_constants(ROOT / "spark/room_memory/room_memory/node.py")
    assert python_node["REQUEST_SERVICE"] == REQUEST_SERVICE
    cpp_node = (ROOT / "spark/room_memory_cpp/node/room_memory_node.cpp").read_text()
    assert f'constexpr const char* kRequestService = "{REQUEST_SERVICE}";' in cpp_node
    swift = (ROOT / "Sources/LabCommandCore/LiveRoom/LiveRoomClient.swift").read_text()
    assert f'public static let requestService = "{REQUEST_SERVICE}"' in swift
    assert 'public static let requestServiceType = "labcommand_msgs/srv/RequestSync"' in swift


def test_service_request_is_exactly_one_request_sync():
    # One nested RequestSync and nothing else: the service request's CDR has the message's layout,
    # so the Mac sends one encoding on every path.
    lines = [line.split("#")[0].strip() for line in (ROOT / "ros/labcommand_msgs/srv/RequestSync.srv").read_text().splitlines()]
    fields = [line for line in lines if line]
    separator = fields.index("---")
    assert fields[:separator] == ["RequestSync request"]
    assert fields[separator + 1:] == ["bool accepted"]
    assert '"srv/RequestSync.srv"' in (ROOT / "ros/labcommand_msgs/CMakeLists.txt").read_text()


def test_the_operator_service_is_opt_in():
    # D46: it changes durable state through a port without authentication. The node offers it and the
    # bridge whitelists it only with operator_actions:=true; the default is off in both launch files.
    constants = _module_constants(CPP_LAUNCH)
    assert constants["OPERATOR_SERVICE"] == OPERATOR_SERVICE
    for launch in (CPP_LAUNCH, ROOT / "spark/room_memory/launch/room_memory.launch.py"):
        assert '"operator_actions": ("false", bool),' in launch.read_text(), launch
        # A node flag, not a MemoryConfig field: it lives in NODE_ARGUMENTS, never in ARGUMENTS.
        tables = {node.targets[0].id: [ast.literal_eval(key) for key in node.value.keys]
                  for node in ast.parse(launch.read_text()).body
                  if isinstance(node, ast.Assign) and isinstance(node.targets[0], ast.Name) and isinstance(node.value, ast.Dict)}
        assert tables["NODE_ARGUMENTS"] == ["operator_actions"] and "operator_actions" not in tables["ARGUMENTS"], launch
    pattern = constants["OPERATOR_SERVICE_PATTERN"]
    assert re.fullmatch(pattern, OPERATOR_SERVICE)
    for other in (REQUEST_SERVICE, "/room_memory/operator", "/room_memory/operator/action_x", "/x/room_memory/operator/action"):
        assert not re.search(pattern, other), other
    source = CPP_LAUNCH.read_text()
    assert 'repr(BRIDGE_SERVICE_WHITELIST + [OPERATOR_SERVICE_PATTERN]), " if \'", LaunchConfiguration("operator_actions")' in source
    assert '"\'.lower() == \'true\' else ", repr(BRIDGE_SERVICE_WHITELIST)' in source
    cpp_node = (ROOT / "spark/room_memory_cpp/node/room_memory_node.cpp").read_text()
    assert 'declare_parameter<bool>("operator_actions", false)' in cpp_node
    python_node = (ROOT / "spark/room_memory/room_memory/node.py").read_text()
    assert 'self.declare_parameter("operator_actions", False)' in python_node


def _braced_block(source, opener):
    """The text between ``opener`` (which ends in ``{``) and its matching ``}``."""
    assert source.count(opener) == 1, opener
    start = source.index(opener) + len(opener)
    depth = 1
    for index in range(start, len(source)):
        depth += {"{": 1, "}": -1}.get(source[index], 0)
        if depth == 0:
            return source[start:index]
    raise AssertionError(f"unbalanced {opener}")


def test_the_operator_topic_is_behind_the_flag_too():
    # Review of #31: the topic changes durable state like the service, and any peer on the ROS domain
    # can publish on it, so without operator_actions:=true the C++ node does not subscribe at all. The
    # C++ node is built only under colcon; test_node_operator_gate.py runs the Python node's twin.
    cpp_node = (ROOT / "spark/room_memory_cpp/node/room_memory_node.cpp").read_text()
    gated = _braced_block(cpp_node, "if (operator_actions) {")
    assert "operator_topic_ = create_generic_subscription(\n                kOperatorTopic," in gated
    assert "operator_service_ = create_service<labcommand_msgs::srv::OperatorAction>(" in gated
    outside = cpp_node.replace(gated, "")
    assert outside.count("kOperatorTopic") == 1, "only the constant's definition is outside the gate"
    assert "create_generic_subscription(\n            kOperatorTopic" not in outside


def test_every_side_names_the_same_operator_service():
    python_node = _module_constants(ROOT / "spark/room_memory/room_memory/node.py")
    assert python_node["OPERATOR_SERVICE"] == OPERATOR_SERVICE
    assert python_node["OPERATOR_TOPIC"] == "/room_memory/operator"
    cpp_node = (ROOT / "spark/room_memory_cpp/node/room_memory_node.cpp").read_text()
    assert f'constexpr const char* kOperatorService = "{OPERATOR_SERVICE}";' in cpp_node
    assert 'constexpr const char* kOperatorTopic = "/room_memory/operator";' in cpp_node
    swift = (ROOT / "Sources/LabCommandCore/LiveRoom/LiveRoomClient.swift").read_text()
    assert f'public static let operatorService = "{OPERATOR_SERVICE}"' in swift
    assert 'public static let operatorServiceType = "labcommand_msgs/srv/OperatorAction"' in swift


def test_operator_service_is_one_action_and_one_outcome():
    lines = [line.split("#")[0].strip() for line in (ROOT / "ros/labcommand_msgs/srv/OperatorAction.srv").read_text().splitlines()]
    fields = [line for line in lines if line]
    separator = fields.index("---")
    assert fields[:separator] == ["OperatorAction action"]
    assert fields[-1] == "uint8 outcome"
    # The outcome constants match the codec's enum, value for value.
    from room_memory.messages import OperatorOutcome
    constants = dict(line.split()[1].split("=") for line in fields[separator + 1:-1])
    assert {name.removeprefix("OUTCOME_"): int(value) for name, value in constants.items()} == {
        outcome.name: int(outcome) for outcome in OperatorOutcome}
    assert '"srv/OperatorAction.srv"' in (ROOT / "ros/labcommand_msgs/CMakeLists.txt").read_text()

