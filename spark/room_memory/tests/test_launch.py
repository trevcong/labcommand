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
NAME_OBJECT_SERVICE = "/room_memory/objects/name"


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
    assert parameters["service_whitelist"] == "BRIDGE_SERVICE_WHITELIST"
    assert parameters["capabilities"] == "ParameterValue(capabilities, value_type=List[str])"


def test_service_whitelist_admits_the_request_and_name_services():
    # The bridge's default is ".*": every node's services, parameter services included.
    patterns = _module_constants(CPP_LAUNCH)["BRIDGE_SERVICE_WHITELIST"]
    assert patterns == ["^/room_memory/sync/request$", "^/room_memory/objects/name$"]
    assert _module_constants(CPP_LAUNCH)["NAME_OBJECT_SERVICE"] == NAME_OBJECT_SERVICE
    assert re.fullmatch(patterns[0], REQUEST_SERVICE)
    assert re.fullmatch(patterns[1], NAME_OBJECT_SERVICE)
    assert not re.fullmatch(patterns[0], NAME_OBJECT_SERVICE)
    assert not re.fullmatch(patterns[1], REQUEST_SERVICE)
    for other in ("/room_memory/sync/request_x", "/x/room_memory/sync/request", "/room_memory/request_sync",
                  "/room_memory/set_parameters", "/hesai_ros_driver_node/set_parameters", "/lidar/sync/request",
                  "/room_memory/objects/name_x", "/room_memory/objects/rename", "/x/room_memory/objects/name"):
        assert not any(re.search(pattern, other) for pattern in patterns), other


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
    # NameObject is whitelisted and named here. The node does not advertise it until the wiring PR.
    assert f'public static let nameObjectService = "{NAME_OBJECT_SERVICE}"' in swift
    assert 'public static let nameObjectServiceType = "labcommand_msgs/srv/NameObject"' in swift


def test_service_request_is_exactly_one_request_sync():
    # One nested RequestSync and nothing else: the service request's CDR has the message's layout,
    # so the Mac sends one encoding on every path.
    lines = [line.split("#")[0].strip() for line in (ROOT / "ros/labcommand_msgs/srv/RequestSync.srv").read_text().splitlines()]
    fields = [line for line in lines if line]
    separator = fields.index("---")
    assert fields[:separator] == ["RequestSync request"]
    assert fields[separator + 1:] == ["bool accepted"]
    assert '"srv/RequestSync.srv"' in (ROOT / "ros/labcommand_msgs/CMakeLists.txt").read_text()


def _nodes():
    """Every Node(...) call in the launch file, as {executable: {keyword: source text}}."""
    tree = ast.parse(CPP_LAUNCH.read_text())
    nodes = {}
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and node.func.id == "Node":
            keywords = {k.arg: ast.unparse(k.value) for k in node.keywords}
            nodes[ast.literal_eval(keywords["executable"])] = keywords
    return nodes


def test_either_process_exiting_ends_the_container():
    # Review round 2: the node logged readiness and then died while the bridge kept ros2 launch, and
    # so the container, running. Docker never restarted it and the deploy's stability window passed.
    tree = ast.parse(CPP_LAUNCH.read_text())
    imports = [n for n in tree.body if isinstance(n, ast.ImportFrom) and n.module == "launch.actions"]
    assert "Shutdown" in {alias.name for n in imports for alias in n.names}
    nodes = _nodes()
    assert set(nodes) == {"room_memory_node", "foxglove_bridge"}
    for executable, keywords in nodes.items():
        assert keywords.get("on_exit", "").startswith("Shutdown("), executable
    # ros2 launch is the container's main process (exec, not a shell's child), so its end is the container's.
    assert (ROOT / "spark/room_memory_cpp/docker-entrypoint.sh").read_text().splitlines()[-1] == 'exec "$@"'
    assert 'CMD ["ros2", "launch", "room_memory_cpp", "room_memory_cpp.launch.py"]' in (
        ROOT / "spark/room_memory_cpp/Dockerfile").read_text()


def test_nodes_log_readiness_when_a_generation_is_allocated():
    # The deploy waits for this line. Its self-test stub writes the line itself, so --self-test stays
    # green even when the nodes never log it. Both nodes log it immediately after allocation.
    python = (ROOT / "spark/room_memory/room_memory/node.py").read_text()
    cpp = (ROOT / "spark/room_memory_cpp/node/room_memory_node.cpp").read_text()
    assert "room-memory ready: publisher_generation {self.generation.generation}" in python
    assert "room-memory ready: publisher_generation %llu" in cpp
