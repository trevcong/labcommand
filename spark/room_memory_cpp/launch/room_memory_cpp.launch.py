"""Launches the C++ room_memory node and, by default, a foxglove_bridge sidecar for the Mac. Every parameter can be overridden on the command line:

    ros2 launch room_memory_cpp room_memory_cpp.launch.py room_id:=lab sensors:="['mid360']"

``max_range`` <= 0 disables the range clip; ``state_dir`` must already exist (it holds the
references, ``map_epoch`` and the publisher generation), and an empty or missing one stops the node;
an empty ``map_sensor`` publishes the JT128's reference when it is configured, else the first
sensor's; ``rig_guard:=true`` turns on the rig-pose guard (``RigPose`` on ``/room_memory/rig``),
fed by ``imu_topic`` (empty: ICP only). Same node parameters and defaults as ``room_memory.launch.py`` (the Python node), which
are ``MemoryConfig``'s, measured on the static recording.

The sidecar (``foxglove_bridge:=false`` turns it off) listens on ``foxglove_port`` (8767) and
relays only topics matching ``^/room_memory/.*``, so the raw clouds never leave through it; the
lidar containers' bridge on :8765 is not touched. The Mac's ``RequestSync`` goes over the same
socket as the node's ``/room_memory/sync/request`` service (protocol D39): the bridge offers
``services`` with a ``service_whitelist`` of that name and ``/room_memory/objects/name``
(protocol D51). The node does not advertise NameObject until the wiring PR. Every other
node's services, the
lidar drivers' parameter services included, stay hidden; the bridge enforces this whitelist
(the default is ``.*``, so it must be set). Client publishing stays off unless
``foxglove_client_publish:=true``: foxglove_bridge parses ``client_topic_whitelist`` but does not
enforce it (3.4.1 through 3.6.2), so client publishing would let any client that reaches the port
publish on any topic.

Either process exiting shuts the whole launch down, and with it the container (``ros2 launch`` is its
main process). Without that, a node that crashed leaves the other one holding the container up:
Docker reports it running, its restart policy never fires, and a deploy's stability window passes.
A bridge exit restarts the node too, so it reclassifies the room under a new publisher generation;
a node the Mac cannot reach is down all the same.
"""

from typing import List

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

ARGUMENTS = {
    "room_id": ("lab", str),
    "sensors": ("['mid360', 'jt128']", None),
    "voxel_size": ("0.05", float),
    "max_range": ("15.0", float),
    "tolerance": ("1", int),
    "min_voxels": ("6", int),
    "min_extent": ("2", int),
    "window_s": ("1.0", float),
    "reference_windows": ("30", int),
    "recent_windows": ("3", int),
    "state_dir": ("~/room-memory", str),
    "occupied_min_hits": ("2", int),
    "occupied_min_ratio": ("0.3", float),
    "free_min_seen": ("2", int),
    "free_max_ratio": ("0.0", float),
    "candidate_windows": ("40", int),
    "expire_windows": ("3", int),
    "fresh_min_voxels": ("3", int),
    "fresh_min_fraction": ("0.5", float),
    "stable_min_fraction": ("0.9", float),
    "activity_radius_m": ("1.0", float),
    "learn_max_offset_m": ("0.02", float),
    "learn_windows": ("0", int),
    "map_sensor": ("", str),
    "delta_period_s": ("5.0", float),
    "keyframe_period_s": ("60.0", float),
    "keyframe_delta_ratio": ("0.5", float),
    "rig_guard": ("false", bool),
    "imu_topic": ("/livox/imu", str),
    "imu_shock_frac": ("0.05", float),
    "imu_gyro_dps": ("2.0", float),
    "imu_quiet_s": ("2.0", float),
    "tilt_trigger_deg": ("0.2", float),
    "pose_trigger_m": ("0.015", float),
    "guard_check_period_s": ("30.0", float),
    "check_max_s": ("10.0", float),
    "move_min_m": ("0.015", float),
    "move_min_deg": ("0.25", float),
    "align_max_m": ("0.3", float),
    "align_max_deg": ("10.0", float),
    "align_inlier_ratio": ("0.9", float),
    "align_max_rms_m": ("0.02", float),
    "align_stable_windows": ("3", int),
    "observe_min_eig": ("0.02", float),
    "gross_inlier_ratio": ("0.5", float),
    "relearn_after_s": ("60.0", float),
    "auto_relearn": ("true", bool),
    "reg_voxel_m": ("0.02", float),
    "reg_max_points": ("300000", int),
    "src_voxel_m": ("0.1", float),
    "src_max_points": ("50000", int),
    "icp_max_range_m": ("30.0", float),
}

# The node's RequestSync service (D39) and the NameObject service (D51). The node advertises
# RequestSync. NameObject is on the whitelist so the bridge will pass it once the wiring PR
# advertises it; until then the bridge has nothing by that name to offer.
REQUEST_SERVICE = "/room_memory/sync/request"
NAME_OBJECT_SERVICE = "/room_memory/objects/name"
# connectionGraph is read-only (topic names, no data). services carries the whitelisted names
# below; clientPublish is added only by foxglove_client_publish:=true.
BRIDGE_CAPABILITIES = ["connectionGraph", "services"]
BRIDGE_SERVICE_WHITELIST = ["^/room_memory/sync/request$", "^/room_memory/objects/name$"]

BRIDGE_ARGUMENTS = {
    "foxglove_bridge": "true",
    "foxglove_port": "8767",
    "foxglove_address": "0.0.0.0",
    "foxglove_client_publish": "false",
}


def generate_launch_description():
    parameters = {
        name: ParameterValue(LaunchConfiguration(name), value_type=kind) if kind else LaunchConfiguration(name)
        for name, (_, kind) in ARGUMENTS.items()
    }
    capabilities = PythonExpression([
        repr(["clientPublish"] + BRIDGE_CAPABILITIES), " if '", LaunchConfiguration("foxglove_client_publish"),
        "'.lower() == 'true' else ", repr(BRIDGE_CAPABILITIES),
    ])
    bridge = Node(
        package="foxglove_bridge", executable="foxglove_bridge", name="room_memory_bridge", output="screen",
        condition=IfCondition(LaunchConfiguration("foxglove_bridge")),
        on_exit=Shutdown(reason="room_memory_bridge exited"),
        parameters=[{
            "port": ParameterValue(LaunchConfiguration("foxglove_port"), value_type=int),
            "address": LaunchConfiguration("foxglove_address"),
            "topic_whitelist": ["^/room_memory/.*"],
            "client_topic_whitelist": ["^/room_memory/request_sync$"],
            "service_whitelist": BRIDGE_SERVICE_WHITELIST,
            "capabilities": ParameterValue(capabilities, value_type=List[str]),
        }],
    )
    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value=default) for name, (default, _) in ARGUMENTS.items()]
        + [DeclareLaunchArgument(name, default_value=default) for name, default in BRIDGE_ARGUMENTS.items()]
        + [Node(package="room_memory_cpp", executable="room_memory_node", name="room_memory", output="screen",
                parameters=[parameters], on_exit=Shutdown(reason="room_memory exited")),
           bridge]
    )
