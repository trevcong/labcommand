"""Launches the C++ room_memory node and, by default, a read-only foxglove_bridge sidecar for the
Mac. Every parameter can be overridden on the command line:

    ros2 launch room_memory_cpp room_memory_cpp.launch.py room_id:=lab sensors:="['mid360']"

``max_range`` <= 0 disables the range clip; ``state_dir`` must already exist (it holds the
references, ``map_epoch`` and the publisher generation), and an empty or missing one stops the node;
an empty ``map_sensor`` publishes the JT128's reference when it is configured, else the first
sensor's. Same node parameters and defaults as ``room_memory.launch.py`` (the Python node), which
are ``MemoryConfig``'s, measured on the static recording.

The sidecar (``foxglove_bridge:=false`` turns it off) listens on ``foxglove_port`` (8767) and
relays only topics matching ``^/room_memory/.*``, so the raw clouds never leave through it; the
lidar containers' bridge on :8765 is not touched. It is read-only unless
``foxglove_client_publish:=true``: foxglove_bridge 3.4.1 parses ``client_topic_whitelist`` but
does not enforce it, so client publishing would let any client that reaches the port publish
on any topic.
"""

from typing import List

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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
}

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
    # connectionGraph is read-only (topic names, no data) and keeps the list non-empty, which
    # ROS 2 parameter files require.
    capabilities = PythonExpression([
        "['clientPublish', 'connectionGraph'] if '", LaunchConfiguration("foxglove_client_publish"),
        "'.lower() == 'true' else ['connectionGraph']",
    ])
    bridge = Node(
        package="foxglove_bridge", executable="foxglove_bridge", name="room_memory_bridge", output="screen",
        condition=IfCondition(LaunchConfiguration("foxglove_bridge")),
        parameters=[{
            "port": ParameterValue(LaunchConfiguration("foxglove_port"), value_type=int),
            "address": LaunchConfiguration("foxglove_address"),
            "topic_whitelist": ["^/room_memory/.*"],
            "client_topic_whitelist": ["^/room_memory/request_sync$"],
            "capabilities": ParameterValue(capabilities, value_type=List[str]),
        }],
    )
    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value=default) for name, (default, _) in ARGUMENTS.items()]
        + [DeclareLaunchArgument(name, default_value=default) for name, default in BRIDGE_ARGUMENTS.items()]
        + [Node(package="room_memory_cpp", executable="room_memory_node", name="room_memory", output="screen",
                parameters=[parameters]),
           bridge]
    )
