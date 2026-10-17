"""Launches the room_memory node. Every parameter can be overridden on the command line:

    ros2 launch room_memory room_memory.launch.py room_id:=lab sensors:="['mid360']"

``max_range`` <= 0 disables the range clip; ``state_dir`` must already exist (it holds the
references, ``map_epoch`` and the publisher generation), and an empty or missing one stops the node;
an empty ``map_sensor`` publishes the JT128's reference when it is configured, else the first
sensor's. The defaults are ``MemoryConfig``'s, measured on the static recording.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
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
    "suppress_clear_windows": ("30", int),
    # The operator service changes durable state: off unless asked (D46).
    "operator_actions": ("false", bool),
}


def generate_launch_description():
    parameters = {
        name: ParameterValue(LaunchConfiguration(name), value_type=kind) if kind else LaunchConfiguration(name)
        for name, (_, kind) in ARGUMENTS.items()
    }
    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value=default) for name, (default, _) in ARGUMENTS.items()]
        + [Node(package="room_memory", executable="room_memory_node", name="room_memory", output="screen",
                parameters=[parameters])]
    )
