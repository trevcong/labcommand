"""Launches the room_memory node. Every parameter can be overridden on the command line:

    ros2 launch room_memory room_memory.launch.py room_id:=lab sensors:="['mid360']"

``max_range`` <= 0 disables the range clip; an empty ``state_dir`` disables saving and loading;
an empty ``map_sensor`` publishes the JT128's reference when it is configured, else the first
sensor's. The defaults are ``MemoryConfig``'s, measured on the static recordings.
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
    "tolerance": ("2", int),
    "min_voxels": ("8", int),
    "min_extent": ("2", int),
    "window_s": ("1.0", float),
    "reference_windows": ("30", int),
    "recent_windows": ("3", int),
    "state_dir": ("~/room-memory", str),
    "occupied_min_hits": ("2", int),
    "occupied_min_ratio": ("0.3", float),
    "free_min_seen": ("2", int),
    "free_max_ratio": ("0.0", float),
    "candidate_windows": ("3", int),
    "expire_windows": ("3", int),
    "learn_max_offset_m": ("0.02", float),
    "near_range_m": ("2.0", float),
    "learn_windows": ("30", int),
    "map_sensor": ("", str),
    "delta_period_s": ("5.0", float),
    "keyframe_period_s": ("60.0", float),
    "keyframe_delta_ratio": ("0.5", float),
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
