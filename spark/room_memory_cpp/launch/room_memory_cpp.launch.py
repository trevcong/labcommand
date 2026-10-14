"""Launches the C++ room_memory node. Every parameter can be overridden on the command line:

    ros2 launch room_memory_cpp room_memory_cpp.launch.py room_id:=lab sensors:="['mid360']"

``max_range`` <= 0 disables the range clip; an empty ``state_dir`` disables saving and loading.
Same parameters, defaults, node name and topics as ``room_memory.launch.py`` (the Python node).
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
    "reference_windows": ("5", int),
    "recent_windows": ("1", int),
    "state_dir": ("~/room-memory", str),
}


def generate_launch_description():
    parameters = {
        name: ParameterValue(LaunchConfiguration(name), value_type=kind) if kind else LaunchConfiguration(name)
        for name, (_, kind) in ARGUMENTS.items()
    }
    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value=default) for name, (default, _) in ARGUMENTS.items()]
        + [Node(package="room_memory_cpp", executable="room_memory_node", name="room_memory", output="screen",
                parameters=[parameters])]
    )
