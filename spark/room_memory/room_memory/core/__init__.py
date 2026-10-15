"""Pure-Python room-memory core: PointCloud2 decoding and the v2 change-detection port.

No ROS imports. ``docs/CHANGE-DETECTION.md`` is the contract the port reproduces.
"""

from .change_detection import (
    ChangeDetectionError,
    Config,
    InvalidConfig,
    OccupancyMap,
    Region,
    Report,
    VoxelSizeMismatch,
    cast_rays,
    compare,
    diff,
    occupancy_map,
    to_json,
)
from .pointcloud2 import PointCloud, PointCloudError, decode_pointcloud2

__all__ = [
    "ChangeDetectionError",
    "Config",
    "InvalidConfig",
    "OccupancyMap",
    "PointCloud",
    "PointCloudError",
    "Region",
    "Report",
    "VoxelSizeMismatch",
    "cast_rays",
    "compare",
    "decode_pointcloud2",
    "diff",
    "occupancy_map",
    "to_json",
]
