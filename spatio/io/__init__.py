"""Reference readers and writers for Spatio trajectory.tum and map.pcd files.

Other workstreams keep their own loaders. This package is the checked
reference for the shared session-file contract. ``spatio`` itself is a
namespace package and has no ``__init__.py``.
"""

from spatio.io.pcd import (
    PcdFormatError,
    PcdHeader,
    read_pcd,
    read_pcd_header,
    write_pcd,
)
from spatio.io.synth import room_map, square_loop, stationary, with_gap, with_reset
from spatio.io.tum import (
    Issue,
    Trajectory,
    TumFormatError,
    check_trajectory,
    read_tum,
    write_tum,
)

__all__ = [
    "Issue",
    "PcdFormatError",
    "PcdHeader",
    "Trajectory",
    "TumFormatError",
    "check_trajectory",
    "read_pcd",
    "read_pcd_header",
    "read_tum",
    "room_map",
    "square_loop",
    "stationary",
    "with_gap",
    "with_reset",
    "write_pcd",
    "write_tum",
]
