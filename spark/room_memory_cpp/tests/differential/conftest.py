"""Differential tests: the C++ binaries against the Python oracle in spark/room_memory.

CTest sets ROOM_MEMORY_OFFLINE, ROOM_MEMORY_REPLAY and ROOM_MEMORY_PYTHON_PACKAGE; the oracle
and its test scenes are imported from that package, unchanged.
"""

import os
import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent
PACKAGE = pathlib.Path(os.environ.get("ROOM_MEMORY_PYTHON_PACKAGE", HERE.parents[2] / "room_memory"))
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(PACKAGE / "tests"))
sys.path.insert(0, str(PACKAGE))
