"""Puts ``acceptance`` and ``room_memory`` on ``sys.path`` for plain pytest runs."""

import pathlib
import sys

ROOM_MEMORY_ROOT = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOM_MEMORY_ROOT))
