"""Atomic JSON writes."""

from __future__ import annotations

import json
import os
import tempfile
from pathlib import Path


def write_json_atomic(path, obj):
    """Write UTF-8 JSON via a temp file in the same directory, then replace."""
    path = Path(path)
    fd, name = tempfile.mkstemp(prefix=path.name + ".", suffix=".tmp", dir=path.parent)
    temporary = Path(name)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as handle:
            json.dump(obj, handle, indent=2, ensure_ascii=False, allow_nan=False)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(str(temporary), str(path))
    finally:
        temporary.unlink(missing_ok=True)
