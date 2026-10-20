"""Atomic JSON writes."""

from __future__ import annotations

import json
import os
from pathlib import Path


def write_json_atomic(path, obj):
    """Write UTF-8 JSON via a temp file in the same directory, then replace."""
    path = Path(path)
    payload = json.dumps(obj, indent=2, ensure_ascii=False, allow_nan=False) + "\n"
    temporary = path.with_name(path.name + ".tmp")
    try:
        temporary.write_text(payload, encoding="utf-8")
        os.replace(str(temporary), str(path))
    except Exception:
        if temporary.exists():
            try:
                temporary.unlink()
            except OSError:
                pass
        raise
