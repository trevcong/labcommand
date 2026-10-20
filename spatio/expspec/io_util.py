"""Small file helpers. Writes are atomic so a failed freeze leaves no half file."""

from __future__ import annotations

import json
import os
from pathlib import Path


def read_json_object(path):
    """Return (object, raw_bytes) or raise ValueError / OSError."""
    path = Path(path)
    raw = path.read_bytes()
    try:
        text = raw.decode("utf-8")
    except UnicodeError as exc:
        raise ValueError("not utf-8: {0}".format(exc))
    try:
        value = json.loads(text)
    except json.JSONDecodeError as exc:
        raise ValueError("invalid JSON: {0}".format(exc))
    if not isinstance(value, dict):
        raise ValueError("JSON value must be an object")
    return value, raw


def write_bytes_atomic(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(".{0}.{1}.tmp".format(path.name, os.getpid()))
    try:
        with open(temporary, "wb") as handle:
            handle.write(data)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    finally:
        if temporary.exists():
            temporary.unlink()
