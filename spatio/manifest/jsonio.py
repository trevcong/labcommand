"""Atomic JSON and text writes."""

from __future__ import annotations

import json
import os
import tempfile
from pathlib import Path


def _fsync_directory(directory):
    """Make a rename in ``directory`` durable. POSIX only; elsewhere there is no directory fd."""
    if os.name != "posix":
        return
    fd = os.open(str(directory), os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def _write_atomic(path, write):
    """Call ``write(handle)`` on a temp file in the same directory, fsync, replace, fsync the directory."""
    path = Path(path)
    fd, name = tempfile.mkstemp(prefix=path.name + ".", suffix=".tmp", dir=path.parent)
    temporary = Path(name)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as handle:
            # mkstemp creates 0600; give the result the mode write_text would (0666 & ~umask).
            umask = os.umask(0)
            os.umask(umask)
            os.fchmod(handle.fileno(), 0o666 & ~umask)
            write(handle)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(str(temporary), str(path))
    finally:
        temporary.unlink(missing_ok=True)
    _fsync_directory(path.parent)


def write_json_atomic(path, obj):
    """Write UTF-8 JSON via a temp file in the same directory, then replace."""
    def write(handle):
        json.dump(obj, handle, indent=2, ensure_ascii=False, allow_nan=False)
        handle.write("\n")

    _write_atomic(path, write)


def write_text_atomic(path, text):
    """Write UTF-8 text via a temp file in the same directory, then replace."""
    _write_atomic(path, lambda handle: handle.write(text))
