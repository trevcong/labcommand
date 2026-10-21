"""Small JSON helpers. Numpy values are turned into plain Python numbers."""

from __future__ import annotations

import os

import numpy as np

from spatio.manifest.jsonio import write_json_atomic


def to_builtin(obj):
    """Return a JSON-safe copy of obj. Non-finite floats become null."""
    if isinstance(obj, dict):
        return {str(key): to_builtin(value) for key, value in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [to_builtin(value) for value in obj]
    if isinstance(obj, np.ndarray):
        return to_builtin(obj.tolist())
    if isinstance(obj, np.floating):
        value = float(obj)
        if not np.isfinite(value):
            return None
        return value
    if isinstance(obj, np.integer):
        return int(obj)
    if isinstance(obj, np.bool_):
        return bool(obj)
    if isinstance(obj, float):
        if not np.isfinite(obj):
            return None
        return obj
    return obj


def write_json(path, doc):
    """Write UTF-8 JSON with a trailing newline. Creates parent directories."""
    parent = os.path.dirname(os.path.abspath(path))
    if parent and not os.path.isdir(parent):
        os.makedirs(parent)
    write_json_atomic(path, to_builtin(doc))
