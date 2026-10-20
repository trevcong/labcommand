"""Canonical JSON for frozen specs.

The hash is the sha256 of these bytes. Key order in the input does not matter.
A trailing newline is part of the hash, so a file that gained or lost one is
a different spec.
"""

from __future__ import annotations

import hashlib
import json


def canonical_bytes(document):
    text = json.dumps(
        document,
        sort_keys=True,
        indent=2,
        ensure_ascii=False,
        allow_nan=False,
    )
    return (text + "\n").encode("utf-8")


def sha256_hex(data):
    return hashlib.sha256(data).hexdigest()


def sidecar_path(spec_path):
    return "{0}.sha256".format(spec_path)
