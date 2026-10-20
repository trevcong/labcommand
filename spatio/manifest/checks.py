"""Small type and time checks shared by the builder and the validator."""

from __future__ import annotations

import json
import math
import re
from datetime import datetime
from pathlib import Path

from spatio.manifest.findings import ManifestUnreadable


# 64 lowercase hex characters. Uppercase is rejected on purpose.
SHA256_RE = re.compile(r"^[0-9a-f]{64}$")

# ISO 8601 with a literal T and an explicit offset (Z or ±HH:MM).
# Python 3.9's fromisoformat does not accept Z, so Z is rewritten first.
_ISO_RE = re.compile(
    r"^(?P<body>\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}(?:\.\d+)?)"
    r"(?P<offset>Z|[+-]\d{2}:\d{2})$"
)


def is_bool(value):
    return isinstance(value, bool)


def is_int(value):
    # bool is a subclass of int. A JSON true must not pass as a byte count.
    return isinstance(value, int) and not isinstance(value, bool)


def is_finite_number(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return False
    return math.isfinite(value)


def is_sha256_hex(value):
    return isinstance(value, str) and SHA256_RE.match(value) is not None


def parse_iso8601_with_offset(value):
    """Return an aware datetime, or None when the text has no offset."""
    if not isinstance(value, str):
        return None
    match = _ISO_RE.match(value)
    if match is None:
        return None
    body = match.group("body")
    offset = match.group("offset")
    text = body + ("+00:00" if offset == "Z" else offset)
    try:
        parsed = datetime.fromisoformat(text)
    except ValueError:
        return None
    if parsed.tzinfo is None:
        return None
    return parsed


def read_json_object(path):
    """Read a UTF-8 JSON object.

    Raises ManifestUnreadable when the file cannot be opened or is not a
    JSON object. Callers that can still validate the rest of a session
    should catch this and turn it into a finding.
    """
    path = Path(path)
    try:
        raw = path.read_bytes()
    except OSError as exc:
        raise ManifestUnreadable("%s is unreadable: %s" % (path, exc))
    try:
        text = raw.decode("utf-8-sig")
    except UnicodeDecodeError as exc:
        raise ManifestUnreadable("%s is not UTF-8: %s" % (path, exc))
    try:
        data = json.loads(text)
    except json.JSONDecodeError as exc:
        raise ManifestUnreadable("%s is not valid JSON: %s" % (path, exc))
    if not isinstance(data, dict):
        raise ManifestUnreadable("%s must be a JSON object" % path)
    return data
