"""Freeze a spec: canonical bytes plus a sidecar hash.

record and runsheet trust the sidecar. Editing the JSON without running
freeze again makes the sidecar disagree, and those commands refuse.
"""

from __future__ import annotations

from pathlib import Path

from spatio.expspec.canonical import canonical_bytes, sha256_hex, sidecar_path
from spatio.expspec.io_util import read_json_object, write_bytes_atomic
from spatio.expspec.issues import errors_of
from spatio.expspec.validate import validate


class FreezeError(Exception):
    def __init__(self, issues):
        self.issues = list(issues)
        super(FreezeError, self).__init__(self._text())

    def _text(self):
        return "\n".join(item.format() for item in errors_of(self.issues))


class FrozenSpecError(Exception):
    """The file is missing, unreadable, or no longer the frozen bytes."""


def freeze_spec(spec, out_path):
    """Write canonical JSON and the sidecar. Return (sha256, issues).

    Raises FreezeError and writes nothing when validation reports an error.
    Warnings are returned and do not block the write.
    """
    issues = validate(spec)
    if errors_of(issues):
        raise FreezeError(issues)
    data = canonical_bytes(spec)
    digest = sha256_hex(data)
    out_path = Path(out_path)
    write_bytes_atomic(out_path, data)
    write_bytes_atomic(Path(sidecar_path(out_path)), (digest + "\n").encode("ascii"))
    return digest, issues


def load_frozen(spec_path):
    """Load a frozen spec whose bytes still match the sidecar.

    Returns (spec, sha256). Raises FrozenSpecError if the file changed
    after freezing, was never frozen, or no longer validates.
    """
    spec_path = Path(spec_path)
    try:
        spec, raw = read_json_object(spec_path)
    except (OSError, ValueError) as exc:
        raise FrozenSpecError(str(exc))
    digest = sha256_hex(raw)
    side = Path(sidecar_path(spec_path))
    if not side.is_file():
        raise FrozenSpecError(
            "spec was not frozen (missing sha256 sidecar); refuse to continue"
        )
    try:
        expected = side.read_text(encoding="ascii").strip()
    except (OSError, UnicodeError) as exc:
        raise FrozenSpecError("unreadable freeze hash: {0}".format(exc))
    if expected != digest:
        raise FrozenSpecError(
            "spec file changed after freezing (sha256 {0} != frozen {1})".format(digest, expected)
        )
    problems = errors_of(validate(spec))
    if problems:
        raise FrozenSpecError(
            "frozen spec no longer validates: {0}".format(problems[0].format())
        )
    return spec, digest
