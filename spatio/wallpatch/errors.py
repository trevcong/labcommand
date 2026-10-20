"""Failures a caller can report. The CLI turns WallpatchError into exit code 1."""

from __future__ import annotations


class WallpatchError(Exception):
    """The map or the patch file cannot be measured as given."""


class MapShaMismatch(WallpatchError):
    """Patch boxes were chosen on a different map file.

    A box is valid only in the session whose map it was drawn on.
    """

    def __init__(self, expected, actual):
        # type: (str, str) -> None
        self.expected = expected
        self.actual = actual
        super(MapShaMismatch, self).__init__(
            "patch file map_sha256 does not match this map "
            "(expected %s, actual %s). Patch boxes are per-session; "
            "a box chosen on another map is not valid here."
            % (expected, actual)
        )
