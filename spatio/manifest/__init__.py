"""Session manifest builder and validator.

Run from the repository root:

    python3 -m spatio.manifest build <session_dir> ...
    python3 -m spatio.manifest validate <session_dir> ...
"""

from __future__ import annotations

from spatio.manifest.build import build_manifest
from spatio.manifest.findings import (
    REPLAY_LABEL,
    Finding,
    ManifestBuildError,
    ManifestUnreadable,
)
from spatio.manifest.hashing import sha256_file
from spatio.manifest.physical import (
    placement_within_tolerance,
    validate_physical_reference,
)
from spatio.manifest.validate import validate_session

__all__ = [
    "REPLAY_LABEL",
    "Finding",
    "ManifestBuildError",
    "ManifestUnreadable",
    "build_manifest",
    "placement_within_tolerance",
    "sha256_file",
    "validate_physical_reference",
    "validate_session",
]
