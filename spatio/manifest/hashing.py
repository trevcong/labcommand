"""Streamed SHA-256. Callers must not load a bag or a map into memory."""

from __future__ import annotations

import hashlib

# One mebibyte. The large-file test asserts reads use this size.
CHUNK_BYTES = 1024 * 1024


def _open_rb(path):
    """Open a file for binary reads. Tests wrap this to count chunk sizes."""
    return open(path, "rb")


def hash_file(path, chunk_size=CHUNK_BYTES):
    """Return ``(lowercase hex sha256, bytes read)``.

    The file is read in fixed chunks. The byte count is the number of
    bytes hashed, not a separate stat, so the two cannot disagree.
    """
    if chunk_size <= 0:
        raise ValueError("chunk_size must be positive")
    digest = hashlib.sha256()
    total = 0
    with _open_rb(path) as handle:
        while True:
            block = handle.read(chunk_size)
            if not block:
                break
            digest.update(block)
            total += len(block)
    return digest.hexdigest(), total


def sha256_file(path, chunk_size=CHUNK_BYTES):
    """Return the lowercase hex SHA-256 of a file, streamed."""
    digest, _total = hash_file(path, chunk_size=chunk_size)
    return digest
