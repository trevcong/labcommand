"""Read and write TUM trajectories: ``t x y z qx qy qz qw``."""

from __future__ import annotations

from pathlib import Path
from typing import Iterable, List, Sequence, Union

import numpy as np

PathLike = Union[str, Path]


def read_tum(path: PathLike) -> np.ndarray:
    """Return ``(N, 8)`` float64 rows sorted by time.

    Blank lines and ``#`` comments are ignored. A non-comment line that does
    not contain eight numbers raises ``ValueError``.
    """
    path = Path(path)
    rows: List[List[float]] = []
    with path.open("r", encoding="utf-8") as handle:
        for lineno, line in enumerate(handle, 1):
            text = line.strip()
            if not text or text.startswith("#"):
                continue
            parts = text.split()
            if len(parts) != 8:
                raise ValueError(
                    f"{path}:{lineno}: expected 8 TUM fields, found {len(parts)}"
                )
            try:
                rows.append([float(part) for part in parts])
            except ValueError as exc:
                raise ValueError(f"{path}:{lineno}: {exc}") from exc
    if not rows:
        return np.zeros((0, 8), dtype=np.float64)
    values = np.asarray(rows, dtype=np.float64)
    order = np.argsort(values[:, 0], kind="mergesort")
    return values[order]


def write_tum(path: PathLike, rows: Iterable[Sequence[float]]) -> None:
    """Write rows in the given order. The reader sorts them on the way back in."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = ["# t x y z qx qy qz qw"]
    for row in rows:
        if len(row) != 8:
            raise ValueError(f"TUM row must have 8 fields, found {len(row)}")
        lines.append(" ".join(format(float(value), ".17g") for value in row))
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
