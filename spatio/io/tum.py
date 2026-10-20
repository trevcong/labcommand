"""Strict TUM trajectory reader and writer.

A pose line is ``t x y z qx qy qz qw``. ``t`` is seconds, position is metres
in the map frame, and the quaternion is xyzw. Lines that are empty or start
with ``#`` are ignored. Nothing is sorted, deduplicated, or interpolated.
"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

import numpy as np

_QUAT_TOL = 1e-3


class TumFormatError(ValueError):
    """A trajectory file broke the strict TUM rules.

    ``line_number`` is 1-based and counts every physical line, including
    comments and blank lines.
    """

    def __init__(self, line_number: int, message: str) -> None:
        self.line_number = int(line_number)
        super().__init__("line %d: %s" % (self.line_number, message))


@dataclass(frozen=True, eq=False)
class Trajectory:
    """One pose per row, in file order.

    ``t`` is shape (N,), ``xyz`` is (N, 3), ``quat_xyzw`` is (N, 4), all
    float64. The arrays are copies. The dataclass is frozen, but the arrays
    themselves remain writable.
    """

    t: np.ndarray
    xyz: np.ndarray
    quat_xyzw: np.ndarray
    source: str

    def __post_init__(self) -> None:
        t = np.array(self.t, dtype=np.float64, copy=True)
        xyz = np.array(self.xyz, dtype=np.float64, copy=True)
        quat = np.array(self.quat_xyzw, dtype=np.float64, copy=True)
        n = int(t.shape[0]) if t.ndim == 1 else -1
        if t.ndim != 1 or xyz.shape != (n, 3) or quat.shape != (n, 4):
            raise ValueError(
                "trajectory arrays must be t (N,), xyz (N, 3), quat_xyzw (N, 4)"
            )
        object.__setattr__(self, "t", t)
        object.__setattr__(self, "xyz", xyz)
        object.__setattr__(self, "quat_xyzw", quat)
        object.__setattr__(self, "source", os.fspath(self.source))

    def __len__(self) -> int:
        return int(self.t.shape[0])


@dataclass(frozen=True)
class Issue:
    """One finding from :func:`check_trajectory`. The trajectory is unchanged.

    ``index`` is the later pose of the pair (0-based). ``dt`` is the signed
    time step for ``BACKWARDS``. ``gap_s`` is the forward step for ``GAP``.
    ``TOO_SHORT`` carries neither.
    """

    code: str
    index: Optional[int] = None
    dt: Optional[float] = None
    gap_s: Optional[float] = None


def read_tum(path: str, normalize_quat: bool = False) -> Trajectory:
    """Read a TUM file. Raise :class:`TumFormatError` on the first bad line.

    Each data line must have exactly eight finite numeric columns. The
    quaternion norm must be within 1e-3 of 1. When ``normalize_quat`` is
    true, a finite non-zero quaternion is divided by its norm instead.
    """

    rows = []
    with open(path, "rb") as handle:
        for line_number, raw in enumerate(handle, start=1):
            try:
                text = raw.decode("utf-8")
            except UnicodeDecodeError as exc:
                raise TumFormatError(line_number, "line is not valid UTF-8") from exc
            text = text.strip()
            if not text or text.startswith("#"):
                continue
            parts = text.split()
            if len(parts) != 8:
                raise TumFormatError(
                    line_number, "expected 8 columns, found %d" % len(parts)
                )
            try:
                vals = [float(part) for part in parts]
            except ValueError as exc:
                raise TumFormatError(line_number, "non-numeric column") from exc
            if not all(math.isfinite(val) for val in vals):
                raise TumFormatError(line_number, "non-finite value")
            norm = math.sqrt(sum(val * val for val in vals[4:8]))
            if not math.isfinite(norm) or norm == 0.0:
                raise TumFormatError(
                    line_number, "quaternion norm %.8g is not within 1e-3 of 1" % norm
                )
            if abs(norm - 1.0) > _QUAT_TOL:
                if not normalize_quat:
                    raise TumFormatError(
                        line_number,
                        "quaternion norm %.8g is not within 1e-3 of 1" % norm,
                    )
                vals[4:8] = [val / norm for val in vals[4:8]]
            elif normalize_quat:
                vals[4:8] = [val / norm for val in vals[4:8]]
            rows.append(vals)
    if rows:
        data = np.asarray(rows, dtype=np.float64)
        t = data[:, 0]
        xyz = data[:, 1:4]
        quat = data[:, 4:8]
    else:
        t = np.zeros((0,), dtype=np.float64)
        xyz = np.zeros((0, 3), dtype=np.float64)
        quat = np.zeros((0, 4), dtype=np.float64)
    return Trajectory(t, xyz, quat, os.fspath(path))


def write_tum(path: str, traj: Trajectory, comments: Sequence[str] = ()) -> None:
    """Write ``traj`` with ``%.9f`` on every column.

    Nine digits after the decimal keep a round trip inside 1e-9 s and
    1e-9 m. The original ``%.9g`` position format does not: on coordinates
    in [1, 10) its absolute error reaches 5e-9 m. Comments are written
    first, one ``#`` line each. The file uses LF newlines.
    """

    if len(traj.t) != len(traj.xyz) or len(traj.t) != len(traj.quat_xyzw):
        raise ValueError("trajectory fields have different lengths")
    if not (
        np.isfinite(traj.t).all()
        and np.isfinite(traj.xyz).all()
        and np.isfinite(traj.quat_xyzw).all()
    ):
        raise ValueError("trajectory values must be finite")
    lines = []
    for comment in comments:
        lines.extend(_comment_lines(comment))
    for i in range(len(traj)):
        t = float(traj.t[i])
        x, y, z = (float(v) for v in traj.xyz[i])
        qx, qy, qz, qw = (float(v) for v in traj.quat_xyzw[i])
        lines.append(
            "%.9f %.9f %.9f %.9f %.9f %.9f %.9f %.9f" % (t, x, y, z, qx, qy, qz, qw)
        )
    payload = ("\n".join(lines) + ("\n" if lines else ""))
    with open(path, "w", encoding="utf-8", newline="\n") as handle:
        handle.write(payload)


def check_trajectory(traj: Trajectory, max_gap_s: float = 0.5) -> list:
    """Report timestamp problems. This does not change ``traj``.

    A step is ``BACKWARDS`` when dt < 0, ``DUPLICATE_T`` when dt == 0, and
    ``GAP`` when dt > ``max_gap_s``. Fewer than two poses is ``TOO_SHORT``
    and nothing else. ``index`` refers to the later pose.
    """

    if max_gap_s < 0 or not math.isfinite(float(max_gap_s)):
        raise ValueError("max_gap_s must be a finite number >= 0")
    n = len(traj)
    if n < 2:
        return [Issue("TOO_SHORT")]
    issues = []
    # Pairwise, in file order. One pair produces at most one issue.
    for i in range(1, n):
        dt = float(traj.t[i] - traj.t[i - 1])
        if dt < 0.0:
            issues.append(Issue("BACKWARDS", index=i, dt=dt))
        elif dt == 0.0:
            issues.append(Issue("DUPLICATE_T", index=i))
        elif dt > float(max_gap_s):
            issues.append(Issue("GAP", index=i, gap_s=dt))
    return issues


def _comment_lines(comment: str) -> Tuple[str, ...]:
    text = str(comment).replace("\r\n", "\n").replace("\r", "\n")
    lines = []
    for part in text.split("\n"):
        part = part.strip()
        if not part:
            continue
        if not part.startswith("#"):
            part = "# " + part
        lines.append(part)
    return tuple(lines)
