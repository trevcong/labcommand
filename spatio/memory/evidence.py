"""Spatio memory v2: the core evidence store (job V1).

See docs/MEMORY-V2-DESIGN.md. Python 3.9 and the stdlib only. Built here, under
the §5 decisions:

- §2.1: one ``transactions`` row per write, ``recorded_txn`` on every row,
  queries at ``known_at``, the append-only look-again lifecycle;
- §2.2: operator-only retractions, with transitive invalidation computed at read
  time (one retraction row per retraction, never a cascade row);
- §2.3: trajectory and map revisions with parent links, active revision, current frame;
- §2.5: ``compare_positions`` rule 2 only, ``MOVED_SIGMA_K``;
- §2.6: immutable rows, the camera purge with its tombstones and blanked reason.

Not built (V2/V3): alignments, absence refutation, status words, reports, body
pose derivation, v1 migration, ingest. Queries take ``now`` (sensor time) and
``known_at`` (a transaction id) and never read a clock.
"""

from __future__ import annotations

import hashlib
import json
import math
import sqlite3
from contextlib import contextmanager
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Dict, Iterator, List, Optional, Sequence, Tuple, Union

__all__ = [
    "MOVED_SIGMA_K",
    "RETENTION_S",
    "SCHEMA_VERSION",
    "TARGET_KINDS",
    "Comparison",
    "EvidenceStore",
    "ValidationError",
    "canonical_json",
    "comparison_sigma",
    "is_moved",
]

SCHEMA_VERSION = 2
# §5 decision 3: "moved" needs distance_m > MOVED_SIGMA_K * sigma_m.
MOVED_SIGMA_K = 3
# v1 DESIGN §8 (store.py at 90ae03e): a camera row expires at acquired_at + RETENTION_S,
# compared with the purge's wall time. Lidar rows never expire.
RETENTION_S = 30 * 86400.0

# §2.2 without alignments (out of V1). Retraction is operator-only for each (§5 decision 4).
TARGET_KINDS = ("interpretation", "observation", "trajectory_revision", "session")

# kind -> (table, primary key). Every kind whose validity can be asked.
_TABLES = {
    "session": ("sessions", "session_id"),
    "trajectory_revision": ("trajectory_revisions", "trajectory_revision_id"),
    "map_revision": ("map_revisions", "map_revision_id"),
    "observation": ("observations", "observation_id"),
    "interpretation": ("interpretations", "interpretation_id"),
}
_HASHABLE = {
    "sessions": "session_id",
    "trajectory_revisions": "trajectory_revision_id",
    "map_revisions": "map_revision_id",
    "observations": "observation_id",
    "interpretations": "interpretation_id",
    "retractions": "retraction_id",
    "look_again_requests": "request_id",
    "look_again_closures": "request_id",
}
# When two observations are not comparable for more than one reason, the first wins.
# no_map_revision outranks superseded_revision (V2-4e at K3).
_REASON_ORDER = ("source_retracted", "no_map_revision", "superseded_revision")
_KNOWLEDGE_BASES = ("live", "migrated")
_GUARD_OPEN = "(SELECT count(*) FROM purge_guard) = 1"


class ValidationError(ValueError):
    """A write or query the design refuses. A refused write leaves no row behind."""


def _sha_check(column: str) -> str:
    return f"CHECK (length({column}) = 64 AND {column} NOT GLOB '*[^0-9a-f]*')"


def _immutable(table: str, update: bool = True, delete: bool = True) -> str:
    out = ""
    if update:
        out += f"""
CREATE TRIGGER {table}_no_update BEFORE UPDATE ON {table}
BEGIN
    SELECT RAISE(ABORT, '{table} are immutable');
END;"""
    if delete:
        out += f"""
CREATE TRIGGER {table}_no_delete BEFORE DELETE ON {table}
BEGIN
    SELECT RAISE(ABORT, '{table} are immutable');
END;"""
    return out


def _schema_script() -> str:
    retention = format(RETENTION_S, ".12g")
    txn = "recorded_txn INTEGER NOT NULL REFERENCES transactions(txn_id)"
    code = "method TEXT NOT NULL, method_version TEXT NOT NULL, params_sha256 TEXT NOT NULL"
    plain = (
        "transactions",
        "sessions",
        "trajectory_revisions",
        "map_revisions",
        "look_again_requests",
        "look_again_closures",
        "purged_observations",
        "purged_associations",
    )
    # purge_guard is internal. Only purge_expired holds its one row, inside its own
    # transaction, so a crash rolls the guard back with everything else.
    return f"""
BEGIN;
CREATE TABLE transactions (
    txn_id INTEGER PRIMARY KEY AUTOINCREMENT,
    recorded_at REAL NOT NULL,
    writer TEXT NOT NULL CHECK (length(writer) >= 1),
    operation TEXT NOT NULL CHECK (length(operation) >= 1),
    knowledge_basis TEXT NOT NULL CHECK (knowledge_basis IN ('live', 'migrated'))
) STRICT;

CREATE TABLE sessions (
    session_id TEXT PRIMARY KEY CHECK (length(session_id) BETWEEN 1 AND 128),
    t_start REAL NOT NULL,
    t_end REAL NOT NULL,
    source_recording_sha256 TEXT NOT NULL {_sha_check("source_recording_sha256")},
    {txn},
    CHECK (t_start <= t_end)
) STRICT;

CREATE TABLE trajectory_revisions (
    trajectory_revision_id INTEGER PRIMARY KEY AUTOINCREMENT,
    session_id TEXT NOT NULL REFERENCES sessions(session_id),
    revision INTEGER NOT NULL CHECK (revision >= 1),
    tum_sha256 TEXT NOT NULL {_sha_check("tum_sha256")},
    -- NULL: rebuilt from the raw recording, not from a parent's poses (§2.3).
    parent_revision_id INTEGER REFERENCES trajectory_revisions(trajectory_revision_id),
    {code} {_sha_check("params_sha256")},
    reason TEXT NOT NULL CHECK (length(reason) >= 1),
    {txn},
    UNIQUE (session_id, revision)
) STRICT;

CREATE TABLE map_revisions (
    map_revision_id INTEGER PRIMARY KEY AUTOINCREMENT,
    map_id TEXT NOT NULL CHECK (length(map_id) BETWEEN 1 AND 128),
    revision INTEGER NOT NULL CHECK (revision >= 1),
    map_pcd_sha256 TEXT NOT NULL {_sha_check("map_pcd_sha256")},
    trajectory_revision_id INTEGER NOT NULL REFERENCES trajectory_revisions(trajectory_revision_id),
    built_by TEXT NOT NULL CHECK (length(built_by) >= 1),
    {code} {_sha_check("params_sha256")},
    {txn},
    UNIQUE (map_id, revision)
) STRICT;

CREATE TABLE observations (
    observation_id INTEGER PRIMARY KEY AUTOINCREMENT,
    session_id TEXT NOT NULL REFERENCES sessions(session_id),
    kind TEXT NOT NULL CHECK (kind IN ('detection', 'absence')),
    sensor_kind TEXT NOT NULL CHECK (sensor_kind IN ('camera', 'lidar')),
    geometry_frame TEXT NOT NULL CHECK (geometry_frame IN ('body', 'map')),
    map_revision_id INTEGER REFERENCES map_revisions(map_revision_id),
    acquired_at REAL NOT NULL,
    aabb_min_x REAL NOT NULL,
    aabb_min_y REAL NOT NULL,
    aabb_min_z REAL NOT NULL,
    aabb_max_x REAL NOT NULL,
    aabb_max_y REAL NOT NULL,
    aabb_max_z REAL NOT NULL,
    sigma_m REAL NOT NULL CHECK (sigma_m >= 0),
    coverage REAL CHECK (coverage IS NULL OR (coverage >= 0 AND coverage <= 1)),
    measurement_confidence REAL NOT NULL
        CHECK (measurement_confidence >= 0 AND measurement_confidence <= 1),
    expires_at REAL,
    derived_from TEXT CHECK (derived_from IS NULL OR length(derived_from) >= 1),
    method TEXT,
    method_version TEXT,
    params_sha256 TEXT CHECK (params_sha256 IS NULL OR
        (length(params_sha256) = 64 AND params_sha256 NOT GLOB '*[^0-9a-f]*')),
    {txn},
    CHECK ((geometry_frame = 'map') = (map_revision_id IS NOT NULL)),
    -- Absences are map-frame only (§2.5).
    CHECK (kind = 'detection' OR geometry_frame = 'map'),
    CHECK ((kind = 'absence') = (coverage IS NOT NULL)),
    CHECK (aabb_min_x <= aabb_max_x AND aabb_min_y <= aabb_max_y AND aabb_min_z <= aabb_max_z),
    CHECK (
        (sensor_kind = 'camera' AND expires_at = acquired_at + {retention})
        OR (sensor_kind = 'lidar' AND expires_at IS NULL)
    )
) STRICT;

CREATE TABLE interpretations (
    interpretation_id INTEGER PRIMARY KEY AUTOINCREMENT,
    observation_id INTEGER NOT NULL REFERENCES observations(observation_id),
    object_id TEXT NOT NULL CHECK (length(object_id) BETWEEN 1 AND 128),
    label TEXT NOT NULL CHECK (length(label) >= 1),
    confidence REAL NOT NULL CHECK (confidence >= 0 AND confidence <= 1),
    source_kind TEXT NOT NULL CHECK (source_kind IN ('model', 'operator')),
    writer TEXT NOT NULL CHECK (length(writer) >= 1),
    {txn},
    CHECK ((source_kind = 'model') = (writer GLOB 'model:*'))
) STRICT;

CREATE TABLE retractions (
    retraction_id INTEGER PRIMARY KEY AUTOINCREMENT,
    target_kind TEXT NOT NULL
        CHECK (target_kind IN ('interpretation', 'observation', 'trajectory_revision', 'session')),
    target_id TEXT NOT NULL,
    {txn},
    -- NULL only after the purge deleted the target (§2.6, §5 decision 5).
    reason TEXT CHECK (reason IS NULL OR length(reason) >= 1),
    retracted_by TEXT NOT NULL CHECK (length(retracted_by) >= 1 AND retracted_by NOT GLOB 'model:*'),
    UNIQUE (target_kind, target_id)
) STRICT;

CREATE TABLE look_again_requests (
    request_id INTEGER PRIMARY KEY AUTOINCREMENT,
    object_id TEXT NOT NULL CHECK (length(object_id) BETWEEN 1 AND 128),
    {txn}
) STRICT;

CREATE TABLE look_again_closures (
    request_id INTEGER PRIMARY KEY REFERENCES look_again_requests(request_id),
    outcome TEXT NOT NULL CHECK (outcome IN ('fulfilled', 'cancelled')),
    -- A plain id, no foreign key: the purge never touches it (§2.1).
    fulfilled_by_observation_id INTEGER,
    {txn},
    CHECK ((outcome = 'fulfilled') = (fulfilled_by_observation_id IS NOT NULL))
) STRICT;

-- §2.6 tombstones: ids and transaction numbers only. No label, confidence,
-- thumbnail or geometry.
CREATE TABLE purged_observations (
    observation_id INTEGER PRIMARY KEY,
    kind TEXT NOT NULL,
    session_id TEXT NOT NULL,
    sensor_kind TEXT NOT NULL,
    acquired_at REAL NOT NULL,
    recorded_txn INTEGER NOT NULL,
    purged_txn INTEGER NOT NULL REFERENCES transactions(txn_id),
    row_sha256 TEXT NOT NULL {_sha_check("row_sha256")}
) STRICT;

CREATE TABLE purged_associations (
    interpretation_id INTEGER PRIMARY KEY,
    observation_id INTEGER NOT NULL,
    object_id TEXT NOT NULL,
    recorded_txn INTEGER NOT NULL,
    purged_txn INTEGER NOT NULL REFERENCES transactions(txn_id)
) STRICT;

CREATE TABLE purge_guard (
    id INTEGER PRIMARY KEY CHECK (id = 1)
) STRICT;

CREATE INDEX trajectory_revisions_session ON trajectory_revisions(session_id);
CREATE INDEX map_revisions_trajectory ON map_revisions(trajectory_revision_id);
CREATE INDEX interpretations_object ON interpretations(object_id);
CREATE INDEX interpretations_observation ON interpretations(observation_id);
CREATE INDEX look_again_requests_object ON look_again_requests(object_id);
CREATE INDEX purged_associations_object ON purged_associations(object_id);
{"".join(_immutable(t) for t in plain)}
{_immutable("observations", delete=False)}
{_immutable("interpretations", delete=False)}
{_immutable("retractions", update=False)}

-- The only deletes: purge_expired, camera rows only, each after its tombstone.
CREATE TRIGGER observations_delete_only_in_purge BEFORE DELETE ON observations
WHEN NOT (
    {_GUARD_OPEN}
    AND OLD.sensor_kind = 'camera'
    AND EXISTS (SELECT 1 FROM purged_observations AS p WHERE p.observation_id = OLD.observation_id)
)
BEGIN
    SELECT RAISE(ABORT, 'only purge_expired deletes observations, and only camera rows');
END;

CREATE TRIGGER interpretations_delete_only_in_purge BEFORE DELETE ON interpretations
WHEN NOT (
    {_GUARD_OPEN}
    AND (SELECT o.sensor_kind FROM observations AS o
         WHERE o.observation_id = OLD.observation_id) = 'camera'
    AND EXISTS (SELECT 1 FROM purged_associations AS p
                WHERE p.interpretation_id = OLD.interpretation_id)
)
BEGIN
    SELECT RAISE(ABORT, 'only purge_expired deletes interpretations, and only of camera rows');
END;

-- The only update ever made to a retraction: the purge blanks reason (§2.6).
CREATE TRIGGER retractions_only_reason_blanked_in_purge BEFORE UPDATE ON retractions
WHEN NOT (
    {_GUARD_OPEN}
    AND NEW.reason IS NULL
    AND NEW.retraction_id IS OLD.retraction_id
    AND NEW.target_kind IS OLD.target_kind
    AND NEW.target_id IS OLD.target_id
    AND NEW.recorded_txn IS OLD.recorded_txn
    AND NEW.retracted_by IS OLD.retracted_by
)
BEGIN
    SELECT RAISE(ABORT, 'retractions are immutable; only the purge blanks reason');
END;

CREATE TRIGGER retractions_need_reason BEFORE INSERT ON retractions
WHEN NEW.reason IS NULL
BEGIN
    SELECT RAISE(ABORT, 'a retraction needs a reason');
END;

PRAGMA user_version = {SCHEMA_VERSION};
COMMIT;
"""


def canonical_json(value: object) -> str:
    """The byte form two answers are compared in (V2-3)."""
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False)


def comparison_sigma(sigma_a: float, sigma_b: float) -> float:
    """``sigma_m`` of a rule-2 comparison (§2.5): the measurement sigmas in quadrature.

    A ``body`` observation's pose adds 0 until a pose sigma is recorded (§5 decision 3).
    """
    return math.sqrt(sigma_a ** 2 + sigma_b ** 2)


def is_moved(distance_m: float, sigma_m: float) -> bool:
    """The §2.5 predicate a future detector must pass to call something moved."""
    return distance_m > MOVED_SIGMA_K * sigma_m


@dataclass(frozen=True)
class Comparison:
    """``compare_positions`` result. ``distance_m`` is None whenever ``comparable`` is False."""

    comparable: bool
    reason: Optional[str]
    frame: Optional[int]
    distance_m: Optional[float]
    sigma_m: Optional[float]
    revisions_used: Dict[str, List[int]] = field(default_factory=dict)


def _not_comparable(reason: str) -> Comparison:
    return Comparison(False, reason, None, None, None, {})


def _is_model(writer: str) -> bool:
    return writer.startswith("model:")


def _text(value: object, name: str) -> str:
    if not isinstance(value, str) or not value:
        raise ValidationError(f"{name} must be a non-empty string")
    return value


def _finite(value: object, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValidationError(f"{name} must be a finite number")
    return float(value)


def _row_id(value: object, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValidationError(f"{name} must be an integer id")
    return value


def _sha(value: object, name: str) -> str:
    text = _text(value, name)
    if len(text) != 64 or any(c not in "0123456789abcdef" for c in text):
        raise ValidationError(f"{name} must be 64 lowercase hex characters")
    return text


def _centroid(row: sqlite3.Row) -> Tuple[float, float, float]:
    return (
        (row["aabb_min_x"] + row["aabb_max_x"]) / 2.0,
        (row["aabb_min_y"] + row["aabb_max_y"]) / 2.0,
        (row["aabb_min_z"] + row["aabb_max_z"]) / 2.0,
    )


class EvidenceStore:
    """SQLite-backed v2 evidence. One transaction per public write unless the
    caller groups writes with ``transaction()`` (a correction is one, §2.2)."""

    def __init__(self, path: Union[str, Path], wallclock: Callable[[], float]) -> None:
        self._path = Path(path)
        self._wallclock = wallclock
        self._txn: Optional[int] = None
        self._txn_writer = ""
        new_file = not self._path.exists() or self._path.stat().st_size == 0
        conn = sqlite3.connect(str(self._path), isolation_level=None)
        conn.row_factory = sqlite3.Row
        conn.execute("PRAGMA foreign_keys = ON")
        try:
            if new_file:
                conn.executescript(_schema_script())
            else:
                version = int(conn.execute("PRAGMA user_version").fetchone()[0])
                if version != SCHEMA_VERSION:
                    raise ValidationError(
                        f"database user_version is {version}, expected {SCHEMA_VERSION}"
                    )
        except Exception:
            if conn.in_transaction:
                conn.execute("ROLLBACK")
            conn.close()
            raise
        self._conn: Optional[sqlite3.Connection] = conn

    def close(self) -> None:
        conn, self._conn = self._conn, None
        if conn is not None:
            conn.close()

    def __enter__(self) -> "EvidenceStore":
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()

    @property
    def connection(self) -> sqlite3.Connection:
        """Raw connection, for tests and read-only inspection only."""
        if self._conn is None:
            raise RuntimeError("EvidenceStore is closed")
        return self._conn

    # Transactions (§2.1) -----------------------------------------------------

    @contextmanager
    def transaction(
        self, writer: str, operation: str, knowledge_basis: str = "live"
    ) -> Iterator[int]:
        """One knowledge transaction. Every row written inside carries its id.

        ``recorded_at = max(previous recorded_at, wallclock())``, so wall time
        never goes back. Any exception rolls back every row, the transaction
        row included.
        """
        if self._txn is not None:
            raise ValidationError("transactions do not nest")
        _text(writer, "writer")
        _text(operation, "operation")
        if knowledge_basis not in _KNOWLEDGE_BASES:
            raise ValidationError(f"knowledge_basis must be one of {_KNOWLEDGE_BASES}")
        wall = _finite(self._wallclock(), "wallclock()")
        conn = self.connection
        conn.execute("BEGIN IMMEDIATE")
        try:
            prev = conn.execute(
                "SELECT recorded_at FROM transactions ORDER BY txn_id DESC LIMIT 1"
            ).fetchone()
            recorded_at = wall if prev is None else max(prev[0], wall)
            txn = conn.execute(
                "INSERT INTO transactions (recorded_at, writer, operation, knowledge_basis)"
                " VALUES (?, ?, ?, ?)",
                (recorded_at, writer, operation, knowledge_basis),
            ).lastrowid
            self._txn, self._txn_writer = txn, writer
            yield txn
            conn.execute("COMMIT")
        except sqlite3.IntegrityError as exc:
            conn.execute("ROLLBACK")
            raise ValidationError(str(exc)) from exc
        except BaseException:
            conn.execute("ROLLBACK")
            raise
        finally:
            self._txn, self._txn_writer = None, ""

    @contextmanager
    def _write(self, writer: str, operation: str) -> Iterator[int]:
        """A public write: its own transaction, or a savepoint in the caller's."""
        if self._txn is None:
            with self.transaction(writer, operation) as txn:
                yield txn
            return
        conn = self.connection
        conn.execute("SAVEPOINT write")
        try:
            yield self._txn
        except BaseException as exc:
            conn.execute("ROLLBACK TO write")
            conn.execute("RELEASE write")
            if isinstance(exc, sqlite3.IntegrityError):
                raise ValidationError(str(exc)) from exc
            raise
        conn.execute("RELEASE write")

    def _uncommitted(self) -> int:
        """The first txn_id not yet committed: the open transaction's, if any."""
        return self._txn if self._txn is not None else 1 << 62

    def latest_txn(self) -> int:
        """The newest committed transaction id, 0 for an empty store. The open
        transaction, if any, is not committed and is never reported."""
        row = self.connection.execute(
            "SELECT max(txn_id) FROM transactions WHERE txn_id < ?", (self._uncommitted(),)
        ).fetchone()
        return int(row[0] or 0)

    def known_at_wall(self, wall_time: float) -> int:
        """The newest committed transaction with ``recorded_at <= wall_time`` (§2.1).
        0, the empty history, when there is none: never None, which queries read
        as the newest transaction."""
        row = self.connection.execute(
            "SELECT max(txn_id) FROM transactions WHERE recorded_at <= ? AND txn_id < ?",
            (_finite(wall_time, "wall_time"), self._uncommitted()),
        ).fetchone()
        return int(row[0] or 0)

    def _resolve(self, known_at: Optional[int]) -> int:
        # Inside an open transaction the connection sees uncommitted rows, and a
        # purge's deletes, at every known_at, so no answer there could be reproduced.
        if self._txn is not None:
            raise ValidationError("queries are not answered inside an open transaction")
        latest = self.latest_txn()
        if known_at is None:
            return latest
        k = _row_id(known_at, "known_at")
        # A known_at past the newest transaction could change answer later.
        if not 0 <= k <= latest:
            raise ValidationError(f"known_at {k} is not a committed transaction (newest {latest})")
        return k

    # Writes ------------------------------------------------------------------

    def add_session(
        self,
        session_id: str,
        t_start: float,
        t_end: float,
        source_recording_sha256: str,
        writer: str = "ingest",
    ) -> str:
        _text(session_id, "session_id")
        start, end = _finite(t_start, "t_start"), _finite(t_end, "t_end")
        if start > end:
            raise ValidationError("t_start is after t_end")
        _sha(source_recording_sha256, "source_recording_sha256")
        with self._write(writer, "add_session") as txn:
            self.connection.execute(
                "INSERT INTO sessions (session_id, t_start, t_end, source_recording_sha256,"
                " recorded_txn) VALUES (?, ?, ?, ?, ?)",
                (session_id, start, end, source_recording_sha256, txn),
            )
        return session_id

    def add_trajectory_revision(
        self,
        session_id: str,
        revision: int,
        tum_sha256: str,
        parent_revision_id: Optional[int],
        method: str,
        method_version: str,
        params_sha256: str,
        reason: str,
        writer: str = "ingest",
    ) -> int:
        """A trajectory revision (§2.3). ``parent_revision_id=None`` records a
        rebuild from the raw recording, which a retraction of an older revision
        does not reach (§5 decision 6)."""
        _text(session_id, "session_id")
        _row_id(revision, "revision")
        _sha(tum_sha256, "tum_sha256")
        if parent_revision_id is not None:
            _row_id(parent_revision_id, "parent_revision_id")
        _text(method, "method")
        _text(method_version, "method_version")
        _sha(params_sha256, "params_sha256")
        _text(reason, "reason")
        with self._write(writer, "add_trajectory_revision") as txn:
            self._existing("session", session_id)
            if parent_revision_id is not None:
                parent = self._existing("trajectory_revision", parent_revision_id)
                if parent["session_id"] != session_id:
                    raise ValidationError("a parent revision must belong to the same session")
            return self.connection.execute(
                "INSERT INTO trajectory_revisions (session_id, revision, tum_sha256,"
                " parent_revision_id, method, method_version, params_sha256, reason, recorded_txn)"
                " VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)",
                (session_id, revision, tum_sha256, parent_revision_id, method,
                 method_version, params_sha256, reason, txn),
            ).lastrowid

    def add_map_revision(
        self,
        map_id: str,
        revision: int,
        map_pcd_sha256: str,
        trajectory_revision_id: int,
        built_by: str,
        method: str,
        method_version: str,
        params_sha256: str,
        writer: str = "ingest",
    ) -> int:
        """A map revision built from one trajectory revision (§2.3). Its frame is
        ``map_revision_id``."""
        _text(map_id, "map_id")
        _row_id(revision, "revision")
        _sha(map_pcd_sha256, "map_pcd_sha256")
        _row_id(trajectory_revision_id, "trajectory_revision_id")
        _text(built_by, "built_by")
        _text(method, "method")
        _text(method_version, "method_version")
        _sha(params_sha256, "params_sha256")
        with self._write(writer, "add_map_revision") as txn:
            self._existing("trajectory_revision", trajectory_revision_id)
            return self.connection.execute(
                "INSERT INTO map_revisions (map_id, revision, map_pcd_sha256,"
                " trajectory_revision_id, built_by, method, method_version, params_sha256,"
                " recorded_txn) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)",
                (map_id, revision, map_pcd_sha256, trajectory_revision_id, built_by, method,
                 method_version, params_sha256, txn),
            ).lastrowid

    def add_observation(
        self,
        session_id: str,
        kind: str,
        sensor_kind: str,
        geometry_frame: str,
        acquired_at: float,
        aabb: Sequence[float],
        sigma_m: float,
        measurement_confidence: float,
        coverage: Optional[float] = None,
        map_revision_id: Optional[int] = None,
        derived_from: Optional[str] = None,
        method: Optional[str] = None,
        method_version: Optional[str] = None,
        params_sha256: Optional[str] = None,
        writer: str = "ingest",
    ) -> int:
        """An immutable measurement. ``aabb`` is (min x, y, z, max x, y, z) in the
        body frame at ``acquired_at``, or in ``map_revision_id``'s frame."""
        _text(session_id, "session_id")
        if kind not in ("detection", "absence"):
            raise ValidationError("kind must be 'detection' or 'absence'")
        if sensor_kind not in ("camera", "lidar"):
            raise ValidationError("sensor_kind must be 'camera' or 'lidar'")
        if geometry_frame not in ("body", "map"):
            raise ValidationError("geometry_frame must be 'body' or 'map'")
        if kind == "absence" and geometry_frame == "body":
            raise ValidationError("a body-frame absence is refused until designed (§2.5)")
        if (geometry_frame == "map") != (map_revision_id is not None):
            raise ValidationError("a map-frame observation names its map revision, a body one does not")
        if map_revision_id is not None:
            _row_id(map_revision_id, "map_revision_id")
        when = _finite(acquired_at, "acquired_at")
        box = [_finite(v, "aabb") for v in aabb]
        if len(box) != 6 or any(box[i] > box[i + 3] for i in range(3)):
            raise ValidationError("aabb must be (min x, y, z, max x, y, z) with min <= max")
        if _finite(sigma_m, "sigma_m") < 0:
            raise ValidationError("sigma_m must be >= 0")
        if not 0 <= _finite(measurement_confidence, "measurement_confidence") <= 1:
            raise ValidationError("measurement_confidence must be in [0, 1]")
        if kind == "absence":
            if coverage is None or not 0 <= _finite(coverage, "coverage") <= 1:
                raise ValidationError("an absence needs a coverage in [0, 1]")
        elif coverage is not None:
            raise ValidationError("a detection has no coverage")
        for name, value in (("derived_from", derived_from), ("method", method),
                            ("method_version", method_version)):
            if value is not None:
                _text(value, name)
        if params_sha256 is not None:
            _sha(params_sha256, "params_sha256")
        expires_at = when + RETENTION_S if sensor_kind == "camera" else None
        with self._write(writer, "add_observation") as txn:
            session = self._existing("session", session_id)
            if not session["t_start"] <= when <= session["t_end"]:
                raise ValidationError("acquired_at is outside its session")
            if map_revision_id is not None:
                if self._session_of_map(map_revision_id) != session_id:
                    raise ValidationError("an observation is bound only to its own session's map revisions")
            return self.connection.execute(
                "INSERT INTO observations (session_id, kind, sensor_kind, geometry_frame,"
                " map_revision_id, acquired_at, aabb_min_x, aabb_min_y, aabb_min_z, aabb_max_x,"
                " aabb_max_y, aabb_max_z, sigma_m, coverage, measurement_confidence, expires_at,"
                " derived_from, method, method_version, params_sha256, recorded_txn)"
                " VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
                (session_id, kind, sensor_kind, geometry_frame, map_revision_id, when, *box,
                 float(sigma_m), None if coverage is None else float(coverage),
                 float(measurement_confidence), expires_at, derived_from, method,
                 method_version, params_sha256, txn),
            ).lastrowid

    def add_interpretation(
        self,
        observation_id: int,
        object_id: str,
        label: str,
        confidence: float,
        source_kind: str,
        writer: str,
    ) -> int:
        """A guess about what an observation is, kept apart from the measurement."""
        _row_id(observation_id, "observation_id")
        _text(object_id, "object_id")
        _text(label, "label")
        if not 0 <= _finite(confidence, "confidence") <= 1:
            raise ValidationError("confidence must be in [0, 1]")
        if source_kind not in ("model", "operator"):
            raise ValidationError("source_kind must be 'model' or 'operator'")
        _text(writer, "writer")
        if (source_kind == "model") != _is_model(writer):
            raise ValidationError("a model interpretation's writer is 'model:<name>@<version>'")
        with self._write(writer, "add_interpretation") as txn:
            self._existing("observation", observation_id)
            return self.connection.execute(
                "INSERT INTO interpretations (observation_id, object_id, label, confidence,"
                " source_kind, writer, recorded_txn) VALUES (?, ?, ?, ?, ?, ?, ?)",
                (observation_id, object_id, label, float(confidence), source_kind, writer, txn),
            ).lastrowid

    def retract(
        self, target_kind: str, target_id: Union[int, str], reason: str, retracted_by: str
    ) -> int:
        """Retract one row, once (§2.2). Operator-only for every kind (§5 decision 4):
        a ``model:`` writer is refused. Writes one row and nothing for what depends on it."""
        _text(retracted_by, "retracted_by")
        if _is_model(retracted_by) or (self._txn is not None and _is_model(self._txn_writer)):
            raise ValidationError("only an operator can retract (§5 decision 4)")
        if target_kind not in TARGET_KINDS:
            raise ValidationError(f"{target_kind!r} cannot be retracted; kinds are {TARGET_KINDS}")
        _text(reason, "reason")
        key = self._key(target_kind, target_id)
        with self._write(retracted_by, "retract_" + target_kind) as txn:
            self._existing(target_kind, target_id)
            if self._retraction_txn(target_kind, key) is not None:
                raise ValidationError(f"{target_kind} {key} is already retracted")
            return self.connection.execute(
                "INSERT INTO retractions (target_kind, target_id, recorded_txn, reason,"
                " retracted_by) VALUES (?, ?, ?, ?, ?)",
                (target_kind, key, txn, reason, retracted_by),
            ).lastrowid

    def request_look_again(self, object_id: str, writer: str) -> int:
        """An append-only request (§2.1). Its creation time is its transaction's recorded_at."""
        _text(object_id, "object_id")
        with self._write(writer, "request_look_again") as txn:
            return self.connection.execute(
                "INSERT INTO look_again_requests (object_id, recorded_txn) VALUES (?, ?)",
                (object_id, txn),
            ).lastrowid

    def fulfil_look_again(self, request_id: int, observation_id: int, writer: str) -> None:
        """Close a request as fulfilled. The observation must be valid at this
        transaction and acquired no earlier than the request was created (§2.1)."""
        _row_id(request_id, "request_id")
        _row_id(observation_id, "observation_id")
        with self._write(writer, "fulfil_look_again") as txn:
            created = self._open_request_created_at(request_id)
            obs = self._existing("observation", observation_id)
            if self._invalid("observation", observation_id, txn):
                raise ValidationError("a retracted observation cannot fulfil a request")
            # Cross-clock comparison, by design (§2.1, as v1): sensor acquired_at vs
            # the request's wall recorded_at.
            if obs["acquired_at"] < created:
                raise ValidationError("the observation was acquired before the request was made")
            self.connection.execute(
                "INSERT INTO look_again_closures (request_id, outcome,"
                " fulfilled_by_observation_id, recorded_txn) VALUES (?, 'fulfilled', ?, ?)",
                (request_id, observation_id, txn),
            )

    def cancel_look_again(self, request_id: int, writer: str) -> None:
        _row_id(request_id, "request_id")
        with self._write(writer, "cancel_look_again") as txn:
            self._open_request_created_at(request_id)
            self.connection.execute(
                "INSERT INTO look_again_closures (request_id, outcome,"
                " fulfilled_by_observation_id, recorded_txn) VALUES (?, 'cancelled', NULL, ?)",
                (request_id, txn),
            )

    def purge_expired(self, wall_now: float, writer: str = "retention") -> List[int]:
        """Delete expired camera observations and their interpretations, in one
        transaction, leaving §2.6 tombstones. A retraction whose target is deleted
        has its ``reason`` blanked (§5 decision 5). Closures are never touched.
        Returns the purged observation ids."""
        now = _finite(wall_now, "wall_now")
        with self._write(writer, "purge_expired") as txn:
            conn = self.connection
            conn.execute("INSERT INTO purge_guard (id) VALUES (1)")
            expired = [
                r[0]
                for r in conn.execute(
                    "SELECT observation_id FROM observations WHERE sensor_kind = 'camera'"
                    " AND expires_at <= ? ORDER BY observation_id",
                    (now,),
                )
            ]
            for obs_id in expired:
                row_hash = self.row_sha256("observations", obs_id)
                interps = conn.execute(
                    "SELECT interpretation_id, object_id, recorded_txn FROM interpretations"
                    " WHERE observation_id = ? ORDER BY interpretation_id",
                    (obs_id,),
                ).fetchall()
                for interp in interps:
                    conn.execute(
                        "INSERT INTO purged_associations (interpretation_id, observation_id,"
                        " object_id, recorded_txn, purged_txn) VALUES (?, ?, ?, ?, ?)",
                        (interp["interpretation_id"], obs_id, interp["object_id"],
                         interp["recorded_txn"], txn),
                    )
                    conn.execute(
                        "DELETE FROM interpretations WHERE interpretation_id = ?",
                        (interp["interpretation_id"],),
                    )
                    self._blank_reason("interpretation", interp["interpretation_id"])
                conn.execute(
                    "INSERT INTO purged_observations (observation_id, kind, session_id,"
                    " sensor_kind, acquired_at, recorded_txn, purged_txn, row_sha256)"
                    " SELECT observation_id, kind, session_id, sensor_kind, acquired_at,"
                    " recorded_txn, ?, ? FROM observations WHERE observation_id = ?",
                    (txn, row_hash, obs_id),
                )
                conn.execute("DELETE FROM observations WHERE observation_id = ?", (obs_id,))
                self._blank_reason("observation", obs_id)
            conn.execute("DELETE FROM purge_guard")
        return expired

    def _blank_reason(self, target_kind: str, target_id: int) -> None:
        self.connection.execute(
            "UPDATE retractions SET reason = NULL WHERE target_kind = ? AND target_id = ?"
            " AND reason IS NOT NULL",
            (target_kind, str(target_id)),
        )

    def _open_request_created_at(self, request_id: int) -> float:
        row = self.connection.execute(
            "SELECT t.recorded_at FROM look_again_requests AS r"
            " JOIN transactions AS t ON t.txn_id = r.recorded_txn WHERE r.request_id = ?",
            (request_id,),
        ).fetchone()
        if row is None:
            raise ValidationError(f"no look-again request {request_id}")
        closed = self.connection.execute(
            "SELECT 1 FROM look_again_closures WHERE request_id = ?", (request_id,)
        ).fetchone()
        if closed is not None:
            raise ValidationError("a request closes once; file a new one to ask again")
        return float(row[0])

    # Row access ----------------------------------------------------------------

    def _key(self, kind: str, row_id: object) -> str:
        """The retractions.target_id spelling of a row id."""
        if kind == "session":
            return _text(row_id, "session_id")
        return str(_row_id(row_id, f"{kind} id"))

    def _existing(self, kind: str, row_id: object, known_at: Optional[int] = None) -> sqlite3.Row:
        """The row, refused if absent or not yet recorded at ``known_at``."""
        table, pk = _TABLES[kind]
        if kind != "session":
            _row_id(row_id, f"{kind} id")
        row = self.connection.execute(
            f"SELECT * FROM {table} WHERE {pk} = ?", (row_id,)
        ).fetchone()
        if row is None:
            raise ValidationError(f"no {kind} {row_id}")
        if known_at is not None and row["recorded_txn"] > known_at:
            raise ValidationError(f"{kind} {row_id} is not recorded at known_at {known_at}")
        return row

    def _session_of_map(self, map_revision_id: int) -> str:
        traj = self._existing("map_revision", map_revision_id)["trajectory_revision_id"]
        return self._existing("trajectory_revision", traj)["session_id"]

    def row_sha256(self, table: str, row_id: Union[int, str]) -> str:
        """SHA-256 of a row's canonical JSON (column name to value)."""
        if table not in _HASHABLE:
            raise ValidationError(f"no row hash for table {table!r}")
        row = self.connection.execute(
            f"SELECT * FROM {table} WHERE {_HASHABLE[table]} = ?", (row_id,)
        ).fetchone()
        if row is None:
            raise ValidationError(f"no row {row_id} in {table}")
        data = {key: row[key] for key in row.keys()}
        return hashlib.sha256(canonical_json(data).encode("utf-8")).hexdigest()

    # Validity and revisions (§2.2, §2.3) ---------------------------------------

    def _retraction_txn(self, kind: str, key: str) -> Optional[int]:
        row = self.connection.execute(
            "SELECT recorded_txn FROM retractions WHERE target_kind = ? AND target_id = ?",
            (kind, key),
        ).fetchone()
        return None if row is None else int(row[0])

    def _retracted(self, kind: str, row_id: object, k: int) -> bool:
        txn = self._retraction_txn(kind, str(row_id))
        return txn is not None and txn <= k

    def _invalid(self, kind: str, row_id: object, k: int) -> bool:
        """§2.2: the row, or a row it depends on, is retracted at ``k``. Computed
        from immutable columns; nothing is stored for dependents."""
        row = self._existing(kind, row_id)
        if kind == "session":
            return self._retracted("session", row_id, k)
        if kind == "trajectory_revision":
            parent = row["parent_revision_id"]
            return (
                self._retracted(kind, row_id, k)
                or self._invalid("session", row["session_id"], k)
                or (parent is not None and self._invalid(kind, parent, k))
            )
        if kind == "map_revision":
            return self._invalid("trajectory_revision", row["trajectory_revision_id"], k)
        if kind == "observation":
            mr = row["map_revision_id"]
            return (
                self._retracted(kind, row_id, k)
                or self._invalid("session", row["session_id"], k)
                or (mr is not None and self._invalid("map_revision", mr, k))
            )
        return self._retracted(kind, row_id, k) or self._invalid(
            "observation", row["observation_id"], k
        )

    def is_invalid(self, kind: str, row_id: Union[int, str], known_at: Optional[int] = None) -> bool:
        """Whether a recorded row counts as retracted at ``known_at`` (§2.2)."""
        if kind not in _TABLES:
            raise ValidationError(f"unknown kind {kind!r}")
        k = self._resolve(known_at)
        self._existing(kind, row_id, k)
        return self._invalid(kind, row_id, k)

    def _active_trajectory(self, session_id: str, k: int) -> Optional[int]:
        rows = self.connection.execute(
            "SELECT trajectory_revision_id FROM trajectory_revisions"
            " WHERE session_id = ? AND recorded_txn <= ?"
            " ORDER BY recorded_txn DESC, trajectory_revision_id DESC",
            (session_id, k),
        ).fetchall()
        for row in rows:
            if not self._invalid("trajectory_revision", row[0], k):
                return int(row[0])
        return None

    def active_trajectory_revision(
        self, session_id: str, known_at: Optional[int] = None
    ) -> Optional[int]:
        """§2.3: the newest valid revision visible at ``known_at``. A retracted
        revision falls back to its parent; None when no valid revision is left."""
        k = self._resolve(known_at)
        self._existing("session", session_id, k)
        return self._active_trajectory(session_id, k)

    def _active_map(self, trajectory_revision_id: int, k: int) -> Optional[int]:
        row = self.connection.execute(
            "SELECT map_revision_id FROM map_revisions"
            " WHERE trajectory_revision_id = ? AND recorded_txn <= ?"
            " ORDER BY recorded_txn DESC, map_revision_id DESC LIMIT 1",
            (trajectory_revision_id, k),
        ).fetchone()
        return None if row is None else int(row[0])

    def _frame_current(self, map_revision_id: int, k: int) -> bool:
        if self._invalid("map_revision", map_revision_id, k):
            return False
        traj = self._existing("trajectory_revision",
                              self._existing("map_revision", map_revision_id)["trajectory_revision_id"])
        return self._active_trajectory(traj["session_id"], k) == traj["trajectory_revision_id"]

    def is_frame_current(self, map_revision_id: int, known_at: Optional[int] = None) -> bool:
        """§2.3: the map revision is valid and built from its session's active
        trajectory revision. A newer revision supersedes; it invalidates nothing."""
        k = self._resolve(known_at)
        self._existing("map_revision", map_revision_id, k)
        return self._frame_current(map_revision_id, k)

    def _resolved_frame(
        self, obs: sqlite3.Row, k: int
    ) -> Tuple[Optional[int], Optional[str], Optional[int]]:
        """(map revision, reason it has none, trajectory revision) of a valid observation (§2.5)."""
        if obs["geometry_frame"] == "map":
            mr = obs["map_revision_id"]
            if not self._frame_current(mr, k):
                return None, "superseded_revision", None
            return mr, None, int(self._existing("map_revision", mr)["trajectory_revision_id"])
        # body: positioned only under the session's active trajectory revision.
        active = self._active_trajectory(obs["session_id"], k)
        if active is None:
            return None, "source_retracted", None
        mr = self._active_map(active, k)
        if mr is None:
            return None, "no_map_revision", active
        return mr, None, active

    # Queries -------------------------------------------------------------------

    def compare_positions(self, a: int, b: int, known_at: Optional[int] = None) -> Comparison:
        """§2.5 rule 2 only: both observations resolve to the same current map revision.

        Rules 1 and 3 (same-session body positions, alignments) are not built.
        A body observation that resolves to the same frame as the other would
        need its pose derived, which V1 does not do: that raises
        NotImplementedError rather than return a distance it cannot measure.
        """
        k = self._resolve(known_at)
        oa = self._existing("observation", a, k)
        ob = self._existing("observation", b, k)
        if self._invalid("observation", a, k) or self._invalid("observation", b, k):
            return _not_comparable("source_retracted")
        fa = self._resolved_frame(oa, k)
        fb = self._resolved_frame(ob, k)
        for reason in _REASON_ORDER:
            if reason in (fa[1], fb[1]):
                return _not_comparable(reason)
        if fa[0] != fb[0]:
            return _not_comparable("different_frames")
        if "body" in (oa["geometry_frame"], ob["geometry_frame"]):
            raise NotImplementedError("a body-frame position needs pose derivation (not in V1)")
        return Comparison(
            comparable=True,
            reason=None,
            frame=fa[0],
            distance_m=math.dist(_centroid(oa), _centroid(ob)),
            sigma_m=comparison_sigma(oa["sigma_m"], ob["sigma_m"]),
            revisions_used={
                "map_revisions": [fa[0]],
                "trajectory_revisions": sorted({fa[2], fb[2]}),
            },
        )

    def active_interpretations(
        self, observation_id: int, known_at: Optional[int] = None
    ) -> List[Dict[str, object]]:
        """The observation's interpretations recorded and valid at ``known_at``."""
        k = self._resolve(known_at)
        self._existing("observation", observation_id, k)
        rows = self.connection.execute(
            "SELECT interpretation_id, object_id, label, confidence, source_kind, writer,"
            " recorded_txn FROM interpretations WHERE observation_id = ? AND recorded_txn <= ?"
            " ORDER BY interpretation_id",
            (observation_id, k),
        ).fetchall()
        return [
            {key: row[key] for key in row.keys()}
            for row in rows
            if not self._invalid("interpretation", row["interpretation_id"], k)
        ]

    def evidence_for(
        self, object_id: str, now: float, known_at: Optional[int] = None
    ) -> Dict[str, object]:
        """The detections supporting ``object_id`` at ``(now, known_at)``, newest first.

        A detection counts when it and its interpretation are recorded and valid at
        ``known_at``, ``acquired_at <= now``, and it has a position: a map-frame row
        in a current frame, or a body-frame row under a valid active trajectory
        revision. ``frame`` is the newest support's map revision. ``complete`` and
        ``purged_ids`` follow §2.6. Status words and absences are V3.
        """
        _text(object_id, "object_id")
        t_now = _finite(now, "now")
        k = self._resolve(known_at)
        rows = self.connection.execute(
            "SELECT i.interpretation_id, o.* FROM interpretations AS i"
            " JOIN observations AS o ON o.observation_id = i.observation_id"
            " WHERE i.object_id = ? AND i.recorded_txn <= ? AND o.kind = 'detection'"
            " AND o.acquired_at <= ? ORDER BY o.observation_id",
            (object_id, k, t_now),
        ).fetchall()
        support: Dict[int, Dict[str, object]] = {}
        for row in rows:
            obs_id = row["observation_id"]
            if obs_id in support or self._invalid("interpretation", row["interpretation_id"], k):
                continue
            frame, reason, traj = self._resolved_frame(row, k)
            if reason in ("source_retracted", "superseded_revision"):
                continue
            support[obs_id] = {
                "observation_id": obs_id,
                "session_id": row["session_id"],
                "acquired_at": row["acquired_at"],
                "geometry_frame": row["geometry_frame"],
                "map_revision_id": frame,
                "trajectory_revision_id": traj,
            }
        ordered = sorted(
            support.values(), key=lambda s: (-s["acquired_at"], -s["observation_id"])
        )
        purged = self._purged_ids(object_id, t_now, k)
        return {
            "object_id": object_id,
            "now": t_now,
            "known_at": k,
            "frame": ordered[0]["map_revision_id"] if ordered else None,
            "support": ordered,
            "complete": not purged,
            "purged_ids": purged,
        }

    def _purged_ids(self, object_id: str, now: float, k: int) -> List[int]:
        """§2.6: purged observations this answer may have depended on. ``purged_txn``
        is never compared. Trajectory retractions cannot be checked from a
        tombstone, so this can over-report and never under-reports."""
        rows = self.connection.execute(
            "SELECT DISTINCT p.observation_id FROM purged_associations AS a"
            " JOIN purged_observations AS p ON p.observation_id = a.observation_id"
            " WHERE a.object_id = :obj AND a.recorded_txn <= :k AND p.recorded_txn <= :k"
            " AND p.acquired_at <= :now"
            " AND NOT EXISTS (SELECT 1 FROM retractions AS r WHERE r.recorded_txn <= :k AND ("
            "   (r.target_kind = 'interpretation' AND r.target_id = CAST(a.interpretation_id AS TEXT))"
            "   OR (r.target_kind = 'observation' AND r.target_id = CAST(p.observation_id AS TEXT))"
            "   OR (r.target_kind = 'session' AND r.target_id = p.session_id)))"
            " ORDER BY p.observation_id",
            {"obj": object_id, "k": k, "now": now},
        ).fetchall()
        return [int(r[0]) for r in rows]

    def open_look_again(self, object_id: str, known_at: Optional[int] = None) -> List[int]:
        """Requests recorded and not closed at ``known_at`` (§2.1). ``now`` does not filter them."""
        _text(object_id, "object_id")
        k = self._resolve(known_at)
        rows = self.connection.execute(
            "SELECT r.request_id FROM look_again_requests AS r"
            " WHERE r.object_id = ? AND r.recorded_txn <= ?"
            " AND NOT EXISTS (SELECT 1 FROM look_again_closures AS c"
            "                 WHERE c.request_id = r.request_id AND c.recorded_txn <= ?)"
            " ORDER BY r.request_id",
            (object_id, k, k),
        ).fetchall()
        return [int(r[0]) for r in rows]
