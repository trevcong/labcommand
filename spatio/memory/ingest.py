"""Offline walk-day measurements in V1, per docs/MEMORY-V2-DESIGN.md §2.3/2.6.

Conventions from PLAN-V2.md [proposal]: map observations are acquired at the
last TUM stamp and measurement_confidence is 1.0. Neither is a calibrated
confidence estimate. No interpretations, series verdicts or revised poses.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import itertools
import json
import math
import os
from pathlib import Path
import re
import sqlite3
import time

import numpy as np

from spatio.clearance import provenance
from spatio.clearance.model import parse_zones
from spatio.clearance.report import build_report
from spatio.io.tum import read_tum
from spatio.manifest.hashing import hash_file
from spatio.manifest.locate import bag_files_next_to, find_metadata_yaml, relative_bag_name
from spatio.memory.evidence import EvidenceStore, ValidationError
from spatio.series.validate import SeriesError, parse_walk_result
from spatio.wallpatch.geometry import box_axes, parse_box
from spatio.wallpatch.errors import WallpatchError


class IngestRefused(ValidationError):
    """Inputs or existing evidence conflict; no ingest transaction is retained."""


@dataclass(frozen=True)
class IngestResult:
    session_id: str
    trajectory_revision_id: int
    map_revision_id: int
    observation_ids: tuple[int, ...]
    inserted_observation_ids: tuple[int, ...]
    skipped: tuple[str, ...]
    transaction_id: int | None


def _text(value, name):
    if not isinstance(value, str) or not value.strip():
        raise IngestRefused(f"{name} must be nonempty text")
    return value


def _sha(value, name):
    if not isinstance(value, str) or not re.fullmatch(r"[0-9a-f]{64}", value):
        raise IngestRefused(f"{name} must be a lowercase SHA-256")
    return value


def _path(value):
    path = Path(value).absolute()
    for member in (path, *path.parents):
        # macOS system aliases, not user-supplied links into another input tree.
        if member.is_symlink() and str(member) not in ("/tmp", "/var"):
            raise IngestRefused(f"symlink input is unsupported: {member}")
    return path.resolve()


def _members(path):
    if path.is_file():
        return [path]
    if not path.is_dir():
        raise IngestRefused(f"input is not a regular file or directory: {path}")
    def unreadable(error):
        raise error
    files = []
    for root, dirs, names in os.walk(path, onerror=unreadable):
        dirs.sort()
        for name in sorted(dirs + names):
            child = Path(root) / name
            if child.is_symlink() or not (child.is_dir() or child.is_file()):
                raise IngestRefused(f"input contains a symlink or special file: {child}")
        files.extend(Path(root) / name for name in sorted(names))
    return files


def _protect(database, inputs):
    database = Path(database).resolve()
    for source in inputs:
        if database == source or (source.is_dir() and source in database.parents):
            raise IngestRefused("database must be outside every input tree/file")
        for member in _members(source):
            if database.exists() and database.samefile(member):
                raise IngestRefused("database is a hard link to an input file")


def _json(path):
    if not path.is_file():
        raise IngestRefused(f"missing regular input file: {path}")
    def pairs(items):
        result = {}
        for key, value in items:
            if key in result:
                raise IngestRefused(f"duplicate JSON key: {key}")
            result[key] = value
        return result
    def invalid(value):
        raise IngestRefused(f"nonfinite JSON value: {value}")
    data = json.loads(path.read_bytes(), object_pairs_hook=pairs, parse_constant=invalid)
    provenance.canonical_sha256(data)  # Also rejects overflowing literals (1e999).
    if not isinstance(data, dict):
        raise IngestRefused("JSON document must be an object")
    return data


def _recording_hash(recording):
    if not recording.is_dir():
        raise IngestRefused("recording_dir must be a directory")
    files = _members(recording)
    if any(p.name == "SYNTHETIC.json" for p in files):
        raise IngestRefused("synthetic recording cannot be ingested as evidence")
    if not files:
        raise IngestRefused("recording directory is empty")
    # Same definition as sha256_path, with os.walk errors and special files refused.
    return provenance.canonical_sha256({"tree": [
        [p.relative_to(recording).as_posix(), provenance.sha256_path(p)] for p in files]})


def _fingerprint(manifest, recording):
    expected = manifest["source_fingerprint"]
    metadata = find_metadata_yaml(None, str(recording))
    if metadata is None or hash_file(metadata)[0] != expected["metadata_yaml_sha256"]:
        raise IngestRefused("source_fingerprint metadata mismatch")
    bags = bag_files_next_to(metadata)
    records = expected["bag_files"]
    if not bags or not isinstance(records, list) or len(records) != len(bags):
        raise IngestRefused("source_fingerprint bag files mismatch")
    by_name = {r["name"]: r for r in records}
    if len(by_name) != len(records) or set(by_name) != {
            relative_bag_name(b, str(recording)) for b in bags}:
        raise IngestRefused("source_fingerprint bag names mismatch")
    for bag in bags:
        record = by_name[relative_bag_name(bag, str(recording))]
        actual, size = hash_file(bag)
        if type(record["bytes"]) is not int or record["bytes"] != size:
            raise IngestRefused("source_fingerprint bag bytes mismatch")
        if record["sha256"] is not None and _sha(record["sha256"], "bag hash") != actual:
            raise IngestRefused("source_fingerprint bag hash mismatch")


def _prepare(session_dir, recording_dir, code_revision, clearance):
    if not isinstance(code_revision, str) or not re.fullmatch(r"[0-9a-f]{40}", code_revision):
        raise IngestRefused("code_revision must be 40 lowercase hex characters")
    session, recording = _path(session_dir), _path(recording_dir)
    _members(session)
    if (session / "series.json").exists() and not (session / "walk_result.json").exists():
        raise IngestRefused("series output: aggregate verdicts, no geometry")
    raw_hash = _recording_hash(recording)
    manifest = _json(session / "manifest.json")
    _fingerprint(manifest, recording)
    walk_data = _json(session / "walk_result.json")
    walk = parse_walk_result(walk_data, str(session / "walk_result.json"))
    if walk.status not in ("PASS", "FAIL"):
        raise IngestRefused("unscored or NOT_EVALUABLE run")
    for key in ("run_id", "fastlio_revision", "config_sha256"):
        if walk_data[key] != manifest[key]:
            raise IngestRefused(f"walk_result {key} differs from manifest")
    sid = _text(manifest["run_id"], "run_id")
    method = dict(method="fast_lio", method_version=_text(manifest["fastlio_revision"],
                  "fastlio_revision"), params_sha256=_sha(manifest["config_sha256"], "config_sha256"))
    hashes = {}
    for name in ("trajectory.tum", "map.pcd"):
        if not (session / name).is_file():
            raise IngestRefused(f"{name} must be a regular file")
        hashes[name] = provenance.sha256_path(session / name)
        if hashes[name] != _sha(manifest["files"][name]["sha256"], name):
            raise IngestRefused(f"manifest {name} sha256 mismatch")
    trajectory = read_tum(session / "trajectory.tum")
    if len(trajectory.t) < 2 or np.any(np.diff(trajectory.t) <= 0):
        raise IngestRefused("trajectory needs at least two strictly increasing stamps")
    start, end = float(trajectory.t[0]), float(trajectory.t[-1])
    s = dict(session_id=sid, t_start=start, t_end=end, source_recording_sha256=raw_hash)
    t = dict(session_id=sid, revision=1, tum_sha256=hashes["trajectory.tum"],
             parent_revision_id=None, reason="walk-day FAST-LIO output", **method)
    m = dict(map_id=sid, revision=1, map_pcd_sha256=hashes["map.pcd"],
             built_by="spatio.walkday", **method)
    observations, skipped, identities, record_keys = [], [], set(), []

    def record_key(source, item, record):
        _text(item, "item identity")
        if "|" in item or (source, item) in identities:
            raise IngestRefused("duplicate or ambiguous item identity")
        identities.add((source, item))
        record_hash = provenance.canonical_sha256({k: v for k, v in record.items() if k != "figure"})
        key = f"recording:{raw_hash}|source:{source}|item:{item}|record:{record_hash}"
        record_keys.append(key)
        return key

    def add(source, key, kind, box, sigma, params, coverage=None):
        if isinstance(sigma, bool) or not isinstance(sigma, (int, float)) or not math.isfinite(sigma) or sigma < 0:
            raise IngestRefused("sigma_m must be finite and nonnegative")
        if len(box) != 6 or not all(math.isfinite(v) for v in box):
            raise IngestRefused("AABB must have six finite coordinates")
        observations.append(dict(session_id=sid, kind=kind, sensor_kind="lidar",
            geometry_frame="map", acquired_at=end, aabb=box, sigma_m=sigma,
            measurement_confidence=1.0, coverage=coverage, derived_from=key,
            method=f"spatio.{source}", method_version=code_revision,
            params_sha256=provenance.canonical_sha256(params)))

    report = _json(session / "wallpatch/wallpatch.json")
    if report.get("schema") != "spatio.wallpatch_report/1":
        raise IngestRefused("unsupported wallpatch schema")
    if report["map_sha256"] != hashes["map.pcd"]:
        raise IngestRefused("wallpatch map sha256 mismatch")
    names = set()
    if not isinstance(report["patches"], list):
        raise IngestRefused("wallpatch patches must be a list")
    for patch in report["patches"]:
        name = _text(patch["name"], "patch name")
        if name in names:
            raise IngestRefused("duplicate patch name")
        names.add(name)
        key = record_key("wallpatch", name, patch)
        if patch["status"] == "NOT_EVALUABLE":
            skipped.append(f"wallpatch:{name}:NOT_EVALUABLE")
            continue
        if patch["status"] != "MEASURED":
            raise IngestRefused("unknown patch status")
        center, hint, half = parse_box(patch["box_map_frame"])
        axes = box_axes(hint)
        corners = np.array([center + sum(sign * extent * axis for sign, extent, axis
            in zip(signs, half, axes)) for signs in itertools.product((-1, 1), repeat=3)])
        box = np.concatenate((corners.min(axis=0), corners.max(axis=0))).tolist()
        add("wallpatch", key, "detection", box, patch["residuals"]["p95_abs_m"], report["gates"])

    if clearance is not None:
        result_path, zones_path = map(_path, clearance)
        results, zones_data = _json(result_path), _json(zones_path)
        if results.get("schema_version") != 1 or "schema" in results:
            raise IngestRefused("unsupported clearance schema (expected classifier report.json)")
        p = results["provenance"]
        if p["limitations"]:
            raise IngestRefused("clearance provenance limitations must be empty")
        # The two-file API cannot re-read scans/extrinsics. Their recorded hashes
        # are structurally validated, not claimed as independently reverified.
        checked = build_report(results, zones_data, sid, {
            "recording": raw_hash, "trajectory": hashes["trajectory.tum"],
            "scans": p["scans_sha256"], "extrinsics": p["extrinsics_sha256"]})
        _text(p["time_basis"], "clearance time_basis")
        times = checked["scan_time_range"]
        if times is not None and not start <= times[0] <= times[1] <= end:
            raise IngestRefused("clearance acquisition times outside trajectory session")
        definitions = {z.id: z for z in parse_zones(zones_data)[1]}
        params = {k: results[k] for k in ("voxel_size_m", "grid_origin", "uncertainty_m", "min_free_rays")}
        for zone in results["zones"]:
            key = record_key("clearance", zone["id"], zone)
            if zone["status"] != "PASS":
                skipped.append(f"clearance:{zone['id']}:{zone['status']}")
                continue
            definition = definitions[zone["id"]]
            add("clearance", key, "absence",
                [*definition.low.tolist(), *definition.high.tolist()], results["uncertainty_m"],
                params, zone["free_coverage"])
    return s, t, m, observations, tuple(skipped), record_keys


def _existing(connection, table, identity, values):
    where = " AND ".join(f"{key} = ?" for key in identity)
    row = connection.execute(f"SELECT * FROM {table} WHERE {where}", tuple(identity.values())).fetchone()
    if row is not None and any(row[key] != value for key, value in values.items()):
        raise IngestRefused(f"conflicting {table} identity {identity}")
    return row


def ingest_walkday(store, session_dir, recording_dir, code_revision, clearance=None):
    """Append one validated run atomically. An identical re-ingest writes nothing.

    clearance is (classifier report.json, zones.json). The store must be opened
    outside input trees; the CLI checks this before opening/creating a database.
    """
    try:
        inputs = [_path(session_dir), _path(recording_dir)]
        if clearance is not None:
            inputs.extend(map(_path, clearance))
        for database in store.connection.execute("PRAGMA database_list"):
            if database[2]:
                _protect(database[2], inputs)
        s, t, m, observations, skipped, record_keys = _prepare(session_dir, recording_dir, code_revision, clearance)
        conn = store.connection
        version = conn.execute("PRAGMA data_version").fetchone()[0]
        session = _existing(conn, "sessions", {"session_id": s["session_id"]}, s)
        if session is not None and store.is_invalid("session", s["session_id"]):
            raise IngestRefused("session evidence is retracted; it needs an operator, not a re-ingest")
        trajectory = _existing(conn, "trajectory_revisions", {"session_id": s["session_id"], "revision": 1}, t)
        tid = trajectory["trajectory_revision_id"] if trajectory else None
        m["trajectory_revision_id"] = tid
        mapping = _existing(conn, "map_revisions", {"map_id": s["session_id"], "revision": 1}, m)
        mid = mapping["map_revision_id"] if mapping else None
        # Include now-skipped items: a changed status cannot silently leave an
        # earlier measurement behind under the same source/item identity.
        for key in record_keys:
            prefix = key.rsplit("|record:", 1)[0] + "|record:"
            prior = conn.execute("SELECT derived_from FROM observations WHERE substr(derived_from, 1, ?) = ?",
                                 (len(prefix), prefix)).fetchall()
            if any(row[0] != key for row in prior):
                raise IngestRefused(f"conflicting derived_from item: {prefix}")
        existing_ids, pending = [], []
        for observation in observations:
            observation["map_revision_id"] = mid
            prefix = observation["derived_from"].rsplit("|record:", 1)[0] + "|record:"
            matches = conn.execute("SELECT * FROM observations WHERE substr(derived_from, 1, ?) = ?",
                                   (len(prefix), prefix)).fetchall()
            values = {k: v for k, v in observation.items() if k != "aabb"}
            values.update(zip(("aabb_min_x", "aabb_min_y", "aabb_min_z", "aabb_max_x", "aabb_max_y", "aabb_max_z"), observation["aabb"]))
            if matches:
                if len(matches) != 1 or any(matches[0][k] != v for k, v in values.items()):
                    raise IngestRefused(f"conflicting derived_from item: {prefix}")
                existing_ids.append(matches[0]["observation_id"])
            else:
                pending.append(observation)
        txn, inserted = None, []
        if session is None or trajectory is None or mapping is None or pending:
            with store.transaction("ingest", "ingest_walkday") as txn:
                if conn.execute("PRAGMA data_version").fetchone()[0] != version:
                    raise IngestRefused("store changed during ingest planning; retry")
                if session is None:
                    store.add_session(**s)
                if trajectory is None:
                    tid = store.add_trajectory_revision(**t)
                if mapping is None:
                    m["trajectory_revision_id"] = tid
                    mid = store.add_map_revision(**m)
                for observation in pending:
                    observation["map_revision_id"] = mid
                    inserted.append(store.add_observation(**observation))
        return IngestResult(s["session_id"], tid, mid, tuple(existing_ids + inserted),
                            tuple(inserted), skipped, txn)
    except (KeyError, TypeError, ValueError, OSError, SeriesError, WallpatchError, sqlite3.Error) as exc:
        raise IngestRefused(str(exc)) from exc


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session_dir", type=Path)
    parser.add_argument("--recording-dir", type=Path, required=True)
    parser.add_argument("--code-revision", required=True)
    parser.add_argument("--database", type=Path, required=True)
    parser.add_argument("--clearance", type=Path, nargs=2, metavar=("REPORT", "ZONES"))
    args = parser.parse_args(argv)
    try:
        inputs = [args.session_dir, args.recording_dir, *(args.clearance or ())]
        _protect(args.database, list(map(_path, inputs)))
        # Validate before EvidenceStore can create a file or schema.
        _prepare(args.session_dir, args.recording_dir, args.code_revision, args.clearance)
        with EvidenceStore(args.database, wallclock=time.time) as store:
            result = ingest_walkday(store, args.session_dir, args.recording_dir,
                                    args.code_revision, args.clearance)
        print(json.dumps(asdict(result), sort_keys=True))
    except (ValidationError, KeyError, TypeError, ValueError, OSError, SeriesError, WallpatchError, sqlite3.Error) as exc:
        parser.exit(2, f"ingest refused: {exc}\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
