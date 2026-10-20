"""Input identities for clearance results, and the check before reuse.

Same pattern as wall scoring's map binding: record the sha256 of the actual
input bytes, then refuse reuse unless the supplied inputs hash the same.
Hashes only, never paths or clocks, so identical inputs give identical bytes.
Hashes detect changed inputs; they are not signatures against forgery.
"""

import hashlib
import json
import os
from pathlib import Path

SCHEMA = "spatio.clearance_provenance.v1"
# Inputs whose change must block reuse (hard error), and inputs checked when supplied.
BOUND = ("trajectory", "extrinsics")
CHECKED_IF_SUPPLIED = ("recording", "scans")
HEX = set("0123456789abcdef")

LIMITATIONS = {
    "no_recording_identity":
        "raw recording not supplied; rays are not tied to a recording",
    "no_time_basis":
        "timestamp interpretation not supplied; times are uninterpreted input seconds",
    "no_extrinsics_identity":
        "calibration/extrinsics not supplied; ray origins and hits are unverified",
    "no_trajectory_identity":
        "trajectory revision not supplied; ray origins and hits are unverified",
}
REUSE_LIMITATIONS = {
    "recording_not_reverified": "recording not resupplied at reuse; its hash was not rechecked",
    "scans_not_reverified": "scans not resupplied at reuse; ray evidence hash was not rechecked",
}


def sha256_path(path):
    """sha256 of a file's bytes, or of a directory tree (relative names and file hashes)."""
    path = Path(path)
    if path.is_dir():
        entries = []
        for root, dirs, files in os.walk(path):
            dirs.sort()
            for name in sorted(files):
                full = Path(root) / name
                entries.append([full.relative_to(path).as_posix(), _sha_file(full)])
        if not entries:
            raise ValueError(f"{path}: directory has no files to identify")
        return canonical_sha256({"tree": entries})
    return _sha_file(path)


def _sha_file(path):
    digest = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def canonical_sha256(value):
    canon = json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False)
    return hashlib.sha256(canon.encode("utf-8")).hexdigest()


def zone_geometry_sha256(frame, zone):
    """Parsed geometry only: IDs, rules and provisions are checked separately."""
    return canonical_sha256({"frame_id": frame, "shape": zone.shape,
                             "low": zone.low.tolist(), "high": zone.high.tolist(),
                             "axis": zone.axis, "direction": zone.direction})


def _hash(value, name):
    if value is not None and (not isinstance(value, str) or len(value) != 64
                              or not set(value) <= HEX):
        raise ValueError(f"{name} sha256 must be null or 64 lowercase hex characters")
    return value


def build(scans_data, recording=None, extrinsics=None, trajectory=None, time_basis=None):
    """Provenance block for a new result. Inputs are sha256 hex strings or None."""
    if time_basis is not None and (not isinstance(time_basis, str) or not time_basis.strip()):
        raise ValueError("time_basis must be nonempty text when given")
    identities = {"recording": _hash(recording, "recording"),
                  "extrinsics": _hash(extrinsics, "extrinsics"),
                  "trajectory": _hash(trajectory, "trajectory")}
    limitations = [key for key, name in (
        ("no_recording_identity", "recording"), ("no_extrinsics_identity", "extrinsics"),
        ("no_trajectory_identity", "trajectory")) if identities[name] is None]
    if time_basis is None:
        limitations.append("no_time_basis")
    return {
        "schema": SCHEMA,
        "recording_sha256": identities["recording"],
        "time_basis": time_basis,
        "extrinsics_sha256": identities["extrinsics"],
        "trajectory_sha256": identities["trajectory"],
        "scans_sha256": canonical_sha256(scans_data),
        "limitations": sorted(limitations),
    }


def check(provenance, supplied):
    """Hard error unless supplied inputs (sha256 hex or None) match the recorded ones.

    Trajectory and extrinsics are bound: a recorded hash must be resupplied and
    match, and one supplied for an unbound result is also a mismatch. Recording
    and scans are compared when supplied. Returns limitation keys for this reuse.
    """
    if not isinstance(provenance, dict) or provenance.get("schema") != SCHEMA:
        raise ValueError(f"results provenance must have schema {SCHEMA}")
    for key in ("recording_sha256", "extrinsics_sha256", "trajectory_sha256", "scans_sha256"):
        _hash(provenance[key], key)
    if provenance["scans_sha256"] is None:
        raise ValueError("results provenance has no scans_sha256 (ray evidence)")
    limitations = provenance["limitations"]
    if (not isinstance(limitations, list) or limitations != sorted(set(limitations))
            or not set(limitations) <= set(LIMITATIONS)):
        raise ValueError("results provenance limitations are not known sorted keys")
    for name in ("recording", "extrinsics", "trajectory"):
        if (provenance[f"{name}_sha256"] is None) != (f"no_{name}_identity" in limitations):
            raise ValueError(f"results provenance {name} limitation disagrees with its hash")
    if (provenance["time_basis"] is None) != ("no_time_basis" in limitations):
        raise ValueError("results provenance time_basis limitation disagrees with its value")
    for name in BOUND:
        recorded, given = provenance[f"{name}_sha256"], supplied.get(name)
        if recorded is None and given is not None:
            raise ValueError(f"{name} supplied (sha256 {given}) but the results are not bound "
                             f"to any {name}; recompute clearance")
        if recorded is not None and given is None:
            raise ValueError(f"results are bound to {name} sha256 {recorded}; supply it "
                             "to confirm it is unchanged")
        if recorded != given:
            raise ValueError(f"{name} changed: sha256 {given} is not the results' "
                             f"{recorded}; recompute clearance")
    reuse = []
    for name in CHECKED_IF_SUPPLIED:
        recorded, given = provenance[f"{name}_sha256"], supplied.get(name)
        if given is None:
            if recorded is not None:
                reuse.append(f"{name}_not_reverified")
        elif given != recorded:
            raise ValueError(f"{name} sha256 {given} is not the results' {recorded}")
    return reuse
