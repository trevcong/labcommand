"""Check a session folder against the manifest rules. This module never writes."""

from __future__ import annotations

from pathlib import Path

from spatio.manifest.checks import (
    is_bool,
    is_int,
    is_sha256_hex,
    parse_iso8601_with_offset,
    read_json_object,
)
from spatio.manifest.findings import (
    REPLAY_LABEL,
    KNOWN_KEYS,
    REQUIRED_KEYS,
    STRING_KEYS,
    ManifestUnreadable,
    error,
    warning,
)
from spatio.manifest.hashing import hash_file
from spatio.manifest.locate import (
    find_metadata_yaml,
    locate_bag_file,
    locate_config_file,
    locate_listed_file,
    safe_relative_name,
)
from spatio.manifest.physical import validate_physical_reference


def _contains_label(value):
    return isinstance(value, str) and REPLAY_LABEL in value


def _check_required(data):
    findings = []
    for key in REQUIRED_KEYS:
        if key not in data:
            findings.append(error("M1", "missing required key: %s" % key))
            continue
        value = data[key]
        if key == "physical_accuracy_evaluated":
            if not is_bool(value):
                findings.append(
                    error("M1", "physical_accuracy_evaluated must be a boolean")
                )
        elif key in STRING_KEYS and not isinstance(value, str):
            findings.append(error("M1", "%s must be a string" % key))
    sha = data.get("config_sha256")
    if isinstance(sha, str) and not is_sha256_hex(sha):
        findings.append(
            error(
                "M1",
                "config_sha256 must be 64 lowercase hexadecimal characters",
            )
        )
    return findings


def _check_extras(data):
    findings = []
    for key in sorted(data):
        if key not in KNOWN_KEYS:
            findings.append(warning("M2", "unknown key: %s" % key))
    return findings


def _check_label(data):
    flag = data.get("physical_accuracy_evaluated")
    if not is_bool(flag) or flag is True:
        return []
    if _contains_label(data.get("accuracy_label")) or _contains_label(data.get("notes")):
        return []
    return [
        error(
            "M3",
            "physical_accuracy_evaluated is false but the exact label %r "
            "is not in accuracy_label or notes" % REPLAY_LABEL,
        )
    ]


def _check_run_id(data, session):
    run_id = data.get("run_id")
    if not isinstance(run_id, str):
        return []
    if run_id != session.name:
        return [
            error(
                "M1",
                "run_id %r does not equal session folder name %r"
                % (run_id, session.name),
            )
        ]
    return []


def _check_created_at(data):
    created_at = data.get("created_at")
    if not isinstance(created_at, str):
        return []
    if parse_iso8601_with_offset(created_at) is None:
        return [
            error("M1", "created_at must be ISO 8601 with a timezone offset")
        ]
    return []


def _check_config_hash(data, session):
    sha = data.get("config_sha256")
    config_file = data.get("config_file")
    if not isinstance(config_file, str) or not is_sha256_hex(sha):
        return []
    found = locate_config_file(config_file, session)
    if found is None:
        return [
            warning(
                "M5",
                "%s: config not verifiable here (%s)" % (session, config_file),
            )
        ]
    try:
        digest, _nbytes = hash_file(found)
    except OSError as exc:
        # The path exists but cannot be read. That is the M5 warning,
        # not an exception that aborts the rest of the sessions.
        return [
            warning(
                "M5",
                "%s: config not verifiable here (%s)" % (session, exc),
            )
        ]
    if digest != sha:
        return [
            error(
                "M5",
                "%s: config sha256 mismatch for %s: manifest %s, file %s"
                % (session, found, sha, digest),
            )
        ]
    return []


def _check_files(data, session):
    if "files" not in data:
        return []
    files = data["files"]
    if not isinstance(files, dict):
        return [error("M6", "%s: files must be an object" % session)]
    findings = []
    source = data.get("source_recording")
    config_file = data.get("config_file")
    for name in sorted(files):
        info = files[name]
        prefix = "%s: files[%r]" % (session, name)
        if not isinstance(info, dict):
            findings.append(error("M6", "%s must be an object" % prefix))
            continue
        for extra in sorted(info):
            if extra not in ("sha256", "bytes"):
                findings.append(
                    warning("M2", "%s: unknown key in %s: %s" % (session, name, extra))
                )
        if not safe_relative_name(name):
            findings.append(error("M6", "%s: file name is not a plain relative path" % prefix))
            continue
        sha = info.get("sha256")
        nbytes = info.get("bytes")
        shape_ok = True
        if not is_sha256_hex(sha):
            findings.append(error("M6", "%s: sha256 must be 64 lowercase hex" % prefix))
            shape_ok = False
        if not is_int(nbytes) or nbytes < 0:
            findings.append(error("M6", "%s: bytes must be an integer >= 0" % prefix))
            shape_ok = False
        if not shape_ok:
            continue
        found = locate_listed_file(name, session, source, config_file)
        if found is None:
            findings.append(error("M6", "%s: listed file is missing" % prefix))
            continue
        try:
            digest, actual = hash_file(found)
        except OSError as exc:
            findings.append(
                error(
                    "M6",
                    "%s: listed file is not readable (%s)" % (prefix, exc),
                )
            )
            continue
        if digest != sha or actual != nbytes:
            findings.append(
                error(
                    "M6",
                    "%s: hash or size mismatch (manifest %s %s bytes, file %s %s bytes)"
                    % (prefix, sha, nbytes, digest, actual),
                )
            )
    return findings


def _check_one_bag(session, source, entry, index):
    prefix = "%s: source_fingerprint.bag_files[%d]" % (session, index)
    findings = []
    if not isinstance(entry, dict):
        return [error("M7", "%s must be an object" % prefix)]
    for extra in sorted(entry):
        if extra not in ("name", "bytes", "sha256"):
            findings.append(
                warning("M2", "%s: unknown key in bag file entry: %s" % (session, extra))
            )
    name = entry.get("name")
    nbytes = entry.get("bytes")
    sha = entry.get("sha256", None)
    if not isinstance(name, str):
        findings.append(error("M7", "%s: name must be a string" % prefix))
        return findings
    if not is_int(nbytes) or nbytes < 0:
        findings.append(error("M7", "%s: bytes must be an integer >= 0" % prefix))
    sha_ok = sha is None or is_sha256_hex(sha)
    if not sha_ok:
        findings.append(
            error("M7", "%s: sha256 must be 64 lowercase hex or null" % prefix)
        )
    if not safe_relative_name(name):
        findings.append(error("M7", "%s: name is not a plain relative path" % prefix))
        return findings
    if not sha_ok or not is_int(nbytes) or nbytes < 0:
        return findings
    found = locate_bag_file(name, session, source)
    if found is None:
        # A null hash means the bag was deliberately not hashed. Absence is OK.
        if sha is not None:
            findings.append(
                warning("M7", "%s: bag file not verifiable here (%s)" % (session, name))
            )
        return findings
    try:
        size = found.stat().st_size
    except OSError as exc:
        findings.append(warning("M7", "%s: bag file not verifiable here (%s)" % (session, exc)))
        return findings
    if size != nbytes:
        findings.append(
            error(
                "M7",
                "%s: byte count mismatch for %s (manifest %s, file %s)"
                % (prefix, name, nbytes, size),
            )
        )
    if sha is not None:
        digest, actual = hash_file(found)
        if digest != sha or actual != nbytes:
            findings.append(
                error(
                    "M7",
                    "%s: hash mismatch for %s (manifest %s, file %s)"
                    % (prefix, name, sha, digest),
                )
            )
    return findings


def _check_fingerprint(data, session):
    if "source_fingerprint" not in data:
        return []
    fingerprint = data["source_fingerprint"]
    if not isinstance(fingerprint, dict):
        return [error("M7", "%s: source_fingerprint must be an object" % session)]
    findings = []
    for extra in sorted(fingerprint):
        if extra not in ("metadata_yaml_sha256", "bag_files"):
            findings.append(
                warning(
                    "M2",
                    "%s: unknown key in source_fingerprint: %s" % (session, extra),
                )
            )
    sha = fingerprint.get("metadata_yaml_sha256", None)
    if not is_sha256_hex(sha):
        findings.append(
            error(
                "M7",
                "%s: metadata_yaml_sha256 must be 64 lowercase hex" % session,
            )
        )
    else:
        source = data.get("source_recording")
        metadata = find_metadata_yaml(
            session, source if isinstance(source, str) else None
        )
        if metadata is None:
            findings.append(
                warning("M7", "%s: metadata.yaml not verifiable here" % session)
            )
        else:
            digest, _nbytes = hash_file(metadata)
            if digest != sha:
                findings.append(
                    error(
                        "M7",
                        "%s: metadata.yaml sha256 mismatch (manifest %s, file %s)"
                        % (session, sha, digest),
                    )
                )
    bags = fingerprint.get("bag_files", None)
    if not isinstance(bags, list):
        findings.append(error("M7", "%s: bag_files must be a list" % session))
        return findings
    source = data.get("source_recording")
    source = source if isinstance(source, str) else None
    for index, entry in enumerate(bags):
        findings.extend(_check_one_bag(session, source, entry, index))
    return findings


def _check_physical(data, session):
    flag = data.get("physical_accuracy_evaluated")
    reference_path = session / "physical_reference.json"
    exists = False
    try:
        exists = reference_path.is_file()
    except OSError:
        exists = False
    findings = []
    if not exists:
        if is_bool(flag) and flag is True:
            findings.append(
                error(
                    "M4",
                    "%s: physical_accuracy_evaluated is true but "
                    "physical_reference.json is missing" % session,
                )
            )
        return findings

    try:
        reference = read_json_object(reference_path)
    except ManifestUnreadable as exc:
        findings.append(
            error(
                "M4" if is_bool(flag) and flag is True else "S3",
                "%s: physical_reference.json could not be read: %s" % (session, exc),
            )
        )
        return findings

    reference_findings = validate_physical_reference(
        reference, expected_run_id=session.name
    )
    findings.extend(reference_findings)
    if is_bool(flag) and flag is True:
        reference_errors = [
            item for item in reference_findings if item.severity == "ERROR"
        ]
        if reference_errors:
            findings.append(
                error(
                    "M4",
                    "%s: physical_accuracy_evaluated is true but "
                    "physical_reference.json is not valid" % session,
                )
            )
    return findings


def validate_session(path):
    """Return findings for one session directory.

    Raises ManifestUnreadable when the directory or manifest.json cannot be
    read as a JSON object. Those are CLI exit 2, not schema errors.
    """
    session = Path(path)
    if not session.exists():
        raise ManifestUnreadable("session directory does not exist: %s" % session)
    if not session.is_dir():
        raise ManifestUnreadable("session path is not a directory: %s" % session)
    manifest_path = session / "manifest.json"
    if not manifest_path.is_file():
        raise ManifestUnreadable("manifest.json is missing in %s" % session)
    try:
        data = read_json_object(manifest_path)
    except ManifestUnreadable:
        raise

    findings = []
    findings.extend(_check_required(data))
    findings.extend(_check_extras(data))
    findings.extend(_check_run_id(data, session))
    findings.extend(_check_created_at(data))
    findings.extend(_check_label(data))
    findings.extend(_check_config_hash(data, session))
    findings.extend(_check_files(data, session))
    findings.extend(_check_fingerprint(data, session))
    findings.extend(_check_physical(data, session))
    return findings
