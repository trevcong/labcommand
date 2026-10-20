"""Write manifest.json for one session folder."""

from __future__ import annotations

import shutil
from datetime import datetime
from pathlib import Path

from spatio.manifest.checks import read_json_object
from spatio.manifest.findings import (
    REPLAY_LABEL,
    ManifestBuildError,
    ManifestUnreadable,
)
from spatio.manifest.hashing import hash_file
from spatio.manifest.jsonio import write_json_atomic
from spatio.manifest.locate import (
    bag_files_next_to,
    find_metadata_yaml,
    locate_listed_file,
    pcd_names_in,
    relative_bag_name,
)
from spatio.manifest.physical import validate_physical_reference


# Output roots owned by other work, or by the raw copy. Never write there.
_FORBIDDEN_ROOTS = (
    Path.home() / "Developer" / "spatio-data" / "sessions",
    Path.home() / "Developer" / "spatio-data" / "recordings",
    Path.home() / "Developer" / "spatio-data" / "spark-src",
)


def assert_output_allowed(session_dir):
    """Refuse destinations under the shared recording and session trees.

    The protected root does not have to exist. Skipping a missing root
    would let ``mkdir`` create it and write the manifest underneath.
    """
    resolved = Path(session_dir).expanduser().resolve()
    for root in _FORBIDDEN_ROOTS:
        root_resolved = root.expanduser().resolve()
        if resolved == root_resolved or root_resolved in resolved.parents:
            raise ManifestBuildError(
                "refusing to write a manifest under %s" % root_resolved
            )


def _require_text(name, value):
    if not isinstance(value, str):
        raise ManifestBuildError("%s must be a string" % name)
    return value


def _local_created_at():
    return datetime.now().astimezone().isoformat(timespec="seconds")


def _artifact_names(session_dir, source_recording, config_path):
    """Names to hash into ``files``, excluding the config basename."""
    names = []

    def add(name):
        if name and name != config_path.name and name not in names:
            names.append(name)

    add("trajectory.tum")
    add("map.pcd")
    for directory in (session_dir, source_recording):
        if directory is None:
            continue
        path = Path(directory)
        if not path.is_dir():
            continue
        for pcd_name in pcd_names_in(path):
            add(pcd_name)
        trajectory = path / "trajectory.tum"
        try:
            if trajectory.is_file():
                add("trajectory.tum")
        except OSError:
            continue
    return names


def _files_block(session_dir, source_recording, config_path, config_digest, config_bytes):
    files = {
        config_path.name: {"sha256": config_digest, "bytes": config_bytes},
    }
    for name in _artifact_names(session_dir, str(source_recording), config_path):
        found = locate_listed_file(name, session_dir, str(source_recording), str(config_path))
        if found is None:
            continue
        digest, nbytes = hash_file(found)
        files[name] = {"sha256": digest, "bytes": nbytes}
    return files


def _source_fingerprint(source_recording, hash_bag):
    metadata = find_metadata_yaml(None, source_recording)
    if metadata is None:
        return None
    metadata_digest, _metadata_bytes = hash_file(metadata)
    bags = []
    for bag in bag_files_next_to(metadata):
        if hash_bag:
            digest, nbytes = hash_file(bag)
        else:
            digest = None
            try:
                nbytes = bag.stat().st_size
            except OSError as exc:
                raise ManifestUnreadable("bag file is unreadable: %s" % exc)
        bags.append(
            {
                "name": relative_bag_name(bag, source_recording),
                "bytes": nbytes,
                "sha256": digest,
            }
        )
    return {
        "metadata_yaml_sha256": metadata_digest,
        "bag_files": bags,
    }


def build_manifest(
    session_dir,
    run_id,
    source_recording,
    sensor,
    fastlio_revision,
    config_file,
    image,
    notes,
    physical_reference=None,
    hash_bag=False,
    force=False,
):
    """Write ``<session_dir>/manifest.json`` and return the object written.

    With no physical reference the manifest says physical accuracy was not
    evaluated and carries the exact replay label. With a reference, the file
    is checked, copied into the session folder, and the flag is set true.
    """
    session_dir = Path(session_dir).expanduser()
    # Before mkdir, and before any large read.
    assert_output_allowed(session_dir)

    run_id = _require_text("run_id", run_id)
    source_recording = _require_text("source_recording", source_recording)
    sensor = _require_text("sensor", sensor)
    fastlio_revision = _require_text("fastlio_revision", fastlio_revision)
    image = _require_text("image", image)
    notes = _require_text("notes", notes)
    if session_dir.name != run_id:
        raise ManifestBuildError(
            "run_id %r does not equal session folder name %r"
            % (run_id, session_dir.name)
        )

    manifest_path = session_dir / "manifest.json"
    if manifest_path.exists() and not force:
        raise ManifestBuildError(
            "manifest.json already exists at %s; pass --force to overwrite"
            % manifest_path
        )

    config_path = Path(config_file).expanduser()
    if not config_path.is_file():
        raise ManifestUnreadable("config file is not readable: %s" % config_path)
    config_path = config_path.resolve()
    config_digest, config_bytes = hash_file(config_path)

    source_path = Path(source_recording).expanduser()
    # Keep a stable absolute path even when the recording directory is absent.
    try:
        source_stored = str(source_path.resolve())
    except OSError:
        source_stored = str(source_path.absolute())

    reference_data = None
    reference_source = None
    if physical_reference is not None:
        reference_source = Path(physical_reference).expanduser()
        if not reference_source.is_file():
            raise ManifestUnreadable(
                "physical reference is not readable: %s" % reference_source
            )
        try:
            reference_data = read_json_object(reference_source)
        except ManifestUnreadable as exc:
            raise ManifestBuildError("physical reference is invalid: %s" % exc)
        reference_findings = validate_physical_reference(
            reference_data, expected_run_id=run_id
        )
        reference_errors = [
            item for item in reference_findings if item.severity == "ERROR"
        ]
        if reference_errors:
            detail = "; ".join(str(item) for item in reference_errors)
            raise ManifestBuildError("physical reference failed validation: %s" % detail)

    evaluated = reference_data is not None
    manifest = {
        "run_id": run_id,
        "source_recording": source_stored,
        "sensor": sensor,
        "fastlio_revision": fastlio_revision,
        "config_file": str(config_path),
        "config_sha256": config_digest,
        "image": image,
        "created_at": _local_created_at(),
        "notes": notes,
        "physical_accuracy_evaluated": evaluated,
    }
    if not evaluated:
        manifest["accuracy_label"] = REPLAY_LABEL

    fingerprint = _source_fingerprint(source_stored, hash_bag=bool(hash_bag))
    manifest["files"] = _files_block(
        session_dir,
        source_stored,
        config_path,
        config_digest,
        config_bytes,
    )
    if fingerprint is not None:
        manifest["source_fingerprint"] = fingerprint

    session_dir.mkdir(parents=True, exist_ok=True)
    # Recheck after mkdir so a race still needs --force.
    if manifest_path.exists() and not force:
        raise ManifestBuildError(
            "manifest.json already exists at %s; pass --force to overwrite"
            % manifest_path
        )

    if reference_source is not None:
        destination = session_dir / "physical_reference.json"
        if reference_source.resolve() != destination.resolve():
            shutil.copyfile(str(reference_source), str(destination))

    write_json_atomic(manifest_path, manifest)
    return manifest
