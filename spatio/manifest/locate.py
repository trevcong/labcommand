"""Find config, bag, and artifact files without leaving the recording tree."""

from __future__ import annotations

from pathlib import Path


def safe_relative_name(name):
    """True for a relative path that cannot climb out of a root."""
    if not isinstance(name, str) or name == "" or len(name) > 512:
        return False
    if name.startswith("/") or name.startswith("\\"):
        return False
    parts = Path(name).parts
    if not parts or ".." in parts:
        return False
    return True


def _first_file(candidates):
    seen = set()
    for candidate in candidates:
        if candidate is None:
            continue
        key = str(candidate)
        if key in seen:
            continue
        seen.add(key)
        try:
            if candidate.is_file():
                return candidate
        except OSError:
            continue
    return None


def find_metadata_yaml(session_dir, source_recording):
    """ROS 2 layout first: ``<recording>/bag/metadata.yaml``."""
    candidates = []
    if source_recording:
        source = Path(source_recording)
        candidates.append(source / "bag" / "metadata.yaml")
        candidates.append(source / "metadata.yaml")
    if session_dir:
        session = Path(session_dir)
        candidates.append(session / "bag" / "metadata.yaml")
        candidates.append(session / "metadata.yaml")
    return _first_file(candidates)


def bag_files_next_to(metadata_path):
    """``*.mcap`` and ``*.db3`` in the same directory as metadata.yaml."""
    folder = Path(metadata_path).parent
    found = []
    try:
        children = list(folder.iterdir())
    except OSError:
        return found
    for child in children:
        try:
            if child.is_file() and child.suffix.lower() in (".mcap", ".db3"):
                found.append(child)
        except OSError:
            continue
    found.sort(key=lambda path: path.name)
    return found


def relative_bag_name(bag_path, source_recording):
    bag_path = Path(bag_path)
    if source_recording:
        try:
            relative = bag_path.resolve().relative_to(Path(source_recording).resolve())
            return relative.as_posix()
        except (ValueError, OSError):
            pass
    return bag_path.name


def locate_config_file(config_file, session_dir):
    if not isinstance(config_file, str) or config_file == "":
        return None
    path = Path(config_file)
    candidates = [path]
    if session_dir is not None and not path.is_absolute():
        candidates.append(Path(session_dir) / path)
    if session_dir is not None:
        candidates.append(Path(session_dir) / path.name)
    return _first_file(candidates)


def locate_listed_file(name, session_dir, source_recording, config_file):
    """Resolve one ``files`` entry.

    The config path is used only when ``name`` is exactly that file's
    basename, so a session file of the same name cannot shadow it. A longer
    relative path is resolved in full: ``archive/sensor.yaml`` is not the
    config, and it is not a file under ``bag/`` that merely shares the
    last component.
    """
    if not safe_relative_name(name):
        return None
    candidates = []
    if isinstance(config_file, str) and config_file and name == Path(config_file).name:
        candidates.append(Path(config_file))
    if session_dir is not None:
        candidates.append(Path(session_dir) / name)
    if isinstance(source_recording, str) and source_recording:
        source = Path(source_recording)
        candidates.append(source / name)
        if Path(name).name == name:
            candidates.append(source / "bag" / name)
    return _first_file(candidates)


def locate_bag_file(name, session_dir, source_recording):
    if not safe_relative_name(name):
        return None
    candidates = []
    basename = Path(name).name
    if isinstance(source_recording, str) and source_recording:
        source = Path(source_recording)
        candidates.append(source / name)
        candidates.append(source / "bag" / basename)
    if session_dir is not None:
        session = Path(session_dir)
        candidates.append(session / name)
        candidates.append(session / "bag" / basename)
    return _first_file(candidates)


def pcd_names_in(directory):
    """Basenames of ``*.pcd`` files one level under ``directory``."""
    directory = Path(directory)
    names = []
    try:
        children = list(directory.iterdir())
    except OSError:
        return names
    for child in children:
        try:
            if child.is_file() and child.suffix.lower() == ".pcd":
                names.append(child.name)
        except OSError:
            continue
    names.sort()
    return names
