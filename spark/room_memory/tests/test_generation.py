"""Durable publisher generation allocation (protocol §1.4)."""

import os

import pytest

from room_memory import generation as gen
from room_memory.generation import GenerationError, allocate_generation, generation_path, load_generation

NOW = 1760400000000


def test_first_start_takes_the_wall_clock(tmp_path):
    allocation = allocate_generation(tmp_path, "lab", NOW)
    try:
        assert allocation.generation == NOW
        assert generation_path(tmp_path, "lab").read_bytes() == f"{NOW}\n".encode()
        assert not generation_path(tmp_path, "lab").with_suffix(".tmp").exists()
    finally:
        allocation.release()


def test_every_start_rises_even_if_the_clock_goes_back(tmp_path):
    generations = []
    for now in (NOW, NOW, NOW - 5000, NOW + 10):
        allocation = allocate_generation(tmp_path, "lab", now)
        generations.append(allocation.generation)
        allocation.release()
    assert generations == [NOW, NOW + 1, NOW + 2, NOW + 10]


def test_wall_clock_floor_survives_a_lost_file(tmp_path):
    allocate_generation(tmp_path, "lab", NOW).release()
    generation_path(tmp_path, "lab").unlink()
    allocation = allocate_generation(tmp_path, "lab", NOW + 60_000)
    assert allocation.generation == NOW + 60_000
    allocation.release()


def test_state_dir_must_exist_and_is_never_created(tmp_path):
    with pytest.raises(GenerationError, match="empty"):
        allocate_generation("", "lab", NOW)
    with pytest.raises(GenerationError, match="empty"):
        allocate_generation(None, "lab", NOW)
    missing = tmp_path / "room-memory"
    with pytest.raises(GenerationError, match="does not exist"):
        allocate_generation(missing, "lab", NOW)
    assert not missing.exists()
    with pytest.raises(GenerationError, match="plain file name"):
        allocate_generation(tmp_path, "../lab", NOW)


@pytest.mark.parametrize("data", [b"", b"12", b"-3\n", b" 3\n", b"3\n\n", b"18446744073709551616\n", b"abc\n"])
def test_malformed_file_is_never_reset(tmp_path, data):
    path = generation_path(tmp_path, "lab")
    path.parent.mkdir()
    path.write_bytes(data)
    with pytest.raises(GenerationError, match="refusing to reset"):
        allocate_generation(tmp_path, "lab", NOW)
    assert path.read_bytes() == data


def test_overflow_is_refused(tmp_path):
    path = generation_path(tmp_path, "lab")
    path.parent.mkdir()
    path.write_bytes(b"18446744073709551615\n")
    assert load_generation(path) == (1 << 64) - 1
    with pytest.raises(GenerationError, match="overflow"):
        allocate_generation(tmp_path, "lab", NOW)
    path.write_bytes(b"5\n")
    with pytest.raises(GenerationError, match="uint64"):
        allocate_generation(tmp_path, "lab", 1 << 64)


def test_a_live_lock_holder_blocks_a_second_publisher(tmp_path):
    first = allocate_generation(tmp_path, "lab", NOW)
    try:
        with pytest.raises(GenerationError, match="another process"):
            allocate_generation(tmp_path, "lab", NOW)
        assert load_generation(generation_path(tmp_path, "lab")) == NOW  # the refused start wrote nothing
        other = allocate_generation(tmp_path, "garage", NOW)  # one lock per room
        other.release()
    finally:
        first.release()
    second = allocate_generation(tmp_path, "lab", NOW)
    assert second.generation == NOW + 1
    second.release()


def test_unwritable_room_dir_fails_closed(tmp_path):
    room = tmp_path / "lab"
    room.mkdir()
    (room / gen.LOCK_FILE).touch()
    os.chmod(room, 0o500)
    try:
        if os.access(room, os.W_OK):
            pytest.skip("running as a user that ignores directory permissions")
        with pytest.raises(GenerationError, match="durably"):
            allocate_generation(tmp_path, "lab", NOW)
    finally:
        os.chmod(room, 0o700)
    assert not generation_path(tmp_path, "lab").exists()


def test_write_failure_releases_the_lock(tmp_path, monkeypatch):
    def fail(*args, **kwargs):
        raise OSError(28, "No space left on device")

    monkeypatch.setattr(gen, "_write_atomic", fail)
    with pytest.raises(GenerationError, match="No space left"):
        allocate_generation(tmp_path, "lab", NOW)
    monkeypatch.undo()
    allocation = allocate_generation(tmp_path, "lab", NOW)  # not blocked by a leaked lock
    allocation.release()
