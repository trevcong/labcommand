"""Atomic writes: directory fsync after the rename, result file mode (issue #102)."""

import json
import os
import stat

import pytest

from spatio.manifest import jsonio

WRITERS = [
    (jsonio.write_json_atomic, {"status": "PASS"}, b'{\n  "status": "PASS"\n}\n'),
    (jsonio.write_text_atomic, "# Walk day\n\nPASS\n", b"# Walk day\n\nPASS\n"),
]


@pytest.fixture
def umask_022():
    old = os.umask(0o022)
    yield
    os.umask(old)


@pytest.mark.skipif(os.name != "posix", reason="directory fsync is POSIX only")
@pytest.mark.parametrize("writer,data,expected", WRITERS)
def test_parent_directory_fsynced_after_replace(tmp_path, monkeypatch, writer, data, expected):
    target = tmp_path / "walk_result.out"
    target.write_bytes(b"earlier\n")
    parent = os.stat(tmp_path)
    real_fsync, real_close = os.fsync, os.close
    synced, closed = [], []

    def fsync(fd):
        st = os.fstat(fd)
        synced.append((stat.S_ISDIR(st.st_mode), (st.st_dev, st.st_ino), fd, target.read_bytes()))
        real_fsync(fd)

    def close(fd):
        closed.append(fd)
        real_close(fd)

    monkeypatch.setattr(jsonio.os, "fsync", fsync)
    monkeypatch.setattr(jsonio.os, "close", close)
    writer(target, data)
    assert [s[0] for s in synced] == [False, True]  # the temp file, then the directory
    is_dir, identity, fd, content_at_sync = synced[1]
    assert identity == (parent.st_dev, parent.st_ino)
    assert content_at_sync == expected  # the directory fsync follows os.replace
    assert fd in closed
    assert target.read_bytes() == expected
    assert list(tmp_path.iterdir()) == [target]


@pytest.mark.parametrize("writer,data,expected", WRITERS)
def test_result_mode_follows_umask_not_mkstemp(tmp_path, umask_022, writer, data, expected):
    fresh, replaced = tmp_path / "fresh.out", tmp_path / "replaced.out"
    replaced.write_bytes(b"earlier\n")
    os.chmod(replaced, 0o600)
    for target in (fresh, replaced):
        writer(target, data)
        assert stat.S_IMODE(os.stat(target).st_mode) == 0o644
        assert target.read_bytes() == expected


@pytest.mark.skipif(os.name != "posix", reason="directory fsync is POSIX only")
def test_directory_fsync_failure_raises_with_new_content_in_place(tmp_path, monkeypatch):
    target = tmp_path / "walk_result.json"
    real_fsync = os.fsync

    def fsync(fd):
        if stat.S_ISDIR(os.fstat(fd).st_mode):
            raise OSError("injected directory fsync")
        real_fsync(fd)

    monkeypatch.setattr(jsonio.os, "fsync", fsync)
    with pytest.raises(OSError, match="directory fsync"):
        jsonio.write_json_atomic(target, {"new": True})
    assert json.loads(target.read_text()) == {"new": True}
    assert list(tmp_path.iterdir()) == [target]
