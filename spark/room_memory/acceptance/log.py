"""JSONL topic log. Stdlib only. NaN is rejected; missing numbers are null."""

import json
import threading
from pathlib import Path

SCHEMA = "room-memory-acceptance-log-v1"


def load_jsonl(path):
    rows = []
    with open(path, "r", encoding="utf-8") as handle:
        for line_number, line in enumerate(handle, start=1):
            if not line.strip():
                continue
            try:
                rows.append(json.loads(line))
            except json.JSONDecodeError as exc:
                raise ValueError(f"{path}:{line_number}: {exc}") from exc
    return rows


def write_jsonl(path, messages):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    text = "".join(json.dumps(message, allow_nan=False) + "\n" for message in messages)
    # Replace in one step so a crash cannot truncate the checkpoint already on disk.
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(text, encoding="utf-8")
    temporary.replace(path)


class TopicLog:
    """Append-only checkpoint. The final ``write_jsonl`` replaces this file.

    ``close`` and ``append`` share a lock so a reader cannot write a line after
    the final replacement has started.
    """

    def __init__(self, path):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._handle = self.path.open("w", encoding="utf-8")
        self._closed = False
        self._lock = threading.Lock()

    def append(self, message):
        line = json.dumps(message, allow_nan=False) + "\n"
        with self._lock:
            if self._closed:
                return
            self._handle.write(line)
            self._handle.flush()

    def close(self):
        with self._lock:
            if not self._closed:
                self._handle.close()
                self._closed = True
