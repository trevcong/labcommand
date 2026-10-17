"""JSONL topic log. Stdlib only. NaN is rejected; missing numbers are null."""

import json
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
    path.write_text(text, encoding="utf-8")
