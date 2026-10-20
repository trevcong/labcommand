#!/bin/bash
# The independent CDR cross-check (issue #73): spark/room_memory/tests/test_rosbags.py checks RigPose and
# the D51 codecs against rosbags. pytest skips it without rosbags; this script fails instead.
# Usage: scripts/check-rosbags.sh   (PY=<python> picks the interpreter; it needs pytest, numpy and rosbags)
# The last line is "rosbags PASS", "rosbags SKIPPED", "rosbags FAIL" or "rosbags NOT RUN"; exit 0 only on PASS.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
PY="${PY:-python3}"
REPORTS="$(mktemp -d)"
trap 'rm -rf "$REPORTS"' EXIT

status=0
(cd "${ROOT}/spark" && "$PY" -m pytest -q -rs -p no:cacheprovider room_memory/tests/test_rosbags.py \
  --junitxml="${REPORTS}/pytest.xml" < /dev/null) || status=$?
summary="$("$PY" "${ROOT}/.github/scripts/ci_checks.py" summary "$REPORTS" --kinds pytest --require-rosbags \
  < /dev/null)" || status=1
echo "$summary"

result="$(sed -n 's/^rosbags independent CDR cross-check: \*\*\([A-Z]*\)\*\*.*/\1/p' <<< "$summary")"
[[ $status -ne 0 && $result == PASS ]] && result=FAIL
version="$("$PY" -c 'import rosbags; from importlib.metadata import version; print(version("rosbags"))' \
  2>/dev/null < /dev/null)" || version="not installed"
python="$("$PY" -c 'import platform; print(platform.python_version())' 2>/dev/null < /dev/null)" || python="unknown"
echo "rosbags ${result:-NOT RUN} (rosbags ${version}, Python ${python})"
exit $status
