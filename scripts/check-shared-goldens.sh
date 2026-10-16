#!/bin/bash
# Every hand golden the Swift suite reads must be byte-identical to the Python/C++ copy.
# Usage: scripts/check-shared-goldens.sh   (exit 0 when every shared file matches)
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SWIFT="${ROOT}/Tests/Fixtures/room/golden/hand"
SPARK="${ROOT}/spark/room_memory/tests/fixtures/hand"

status=0
count=0
for file in "${SWIFT}"/*.hex; do
  name="$(basename "$file")"
  if [[ ! -f "${SPARK}/${name}" ]]; then
    echo "missing: ${SPARK}/${name}" >&2
    status=1
  elif ! cmp -s "$file" "${SPARK}/${name}"; then
    echo "differs: ${name}" >&2
    status=1
  fi
  count=$((count + 1))
done
if [[ $count -eq 0 ]]; then
  echo "no goldens in ${SWIFT}" >&2
  exit 1
fi
[[ $status -eq 0 ]] && echo "shared goldens identical (${count} files)"
exit $status
