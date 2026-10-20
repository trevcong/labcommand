#!/bin/bash
# Local non-ROS tests. See README "Tests" for setup and interpreter overrides.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

stages=(
  "CI helper unittests"
  "Committed fixtures"
  "Spatio pytest"
  "Spark pytest"
  "rosbags cross-check"
  "C++ configure"
  "C++ build"
  "C++ ctest"
  "Swift core tests"
  "change-detect build"
  "Shared goldens"
  "Spatio fixture regeneration check"
)
results=()
for stage in "${stages[@]}"; do
  results+=("SKIPPED (not reached)")
done
current=0
absent_recordings=""
# SKIP_CPP=1 ends a run whose other stages pass with this code, never 0.
# A stage failure that returns the same code is reported as 1 instead.
PARTIAL_EXIT=10
partial=""

summary() {
  local status=$?
  trap - EXIT
  if [[ -n "$absent_recordings" ]]; then
    rmdir "$absent_recordings" || true
  fi
  local overall=FAIL
  if [[ "$status" == 0 ]]; then
    overall=PASS
  elif [[ "$status" == "$PARTIAL_EXIT" && -n "$partial" && "$current" == "${#stages[@]}" ]]; then
    overall="PARTIAL ($partial)"
  elif [[ "$status" == "$PARTIAL_EXIT" ]]; then
    status=1
  fi
  printf '\nLocal test summary (exit %s)\n' "$status"
  if [[ "$status" != 0 && "${results[$current]:-}" == "SKIPPED (not reached)" ]]; then
    printf 'FAIL: runner setup\n'
  fi
  local i
  for ((i = 0; i < ${#stages[@]}; i++)); do
    printf '%s: %s\n' "${results[$i]}" "${stages[$i]}"
  done
  printf 'Overall: %s\n' "$overall"
  exit "$status"
}
trap summary EXIT

run_stage() {
  printf '\n=== %s ===\n' "${stages[$current]}"
  results[$current]="FAIL"
  "$@" < /dev/null
  results[$current]="PASS"
  current=$((current + 1))
}

# Resolve executables before changing into spark/. Keep the venv path (not its
# realpath) so Python still discovers pyvenv.cfg.
PYTHON="${PYTHON:-$ROOT/spark/.venv-local/bin/python}"
SPATIO_PYTHON="${SPATIO_PYTHON:-$PYTHON}"
resolve_python() {
  local executable
  if ! executable="$(command -v "$1")"; then
    printf 'Python executable not found: %s (see README Tests setup)\n' "$1" >&2
    return 127
  fi
  printf '%s/%s\n' "$(cd "$(dirname "$executable")" && pwd)" "$(basename "$executable")"
}
PYTHON="$(resolve_python "$PYTHON")"
SPATIO_PYTHON="$(resolve_python "$SPATIO_PYTHON")"
printf 'Local gate Python: %s\nSpatio Python: %s\n' "$PYTHON" "$SPATIO_PYTHON"
"$PYTHON" --version < /dev/null
"$SPATIO_PYTHON" --version < /dev/null

spark_pytest() {
  cd "$ROOT/spark"
  "$PYTHON" -m pytest -q -rs
  cd "$ROOT"
}

cpp_ctest() {
  # Never fall back to the developer's full recordings. A fresh temporary
  # parent guarantees these paths are absent without deleting any user data.
  absent_recordings="$(mktemp -d "${TMPDIR:-/tmp}/spatio-test-all.XXXXXX")"
  export ROOM_MEMORY_RECORDING="$absent_recordings/static5m"
  export ROOM_MEMORY_RECORDING_B="$absent_recordings/static5m-b"
  ctest --test-dir "$ROOT/build/room_memory_cpp" --output-on-failure \
    --verbose --no-tests=error
}

run_stage "$PYTHON" -m unittest discover -s .github/scripts -p 'test_*.py'
run_stage "$PYTHON" .github/scripts/ci_checks.py fixtures "$ROOT"
run_stage "$SPATIO_PYTHON" -m pytest -q -rs spatio .github/scripts
run_stage spark_pytest
# pytest skips test_rosbags.py without rosbags; this script fails instead (issue #73).
run_stage env PY="$PYTHON" bash scripts/check-rosbags.sh

if [[ "${SKIP_CPP:-0}" == 1 ]]; then
  partial="SKIP_CPP=1: 3 C++ stages skipped"
  for ((i = 0; i < 3; i++)); do
    results[$current]="SKIPPED (SKIP_CPP=1)"
    current=$((current + 1))
  done
else
  run_stage cmake -S spark/room_memory_cpp -B build/room_memory_cpp \
    -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
    -DROOM_MEMORY_BUILD_NODE=OFF -DROOM_MEMORY_NATIVE=OFF \
    -DROOM_MEMORY_OPENMP="${ROOM_MEMORY_OPENMP:-ON}" \
    -DROOM_MEMORY_PYTHON="$PYTHON"
  run_stage cmake --build build/room_memory_cpp --parallel
  run_stage cpp_ctest
fi

run_stage swift run LabCommandCoreTestsRunner
run_stage swift build --product change-detect
run_stage bash scripts/check-shared-goldens.sh
run_stage "$SPATIO_PYTHON" -m spatio.io.make_fixtures --out spatio/io/fixtures --check

if [[ -n "$partial" ]]; then
  exit "$PARTIAL_EXIT"
fi
