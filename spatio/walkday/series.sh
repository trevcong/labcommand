#!/usr/bin/env bash
# Walk day: the three-run table (g3) from runs already scored by process.sh.
#
#   spatio/walkday/series.sh <run-id> <run-id> <run-id> [...] [--sessions-root DIR] [--series-id ID]
#
# List every attempt in the order it was walked, failed ones included: the
# series needs three CONSECUTIVE counted runs, and dropping a failure would hide it.
# Writes <sessions-root>/series/<series-id>/series.{md,json}.
# Exit: 0 PASS; 1 FAIL, INCOMPLETE or REPLAY_ONLY (no physical evidence);
#       2 NOT_EVALUABLE or a run that was never scored; 3 usage error.
set -uo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
export PYTHONPATH="$REPO${PYTHONPATH:+:$PYTHONPATH}"
ROOT="${SPATIO_WALKDAY_ROOT:-$HOME/Developer/spatio-data/walkday}"
SERIES_ID=""
RUNS=()
while [[ $# -gt 0 ]]; do
  case $1 in
    --sessions-root) ROOT=$2; shift 2 ;;
    --series-id) SERIES_ID=$2; shift 2 ;;
    -*) sed -n '2,11p' "$0" | sed 's/^# \{0,1\}//'; exit 3 ;;
    *) RUNS+=("$1"); shift ;;
  esac
done
[[ ${#RUNS[@]} -ge 1 ]] || { sed -n '2,11p' "$0" | sed 's/^# \{0,1\}//'; exit 3; }
[[ -n $SERIES_ID ]] || SERIES_ID="$(IFS=+; echo "${RUNS[*]}")"
[[ $SERIES_ID =~ ^[A-Za-z0-9][A-Za-z0-9_.+-]*$ ]] || { echo "bad series id: $SERIES_ID" >&2; exit 3; }

FILES=() MISSING=0
for r in "${RUNS[@]}"; do
  f="$ROOT/$r/walk_result.json"
  if [[ -f $f ]]; then
    FILES+=("$f")
  else
    MISSING=1
    why="no session folder"
    [[ -f $ROOT/$r/walkday.md ]] && why="$(sed -n '3p;/stopped at/p' "$ROOT/$r/walkday.md" | tr '\n' ' ')"
    echo "!! run $r has no walk_result.json ($why)" >&2
  fi
done
if [[ $MISSING == 1 ]]; then
  echo "!! NOT-EVALUABLE series: every listed attempt must have been scored by process.sh" >&2
  exit 2
fi

OUT="$ROOT/series/$SERIES_ID"
mkdir -p "$OUT"
python3 -m spatio.series compare "${FILES[@]}" --out "$OUT" --series-id "$SERIES_ID"
rc=$?
[[ $rc == 0 ]] || { echo "!! spatio.series rejected the inputs (exit $rc)" >&2; exit 2; }
VERDICT="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["verdict"])' "$OUT/series.json")"
echo "========================================================================"
echo "SERIES $SERIES_ID: $VERDICT"
echo "table: $OUT/series.md"
echo "========================================================================"
case $VERDICT in
  PASS) exit 0 ;;
  NOT_EVALUABLE) exit 2 ;;
  *) exit 1 ;;
esac
