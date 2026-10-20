#!/usr/bin/env bash
# Walk day: the three-run table (g3) from runs already scored by process.sh.
#
#   spatio/walkday/series.sh <run-id> <run-id> <run-id> [...] [--sessions-root DIR] [--series-id ID]
#
# List every attempt, failed ones included: the series needs three CONSECUTIVE
# counted runs, and dropping a failure would hide it. spatio.series orders them
# by each walk_result's recorded_at (the bag's start time), not by this list.
# Writes <sessions-root>/series/<series-id>/series.{md,json} (the walk-day verdict:
# spatio.series plus spatio.walkday.seriesgate G1/G2) and spatio_series/ (its own).
# Exit codes are spatio.series compare's own: 0 PASS, 3 FAIL, 4 INCOMPLETE,
#   5 NOT_EVALUABLE, 6 REPLAY_ONLY (no physical evidence), 1 inputs rejected
#   (e.g. missing or tied recorded_at), 2 unreadable input (a run never scored)
#   or usage error. Note 3 here is FAIL, not process.sh's usage error.
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
    -*) sed -n '2,15p' "$0" | sed 's/^# \{0,1\}//'; exit 2 ;;
    *) RUNS+=("$1"); shift ;;
  esac
done
[[ ${#RUNS[@]} -ge 1 ]] || { sed -n '2,15p' "$0" | sed 's/^# \{0,1\}//'; exit 2; }
[[ -n $SERIES_ID ]] || SERIES_ID="$(IFS=+; echo "${RUNS[*]}")"
[[ $SERIES_ID =~ ^[A-Za-z0-9][A-Za-z0-9_.+-]*$ ]] || { echo "bad series id: $SERIES_ID" >&2; exit 2; }

OUT="$ROOT/series/$SERIES_ID"
STAGED="$OUT/spatio_series"   # spatio.series' own verdict; the walk-day gate writes the final one
# Never leave an old verdict, including when a run below has lost its walk_result.json.
rm -f "$OUT/series.md" "$OUT/series.json" "$STAGED/series.md" "$STAGED/series.json"

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

mkdir -p "$STAGED"
python3 -m spatio.series compare "${FILES[@]}" --out "$STAGED" --series-id "$SERIES_ID"
rc=$?
case $rc in
  0|3|4|5|6) ;;
  1) echo "!! spatio.series rejected the inputs (exit 1; see the message above)" >&2; exit 1 ;;
  *) echo "!! spatio.series could not read the inputs (exit $rc)" >&2; exit "$rc" ;;
esac
# Walk-day rules spatio.series does not know (seriesgate.py docstring): the same
# walk-day settings and driver config in every run (G1), and a run counts only
# with both of the spec's preselected wall patches verified (G2).
python3 -m spatio.walkday.seriesgate "$STAGED" "$OUT" "${FILES[@]}"
rc=$?
case $rc in
  0|3|4|5|6) ;;
  *) echo "!! walk-day series gate failed (exit $rc); no series verdict written" >&2; exit "$rc" ;;
esac
VERDICT="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["verdict"])' "$OUT/series.json")"
[[ $VERDICT == NOT_EVALUABLE ]] && echo "!! NOT-EVALUABLE series: the runs do not share one config, FAST-LIO revision, walk-day settings or driver config (see series.md)"
echo "========================================================================"
echo "SERIES $SERIES_ID: $VERDICT"
echo "table: $OUT/series.md"
echo "========================================================================"
exit "$rc"
