#!/usr/bin/env bash
# End-to-end walk-day dry run on SYNTHETIC marked loops (no physical walk).
#
#   spatio/walkday/synth/dryrun.sh [out-root]      (default ~/Developer/spatio-data/walkday-dryrun)
#
# Builds four synthetic recordings (topics /synth/lidar_points + /synth/imu_data
# at 250 Hz, so nothing can lean on Mid-360 names or rates), then runs the real
# process.sh on each and series.sh on all four in walking order:
#   pass-a  true return 5 cm, placement path exercised (recorded, never applied)
#   fail    true return 15 cm
#   pass-b, pass-c  true return 5 cm, other seeds
# Expected: pass-* PASS, fail FAIL, series REPLAY_ONLY (synthetic is never physical).
set -uo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
WD="$(cd "$HERE/.." && pwd)"
OUT="${1:-$HOME/Developer/spatio-data/walkday-dryrun}"
SY="$OUT/recordings"
SPEC="$OUT/spec/SPEC-synthetic.frozen.json"
mkdir -p "$SY" "$OUT/spec" "$OUT/sessions"
STAMP="$(date +%Y%m%dT%H%M%S)"

mkrec() {  # name offset seed
  [[ -d $SY/$1 ]] || "$HERE/make_recording.sh" "$SY/$1" "$2" --seed "$3" || exit 3
}
mkrec pass-a 0.05 11
mkrec fail 0.15 11
mkrec pass-b 0.05 12
mkrec pass-c 0.05 13

if [[ ! -f $SPEC ]]; then
  REV="$(git -C "${SPATIO_SRC:-$HOME/Developer/spatio-data/spark-src}/FAST_LIO_ROS2" rev-parse HEAD)"
  ( cd "$WD/../.." && python3 -m spatio.walkday.spec_draft --config "$SY/pass-a/fastlio_synthetic.yaml" \
      --fastlio-revision "$REV" --fill "$HERE/spec_fill.json" --out "$OUT/spec/SPEC-synthetic.draft.json" \
    && python3 -m spatio.expspec freeze "$OUT/spec/SPEC-synthetic.draft.json" --out "$SPEC" ) || exit 3
fi

declare -a IDS
run() {  # name [process.sh options]
  local name=$1; shift
  local id="dry-$STAMP-$name"
  IDS+=("$id")
  "$WD/process.sh" "$SY/$name" "$id" --sessions-root "$OUT/sessions" "$@"
  echo "DRYRUN $name -> exit $? ($OUT/sessions/$id/walkday.md)"
  python3 "$HERE/truth.py" "$SY/$name" "$OUT/sessions/$id" --json "$OUT/sessions/$id/logs/synthetic_truth.json" \
    | python3 -c 'import json,sys; d=json.load(sys.stdin); e=d["position_error_m"]; print("DRYRUN %s estimate vs truth: median %.1f cm, max %.1f cm, parked drift %.1f cm" % (sys.argv[1], e["median"]*100, e["max"]*100, d["drift_while_parked_at_start_m"]*100))' "$name"
}
run pass-a --spec "$SPEC" --placement-cm 0.4 --start-placement-cm 0.3 --heading ok --operator dryrun \
  --method "synthetic: no physical measurement"
run fail
run pass-b
run pass-c
"$WD/series.sh" "${IDS[@]}" --sessions-root "$OUT/sessions" --series-id "dry-$STAMP"
echo "DRYRUN series -> exit $?"
