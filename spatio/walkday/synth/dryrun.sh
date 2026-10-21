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
# Every run gets the frozen synthetic spec and its preselected wall patches
# (spec_fill.json fixes the boxes in the map frame, which is the start pose;
# process.sh binds them to each run's map), so the walls can be verified.
# Expected: pass-* PASS, fail FAIL, series REPLAY_ONLY (synthetic is never physical).
set -uo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
WD="$(cd "$HERE/.." && pwd)"
OUT="${1:-$HOME/Developer/spatio-data/walkday-dryrun}"
SY="$OUT/recordings"
# A changed spec_fill.json freezes a new spec instead of reusing an old one.
FILL_SHA="$(python3 -c 'import hashlib,sys; print(hashlib.sha256(open(sys.argv[1],"rb").read()).hexdigest()[:12])' "$HERE/spec_fill.json")" || exit 3
SPEC="$OUT/spec/SPEC-synthetic-$FILL_SHA.frozen.json"
PATCHES="$OUT/spec/patches-synthetic-$FILL_SHA.json"
mkdir -p "$SY" "$OUT/spec" "$OUT/sessions"
STAMP="$(date +%Y%m%dT%H%M%S)"

mkrec() {  # name offset seed t0 (synthetic clock; the series orders runs by the bag's start time)
  [[ -d $SY/$1 ]] || "$HERE/make_recording.sh" "$SY/$1" "$2" --seed "$3" --set "t0=$4" || exit 3
}
mkrec pass-a 0.05 11 1800000000
mkrec fail 0.15 11 1800003600
mkrec pass-b 0.05 12 1800007200
mkrec pass-c 0.05 13 1800010800
STARTS="$(for n in pass-a fail pass-b pass-c; do python3 "$WD/../replay/recording.py" recorded-at "$SY/$n" || echo "missing-$n"; done)"
if [[ $(sort <<<"$STARTS" | uniq | wc -l) -ne 4 || $STARTS == *missing-* ]]; then
  echo "dryrun: the synthetic recordings in $SY do not have four distinct bag start times" >&2
  echo "        (built before per-recording t0). Delete $SY and rerun." >&2
  exit 3
fi

if [[ ! -f $SPEC ]]; then
  REV="$(git -C "${SPATIO_SRC:-$HOME/Developer/spatio-data/spark-src}/FAST_LIO_ROS2" rev-parse HEAD)"
  ( cd "$WD/../.." && python3 -m spatio.walkday.spec_draft --config "$SY/pass-a/fastlio_synthetic.yaml" \
      --fastlio-revision "$REV" --fill "$HERE/spec_fill.json" --out "$OUT/spec/SPEC-synthetic-$FILL_SHA.draft.json" \
    && python3 -m spatio.expspec freeze "$OUT/spec/SPEC-synthetic-$FILL_SHA.draft.json" --out "$SPEC" ) || exit 3
fi
# The spec's preselected patches as a g8 patches file. No map_sha256: process.sh
# binds the spec's fixed boxes to each run's own map.
python3 - "$SPEC" "$PATCHES" <<'PY' || exit 3
import json, sys
spec = json.load(open(sys.argv[1]))
doc = {"schema": "spatio.wall_patches/1",
       "notes": "SYNTHETIC: the frozen spec's preselected patches (boxes fixed in the map frame)",
       "patches": [{"name": p["name"], "box_map_frame": p["box_map_frame"]} for p in spec["wall_patches"]]}
json.dump(doc, open(sys.argv[2], "w"), indent=2)
PY

declare -a IDS
run() {  # name [process.sh options]
  local name=$1; shift
  local id="dry-$STAMP-$name"
  IDS+=("$id")
  "$WD/process.sh" "$SY/$name" "$id" --sessions-root "$OUT/sessions" --spec "$SPEC" --patches "$PATCHES" "$@"
  echo "DRYRUN $name -> exit $? ($OUT/sessions/$id/walkday.md)"
  python3 "$HERE/truth.py" "$SY/$name" "$OUT/sessions/$id" --json "$OUT/sessions/$id/logs/synthetic_truth.json" \
    | python3 -c 'import json,sys; d=json.load(sys.stdin); e=d["position_error_m"]; print("DRYRUN %s estimate vs truth: median %.1f cm, max %.1f cm, parked drift %.1f cm" % (sys.argv[1], e["median"]*100, e["max"]*100, d["drift_while_parked_at_start_m"]*100))' "$name"
}
run pass-a --placement-cm 0.4 --start-placement-cm 0.3 --heading ok --operator dryrun \
  --method "synthetic: no physical measurement"
run fail
run pass-b
run pass-c
"$WD/series.sh" "${IDS[@]}" --sessions-root "$OUT/sessions" --series-id "dry-$STAMP"
echo "DRYRUN series -> exit $?"
