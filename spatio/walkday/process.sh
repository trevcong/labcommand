#!/usr/bin/env bash
# Walk day: score one copied recording from start to finish.
#
#   spatio/walkday/process.sh <recording-dir> <run-id> [options]
#
# Stages: copy check + recording start time -> x1 bag timing + input gate -> FAST-LIO replay (A)
#   -> physical reference (g5, only with --placement-cm) -> manifest (g2)
#   -> walk check (B) -> wall patches (g8) -> raw-scan start->end registration
#   (information only) -> per-run result + report.
# Output: <sessions-root>/<run-id>/ with trajectory.tum, map.pcd, manifest.json,
#   walkcheck.{json,md}, topdown.png, wallpatch/, walk_result.json (for series.sh),
#   walkday.md (read this first), logs/.
#
# Options:
#   --placement-cm X        measured END placement residual (cm): cart reference
#                           point to the floor marks after the walk
#   --start-placement-cm Y  measured START residual (cm). Not given = not measured,
#                           and the run then cannot count toward the series
#   --heading ok|off|not_checked   heading check against the second mark
#                           (default not_checked, which cannot count either)
#   --spec FILE             frozen experiment spec (g5); required with a placement.
#                           Default $SPATIO_WALK_SPEC. Its config sha must match.
#   --operator NAME         who measured (default $USER)
#   --method TEXT           how the placement was measured
#   --config FILE           FAST-LIO config. Default: the one *.yaml with lid_topic
#                           inside the recording (the live recorder copies it there)
#   --driver-config FILE    the sensor driver's own config upstream of FAST-LIO
#                           (JT128: the Hesai driver's jt128_post.yaml). Not read by
#                           replay; hashed, checked against the bag, kept in logs/
#   --patches FILE          the spec's preselected wall patches, boxed on THIS run's
#                           map (g8 format); a file without map_sha256 is bound to
#                           this run's map only if every box is the spec's fixed box.
#                           Default: auto-suggest (labelled NOT preselected; such a
#                           run never counts toward a series)
#   --sessions-root DIR     default $SPATIO_WALKDAY_ROOT or ~/Developer/spatio-data/walkday
#   --sensor TEXT           manifest sensor text (default derived from the bag)
#   --settings FILE         settings (default spatio/walkday/walkday.conf)
#
# Exit: 0 PASS, 1 FAIL, 2 NOT-EVALUABLE (stops loudly at the first stage that
# cannot be trusted), 3 usage or setup error.
# A recording containing SYNTHETIC.json never claims physical accuracy.
set -uo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
export PYTHONPATH="$REPO${PYTHONPATH:+:$PYTHONPATH}"
usage() { sed -n '2,41p' "$0" | sed 's/^# \{0,1\}//'; exit 3; }
die() { echo "process.sh: $*" >&2; exit 3; }

[[ $# -ge 2 ]] || usage
REC_ARG=$1 RUN_ID=$2; shift 2
END_CM="" START_CM="" HEADING=not_checked SPEC="${SPATIO_WALK_SPEC:-}" OPERATOR="${USER:-unknown}"
METHOD="tape measure from the cart reference point to the floor marks"
CONFIG="" DRIVER_CONFIG="" PATCHES="" SENSOR="" SETTINGS="$HERE/walkday.conf"
ROOT="${SPATIO_WALKDAY_ROOT:-$HOME/Developer/spatio-data/walkday}"
while [[ $# -gt 0 ]]; do
  [[ $# -ge 2 ]] || usage
  case $1 in
    --placement-cm) END_CM=$2 ;;
    --start-placement-cm) START_CM=$2 ;;
    --heading) HEADING=$2 ;;
    --spec) SPEC=$2 ;;
    --operator) OPERATOR=$2 ;;
    --method) METHOD=$2 ;;
    --config) CONFIG=$2 ;;
    --driver-config) DRIVER_CONFIG=$2 ;;
    --patches) PATCHES=$2 ;;
    --sessions-root) ROOT=$2 ;;
    --sensor) SENSOR=$2 ;;
    --settings) SETTINGS=$2 ;;
    *) usage ;;
  esac
  shift 2
done

[[ $RUN_ID =~ ^[A-Za-z0-9][A-Za-z0-9_.-]*$ ]] || die "run-id must match [A-Za-z0-9][A-Za-z0-9_.-]*"
[[ -d $REC_ARG ]] || die "recording folder not found: $REC_ARG"
REC="$(cd "$REC_ARG" && pwd)"
[[ -f $SETTINGS ]] || die "settings not found: $SETTINGS"
# shellcheck source=walkday.conf
source "$SETTINGS"
case $HEADING in ok|off|not_checked) ;; *) die "--heading must be ok, off or not_checked" ;; esac
[[ -z $START_CM || -n $END_CM ]] || die "--start-placement-cm needs --placement-cm"
if [[ -n $END_CM ]]; then
  [[ -n $SPEC && -f $SPEC ]] || die "--placement-cm needs a frozen spec (--spec or SPATIO_WALK_SPEC)"
  END_M="$(python3 -c 'import math,sys; v=float(sys.argv[1]); assert math.isfinite(v) and v>=0; print(repr(v/100))' "$END_CM" 2>/dev/null)" \
    || die "--placement-cm must be a number >= 0 (cm)"
  START_M=null
  if [[ -n $START_CM ]]; then
    START_M="$(python3 -c 'import math,sys; v=float(sys.argv[1]); assert math.isfinite(v) and v>=0; print(repr(v/100))' "$START_CM" 2>/dev/null)" \
      || die "--start-placement-cm must be a number >= 0 (cm)"
  fi
fi
[[ -z $PATCHES || -f $PATCHES ]] || die "patches file not found: $PATCHES"
[[ -z $PATCHES ]] || PATCHES="$(cd "$(dirname "$PATCHES")" && pwd)/$(basename "$PATCHES")"
[[ -z $DRIVER_CONFIG || -f $DRIVER_CONFIG ]] || die "driver config not found: $DRIVER_CONFIG"
[[ -z $DRIVER_CONFIG ]] || DRIVER_CONFIG="$(cd "$(dirname "$DRIVER_CONFIG")" && pwd)/$(basename "$DRIVER_CONFIG")"
python3 -c 'import numpy, matplotlib, mcap, mcap_ros2' 2>/dev/null || die "python3 needs numpy, matplotlib, mcap, mcap_ros2"
# The effective settings identity (walk_result.json carries it; a change starts a new series).
SETTING_PAIRS=()
for k in $(python3 -c 'from spatio.walkday.provenance import SETTINGS_KEYS as K; print(*K)'); do
  SETTING_PAIRS+=("$k=${!k:-}")
done
SETTINGS_SHA="$(python3 -m spatio.walkday.provenance settings "${SETTING_PAIRS[@]}")" || die "bad settings in $SETTINGS"

mkdir -p "$ROOT" || die "cannot create $ROOT"
ROOT="$(cd "$ROOT" && pwd)"
SESSION="$ROOT/$RUN_ID"
[[ -e $SESSION ]] && die "session exists, refusing to overwrite: $SESSION"
mkdir -p "$SESSION/logs"
cp "$SETTINGS" "$SESSION/logs/walkday.conf"
exec > >(tee -a "$SESSION/logs/process.log") 2>&1

SYNTHETIC=0; [[ -f $REC/SYNTHETIC.json ]] && SYNTHETIC=1
say() { echo "== [walkday $(date +%H:%M:%S)] $*"; }
stop() {  # stage reason
  echo
  echo "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
  echo "!! NOT-EVALUABLE: stopped at $1"
  echo "!! $2"
  echo "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
  # "written": result.py already wrote the report with its own reason (exit 3).
  [[ ${3:-} == written ]] || python3 -m spatio.walkday.result "$SESSION" --stopped-at "$1" --reason "$2" >/dev/null
  echo "report: $SESSION/walkday.md"
  exit 2
}
say "run $RUN_ID  recording $REC  -> $SESSION"
[[ $SYNTHETIC == 1 ]] && say "SYNTHETIC recording: physical accuracy will not be claimed"

say "[1/9] copy check (bag files vs metadata.yaml)"
python3 "$REPO/spatio/replay/recording.py" complete "$REC" || stop copy-check "recording copy incomplete or unreadable"
# The series orders runs by this; result.py reads it again from the recording.
RECORDED_AT="$(python3 "$REPO/spatio/replay/recording.py" recorded-at "$REC")" \
  || stop recording-time "bag metadata.yaml has no starting_time; the series orders runs by it"
say "recording started $RECORDED_AT (bag starting_time: recorder clock, first message received)"
if [[ -z $CONFIG ]]; then
  CONFIG="$(python3 "$REPO/spatio/replay/recording.py" find-config "$REC")" || stop config "no FAST-LIO config in the recording; pass --config"
fi
[[ -f $CONFIG ]] || stop config "config not found: $CONFIG"
CONFIG="$(cd "$(dirname "$CONFIG")" && pwd)/$(basename "$CONFIG")"
PROFILE="$(python3 "$REPO/spatio/replay/recording.py" profile "$REC" --config "$CONFIG")" || stop config "config does not match the bag"
eval "$PROFILE"
say "config $CONFIG: lidar $LIDAR ($LIDAR_MSG_TYPE), imu $IMU"
python3 -m spatio.walkday.provenance write "$SESSION/logs/provenance.json" --rec "$REC" --config "$CONFIG" \
  ${DRIVER_CONFIG:+--driver-config "$DRIVER_CONFIG"} "${SETTING_PAIRS[@]}" \
  || stop config "provenance not written: driver config rejected or unreadable (see above)"
[[ -z $DRIVER_CONFIG ]] || cp "$DRIVER_CONFIG" "$SESSION/logs/driver_config.$(basename "$DRIVER_CONFIG")"
if [[ -n $SPEC && -f $SPEC ]]; then
  python3 - "$SPEC" "$CONFIG" <<'PY' || stop spec "config does not match the frozen spec (a changed config starts a new series)"
import hashlib, json, sys
spec = json.load(open(sys.argv[1]))
have = hashlib.sha256(open(sys.argv[2], "rb").read()).hexdigest()
want = spec.get("config_sha256")
print(f"spec {spec.get('spec_id')}: config_sha256 {want}; this config {have}")
sys.exit(0 if want == have else 1)
PY
fi

say "[2/9] bag timing (x1) and input gate"
python3 -m spatio.bagtiming "$REC/bag" --out "$SESSION/logs/bagtiming" \
  --point-timing-sampled-every "$BAGTIMING_SAMPLE_EVERY" || stop bag-timing "x1 could not inspect the bag"
python3 -m spatio.walkday.gate "$SESSION/logs/bagtiming/bagtiming.json" --lidar "$LIDAR" --imu "$IMU" \
  --out "$SESSION/logs/input_gate.json"
rc=$?; [[ $rc == 0 ]] || stop input-gate "input is not evaluable (see logs/input_gate.json)"

say "[3/9] FAST-LIO replay (A)"
"$REPO/spatio/replay/replay.sh" "$REC" "$RUN_ID" --sessions-root "$ROOT" --config "$CONFIG" \
  --rate "$PLAYBACK_RATE" --map-voxel "$MAP_VOXEL_M" ${SENSOR:+--sensor "$SENSOR"}
rc=$?
[[ $rc == 0 ]] || stop replay "replay.sh exit $rc (see logs/runner.log)"
[[ -s $SESSION/trajectory.tum && -s $SESSION/map.pcd && -f $SESSION/manifest.json ]] \
  || stop replay "replay produced no trajectory/map/manifest"
mv "$SESSION/manifest.json" "$SESSION/logs/replay_manifest.json"

REF_ARGS=()
if [[ -n $END_CM ]]; then
  say "[4/9] physical reference (g5): start ${START_CM:-not measured} cm, end $END_CM cm, heading $HEADING"
  REF="$SESSION/logs/physical_reference.input.json"
  python3 -m spatio.expspec record --spec "$SPEC" --run-id "$RUN_ID" --start-residual-m "$START_M" \
    --end-residual-m "$END_M" --heading "$HEADING" --operator "$OPERATOR" --method "$METHOD" --out "$REF" \
    || stop physical-reference "g5 refused the placement record"
  if [[ $SYNTHETIC == 1 ]]; then
    mv "$REF" "$SESSION/logs/physical_reference.not_applied.json"
    say "synthetic: placement recorded in logs/ only, NOT applied to the manifest"
  else
    REF_ARGS=(--physical-reference "$REF")
  fi
else
  say "[4/9] no placement measurement given: physical accuracy not evaluated"
fi

say "[5/9] manifest (g2)"
eval "$(python3 - "$SESSION/logs/replay_manifest.json" <<'PY'
import json, shlex, sys
m = json.load(open(sys.argv[1]))
for k in ("source_recording", "sensor", "fastlio_revision", "config_file", "image", "notes"):
    print(f"M_{k.upper()}={shlex.quote(str(m[k]))}")
PY
)" || stop manifest "cannot read the replay manifest"
if [[ -n $SPEC && -f $SPEC ]]; then
  python3 - "$SPEC" "$M_FASTLIO_REVISION" <<'PY' || stop spec "FAST-LIO revision differs from the frozen spec"
import json, sys
want = str(json.load(open(sys.argv[1])).get("fastlio_revision", ""))
have = sys.argv[2]
print(f"spec fastlio_revision {want}; replay image {have}")
sys.exit(0 if want and (have.startswith(want) or want.startswith(have)) else 1)
PY
fi
[[ $SYNTHETIC == 1 ]] && M_NOTES="SYNTHETIC recording (spatio/walkday/synth), not a physical walk. $M_NOTES"
if [[ -n $DRIVER_CONFIG ]]; then
  M_NOTES="$M_NOTES Driver config $DRIVER_CONFIG sha256 $(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["driver_config"]["sha256"])' "$SESSION/logs/provenance.json") (not read by replay; see logs/provenance.json)."
fi
M_NOTES="$M_NOTES Walk-day settings sha256 $SETTINGS_SHA."
python3 -m spatio.manifest build "$SESSION" --run-id "$RUN_ID" --source-recording "$M_SOURCE_RECORDING" \
  --sensor "$M_SENSOR" --fastlio-revision "$M_FASTLIO_REVISION" --config-file "$M_CONFIG_FILE" \
  --image "$M_IMAGE" --notes "$M_NOTES" ${REF_ARGS[@]+"${REF_ARGS[@]}"} || stop manifest "g2 build failed"
python3 -m spatio.manifest validate "$SESSION" || stop manifest "g2 validate found an ERROR"

say "[6/9] walk check (B)"
python3 -m spatio.walkcheck "$SESSION" --stationary-window "$STATIONARY_WINDOW_S" --max-gap "$MAX_GAP_S" \
  --min-path-length "$MIN_PATH_LENGTH_M" --min-excursion "$MIN_EXCURSION_M" \
  --max-endpoint-spread "$MAX_ENDPOINT_SPREAD_M" --max-endpoint-speed "$MAX_ENDPOINT_SPEED_M_S" \
  --max-hold-speed "$MAX_HOLD_SPEED_M_S" --min-hold "$MIN_HOLD_S" --wall-gate-m "$WALL_GATE_M"
rc=$?
case $rc in 0|1|2) say "walk checker exit $rc" ;; *) stop walk-check "walkcheck could not use the session (exit $rc)" ;; esac

say "[7/9] wall patches (g8)"
mkdir -p "$SESSION/wallpatch"
if [[ -n $PATCHES ]]; then
  cp "$PATCHES" "$SESSION/wallpatch/patches.json" || stop wall-patches "cannot copy patches from $PATCHES"
  PATCH_SPEC_ARGS=()
  [[ -n $SPEC ]] && PATCH_SPEC_ARGS=(--spec "$SPEC")
  PATCH_ORIGIN="$(python3 -m spatio.walkday.patch_origin "$SESSION/wallpatch/patches.json" \
    ${PATCH_SPEC_ARGS[@]+"${PATCH_SPEC_ARGS[@]}"} --map "$SESSION/map.pcd")" \
    || stop wall-patches "supplied patches cannot establish preselection (see above)"
  PATCH_SOURCE="$PATCH_ORIGIN patches from $PATCHES"
  [[ $PATCH_ORIGIN != auto_suggested ]] || say "auto_suggested patches cannot be labelled preselected; this run cannot count toward a series"
else
  PATCH_ORIGIN=auto_suggested
  PATCH_SOURCE="auto-suggested from this map (top $SUGGEST_PATCHES), NOT Trevor's preselected patches"
  python3 -m spatio.wallpatch suggest "$SESSION/map.pcd" --n "$SUGGEST_PATCHES" --out "$SESSION/wallpatch/patches.json" \
    || say "WARNING: wall-patch suggestion failed; walls will be not evaluable"
fi
if [[ -f $SESSION/wallpatch/patches.json ]]; then
  ACC_LABEL="replay demonstrated; physical accuracy not evaluated"
  [[ ${#REF_ARGS[@]} -gt 0 ]] && ACC_LABEL="physical reference recorded (see physical_reference.json)"
  [[ $SYNTHETIC == 1 ]] && ACC_LABEL="SYNTHETIC recording; $ACC_LABEL"
  python3 -m spatio.wallpatch measure "$SESSION" --patches "$SESSION/wallpatch/patches.json" \
    --out "$SESSION/wallpatch" --wall-gate-m "$WALL_GATE_M" --accuracy-label "$ACC_LABEL; patches: $PATCH_SOURCE" \
    || say "WARNING: wall-patch measurement failed; walls will be not evaluable"
fi

say "[8/9] raw-scan start->end registration (information only, never scored)"
python3 -m spatio.walkday.startend "$REC/bag" "$SESSION/trajectory.tum" "$SESSION/startend" \
  --lidar "$LIDAR" --hold "$STATIONARY_WINDOW_S" || say "start->end registration not available (see above)"

say "[9/9] result"
SPEC_ARGS=()
[[ -n $SPEC && -f $SPEC ]] && SPEC_ARGS=(--spec "$SPEC")
python3 -m spatio.walkday.result "$SESSION" --wall-gate-m "$WALL_GATE_M" --wall-gate-state "$WALL_GATE_STATE" \
  --patch-source "$PATCH_SOURCE" --patch-origin "$PATCH_ORIGIN" ${SPEC_ARGS[@]+"${SPEC_ARGS[@]}"}
rc=$?
[[ $rc == 3 ]] && stop result "could not build walk_result.json (see output above)" written
echo
if [[ $rc == 2 ]]; then
  echo "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
  echo "!! NOT-EVALUABLE: scored, but the evidence is incomplete (see Why: in walkday.md)"
  echo "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
fi
echo "========================================================================"
sed -n '3p' "$SESSION/walkday.md"
echo "report:  $SESSION/walkday.md"
echo "trail:   LABCOMMAND_TRAIL_SESSION=$SESSION <built app binary>"
echo "========================================================================"
exit $rc
