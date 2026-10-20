#!/usr/bin/env bash
# Offline FAST-LIO replay of a copied recording, inside a local colima container.
#
#   spatio/replay/replay.sh <recording> <run-id> [options]
#
#   <recording>  folder with bag/ (path, or a name under ~/Developer/spatio-data/recordings)
#   <run-id>     session name; output goes to ~/Developer/spatio-data/sessions/<run-id>/
#
# Options:
#   --rate R          playback rate (default 1.0)
#   --duration S      play only the first S seconds (smoke tests)
#   --config FILE     FAST-LIO params file (default: <recording>/mid360.yaml for Mid-360;
#                     required for JT128, no copied JT128 config exists)
#   --config-note T   provenance note for --config (goes into manifest notes)
#   --historical      do not run FAST-LIO; export the bag's own /Odometry from the
#                     original run, and copy the recording's historical map
#   --historical-map FILE  map to copy for --historical (relative to <recording>)
#   --max-seconds N   hard limit for the container run (default 3600)
#   --fastdds-profile FILE  Fast DDS XML profile for every node in the container
#                     (e.g. configs/fastdds_big_shm.xml for large PointCloud2)
#
# Isolation: --network none, private ROS_DOMAIN_ID, recording mounted read-only
# (as a named volume copy), CPU/memory/pids limits, all capabilities dropped.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
DATA="${SPATIO_DATA:-$HOME/Developer/spatio-data}"
DCTX="${DOCKER_CONTEXT:-colima}"
DOMAIN_ID=87
CPUS=4
MEM=8g

usage() { sed -n '2,24p' "$0" | sed 's/^# \{0,1\}//'; exit 2; }
[[ $# -ge 2 ]] || usage
REC_ARG=$1 RUN_ID=$2; shift 2
RATE=1.0 DURATION="" CONFIG="" CONFIG_NOTE="" MODE=replay HIST_MAP="" MAX_SECONDS=3600 DDS_PROFILE=""
while [[ $# -gt 0 ]]; do
  case $1 in
    --rate) RATE=$2; shift 2 ;;
    --duration) DURATION=$2; shift 2 ;;
    --config) CONFIG=$2; shift 2 ;;
    --config-note) CONFIG_NOTE=$2; shift 2 ;;
    --historical) MODE=historical; shift ;;
    --historical-map) HIST_MAP=$2; shift 2 ;;
    --max-seconds) MAX_SECONDS=$2; shift 2 ;;
    --fastdds-profile) DDS_PROFILE=$2; shift 2 ;;
    *) usage ;;
  esac
done

[[ $RUN_ID =~ ^[A-Za-z0-9][A-Za-z0-9_.-]*$ ]] || { echo "run-id must match [A-Za-z0-9][A-Za-z0-9_.-]*" >&2; exit 2; }
if [[ -d $REC_ARG ]]; then REC="$(cd "$REC_ARG" && pwd)"; else REC="$DATA/recordings/$REC_ARG"; fi
[[ -f $REC/bag/metadata.yaml ]] || { echo "no bag/metadata.yaml in $REC" >&2; exit 2; }
SESSION="$DATA/sessions/$RUN_ID"
[[ -e $SESSION ]] && { echo "session exists, refusing to overwrite: $SESSION" >&2; exit 2; }
grep -q "COPY_DONE 0" "$HOME/Developer/lc-tasks/spatio-copy.log" 2>/dev/null \
  || { echo "recording copy not finished (no COPY_DONE 0)" >&2; exit 2; }

# Sensor profile from the bag's topics.
HIST_PATH_TOPIC=""
if grep -q "name: /livox/lidar$" "$REC/bag/metadata.yaml"; then
  SENSOR="Livox Mid-360 + built-in IMU (livox_ros_driver2 CustomMsg)"
  LIDAR=/livox/lidar IMU=/livox/imu TS_UNIT=3
  [[ -z $CONFIG && -f $REC/mid360.yaml ]] && CONFIG="$REC/mid360.yaml"
  [[ -z $HIST_MAP ]] && HIST_MAP=map.pcd
elif grep -q "name: /slam/points$" "$REC/bag/metadata.yaml"; then
  SENSOR="Hesai JT128 via jt128_slam_adapter output recorded in bag (/slam/points PointCloud2, /slam/imu)"
  LIDAR=/slam/points IMU=/slam/imu TS_UNIT=3 HIST_PATH_TOPIC=/path
  [[ -z $HIST_MAP ]] && HIST_MAP=jt128_map.pcd
else
  echo "unrecognised recording (no /livox/lidar or /slam/points in bag)" >&2; exit 2
fi
if [[ $MODE == replay ]]; then
  [[ -n $CONFIG && -f $CONFIG ]] || { echo "need --config for this recording" >&2; exit 2; }
  CONFIG="$(cd "$(dirname "$CONFIG")" && pwd)/$(basename "$CONFIG")"
  # The timestamp unit for PointCloud2 'time' fields comes from the config.
  u=$(sed -n 's/^ *timestamp_unit: *\([0-9]\).*/\1/p' "$CONFIG" | head -1); [[ -n $u ]] && TS_UNIT=$u
fi

IMAGE_TAG="spatio/replay:$(git -C "$DATA/spark-src/FAST_LIO_ROS2" rev-parse --short=7 HEAD)"
IMAGE_ID="$(docker --context "$DCTX" image inspect -f '{{.Id}}' "$IMAGE_TAG")" \
  || { echo "image $IMAGE_TAG missing; run build_image.sh" >&2; exit 2; }
FL_REV="$(docker --context "$DCTX" image inspect -f '{{index .Config.Labels "spatio.fastlio_revision"}}' "$IMAGE_TAG")"

# colima on this Mac shares no host folders we can use, so bind mounts would
# silently create empty VM-local dirs. Instead: the recording goes into a named
# volume (copied once with docker cp, sha256-verified, then mounted read-only);
# tools + config are docker-cp'd into a created container; outputs are
# docker-cp'd back out.
dk() { docker --context "$DCTX" "$@"; }
VOL="spatio-rec-$(basename "$REC" | tr -c 'A-Za-z0-9_.-\n' '_')"
BAGFILE="$(cd "$REC/bag" && ls | grep -E '\.(mcap|db3)$' | head -1)"
want_sha="$(shasum -a 256 "$REC/bag/$BAGFILE" | cut -d' ' -f1)"
have_sha="$(dk run --rm --network none -v "$VOL:/rec:ro" "$IMAGE_TAG" cat /rec/.spatio_loaded 2>/dev/null || true)"
if [[ $have_sha != "$want_sha" ]]; then
  echo "== loading $REC into volume $VOL"
  dk volume rm "$VOL" >/dev/null 2>&1 || true
  dk volume create --label spatio.source="$REC" "$VOL" >/dev/null
  dk create --name "spatio-load-$RUN_ID" --network none -v "$VOL:/rec" "$IMAGE_TAG" true >/dev/null
  dk cp "$REC/." "spatio-load-$RUN_ID:/rec/"
  dk rm "spatio-load-$RUN_ID" >/dev/null
  got="$(dk run --rm --network none -v "$VOL:/rec:ro" "$IMAGE_TAG" sha256sum "/rec/bag/$BAGFILE" | cut -d' ' -f1)"
  [[ $got == "$want_sha" ]] || { echo "bag sha256 mismatch in volume ($got != $want_sha)" >&2; exit 3; }
  dk run --rm --network none -v "$VOL:/rec" "$IMAGE_TAG" sh -c "echo $want_sha > /rec/.spatio_loaded"
fi

STAGE="$(mktemp -d "${TMPDIR:-/tmp}/spatio-stage.XXXXXX")"
trap 'rm -rf "$STAGE"' EXIT
mkdir -p "$STAGE/tools" "$STAGE/cfg"
cp "$HERE"/*.py "$HERE"/run_in_container.sh "$STAGE/tools/"
[[ -n $CONFIG ]] && cp "$CONFIG" "$STAGE/cfg/config.yaml"
DDS_ENV=()
if [[ -n $DDS_PROFILE ]]; then
  cp "$DDS_PROFILE" "$STAGE/cfg/fastdds.xml"
  DDS_ENV=(-e FASTRTPS_DEFAULT_PROFILES_FILE=/cfg/fastdds.xml -e FASTDDS_DEFAULT_PROFILES_FILE=/cfg/fastdds.xml)
fi

CONTAINER="spatio-replay-$RUN_ID"
mkdir -p "$SESSION"
echo "== $CONTAINER: $MODE $REC -> $SESSION"
dk create --name "$CONTAINER" \
  --network none --cpus "$CPUS" --memory "$MEM" --memory-swap "$MEM" --shm-size 1g \
  --pids-limit 1024 --cap-drop ALL --security-opt no-new-privileges \
  -e ROS_DOMAIN_ID=$DOMAIN_ID -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST -e ROS_LOG_DIR=/out/logs/ros \
  -e MODE="$MODE" -e LIDAR="$LIDAR" -e IMU="$IMU" -e RATE="$RATE" -e DURATION="$DURATION" \
  -e TIMESTAMP_UNIT="$TS_UNIT" -e HIST_PATH_TOPIC="$HIST_PATH_TOPIC" -e MAP_VOXEL=0.03 \
  ${DDS_ENV[@]+"${DDS_ENV[@]}"} -v "$VOL:/rec:ro" \
  "$IMAGE_TAG" timeout --signal=TERM --kill-after=60 "$MAX_SECONDS" \
  bash /opt/spatio/tools/run_in_container.sh >/dev/null
dk cp "$STAGE/tools" "$CONTAINER:/opt/spatio/tools"
dk cp "$STAGE/cfg" "$CONTAINER:/cfg"
set +e
dk start -a "$CONTAINER"
RC=$?
dk cp "$CONTAINER:/out/." "$SESSION/" || echo "WARNING: could not copy /out from $CONTAINER"
dk rm "$CONTAINER" >/dev/null
set -e
mkdir -p "$SESSION/logs"
echo "== container exit $RC"

if [[ $MODE == historical ]]; then
  if [[ -f $REC/$HIST_MAP ]]; then cp "$REC/$HIST_MAP" "$SESSION/map.pcd"; else echo "WARNING: no $REC/$HIST_MAP"; fi
fi

# Provenance and run details (outside the contract manifest).
TOOL_REV="$(git -C "$HERE" rev-parse HEAD)"
[[ -n $(git -C "$HERE" status --porcelain -- .) ]] && TOOL_REV="$TOOL_REV-dirty" || true
dk run --rm --network none "$IMAGE_TAG" cat /opt/spatio/provenance.json > "$SESSION/logs/image_provenance.json"
python3 - "$SESSION/logs/run_params.json" <<EOF
import json, sys
json.dump({
  "mode": "$MODE", "container": "$CONTAINER", "container_exit": $RC,
  "image_tag": "$IMAGE_TAG", "image_id": "$IMAGE_ID",
  "replay_tool_revision": "$TOOL_REV",
  "playback_topics": ["$LIDAR", "$IMU"], "rate": $RATE,
  "duration_s": "${DURATION:-all}", "ros_domain_id": $DOMAIN_ID,
  "recording_volume": "$VOL", "bag_sha256": "$want_sha",
  "limits": {"cpus": $CPUS, "memory": "$MEM", "shm": "1g", "pids": 1024, "max_seconds": $MAX_SECONDS},
  "fastdds_profile": "${DDS_PROFILE:-default}",
  "fastlio_param_overrides": {"runtime_pos_log_enable": True} if "$MODE" == "replay" else {},
  "map": "map.pcd = copied map_saver.py (0.03 m voxel) fed offline from recorded /cloud_registered" if "$MODE" == "replay" else "map.pcd = copy of recording's $HIST_MAP",
}, open(sys.argv[1], "w"), indent=2)
EOF
echo "$RC" > "$SESSION/logs/container_exit_code"

# Contract manifest.
if [[ $MODE == replay ]]; then
  M_REV=$FL_REV M_IMAGE="$IMAGE_TAG@$IMAGE_ID" M_CONFIG=$CONFIG
  M_NOTES="Offline FAST-LIO re-run on the Mac (colima, aarch64) of the copied recording; played ${LIDAR} + ${IMU} only at rate ${RATE}${DURATION:+, first ${DURATION} s only}. trajectory.tum = recorded /Odometry header stamps. map.pcd = copied map_saver.py (0.03 m voxel) over recorded /cloud_registered. Container exit ${RC}. ${CONFIG_NOTE}"
else
  # Original Spark run: revision only known if the recording says so.
  M_REV="$(sed -n 's/^FAST_LIO_ROS2: *//p' "$REC/provenance.txt" 2>/dev/null | head -1 || true)"
  M_REV="${M_REV:-unknown (not recorded with this recording)}"
  M_IMAGE="original Spark run (image not recorded); exported with $IMAGE_TAG@$IMAGE_ID"
  M_CONFIG="${CONFIG:-unknown (no config copied with this recording)}"
  M_NOTES="Historical: trajectory.tum = /Odometry recorded in the bag by the original Spark FAST-LIO run (not re-run here). map.pcd = copy of the recording's $HIST_MAP, which may come from a different run than this trajectory. ${CONFIG_NOTE}"
fi
python3 "$HERE/manifest.py" "$SESSION" --run-id "$RUN_ID" --source-recording "$REC" \
  --sensor "$SENSOR" --fastlio-revision "$M_REV" --config-file "$M_CONFIG" \
  --image "$M_IMAGE" --notes "$M_NOTES"
exit $RC
