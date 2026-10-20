#!/usr/bin/env bash
# Offline FAST-LIO replay of a copied recording, inside a local colima container.
#
#   spatio/replay/replay.sh <recording> <run-id> [options]
#
#   <recording>  folder with bag/ (path, or a name under ~/Developer/spatio-data/recordings)
#   <run-id>     session name; output goes to <sessions-root>/<run-id>/
#
# Nothing here is specific to one recording: the lidar/IMU topics come from the
# FAST-LIO config (common.lid_topic / common.imu_topic), the sensor kind from the
# message types in bag/metadata.yaml, and the copy check from the bag's own MCAP
# summaries (recording.py).
#
# Options:
#   --config FILE     FAST-LIO params file. Default: the one *.yaml with lid_topic
#                     inside <recording>; anything else needs --config
#   --sessions-root D output root (default $SPATIO_DATA/sessions)
#   --sensor TEXT     manifest sensor text (default: derived from the bag's types)
#   --rate R          playback rate (default 1.0)
#   --duration S      play only the first S seconds (smoke tests)
#   --config-note T   provenance note for --config (goes into manifest notes)
#   --historical      do not run FAST-LIO; export the bag's own odometry from the
#                     original run, and copy the recording's historical map
#   --historical-map FILE   map to copy for --historical (relative to <recording>; required)
#   --historical-odom-topic T  odometry topic for --historical (default /Odometry,
#                     FAST-LIO's own output name)
#   --historical-path-topic T  optional nav_msgs/Path topic for --historical
#   --max-seconds N   hard limit for the container run (default 3600)
#   --fastdds-profile FILE  Fast DDS XML profile for every node in the container
#                     (e.g. configs/fastdds_big_shm.xml for large PointCloud2)
#   --map-voxel M     map_display.pcd voxel size in metres (default 0.03)
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

usage() { sed -n '2,38p' "$0" | sed 's/^# \{0,1\}//'; exit 2; }
[[ $# -ge 2 ]] || usage
REC_ARG=$1 RUN_ID=$2; shift 2
RATE=1.0 DURATION="" CONFIG="" CONFIG_NOTE="" MODE=replay HIST_MAP="" MAX_SECONDS=3600 DDS_PROFILE="" MAP_VOXEL=0.03
SESSIONS_ROOT="$DATA/sessions" SENSOR="" HIST_ODOM_TOPIC=/Odometry HIST_PATH_TOPIC=""
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
    --map-voxel) MAP_VOXEL=$2; shift 2 ;;
    --sessions-root) SESSIONS_ROOT=$2; shift 2 ;;
    --sensor) SENSOR=$2; shift 2 ;;
    --historical-odom-topic) HIST_ODOM_TOPIC=$2; shift 2 ;;
    --historical-path-topic) HIST_PATH_TOPIC=$2; shift 2 ;;
    *) usage ;;
  esac
done

[[ $RUN_ID =~ ^[A-Za-z0-9][A-Za-z0-9_.-]*$ ]] || { echo "run-id must match [A-Za-z0-9][A-Za-z0-9_.-]*" >&2; exit 2; }
if [[ -d $REC_ARG ]]; then REC="$(cd "$REC_ARG" && pwd)"; else REC="$DATA/recordings/$REC_ARG"; fi
[[ -f $REC/bag/metadata.yaml ]] || { echo "no bag/metadata.yaml in $REC" >&2; exit 2; }
mkdir -p "$SESSIONS_ROOT"
SESSION="$(cd "$SESSIONS_ROOT" && pwd)/$RUN_ID"
# A session folder may already hold logs/ from an earlier pipeline stage
# (walk-day input check); outputs are never overwritten.
for f in trajectory.tum map.pcd map_display.pcd manifest.json logs/output_bag; do
  [[ -e $SESSION/$f ]] && { echo "session already has $f, refusing to overwrite: $SESSION" >&2; exit 2; }
done
# The copy must be complete: every bag file present and its MCAP summary
# matching metadata.yaml (an interrupted rsync fails this).
python3 "$HERE/recording.py" complete "$REC" >/dev/null || { echo "recording copy incomplete or unreadable: $REC" >&2; exit 2; }

# Topics from the config, sensor kind from the bag's message types.
if [[ -z $CONFIG ]]; then CONFIG="$(python3 "$HERE/recording.py" find-config "$REC")" || exit 2; fi
[[ -f $CONFIG ]] || { echo "config not found: $CONFIG" >&2; exit 2; }
CONFIG="$(cd "$(dirname "$CONFIG")" && pwd)/$(basename "$CONFIG")"
PROFILE="$(python3 "$HERE/recording.py" profile "$REC" --config "$CONFIG")" || exit 2
eval "$PROFILE"   # LIDAR IMU LIDAR_MSG_TYPE TS_UNIT SENSOR_KIND (shell-quoted)
[[ -n $SENSOR ]] || SENSOR="$SENSOR_KIND"
if [[ $MODE == historical ]]; then
  [[ -n $HIST_MAP ]] || { echo "--historical needs --historical-map FILE" >&2; exit 2; }
  grep -q "name: $HIST_ODOM_TOPIC\$" "$REC/bag/metadata.yaml" || { echo "no $HIST_ODOM_TOPIC in the bag" >&2; exit 2; }
else
  HIST_PATH_TOPIC=""
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
# Cache key: every bag file plus metadata.yaml (not just the first segment).
BAGFILES=()
while IFS= read -r f; do BAGFILES+=("$f"); done < <(cd "$REC/bag" && ls | grep -E '\.(mcap|db3)$|^metadata\.yaml$' | sort)
want_sha="$(cd "$REC/bag" && shasum -a 256 "${BAGFILES[@]}" | shasum -a 256 | cut -d' ' -f1)"
have_sha="$(dk run --rm --name "spatio-check-$RUN_ID" --network none -v "$VOL:/rec:ro" "$IMAGE_TAG" cat /rec/.spatio_loaded 2>/dev/null || true)"
if [[ $have_sha != "$want_sha" ]]; then
  echo "== loading $REC into volume $VOL"
  dk volume rm "$VOL" >/dev/null 2>&1 || true
  dk volume create --label spatio.source="$REC" "$VOL" >/dev/null
  dk create --name "spatio-load-$RUN_ID" --network none -v "$VOL:/rec" "$IMAGE_TAG" true >/dev/null
  dk cp "$REC/." "spatio-load-$RUN_ID:/rec/"
  dk rm "spatio-load-$RUN_ID" >/dev/null
  got="$(dk run --rm --name "spatio-verify-$RUN_ID" --network none -v "$VOL:/rec:ro" -w /rec/bag "$IMAGE_TAG" \
        sha256sum "${BAGFILES[@]}" | shasum -a 256 | cut -d' ' -f1)"
  [[ $got == "$want_sha" ]] || { echo "bag sha256 mismatch in volume ($got != $want_sha)" >&2; exit 3; }
  dk run --rm --name "spatio-mark-$RUN_ID" --network none -v "$VOL:/rec" "$IMAGE_TAG" sh -c "echo $want_sha > /rec/.spatio_loaded"
fi

# Tool revision as staged into the container (later edits don't reach it).
TOOL_REV="$(git -C "$HERE" rev-parse HEAD)"
[[ -n $(git -C "$HERE" status --porcelain -- .) ]] && TOOL_REV="$TOOL_REV-dirty" || true

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

# One replay at a time on this machine: two would oversubscribe the VM and,
# at playback rate 1, CPU contention can change FAST-LIO's result.
LOCK="${TMPDIR:-/tmp}/spatio-replay.lock"
until mkdir "$LOCK" 2>/dev/null; do
  holder="$(command cat "$LOCK/owner" 2>/dev/null || true)"
  if [[ -n $holder ]] && ! kill -0 "${holder%% *}" 2>/dev/null; then rm -rf "$LOCK"; continue; fi
  echo "== waiting for another replay to finish ($holder)"; sleep 15
done
echo "$$ $RUN_ID" > "$LOCK/owner"
trap 'rm -rf "$STAGE"; rm -rf "$LOCK"' EXIT

CONTAINER="spatio-replay-$RUN_ID"
mkdir -p "$SESSION"
echo "== $CONTAINER: $MODE $REC -> $SESSION"
dk create --name "$CONTAINER" \
  --network none --cpus "$CPUS" --memory "$MEM" --memory-swap "$MEM" --shm-size 1g \
  --pids-limit 1024 --cap-drop ALL --security-opt no-new-privileges \
  -e ROS_DOMAIN_ID=$DOMAIN_ID -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST -e ROS_LOG_DIR=/out/logs/ros \
  -e MODE="$MODE" -e LIDAR="$LIDAR" -e IMU="$IMU" -e RATE="$RATE" -e DURATION="$DURATION" \
  -e TIMESTAMP_UNIT="$TS_UNIT" -e HIST_PATH_TOPIC="$HIST_PATH_TOPIC" -e HIST_ODOM_TOPIC="$HIST_ODOM_TOPIC" \
  -e MAP_VOXEL="$MAP_VOXEL" \
  ${DDS_ENV[@]+"${DDS_ENV[@]}"} -v "$VOL:/rec:ro" \
  "$IMAGE_TAG" timeout --signal=TERM --kill-after=60 "$MAX_SECONDS" \
  bash /opt/spatio/tools/run_in_container.sh >/dev/null
dk cp "$STAGE/tools" "$CONTAINER:/opt/spatio/tools"
dk cp "$STAGE/cfg" "$CONTAINER:/cfg"
set +e
dk start -a "$CONTAINER"
RC=$?
if ! dk cp "$CONTAINER:/out/." "$SESSION/"; then
  echo "ERROR: could not copy /out from $CONTAINER" >&2; [[ $RC == 0 ]] && RC=4
fi
dk rm "$CONTAINER" >/dev/null
set -e
mkdir -p "$SESSION/logs"
echo "== container exit $RC"

if [[ $MODE == historical ]]; then
  if [[ -f $REC/$HIST_MAP ]]; then cp "$REC/$HIST_MAP" "$SESSION/map.pcd"; else echo "ERROR: no $REC/$HIST_MAP" >&2; [[ $RC == 0 ]] && RC=4; fi
fi
for f in trajectory.tum map.pcd; do
  [[ -s $SESSION/$f ]] || { echo "ERROR: replay produced no $f" >&2; [[ $RC == 0 ]] && RC=4; }
done
if [[ $MODE == replay && ! -s $SESSION/map_display.pcd ]]; then
  echo "ERROR: replay produced no map_display.pcd" >&2
  if [[ $RC == 0 ]]; then RC=4; fi
fi

# Provenance and run details (outside the contract manifest).
dk run --rm --name "spatio-prov-$RUN_ID" --network none "$IMAGE_TAG" cat /opt/spatio/provenance.json > "$SESSION/logs/image_provenance.json"
python3 - "$SESSION/logs/run_params.json" <<EOF
import json, sys
json.dump({
  "mode": "$MODE", "container": "$CONTAINER", "container_exit": $RC,
  "image_tag": "$IMAGE_TAG", "image_id": "$IMAGE_ID",
  "replay_tool_revision": "$TOOL_REV",
  "playback_topics": ["$LIDAR", "$IMU"], "lidar_msg_type": "$LIDAR_MSG_TYPE", "rate": $RATE,
  "config_file": "$CONFIG",
  "duration_s": "${DURATION:-all}", "ros_domain_id": $DOMAIN_ID,
  "recording_volume": "$VOL", "bag_files_sha256": "$want_sha", "bag_files_sha256_definition": "sha256 of the sha256sum listing of bag/*.mcap|*.db3 and metadata.yaml",
  "limits": {"cpus": $CPUS, "memory": "$MEM", "shm": "1g", "pids": 1024, "max_seconds": $MAX_SECONDS},
  "fastdds_profile": "${DDS_PROFILE:-default}",
  "fastlio_param_overrides": {"runtime_pos_log_enable": True} if "$MODE" == "replay" else {},
  "map": "map.pcd = all finite registered XYZ points for scoring; map_display.pcd = copied map_saver.py ($MAP_VOXEL m voxel) for display" if "$MODE" == "replay" else "map.pcd = copy of recording's $HIST_MAP (filtering provenance unknown)",
}, open(sys.argv[1], "w"), indent=2)
EOF
echo "$RC" > "$SESSION/logs/container_exit_code"

# Contract manifest.  (SENSOR: --sensor, else derived from the bag's message types.)
if [[ $MODE == replay ]]; then
  M_REV=$FL_REV M_IMAGE="$IMAGE_TAG@$IMAGE_ID" M_CONFIG=$CONFIG
  M_NOTES="Offline FAST-LIO re-run on the Mac (colima, aarch64) of the copied recording; played ${LIDAR} + ${IMU} only at rate ${RATE}${DURATION:+, first ${DURATION} s only}. trajectory.tum = recorded /Odometry header stamps. map.pcd = all finite registered XYZ points for scoring. map_display.pcd = copied map_saver.py (${MAP_VOXEL} m voxel) over recorded /cloud_registered, for display only. Container exit ${RC}. ${CONFIG_NOTE}"
else
  # Original Spark run: revision only known if the recording says so.
  M_REV="$(sed -n 's/^FAST_LIO_ROS2: *//p' "$REC/provenance.txt" 2>/dev/null | head -1 || true)"
  M_REV="${M_REV:-unknown (not recorded with this recording)}"
  M_IMAGE="original Spark run (image not recorded); exported with $IMAGE_TAG@$IMAGE_ID"
  M_CONFIG="$CONFIG"
  M_NOTES="Historical: trajectory.tum = $HIST_ODOM_TOPIC recorded in the bag by the original Spark FAST-LIO run (not re-run here). map.pcd = copy of the recording's $HIST_MAP, which may come from a different run than this trajectory. ${CONFIG_NOTE}"
fi
python3 "$HERE/manifest.py" "$SESSION" --run-id "$RUN_ID" --source-recording "$REC" \
  --sensor "$SENSOR" --fastlio-revision "$M_REV" --config-file "$M_CONFIG" \
  --image "$M_IMAGE" --notes "$M_NOTES"
exit $RC
