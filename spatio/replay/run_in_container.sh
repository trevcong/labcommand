#!/usr/bin/env bash
# Entry point inside the spatio replay container. Started by replay.sh.
#
# Mounts: /rec (recording, read-only), /cfg/config.yaml (read-only),
#         /opt/spatio/tools (this directory, read-only), /out (session dir).
# Env:    MODE=replay|historical  LIDAR IMU  RATE  DURATION (s, empty = all)
#         MAP_VOXEL  TIMESTAMP_UNIT  HIST_ODOM_TOPIC  HIST_PATH_TOPIC
source /opt/ros/jazzy/setup.bash   # ROS setup scripts are not set -u clean
source /ws/install/setup.bash
set -uo pipefail

T=/opt/spatio/tools
OUT=/out
LOGS=$OUT/logs
BAG=/rec/bag
mkdir -p "$LOGS"
say() { echo "[spatio $(date -u +%H:%M:%S)] $*" | tee -a "$LOGS/runner.log"; }

say "mode=$MODE lidar=$LIDAR imu=$IMU rate=${RATE:-} duration=${DURATION:-all} domain=$ROS_DOMAIN_ID"

say "inspecting input timing"
python3 "$T/inspect_bag.py" "$BAG" --lidar "$LIDAR" --imu "$IMU" \
    --timestamp-unit "${TIMESTAMP_UNIT:-3}" --out "$LOGS/input_timing.json" \
    >>"$LOGS/runner.log" 2>&1 || say "WARNING: timing inspection failed (see runner.log)"

if [[ "$MODE" == historical ]]; then
  say "exporting the bag's own ${HIST_ODOM_TOPIC:-/Odometry} (original run)"
  python3 "$T/bag_to_tum.py" "$BAG" "$OUT/trajectory.tum" --topic "${HIST_ODOM_TOPIC:-/Odometry}" \
      ${HIST_PATH_TOPIC:+--path-topic "$HIST_PATH_TOPIC"} \
      --summary "$LOGS/trajectory_summary.json" 2>&1 | tee -a "$LOGS/runner.log"
  exit "${PIPESTATUS[0]}"
fi

# Job control on: without it bash starts background jobs with SIGINT ignored,
# and the bag recorder (Python) then never sees our SIGINT and can't finalize.
set -m
pids=()
cleanup() { for p in "${pids[@]}"; do kill -INT "$p" 2>/dev/null; done; }
trap cleanup EXIT

wait_for_line() {  # file pattern timeout_s
  local i=0
  until grep -q "$2" "$1" 2>/dev/null; do
    sleep 1; i=$((i + 1))
    [[ $i -ge $3 ]] && return 1
  done
  return 0  # an until loop's status is its body's last command, not the test
}

stop_proc() {  # pid name timeout_s ; SIGINT then wait, SIGKILL as last resort
  local pid=$1 name=$2 t=$3 i=0
  kill -INT "$pid" 2>/dev/null || { say "$name already exited"; wait "$pid" 2>/dev/null; return; }
  while kill -0 "$pid" 2>/dev/null; do
    sleep 1; i=$((i + 1))
    if [[ $i -ge $t ]]; then say "WARNING: $name did not stop in ${t}s; SIGKILL"; kill -KILL "$pid"; break; fi
  done
  wait "$pid" 2>/dev/null; say "$name stopped (exit $?) after ${i}s"
}

say "starting FAST-LIO (params: /cfg/config.yaml; runtime_pos_log_enable:=true for per-scan timing)"
# Binary directly, not `ros2 run`: the ros2 wrapper does not forward SIGINT.
/ws/install/fast_lio/lib/fast_lio/fastlio_mapping --ros-args --params-file /cfg/config.yaml \
    -p runtime_pos_log_enable:=true >"$LOGS/fastlio.log" 2>&1 &
FL=$!; pids+=("$FL")

ros2 bag record -s mcap -o "$LOGS/output_bag" --topics /Odometry /cloud_registered \
    >"$LOGS/record.log" 2>&1 &
REC=$!; pids+=("$REC")

python3 "$T/odom_watch.py" /tmp/odom_state >"$LOGS/odom_watch.log" 2>&1 &
WATCH=$!; pids+=("$WATCH")

wait_for_line "$LOGS/fastlio.log" "Node init finished" 60 || { say "FAST-LIO did not start"; exit 3; }
wait_for_line "$LOGS/record.log" "Subscribed to topic '/Odometry'" 60 || { say "recorder never subscribed to /Odometry (discovery?)"; exit 3; }
wait_for_line "$LOGS/record.log" "Subscribed to topic '/cloud_registered'" 30 || { say "recorder never subscribed to /cloud_registered"; exit 3; }
sleep 2

say "playing $LIDAR $IMU at rate $RATE"
ros2 bag play "$BAG" --topics "$LIDAR" "$IMU" --rate "$RATE" --disable-keyboard-controls \
    ${DURATION:+--playback-duration "$DURATION"} >"$LOGS/play.log" 2>&1
say "playback finished (exit $?)"

# Drain: wait until /Odometry has been quiet for 5 s (max 300 s).
i=0
while :; do
  sleep 1; i=$((i + 1))
  read -r n last </tmp/odom_state
  now=$(python3 -c 'import time; print(f"{time.monotonic():.3f}")')
  quiet=$(python3 -c "print(int($now - $last >= 5))")
  [[ $quiet == 1 ]] && break
  [[ $i -ge 300 ]] && { say "WARNING: /Odometry still active after 300 s drain"; break; }
done
say "drained: odom_watch saw $n /Odometry messages"

stop_proc "$REC" recorder 60
stop_proc "$WATCH" odom_watch 10
stop_proc "$FL" fastlio 120
trap - EXIT
cp -a /ws/src/FAST_LIO_ROS2/Log "$LOGS/fastlio_Log" 2>/dev/null || true

say "exporting trajectory"
python3 "$T/bag_to_tum.py" "$LOGS/output_bag" "$OUT/trajectory.tum" --topic /Odometry \
    --summary "$LOGS/trajectory_summary.json" 2>&1 | tee -a "$LOGS/runner.log"
rc1=${PIPESTATUS[0]}

say "building map from recorded /cloud_registered"
python3 "$T/bag_to_map.py" "$LOGS/output_bag" "$OUT/map.pcd" --voxel "${MAP_VOXEL:-0.03}" \
    --summary "$LOGS/map_summary.json" >>"$LOGS/runner.log" 2>&1
rc2=$?
say "done (trajectory rc=$rc1, map rc=$rc2)"
[[ $rc1 == 0 && $rc2 == 0 ]]
