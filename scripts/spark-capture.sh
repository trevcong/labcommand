#!/usr/bin/env bash
# Mid-360 capture control on the DGX Spark.
# Piped over SSH from LabCommand:  ssh spark 'bash -s -- status' < spark-capture.sh
# Safe to run on a Mac for a dry status (reports enP7s7 missing).
set -u

NIC="${CAPTURE_NIC:-enP7s7}"
SENSOR_IP="${CAPTURE_SENSOR_IP:-192.168.1.135}"
HOST_IP="${CAPTURE_HOST_IP:-192.168.1.5}"
IMAGE="${CAPTURE_IMAGE:-livox:jazzy}"
CONTAINER="${CAPTURE_CONTAINER:-livox-slam}"
WS="${CAPTURE_WS:-/home/tcong/livox_ws}"
DATA_ROOT="${CAPTURE_DATA:-/home/tcong/datasets/home_lab}"
LOCK="${DATA_ROOT}/.record.lock"

# Rate-probe windows (seconds). Lidar is 10 Hz, IMU ~200 Hz, so these windows
# are already many samples. Probing costs ~5 s, so cmd_status only pays it when
# carrier is up — a NO-CARRIER link is already reported by the link stage.
RATE_WINDOW="${CAPTURE_RATE_WINDOW:-2}"
IMU_WINDOW="${CAPTURE_IMU_WINDOW:-1}"

json_escape() {
  python3 -c 'import json,sys; print(json.dumps(sys.stdin.read()[:-1] if False else sys.argv[1]))' "$1" 2>/dev/null \
    || printf '"%s"' "${1//\"/\\\"}"
}

has_cmd() { command -v "$1" >/dev/null 2>&1; }

nic_state() {
  OPER="missing"
  CARRIER=0
  ADDR=""
  if [[ -d "/sys/class/net/${NIC}" ]]; then
    OPER="$(cat "/sys/class/net/${NIC}/operstate" 2>/dev/null || echo unknown)"
    CARRIER="$(cat "/sys/class/net/${NIC}/carrier" 2>/dev/null || echo 0)"
    if has_cmd ip; then
      ADDR="$(ip -br addr show "${NIC}" 2>/dev/null | awk '{print $3}')"
    fi
  fi
}

sensor_ping() {
  if has_cmd ping; then
    ping -c 1 -W 2 "${SENSOR_IP}" >/dev/null 2>&1 && echo 1 || echo 0
  else
    echo 0
  fi
}

stream_state() {
  if ! has_cmd docker; then
    echo absent
    return
  fi
  local st
  st="$(docker inspect -f '{{.State.Status}}' "${CONTAINER}" 2>/dev/null || true)"
  case "${st}" in
    running) echo running ;;
    exited|created|dead|paused) echo exited ;;
    *) echo absent ;;
  esac
}

ptp_on() {
  if pgrep -x ptp4l >/dev/null 2>&1; then echo 1; else echo 0; fi
}

git_short() {
  local dir="$1"
  if [[ -d "${dir}/.git" ]] && has_cmd git; then
    git -C "${dir}" rev-parse --short HEAD 2>/dev/null || true
  fi
}

find_src() {
  local name="$1"
  local c
  for c in \
    "${WS}/src/${name}" \
    "${WS}/${name}" \
    "${WS}/src/livox_ros_driver2" \
    ; do
    if [[ -d "${c}/.git" ]] || [[ -d "${c}" ]]; then
      echo "${c}"
      return
    fi
  done
}

record_fields() {
  REC_ACTIVE=0
  REC_SESSION=""
  REC_PID=""
  if [[ -f "${LOCK}" ]]; then
    # lock: session_id pid
    REC_SESSION="$(awk 'NR==1{print $1}' "${LOCK}" 2>/dev/null || true)"
    REC_PID="$(awk 'NR==1{print $2}' "${LOCK}" 2>/dev/null || true)"
    if [[ -n "${REC_PID}" ]] && kill -0 "${REC_PID}" 2>/dev/null; then
      REC_ACTIVE=1
    elif has_cmd docker && [[ "$(stream_state)" == "running" ]]; then
      if docker exec "${CONTAINER}" bash -lc 'pgrep -f "ros2 bag record" >/dev/null' 2>/dev/null; then
        REC_ACTIVE=1
      else
        REC_ACTIVE=0
      fi
    else
      REC_ACTIVE=0
    fi
  fi
}

mem_pct() {
  if [[ -r /proc/meminfo ]]; then
    awk '/^MemTotal:/{t=$2} /^MemAvailable:/{a=$2} END{if (t>0) printf "%.0f", (t-a)*100/t}' \
      /proc/meminfo 2>/dev/null
  fi
}

# Measures what the container is actually publishing. A container can be "running"
# while the driver never bound to the sensor — that is the state this catches.
# Sets RATES_MEASURED=1 only when the probe completed; callers must omit the JSON
# keys otherwise so the panel prints "unavailable" rather than a fabricated 0.
rate_probe() {
  CLOUD_HZ=""
  IMU_HZ=""
  DRIVER_ST=""
  FASTLIO_ST=""
  RATES_MEASURED=0

  has_cmd docker || return 0
  [[ "$(stream_state)" == "running" ]] || return 0

  local inner out
  inner="$(cat <<EOS
source /opt/ros/jazzy/setup.sh >/dev/null 2>&1
source /ws/install/setup.sh >/dev/null 2>&1
C=\$(timeout -k 1 ${RATE_WINDOW} ros2 topic hz /livox/lidar 2>/dev/null | grep -m1 'average rate:' | awk '{print \$3}')
I=\$(timeout -k 1 ${IMU_WINDOW} ros2 topic hz /livox/imu 2>/dev/null | grep -m1 'average rate:' | awk '{print \$3}')
echo "cloud=\$C"
echo "imu=\$I"
pgrep -f 'livox_ros_driver2_nod[e]' >/dev/null 2>&1 && echo driver=running || echo driver=absent
pgrep -f 'fastlio_mappin[g]' >/dev/null 2>&1 && echo fastlio=running || echo fastlio=absent
EOS
)"
  out="$(docker exec "${CONTAINER}" bash -lc "${inner}" </dev/null 2>/dev/null)" || return 0

  RATES_MEASURED=1
  CLOUD_HZ="$(printf '%s\n' "${out}" | sed -n 's/^cloud=//p' | head -1)"
  IMU_HZ="$(printf '%s\n' "${out}" | sed -n 's/^imu=//p' | head -1)"
  DRIVER_ST="$(printf '%s\n' "${out}" | sed -n 's/^driver=//p' | head -1)"
  FASTLIO_ST="$(printf '%s\n' "${out}" | sed -n 's/^fastlio=//p' | head -1)"
  # Probe ran and saw nothing: a measured zero, not a missing key.
  [[ -n "${CLOUD_HZ}" ]] || CLOUD_HZ=0
  [[ -n "${IMU_HZ}" ]] || IMU_HZ=0
}

cmd_status() {
  nic_state
  local ping strm ptp
  ping="$(sensor_ping)"
  strm="$(stream_state)"
  ptp="$(ptp_on)"
  record_fields
  # Rates only buy information when the link is up; NO-CARRIER is already reported.
  RATES_MEASURED=0
  if [[ "${CARRIER}" == "1" || "${CAPTURE_FORCE_RATES:-0}" == "1" ]]; then
    rate_probe
  fi
  printf '{'
  printf '"ok":true,'
  printf '"enP7s7":{"operstate":%s,"carrier":%s,"addr":%s},' \
    "$(json_escape "${OPER}")" "${CARRIER}" "$(json_escape "${ADDR}")"
  printf '"sensor_ping":%s,' "$([[ "${ping}" == 1 ]] && echo true || echo false)"
  printf '"stream":%s,' "$(json_escape "${strm}")"
  printf '"recording":{"active":%s,"session":%s,"pid":%s},' \
    "$([[ "${REC_ACTIVE}" == 1 ]] && echo true || echo false)" \
    "$(json_escape "${REC_SESSION}")" \
    "$(json_escape "${REC_PID}")"
  printf '"ptp4l":%s' "$([[ "${ptp}" == 1 ]] && echo true || echo false)"
  # Optional keys only when this host can measure them. Never *fabricate* a rate —
  # but a probe that ran and saw no messages reports a real 0, which is the whole
  # point: a container publishing nothing must not read as a healthy stream.
  if [[ -f /proc/loadavg ]]; then
    local load
    load="$(awk '{print $1}' /proc/loadavg 2>/dev/null || true)"
    if [[ -n "${load}" ]]; then
      printf ',"spark_load":%s' "${load}"
    fi
  fi
  local mem
  mem="$(mem_pct)"
  if [[ -n "${mem}" ]]; then
    printf ',"spark_mem_pct":%s' "${mem}"
  fi
  if [[ "${RATES_MEASURED}" == "1" ]]; then
    printf ',"cloud_hz":%s' "${CLOUD_HZ}"
    printf ',"imu_hz":%s' "${IMU_HZ}"
    [[ -n "${DRIVER_ST}" ]] && printf ',"driver_status":%s' "$(json_escape "${DRIVER_ST}")"
    [[ -n "${FASTLIO_ST}" ]] && printf ',"fastlio_status":%s' "$(json_escape "${FASTLIO_ST}")"
  fi
  if [[ "${REC_ACTIVE}" == 1 && -n "${REC_SESSION}" ]]; then
    local bagdir="${DATA_ROOT}/sessions/${REC_SESSION}"
    if [[ -d "${bagdir}" ]]; then
      local b
      b="$(du -sk "${bagdir}" 2>/dev/null | awk '{print $1 * 1024}')"
      if [[ -n "${b}" && "${b}" != "0" ]]; then
        printf ',"bag_bytes":%s' "${b}"
        printf ',"write_health":"ok"'
      fi
    fi
  fi
  printf '}\n'
}

cmd_start_stream() {
  nic_state
  if [[ "${CARRIER}" != "1" ]]; then
    printf '{"ok":false,"error":"NO-CARRIER on %s (operstate=%s)"}\n' "${NIC}" "${OPER}"
    return 2
  fi
  if ! has_cmd docker; then
    printf '{"ok":false,"error":"docker not found"}\n'
    return 2
  fi
  mkdir -p "${DATA_ROOT}/sessions"
  docker rm -f livox-slam livox-ros >/dev/null 2>&1 || true
  if ! docker run -d --name "${CONTAINER}" --net=host \
      -v "${WS}:/ws" \
      -v "${DATA_ROOT}:/data" \
      -w /ws \
      "${IMAGE}" bash -c \
      "source /opt/ros/jazzy/setup.sh && source /ws/install/setup.sh && ros2 launch /ws/slam_launch.py" \
      >/tmp/labcommand-stream.out 2>/tmp/labcommand-stream.err; then
    local err
    err="$(tr '\n' ' ' </tmp/labcommand-stream.err 2>/dev/null | cut -c1-160)"
    printf '{"ok":false,"error":%s}\n' "$(json_escape "${err:-docker run failed}")"
    return 2
  fi
  printf '{"ok":true,"action":"start-stream","container":%s}\n' "$(json_escape "${CONTAINER}")"
}

cmd_stop_record() {
  record_fields
  if has_cmd docker && [[ "$(stream_state)" == "running" ]]; then
    docker exec "${CONTAINER}" bash -lc 'pkill -INT -f "ros2 bag record" || true' >/dev/null 2>&1 || true
    sleep 1
    docker exec "${CONTAINER}" bash -lc 'pkill -TERM -f "ros2 bag record" || true' >/dev/null 2>&1 || true
  fi
  if [[ -n "${REC_PID}" ]]; then
    kill -INT "${REC_PID}" 2>/dev/null || true
  fi
  rm -f "${LOCK}"
  printf '{"ok":true,"action":"stop-record"}\n'
}

cmd_stop_stream() {
  cmd_stop_record >/dev/null 2>&1 || true
  if has_cmd docker; then
    docker stop -t 60 "${CONTAINER}" >/dev/null 2>&1 || true
  fi
  printf '{"ok":true,"action":"stop-stream"}\n'
}

write_sidecar() {
  local dest="$1"
  local session_id="$2"
  local layout="$3"
  local duration="$4"
  local notes="$5"
  local path_script="$6"
  local site="$7"

  mkdir -p "${dest}"
  local driver slam sdk ptp
  driver="$(git_short "${WS}/src/livox_ros_driver2")"
  [[ -n "${driver}" ]] || driver="$(git_short "${WS}/livox_ros_driver2")"
  slam="$(git_short "${WS}/src/FAST_LIO_ROS2")"
  [[ -n "${slam}" ]] || slam="$(git_short "${WS}/FAST_LIO_ROS2")"
  sdk="$(git_short "${WS}/src/Livox-SDK2")"
  [[ -n "${sdk}" ]] || sdk="$(git_short "${WS}/Livox-SDK2")"
  ptp="$(ptp_on)"

  # Copy configs if present (DATA-CAPTURE §3).
  local cfg yaml
  for cfg in \
      "${WS}/src/livox_ros_driver2/config/MID360_config.json" \
      "${WS}/livox_ros_driver2/config/MID360_config.json" \
      "${WS}/config/MID360_config.json"; do
    if [[ -f "${cfg}" ]]; then
      cp -f "${cfg}" "${dest}/MID360_config.json"
      break
    fi
  done
  for yaml in \
      "${WS}/src/FAST_LIO_ROS2/config/mid360.yaml" \
      "${WS}/FAST_LIO_ROS2/config/mid360.yaml" \
      "${WS}/config/mid360.yaml"; do
    if [[ -f "${yaml}" ]]; then
      cp -f "${yaml}" "${dest}/mid360.yaml"
      break
    fi
  done

  cat > "${dest}/meta.yaml" <<EOF
# capture provenance — DATA-CAPTURE.md §3
session_id: ${session_id}
site: ${site}
layout_tag: ${layout}
path_script: ${path_script}
duration_s: ${duration}
deliberate_change: ${notes}
operator: tcong
ptp_on: $([[ "${ptp}" == 1 ]] && echo true || echo false)
xfer_format: 1
xfer_format_name: CustomMsg
record_topics: /livox/lidar /livox/imu /tf /tf_static /Odometry /path /cloud_registered
forbidden_topics: /Laser_map
git_hash_driver: ${driver}
git_hash_slam: ${slam}
git_hash_sdk: ${sdk}
container: ${CONTAINER}
image: ${IMAGE}
EOF
}

cmd_start_record() {
  local session_id="${2:-}"
  local layout="${3:-baseline}"
  local duration="${4:-0}"
  local notes="${5:-}"
  local path_script="${6:-fixed loop}"
  local site="${7:-home_lab}"

  if [[ -z "${session_id}" ]]; then
    printf '{"ok":false,"error":"session_id required"}\n'
    return 2
  fi
  if [[ "$(stream_state)" != "running" ]]; then
    printf '{"ok":false,"error":"stream is not running"}\n'
    return 2
  fi
  record_fields
  if [[ "${REC_ACTIVE}" == 1 ]]; then
    printf '{"ok":false,"error":"already recording %s"}\n' "${REC_SESSION}"
    return 2
  fi

  local dest="${DATA_ROOT}/sessions/${session_id}"
  if [[ -e "${dest}/bag" ]]; then
    session_id="${session_id}_$(date -u +%H%M%S)"
    dest="${DATA_ROOT}/sessions/${session_id}"
  fi
  write_sidecar "${dest}" "${session_id}" "${layout}" "${duration}" "${notes}" "${path_script}" "${site}"

  local dur_flag=""
  if [[ "${duration}" =~ ^[0-9]+$ ]] && [[ "${duration}" -gt 0 ]]; then
    dur_flag="-d ${duration}"
  fi

  # Record inside the container so ROS 2 + CustomMsg types exist.
  # Topics match DATA-CAPTURE.md; /Laser_map is excluded on purpose.
  docker exec -d "${CONTAINER}" bash -lc "\
    source /opt/ros/jazzy/setup.sh && source /ws/install/setup.sh && \
    mkdir -p /data/sessions/${session_id} && \
    ros2 bag record -s mcap -o /data/sessions/${session_id}/bag \
      --max-bag-size 2147483648 ${dur_flag} \
      /livox/lidar /livox/imu /tf /tf_static \
      /Odometry /path /cloud_registered \
      > /data/sessions/${session_id}/record.log 2>&1"

  # Host-side lock (best-effort pid of docker exec is not the bag; status uses pgrep).
  printf '%s host\n' "${session_id}" > "${LOCK}"
  printf '{"ok":true,"action":"start-record","session":%s,"path":%s}\n' \
    "$(json_escape "${session_id}")" \
    "$(json_escape "${dest}")"
}

usage() {
  printf '{"ok":false,"error":"usage: spark-capture.sh status|start-stream|stop-stream|start-record|stop-record"}\n'
}

main() {
  local op="${1:-status}"
  case "${op}" in
    status) cmd_status ;;
    start-stream) cmd_start_stream ;;
    stop-stream) cmd_stop_stream ;;
    start-record) cmd_start_record "$@" ;;
    stop-record) cmd_stop_record ;;
    *) usage; return 2 ;;
  esac
}

main "$@"
