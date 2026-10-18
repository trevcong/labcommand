#!/usr/bin/env bash
# Build and replace only the room-memory container on the Spark.
# Reads like ~/Developer/lc-tasks/room-memory-deploy.md. Does not stop lidar-center,
# lidar-dual, or ptp-master, and does not delete ~/room-memory.
# --dry-run prints the archive pipeline and the remote script. --self-test checks the
# guard and failure order with local stand-ins. Neither talks to the Spark.
# A real deploy requires --yes or the operator to type room-memory. The replacement
# is created and inspected while the original still runs: its effective entrypoint,
# command and arguments (rig_guard:=true, ..., even when they are the old image's
# default CMD), whole environment, user, working directory, mounts, Config.Volumes,
# resolved volumes, networks and runtime settings must equal the original's. Only the
# image and the container may differ; anything a replacement cannot reproduce (an
# anonymous volume, a --mount, a two-element entrypoint) refuses the deploy before
# anything stops. The original is stopped and renamed room-memory-previous, never
# removed and never with its volumes. Rollback stays armed through the replacement's
# readiness line and a stability window: a failed rename, scratch cleanup or start,
# an exit, a restart, an unhealthy status, a launched process that ends while the
# container runs, no readiness in time, or a failure in the window removes the
# replacement and starts that same container again. If that rollback's rm, rename,
# or start fails, or room-memory is not running afterwards, the script exits 3 and
# prints a recovery command for the containers that are left. The original is already
# named room-memory: `docker start` of its id. The rename back did not land: remove
# the replacement by id, rename the original back, and start it. A fresh host has no
# original, so the command only removes the failed container. A deploy that already
# came up still exits 0 when listing image tags to prune fails. Scratch files
# are removed as root on the original's own mounts: the container creates root-owned
# room directories, and an ssh user cannot unlink what it left there.
# OpenSSH joins its command arguments and the remote login shell parses that
# string. The script is one single-quoted word after `bash -c`, so `set -euo pipefail`
# applies to the whole remote script. An unquoted `bash -c "$remote"` does not.
set -euo pipefail

HOST="spark"
SHA=""
DRY=0
SELF=0
YES=0

usage() {
  echo "usage: deploy-room-memory.sh [--host HOST] [--sha COMMIT] [--yes] [--dry-run] [--self-test]" >&2
  exit 2
}

# Seconds baked into the remote script. Self-test sets 0. A real deploy waits this
# long after stop and before a rollback run.
backoff_seconds() {
  local value=${ROOM_MEMORY_BACKOFF_S:-5}
  if [[ ! "$value" =~ ^[0-9]+$ ]]; then
    echo "ROOM_MEMORY_BACKOFF_S must be a whole number of seconds" >&2
    exit 2
  fi
  printf '%s' "$value"
}

# The replacement's readiness window (ROOM_MEMORY_READY_S, default 120: its readiness line, one
# check a second), the stability window after it (ROOM_MEMORY_STABLE_S, default 30) and the pause
# between checks (ROOM_MEMORY_POLL_S, default 1), baked into the remote script.
whole_seconds() {
  local name=$1 fallback=$2 value
  case "$name" in
    ROOM_MEMORY_READY_S) value=${ROOM_MEMORY_READY_S:-120} ;;
    ROOM_MEMORY_STABLE_S) value=${ROOM_MEMORY_STABLE_S:-30} ;;
    *) value=${ROOM_MEMORY_POLL_S:-$fallback} ;;
  esac
  if [[ ! "$value" =~ ^[0-9]+$ ]]; then
    echo "$name must be a whole number of seconds" >&2
    exit 2
  fi
  printf '%s' "$value"
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --host)
      [[ $# -ge 2 && -n ${2:-} ]] || usage
      HOST=$2
      shift 2
      ;;
    --sha)
      [[ $# -ge 2 && -n ${2:-} ]] || usage
      SHA=$2
      shift 2
      ;;
    --dry-run)
      DRY=1
      shift
      ;;
    --yes)
      YES=1
      shift
      ;;
    --self-test)
      SELF=1
      shift
      ;;
    *)
      echo "unknown argument: $1" >&2
      usage
      ;;
  esac
done

# A rendered command that names any of these is a mistake. Exit before ssh.
reject_forbidden() {
  local text=$1 name
  for name in lidar-center lidar-dual ptp-master; do
    if [[ "$text" == *"$name"* ]]; then
      echo "refusing a remote command that names ${name}" >&2
      exit 2
    fi
  done
}

# One single-quoted literal for a remote POSIX shell. ' inside the text is '\'' .
shell_quote() {
  local s=$1
  s=${s//\'/\'\\\'\'}
  printf "'%s'" "$s"
}

# One remote bash script. stdin is the git archive. The image tag replaces __SHORT__.
render_remote() {
  local short=$1
  local backoff
  backoff=$(backoff_seconds)
  local template
  template=$(cat <<'END'
set -euo pipefail
BACKOFF=__BACKOFF__
READY_S=__READY__
STABLE_S=__STABLE__
POLL_S=__POLL__
STATE_DEST=/root/room-memory
D=$(mktemp -d /tmp/rmbuild.XXXX)
trap 'rm -rf "$D"' EXIT
tar -xzf - -C "$D"
cd "$D"
docker build -f spark/room_memory_cpp/Dockerfile -t labcommand/room-memory-cpp:__SHORT__ .
cd /
rm -rf "$D"
trap - EXIT
image=labcommand/room-memory-cpp:__SHORT__
names=$(docker ps -a --format '{{.Names}}')
has_name() {
  local wanted=$1 name
  while IFS= read -r name; do
    if [ "$name" = "$wanted" ]; then
      return 0
    fi
  done <<ENDNAMES
$names
ENDNAMES
  return 1
}
refuse() {
  echo "room-memory deploy: $*" >&2
  exit 1
}
# One inspect list, one value per line, into LINES. A value with a line break in it
# (or an empty last value) does not read back as itself: refuse it.
LINES=()
split_lines() {
  local line
  LINES=()
  if [ -n "$1" ]; then
    while IFS= read -r line; do
      LINES+=("$line")
    done < <(printf '%s\n' "$1")
  fi
  if [ "${#LINES[@]}" -ne "$2" ]; then
    refuse "$3 does not read back exactly; replace room-memory by hand"
  fi
}
in_lines() {
  local wanted=$1 line
  while IFS= read -r line; do
    if [ "$line" = "$wanted" ]; then
      return 0
    fi
  done < <(printf '%s\n' "$2")
  return 1
}
# A fresh host has no container to copy: the documented run command.
run_room() {
  docker run -d --name room-memory --restart unless-stopped --net host \
    -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 --cpus 4 --memory 2g --memory-swap 2g --pids-limit 256 \
    --log-opt max-size=10m --log-opt max-file=3 \
    -v "$HOME/room-memory:/root/room-memory" \
    "$1"
}
for leftover in room-memory-previous room-memory-next; do
  if has_name "$leftover"; then
    running=$(docker inspect -f '{{.State.Running}}' "$leftover")
    if [ "$running" = "true" ]; then
      refuse "$leftover is running; stop it before a deploy"
    fi
  fi
done
if ! has_name room-memory && has_name room-memory-previous; then
  refuse "room-memory is missing and room-memory-previous is kept from a deploy; rename it back to room-memory or remove it"
fi
# With a room-memory to replace, the replacement is that container's configuration on the new
# image: its entrypoint, command and arguments (rig_guard:=true, ...), its whole environment, user,
# working directory, mounts, network and runtime settings. It is created and checked while the
# original still runs; whatever cannot be reproduced exactly refuses the deploy before anything
# stops. The original is stopped and renamed room-memory-previous, never removed and never with its
# volumes, so a rollback starts that same container.
orig_id=""
next_id=""
previous_tag=""
renamed=0
run_args=()
run_cmd=()
cleanup_args=(-v "$HOME/room-memory:$STATE_DEST")
# One line per resolved mount: type|name|source|destination|mode|rw|propagation.
MOUNTS='{{range .Mounts}}{{.Type}}|{{.Name}}|{{.Source}}|{{.Destination}}|{{.Mode}}|{{.RW}}|{{.Propagation}}{{println}}{{end}}'
NETWORKS='{{range $k, $v := .NetworkSettings.Networks}}{{$k}}|{{json $v.IPAMConfig}}|{{json $v.Aliases}}{{println}}{{end}}'
SHAPE='{{json .HostConfig}} {{json .Config.Entrypoint}} {{json .Config.Cmd}} {{json .Config.User}} {{json .Config.WorkingDir}} {{json .Config.Volumes}} {{json .Config.Tty}} {{json .Config.OpenStdin}} {{json .Config.StdinOnce}}'
sorted_lines() {
  printf '%s\n' "$1" | LC_ALL=C sort
}
if has_name room-memory; then
  orig_id=$(docker inspect -f '{{.Id}}' room-memory)
  previous_tag=$(docker inspect -f '{{.Config.Image}}' "$orig_id")
  # The entrypoint it runs, when the new image's differs: one element is --entrypoint; more cannot
  # be given to docker create.
  entrypoint=$(docker inspect -f '{{json .Config.Entrypoint}}' "$orig_id")
  image_entrypoint=$(docker image inspect -f '{{json .Config.Entrypoint}}' "$image")
  if [ "$entrypoint" != "$image_entrypoint" ]; then
    entry_text=$(docker inspect -f '{{range .Config.Entrypoint}}{{println .}}{{end}}' "$orig_id")
    entry_count=$(docker inspect -f '{{len .Config.Entrypoint}}' "$orig_id")
    if [ "$entry_count" != "1" ]; then
      refuse "room-memory's entrypoint $entrypoint cannot be given to a replacement; replace it by hand"
    fi
    split_lines "$entry_text" 1 "the entrypoint"
    run_args+=(--entrypoint "${LINES[0]}")
  fi
  # The command the container runs, always, even when it is the old image's default: that
  # default can carry arguments (rig_guard:=true) the new image's does not.
  cmd_text=$(docker inspect -f '{{range .Config.Cmd}}{{println .}}{{end}}' "$orig_id")
  cmd_count=$(docker inspect -f '{{len .Config.Cmd}}' "$orig_id")
  split_lines "$cmd_text" "$cmd_count" "the command"
  run_cmd=(${LINES[@]+"${LINES[@]}"})
  # The whole environment, the old image's own variables included: the replacement runs with exactly
  # it. A variable only the new image has shows up in the check below and refuses the deploy.
  env_text=$(docker inspect -f '{{range .Config.Env}}{{println .}}{{end}}' "$orig_id")
  env_count=$(docker inspect -f '{{len .Config.Env}}' "$orig_id")
  split_lines "$env_text" "$env_count" "the environment"
  for entry in ${LINES[@]+"${LINES[@]}"}; do
    run_args+=(--env "$entry")
  done
  user=$(docker inspect -f '{{.Config.User}}' "$orig_id")
  if [ "$user" != "$(docker image inspect -f '{{.Config.User}}' "$image")" ]; then
    run_args+=(--user "$user")
  fi
  workdir=$(docker inspect -f '{{.Config.WorkingDir}}' "$orig_id")
  if [ "$workdir" != "$(docker image inspect -f '{{.Config.WorkingDir}}' "$image")" ]; then
    run_args+=(--workdir "$workdir")
  fi
  restart=$(docker inspect -f '{{.HostConfig.RestartPolicy.Name}}' "$orig_id")
  retries=$(docker inspect -f '{{.HostConfig.RestartPolicy.MaximumRetryCount}}' "$orig_id")
  if [ "$restart" = "on-failure" ] && [ "$retries" != "0" ]; then
    restart="on-failure:$retries"
  fi
  if [ -n "$restart" ]; then
    run_args+=(--restart "$restart")
  fi
  network=$(docker inspect -f '{{.HostConfig.NetworkMode}}' "$orig_id")
  if [ -n "$network" ]; then
    run_args+=(--network "$network")
  fi
  nano=$(docker inspect -f '{{.HostConfig.NanoCpus}}' "$orig_id")
  if [ "$nano" != "0" ]; then
    cpus=$(awk -v n="$nano" 'BEGIN { printf "%.9f", n / 1e9 }')
    run_args+=(--cpus "$cpus")
  fi
  memory=$(docker inspect -f '{{.HostConfig.Memory}}' "$orig_id")
  if [ "$memory" != "0" ]; then
    run_args+=(--memory "$memory")
  fi
  swap=$(docker inspect -f '{{.HostConfig.MemorySwap}}' "$orig_id")
  if [ "$swap" != "0" ]; then
    run_args+=(--memory-swap "$swap")
  fi
  pids=$(docker inspect -f '{{if .HostConfig.PidsLimit}}{{.HostConfig.PidsLimit}}{{end}}' "$orig_id")
  if [ -n "$pids" ]; then
    run_args+=(--pids-limit "$pids")
  fi
  log_driver=$(docker inspect -f '{{.HostConfig.LogConfig.Type}}' "$orig_id")
  if [ -n "$log_driver" ]; then
    run_args+=(--log-driver "$log_driver")
  fi
  log_text=$(docker inspect -f '{{range $k, $v := .HostConfig.LogConfig.Config}}{{$k}}={{$v}}{{println}}{{end}}' "$orig_id")
  log_count=$(docker inspect -f '{{len .HostConfig.LogConfig.Config}}' "$orig_id")
  split_lines "$log_text" "$log_count" "the log options"
  for entry in ${LINES[@]+"${LINES[@]}"}; do
    run_args+=(--log-opt "$entry")
  done
  # Mounts: binds and named volumes (-v) are carried. A --mount, or an anonymous volume (-v <path>,
  # or an image VOLUME nothing was mounted on), has no -v that keeps the same volume and the same
  # Config.Volumes: refused. The state directory has to be one of them.
  if [ "$(docker inspect -f '{{len .HostConfig.Mounts}}' "$orig_id")" != "0" ]; then
    refuse "room-memory has --mount mounts, which a replacement does not keep; replace it by hand"
  fi
  binds_text=$(docker inspect -f '{{range .HostConfig.Binds}}{{println .}}{{end}}' "$orig_id")
  binds_count=$(docker inspect -f '{{len .HostConfig.Binds}}' "$orig_id")
  split_lines "$binds_text" "$binds_count" "the volumes"
  binds=(${LINES[@]+"${LINES[@]}"})
  for entry in ${binds[@]+"${binds[@]}"}; do
    run_args+=(--volume "$entry")
  done
  orig_mounts=$(docker inspect -f "$MOUNTS" "$orig_id")
  state_mounted=0
  while IFS='|' read -r type name source dest mode rw propagation; do
    [ -n "$type" ] || continue
    if [ "$type" = volume ]; then
      named=0
      for entry in ${binds[@]+"${binds[@]}"}; do
        case "$entry" in "$name:"*) named=1 ;; esac
      done
      if [ "$named" -eq 0 ]; then
        refuse "room-memory keeps $dest in anonymous volume $name, which a replacement cannot keep; replace it by hand"
      fi
    elif [ "$type" != bind ]; then
      refuse "room-memory has a $type mount at $dest; replace it by hand"
    fi
    if [ "$dest" = "$STATE_DEST" ]; then
      state_mounted=1
    fi
  done < <(printf '%s\n' "$orig_mounts")
  if [ "$state_mounted" -eq 0 ]; then
    refuse "room-memory has no mount at $STATE_DEST: its saved room would not reach a replacement; replace it by hand"
  fi
  # The scratch cleanup reaches the state through the original's own mounts.
  cleanup_args=(--volumes-from "$orig_id")
fi
# Left by a deploy that stopped before it started anything: never ran as the service.
if has_name room-memory-next; then
  docker rm room-memory-next
fi
# Before stop: a failure removes the replacement this deploy created and leaves room-memory as it was.
discard_next() {
  local status=${1:-$?}
  if [ "$status" -eq 0 ]; then
    status=1
  fi
  trap - ERR
  set +e
  if [ -n "$next_id" ]; then
    docker rm -f "$next_id"
  else
    docker rm -f room-memory-next >/dev/null 2>&1
  fi
  exit "$status"
}
# After stop: a failure removes the replacement and starts the original container, under its
# own name, with its own command, settings and volumes. A fresh host had none: only the failed one goes.
# rm, rename, and start each have to succeed, and room-memory has to be running afterwards.
# Any failure exits 3 and prints a recovery command built from the ids and the name the original
# actually has. One fixed `docker rm -f room-memory && docker rename room-memory-previous ...`
# deletes the original when the start is what failed: it is already named room-memory, and
# room-memory-previous is gone. The same string is wrong when the rename back did not land
# (the original is running as room-memory-previous) and on a fresh host (there is no previous).
# The original failure status is kept only when the rollback itself worked.
rollback_service() {
  local status=${1:-$?}
  if [ "$status" -eq 0 ]; then
    status=1
  fi
  trap - ERR
  set +e
  local failed=0 running="" recovery="" name_now=""
  if [ -n "$orig_id" ]; then
    if [ -n "$next_id" ]; then
      docker rm -f "$next_id" || failed=1
    fi
    if [ "$renamed" -eq 1 ]; then
      docker rename "$orig_id" room-memory || failed=1
    fi
    sleep "$BACKOFF"
    docker start "$orig_id" || failed=1
    running=$(docker inspect -f '{{.State.Running}}' room-memory) || failed=1
    if [ "$running" != "true" ]; then
      failed=1
    fi
  else
    docker rm -f room-memory || failed=1
  fi
  if [ "$failed" -ne 0 ]; then
    if [ -z "$orig_id" ]; then
      recovery="docker rm -f room-memory"
    else
      # Ground truth, not the rename's exit code: a failed rename leaves the previous name.
      name_now=$(docker inspect -f '{{.Name}}' "$orig_id" 2>/dev/null) || true
      if [ "$name_now" = "/room-memory" ]; then
        recovery="docker start $orig_id"
      elif [ -n "$next_id" ]; then
        recovery="docker rm -f $next_id; docker rename $orig_id room-memory; docker start $orig_id"
      else
        recovery="docker rename $orig_id room-memory; docker start $orig_id"
      fi
    fi
    if [ "$running" = "true" ]; then
      echo "ROLLBACK FAILED: room-memory is running, but an earlier rollback step failed" >&2
    else
      echo "ROLLBACK FAILED: room-memory is not running" >&2
    fi
    echo "$recovery" >&2
    exit 3
  fi
  echo "rolled back" >&2
  exit "$status"
}
# Empty while the container is up: running, never restarted, not unhealthy. Else why not.
health_failure() {
  local state status restarts health
  state=$(docker inspect -f '{{.State.Status}}|{{.RestartCount}}|{{if .State.Health}}{{.State.Health.Status}}{{end}}' "$1") || {
    echo "cannot be inspected"
    return
  }
  IFS='|' read -r status restarts health <<<"$state"
  if [ "$status" != "running" ]; then
    echo "is $status"
  elif [ "$restarts" != "0" ]; then
    echo "restarted $restarts times"
  elif [ "$health" = "unhealthy" ]; then
    echo "is unhealthy"
  fi
}
# Empty while every process ros2 launch started still runs, from the container's logs. Else the
# line launch logs when one ends. The image's launch shuts down when either process exits; one
# built without that keeps the container running around a dead node, so Docker reports nothing.
process_failure() {
  local line
  line=$(grep -m1 -e ']: process has died' -e ']: process has finished cleanly' <<<"$1" || true)
  if [ -n "$line" ]; then
    echo "lost a process (${line})"
  fi
}
# docker start returning is not the service up. The node logs its readiness line right after it
# allocates a publisher generation. This PR does not classify the room: every start that allocates
# a generation logs the line, including a room PR-B will call UNCERTAIN. Ready
# within READY_S seconds, then up for STABLE_S more with every launched process alive, or this fails.
wait_ready() {
  local id=$1 n failure logs ready=0
  for ((n = 0; n < READY_S; n++)); do
    failure=$(health_failure "$id")
    if [ -n "$failure" ]; then
      echo "room-memory deploy: the replacement $failure before it was ready" >&2
      return 1
    fi
    logs=$(docker logs "$id" 2>&1 || true)
    failure=$(process_failure "$logs")
    if [ -n "$failure" ]; then
      echo "room-memory deploy: the replacement $failure before it was ready" >&2
      return 1
    fi
    case "$logs" in
      *"room-memory ready: publisher_generation"*) ready=1 ;;
    esac
    if [ "$ready" -eq 1 ]; then
      break
    fi
    sleep "$POLL_S"
  done
  if [ "$ready" -eq 0 ]; then
    echo "room-memory deploy: the replacement was not ready after ${READY_S} s" >&2
    return 1
  fi
  for ((n = 0; n < STABLE_S; n++)); do
    sleep "$POLL_S"
    failure=$(health_failure "$id")
    if [ -z "$failure" ]; then
      failure=$(process_failure "$(docker logs "$id" 2>&1 || true)")
    fi
    if [ -n "$failure" ]; then
      echo "room-memory deploy: the replacement $failure within ${STABLE_S} s of ready" >&2
      return 1
    fi
  done
}
if [ -n "$orig_id" ]; then
  trap discard_next ERR
  next_id=$(docker create --name room-memory-next ${run_args[@]+"${run_args[@]}"} "$image" ${run_cmd[@]+"${run_cmd[@]}"})
  # Every setting, not only the ones copied above: a setting this script does not carry (a
  # capability, a device), a variable or volume only the new image declares, or another network
  # shows up here and stops the deploy before stop. Only the image and the container differ.
  mismatch() {
    printf 'room-memory deploy: the replacement does not match room-memory (%s)\n  room-memory: %s\n  replacement: %s\n' \
      "$1" "$2" "$3" >&2
    discard_next 1
  }
  orig_shape=$(docker inspect -f "$SHAPE" "$orig_id")
  next_shape=$(docker inspect -f "$SHAPE" "$next_id")
  [ "$orig_shape" = "$next_shape" ] || mismatch "settings" "$orig_shape" "$next_shape"
  orig_env=$(sorted_lines "$env_text")
  next_env=$(sorted_lines "$(docker inspect -f '{{range .Config.Env}}{{println .}}{{end}}' "$next_id")")
  [ "$orig_env" = "$next_env" ] || mismatch "environment" "$orig_env" "$next_env"
  orig_mounts=$(sorted_lines "$orig_mounts")
  next_mounts=$(sorted_lines "$(docker inspect -f "$MOUNTS" "$next_id")")
  [ "$orig_mounts" = "$next_mounts" ] || mismatch "mounts" "$orig_mounts" "$next_mounts"
  orig_networks=$(sorted_lines "$(docker inspect -f "$NETWORKS" "$orig_id")")
  next_networks=$(sorted_lines "$(docker inspect -f "$NETWORKS" "$next_id")")
  [ "$orig_networks" = "$next_networks" ] || mismatch "networks" "$orig_networks" "$next_networks"
  # The last deploy's rollback point, kept until this one is sure to replace room-memory.
  if has_name room-memory-previous; then
    docker rm room-memory-previous
  fi
  docker stop "$orig_id"
  # Once stop has succeeded, rename, scratch cleanup, start and readiness are one unit.
  trap rollback_service ERR
  sleep "$BACKOFF"
  docker rename "$orig_id" room-memory-previous
  renamed=1
else
  mkdir -p "$HOME/room-memory"
fi
# Scratch names are the atomic-write destinations plus ".tmp": reference-<sensor>.cdr.tmp,
# registration-<sensor>.bin.tmp, rig_state.tmp, map_epoch.tmp, publisher_generation.tmp. A room_id
# may itself end in .tmp (lab.tmp), and that room is a direct child of the state dir. Remove scratch
# names inside a room, including a directory that blocks the next rename. Never remove a room
# directory. A scratch file in the state dir is not a room.
# The room-memory container runs as root and leaves root-owned room directories on its state
# mount. find as the ssh user then fails under set -e, after the old container is already stopped.
# The same find runs as root in a one-shot container, on the original's own mounts.
docker run --rm --network none --user 0 \
  --entrypoint bash \
  "${cleanup_args[@]}" \
  "$image" -c 'set -euo pipefail
find /root/room-memory -mindepth 2 -depth \( \
  -name reference-mid360.cdr.tmp -o \
  -name reference-jt128.cdr.tmp -o \
  -name registration-mid360.bin.tmp -o \
  -name registration-jt128.bin.tmp -o \
  -name rig_state.tmp -o \
  -name map_epoch.tmp -o \
  -name publisher_generation.tmp \
\) -exec rm -rf {} +
find /root/room-memory -mindepth 1 -maxdepth 1 -type f \( \
  -name reference-mid360.cdr.tmp -o \
  -name reference-jt128.cdr.tmp -o \
  -name registration-mid360.bin.tmp -o \
  -name registration-jt128.bin.tmp -o \
  -name rig_state.tmp -o \
  -name map_epoch.tmp -o \
  -name publisher_generation.tmp \
\) -exec rm -f {} +'
if [ -n "$orig_id" ]; then
  docker rename "$next_id" room-memory
  if ! docker start "$next_id"; then
    rollback_service 1
  fi
  if ! wait_ready "$next_id"; then
    rollback_service 1
  fi
else
  if ! run_room "$image"; then
    rollback_service 1
  fi
  if ! wait_ready room-memory; then
    rollback_service 1
  fi
fi
trap - ERR
# room-memory-previous stays, stopped, for a manual rollback after a deploy that came up:
#   docker rm -f room-memory && docker rename room-memory-previous room-memory && docker start room-memory
# The next deploy removes it. A failed rollback does not print this. The original may already
# hold the name room-memory, and this rm would delete it. Drop other tags of this image only,
# not the one it runs; leave
# every other image on the host. Listing those tags is best-effort: room-memory is already the
# new container, so a failed `docker images` must not fail the deploy.
image_tags=$(docker images "labcommand/room-memory-cpp" --format '{{.Repository}}:{{.Tag}}') \
  || echo "room-memory deploy: could not list image tags to prune; leaving them" >&2
while IFS= read -r tag; do
  if [ -n "$tag" ] && [ "$tag" != "$image" ] && [ "$tag" != "$previous_tag" ]; then
    docker rmi "$tag" || true
  fi
done <<ENDTAGS
$image_tags
ENDTAGS
END
)
  template=${template//__SHORT__/$short}
  template=${template//__READY__/$(whole_seconds ROOM_MEMORY_READY_S 120)}
  template=${template//__STABLE__/$(whole_seconds ROOM_MEMORY_STABLE_S 30)}
  template=${template//__POLL__/$(whole_seconds ROOM_MEMORY_POLL_S 1)}
  printf '%s\n' "${template//__BACKOFF__/$backoff}"
}

self_test() {
  export ROOM_MEMORY_BACKOFF_S=0 ROOM_MEMORY_READY_S=3 ROOM_MEMORY_STABLE_S=2 ROOM_MEMORY_POLL_S=0
  local script work repo bin ssh_log docker_log state host
  script=$(cd "$(dirname "$0")" && pwd)/$(basename "$0")
  work=$(mktemp -d "${TMPDIR:-/tmp}/room-memory-deploy-test.XXXXXX")
  # Expand the path now. `work` is local, and the EXIT trap runs after that scope is gone.
  trap "rm -rf $(printf '%q' "$work")" EXIT
  mkdir -p "$work/home" "$work/bin" "$work/volumes"
  bin=$work/bin
  ssh_log=$work/ssh.log
  docker_log=$work/docker.log
  state=$work/docker-state.json
  # Not the Spark. A stub that failed to shadow ssh still could not reach it.
  host=room-memory-self-test.invalid
  : >"$ssh_log"
  : >"$docker_log"

  fail() {
    echo "self-test: $*" >&2
    exit 1
  }

  local sample
  sample=$(render_remote abcdef1)
  [[ "$sample" == *"--name room-memory"* ]] || fail "missing container name"
  [[ "$sample" == *"labcommand/room-memory-cpp:abcdef1"* ]] || fail "missing image tag"
  [[ "$sample" == *"--restart unless-stopped"* ]] || fail "missing restart policy"
  [[ "$sample" == *"--cpus 4"* && "$sample" == *"--memory 2g"* && "$sample" == *"--memory-swap 2g"* ]] || fail "missing limits"
  [[ "$sample" == *"--pids-limit 256"* ]] || fail "missing pids limit"
  [[ "$sample" == *"--log-opt max-size=10m"* && "$sample" == *"--log-opt max-file=3"* ]] || fail "missing log opts"
  [[ "$sample" == *'-v "$HOME/room-memory:/root/room-memory"'* ]] || fail "missing state mount"
  [[ "$sample" == *"FASTDDS_BUILTIN_TRANSPORTS=UDPv4"* ]] || fail "missing transport"
  [[ "$sample" != *"--ipc=host"* ]] || fail "ipc host is not part of this deploy"
  [[ "$sample" != *lidar-center* && "$sample" != *lidar-dual* && "$sample" != *ptp-master* ]] || fail "rendered script names a prohibited container"
  [[ "$sample" != *"docker compose"* && "$sample" != *"docker system prune"* ]] || fail "rendered script prunes the docker host or uses compose"
  [[ "$sample" == *"docker rmi "* ]] || fail "rendered script does not remove old room-memory tags"
  [[ "$sample" == *'docker images "labcommand/room-memory-cpp"'* ]] || fail "rendered script removes tags outside this image"
  [[ "$sample" == *"BACKOFF=0"* && "$sample" == *'sleep "$BACKOFF"'* ]] || fail "rendered script does not wait out the backoff"
  grep -q 'ROOM_MEMORY_BACKOFF_S:-5' "$script" || fail "the default backoff is not 5 seconds"
  grep -q 'ROOM_MEMORY_READY_S:-120' "$script" && grep -q 'ROOM_MEMORY_STABLE_S:-30' "$script" \
    || fail "the default readiness and stability windows are not 120 s and 30 s"
  [[ "$sample" == *"READY_S=3"* && "$sample" == *"STABLE_S=2"* && "$sample" == *"POLL_S=0"* ]] || fail "the windows are not baked in"
  local scratch
  for scratch in reference-mid360.cdr.tmp reference-jt128.cdr.tmp registration-mid360.bin.tmp \
    registration-jt128.bin.tmp rig_state.tmp map_epoch.tmp publisher_generation.tmp; do
    [[ "$sample" == *"-name ${scratch}"* ]] || fail "rendered script does not clear ${scratch}"
  done
  [[ "$sample" == *'-mindepth 2'* ]] || fail "rendered script can remove a room directory"
  [[ "$sample" != *"-name '*.tmp'"* ]] || fail "rendered script still removes every .tmp name"
  [[ "$sample" != *'find "$HOME/room-memory"'* ]] || fail "scratch cleanup still runs as the ssh user"
  [[ "$sample" == *'--user 0'* && "$sample" == *'--entrypoint bash'* && "$sample" == *'find /root/room-memory'* ]] || fail "scratch cleanup does not run as root in the container"
  [[ "$sample" == *'--volumes-from "$orig_id"'* ]] || fail "scratch cleanup does not use the original's own mounts"
  [[ "$sample" == *'.Config.Cmd'* && "$sample" == *'docker create --name room-memory-next'* ]] || fail "rendered script does not copy the running container"
  local before_stop=${sample%%'docker stop "$orig_id"'*}
  local after_stop=${sample#*'docker stop "$orig_id"'}
  [[ "$before_stop" != "$sample" ]] || fail "rendered script does not stop the original by id"
  [[ "$before_stop" == *'{{json .HostConfig}}'* && "$before_stop" == *'discard_next 1'* ]] || fail "the replacement is not checked before stop"
  [[ "$before_stop" == *'{{json .Config.Volumes}}'* && "$before_stop" == *'$MOUNTS'* && "$before_stop" == *'$NETWORKS'* ]] \
    || fail "the volumes, mounts and networks are not checked before stop"
  [[ "$after_stop" == *'trap rollback_service ERR'* ]] || fail "rollback is not armed after stop"
  [[ "$after_stop" == *'docker rename "$orig_id" room-memory-previous'* ]] || fail "the original is not kept for rollback"
  [[ "$after_stop" == *'find /root/room-memory'* && "$after_stop" == *'rollback_service 1'* ]] || fail "a failure after stop is outside the rollback"
  [[ "$sample" == *'docker rm -f "$next_id" || failed=1'* ]] || fail "rollback ignores a failed rm"
  [[ "$sample" == *'docker rename "$orig_id" room-memory || failed=1'* ]] || fail "rollback ignores a failed rename"
  [[ "$sample" == *'docker start "$orig_id" || failed=1'* ]] || fail "rollback ignores a failed start"
  [[ "$sample" == *"docker inspect -f '{{.State.Running}}' room-memory"* ]] || fail "rollback does not confirm room-memory is running"
  [[ "$sample" == *'ROLLBACK FAILED: room-memory is not running'* ]] || fail "a failed rollback does not say room-memory is not running"
  [[ "$sample" == *'ROLLBACK FAILED: room-memory is running, but an earlier rollback step failed'* ]] \
    || fail "a rollback that left room-memory running still says it is not"
  [[ "$sample" == *'recovery="docker start $orig_id"'* ]] \
    || fail "a failed rollback does not start the original by id once it has its name back"
  [[ "$sample" == *'recovery="docker rm -f $next_id; docker rename $orig_id room-memory; docker start $orig_id"'* ]] \
    || fail "a failed rename back does not recover the original by id"
  [[ "$sample" == *'recovery="docker rm -f room-memory"'* ]] \
    || fail "a fresh host whose removal fails does not say to remove that container"
  [[ "$sample" != *'echo "docker rm -f room-memory && docker rename room-memory-previous room-memory && docker start room-memory"'* ]] \
    || fail "a failed rollback still prints one recovery command for every state"
  [[ "$sample" == *'echo "rolled back"'* ]] || fail "a successful rollback does not say so"
  [[ "$sample" == *'could not list image tags to prune'* ]] || fail "a failed image list can fail the deploy"
  local after_start=${after_stop#*'docker start "$next_id"'}
  [[ "$after_start" == *'wait_ready "$next_id"'* && "$after_start" == *"trap - ERR"* ]] || fail "the rollback is disarmed before readiness"
  [[ "${after_start%%trap - ERR*}" == *'rollback_service 1'* ]] || fail "readiness is outside the rollback"
  local stability=${sample#*'for ((n = 0; n < STABLE_S; n++))'}
  stability=${stability%%done*}
  [[ "$stability" == *'process_failure'* && "$sample" == *"']: process has died'"* ]] \
    || fail "the stability window does not watch the launched processes"
  [[ "$sample" != *'docker rm "$orig_id"'* && "$sample" != *'docker rm -f "$orig_id"'* ]] || fail "rendered script removes the original container"
  [[ "$sample" != *"rm -v"* && "$sample" != *"rm -fv"* && "$sample" != *"rm -f -v"* ]] || fail "rendered script removes a container's volumes"
  reject_forbidden "$sample"

  local banned code
  for banned in \
    "docker stop lidar-center" \
    "docker rm lidar-dual" \
    "docker run --name ptp-master" \
    "docker build -t labcommand/lidar-center:abc" \
    "labcommand/lidar-dual:abc" \
    "ptp-master"; do
    set +e
    (reject_forbidden "$banned" >/dev/null 2>&1)
    code=$?
    set -e
    [[ "$code" -eq 2 ]] || fail "guard returned ${code} for: ${banned}"
  done

  repo=$work/repo
  mkdir -p "$repo"
  git -C "$repo" init -q
  git -C "$repo" config user.email "room-memory-deploy@example.com"
  git -C "$repo" config user.name "room-memory-deploy"
  echo one >"$repo/file"
  git -C "$repo" add file
  git -C "$repo" -c commit.gpgsign=false commit -q -m first
  local first first_short
  first=$(git -C "$repo" rev-parse HEAD)
  first_short=$(git -C "$repo" rev-parse --short=7 HEAD)
  echo two >>"$repo/file"
  git -C "$repo" add file
  git -C "$repo" -c commit.gpgsign=false commit -q -m second
  local head_short
  head_short=$(git -C "$repo" rev-parse --short=7 HEAD)
  [[ "$first_short" != "$head_short" ]] || fail "the two commits share a short hash"

  local out expected_dry
  out=$(ROOM_MEMORY_REPO="$repo" "$script" --dry-run --sha "$first" --host "$host")
  expected_dry=$(printf 'git archive --format=tar.gz %s | ssh -- %s bash -c %s\n' \
    "$first" "$host" "$(shell_quote "$(render_remote "$first_short")")")
  if [[ "$out" != "$expected_dry" ]]; then
    printf 'dry-run mismatch\n--- got ---\n%s\n--- expected ---\n%s\n' "$out" "$expected_dry" >&2
    fail "dry-run is not the quoted archive pipeline"
  fi
  [[ "$out" == *"labcommand/room-memory-cpp:${first_short}"* ]] || fail "image tag is not --short=7 of the --sha commit"
  [[ "$out" != *"labcommand/room-memory-cpp:${head_short}"* ]] || fail "image tag followed HEAD instead of --sha"
  [[ "$out" == "git archive --format=tar.gz ${first} | ssh -- ${host} bash -c '"* ]] || fail "remote script is not one quoted bash -c argument"

  echo dirty >>"$repo/file"
  cat >"$bin/ssh" <<EOS
#!/bin/bash
printf '%s\n' "\$*" >>"$ssh_log"
echo invoked >>"$work/marker"
exit 99
EOS
  chmod +x "$bin/ssh"
  set +e
  PATH="$bin:$PATH" ROOM_MEMORY_REPO="$repo" "$script" --dry-run --host "$host" >/dev/null 2>&1
  code=$?
  set -e
  [[ "$code" -ne 0 ]] || fail "a dirty worktree was archived"
  [[ ! -e "$work/marker" ]] || fail "dirty --dry-run invoked ssh"

  # OpenSSH sends one string: the command arguments joined by a single space.
  # The remote login shell then evaluates that string. Running only the last
  # argument would hide `bash -c` losing every word after the first.
  cat >"$bin/ssh" <<'EOS'
#!/bin/bash
# A leading -- ends options, matching OpenSSH. The next word is the host.
if [[ "${1:-}" == "--" ]]; then
  shift
fi
if [[ $# -lt 1 ]]; then
  echo "ssh: missing host" >&2
  exit 2
fi
shift
joined=""
for arg in "$@"; do
  if [[ -n "$joined" ]]; then
    joined="${joined} ${arg}"
  else
    joined=$arg
  fi
done
printf '%s\n' "$joined" >>"${SSH_LOG:?}"
exec bash -c "$joined"
EOS
  # A docker with state: containers, images and their mounts live in DOCKER_STATE, named and
  # anonymous volumes under DOCKER_VOLUMES, and `inspect -f` evaluates Go templates (fields, json,
  # len, println, if, range). Any other form or flag fails, so the deploy cannot depend on something
  # this stub does not model. DOCKER_MODE fails one step, or tells the new image's container how it
  # comes up (health-*), or changes the new image (image-*).
  cat >"$bin/docker" <<'EOS'
#!/usr/bin/env python3
import json, os, re, subprocess, sys

args = sys.argv[1:]
with open(os.environ["DOCKER_LOG"], "a") as log:
    log.write(" ".join(args) + "\n")
mode = os.environ.get("DOCKER_MODE", "ok")
expected = os.environ.get("EXPECTED_TAG", "")
path = os.environ["DOCKER_STATE"]
volumes_dir = os.environ.get("DOCKER_VOLUMES", "")
try:
    with open(path) as f:
        state = json.load(f)
except FileNotFoundError:
    state = {"containers": [], "images": {}, "serial": 0}


def save():
    with open(path, "w") as f:
        json.dump(state, f)


def die(message, code=1):
    print("docker stub: " + message, file=sys.stderr)
    sys.exit(code)


def container(ref):
    for c in state["containers"]:
        if c["Id"] == ref or c["Name"] == "/" + ref:
            return c
    die("No such container: " + ref)


def image(ref):
    for tag, img in state["images"].items():
        if tag == ref or img["Id"] == ref:
            return img
    die("No such image: " + ref)


def new_image(c):
    return bool(expected) and c["Config"]["Image"].endswith(":" + expected)


# Go text/template, the forms the deploy uses: fields, json, len, println, if, range over a list or
# a map ($k, $v). Anything else fails, so the deploy cannot depend on something this does not model.
def parse(tokens, at, stop):
    nodes = []
    while at < len(tokens):
        token = tokens[at]
        at += 1
        if not token.startswith("{{"):
            nodes.append(("text", token))
            continue
        action = token[2:-2].strip()
        if action == "end":
            if not stop:
                die("unexpected end", 3)
            return nodes, at
        if action.startswith("range ") or action.startswith("if "):
            body, at = parse(tokens, at, True)
            nodes.append((action.split(" ", 1)[0], action.split(" ", 1)[1], body))
        else:
            nodes.append(("action", action))
    if stop:
        die("missing end", 3)
    return nodes, at


def resolve(expr, dot, scope):
    if expr == ".":
        return dot
    m = re.fullmatch(r"(\$\w+)?((?:\.\w+)*)", expr)
    if not m:
        die("cannot evaluate " + expr, 3)
    value = scope[m[1]] if m[1] else dot
    for part in m[2].split(".")[1:]:
        value = None if value is None else value.get(part)
    return value


def text(v):
    if v is None:
        return "<nil>"
    if isinstance(v, bool):
        return "true" if v else "false"
    return str(v)


def truthy(v):
    return v is not None and v != "" and v != [] and v != {} and v is not False


def evaluate(nodes, dot, scope):
    out = []
    for node in nodes:
        if node[0] == "text":
            out.append(node[1])
        elif node[0] == "if":
            if truthy(resolve(node[1], dot, scope)):
                out.append(evaluate(node[2], dot, scope))
        elif node[0] == "range":
            m = re.fullmatch(r"(\$\w+), (\$\w+) := (\S+)", node[1])
            if m:
                items = resolve(m[3], dot, scope) or {}
                for k in sorted(items):
                    out.append(evaluate(node[2], items[k], dict(scope, **{m[1]: k, m[2]: items[k]})))
            else:
                for item in resolve(node[1], dot, scope) or []:
                    out.append(evaluate(node[2], item, scope))
        else:
            action = node[1]
            if action == "println":
                out.append("\n")
            elif action.startswith("println "):
                out.append(text(resolve(action[8:], dot, scope)) + "\n")
            elif action.startswith("json "):
                out.append(json.dumps(resolve(action[5:], dot, scope), separators=(",", ":"), sort_keys=True))
            elif action.startswith("len "):
                out.append(str(len(resolve(action[4:], dot, scope) or [])))
            else:
                out.append(text(resolve(action, dot, scope)))
    return "".join(out)


def render(template, obj):
    nodes, _ = parse(re.split(r"(\{\{.*?\}\})", template), 0, False)
    return evaluate(nodes, obj, {})


def size(v):
    units = {"b": 1, "k": 1024, "m": 1024 ** 2, "g": 1024 ** 3}
    if v[-1].lower() in units:
        return int(v[:-1]) * units[v[-1].lower()]
    return int(v)


def create(opts):
    hc = {"Binds": None, "CapAdd": None, "LogConfig": {"Type": "json-file", "Config": {}}, "Memory": 0,
          "MemorySwap": 0, "NanoCpus": 0, "NetworkMode": "bridge", "PidsLimit": None, "Mounts": None,
          "VolumesFrom": None, "RestartPolicy": {"Name": "no", "MaximumRetryCount": 0}}
    env, name, user, workdir, entrypoint, anonymous = [], None, None, None, None, []
    i = 0
    while i < len(opts) and opts[i].startswith("-"):
        flag = opts[i]
        if flag in ("-d", "--rm"):
            i += 1
            continue
        if i + 1 >= len(opts):
            die("flag needs a value: " + flag, 125)
        v = opts[i + 1]
        i += 2
        if flag == "--name":
            name = v
        elif flag == "--restart":
            policy, _, count = v.partition(":")
            hc["RestartPolicy"] = {"Name": policy, "MaximumRetryCount": int(count or 0)}
        elif flag in ("--net", "--network"):
            hc["NetworkMode"] = v
        elif flag in ("-e", "--env"):
            env.append(v)
        elif flag == "--cpus":
            hc["NanoCpus"] = int(round(float(v) * 1e9))
        elif flag == "--memory":
            hc["Memory"] = size(v)
        elif flag == "--memory-swap":
            hc["MemorySwap"] = size(v)
        elif flag == "--pids-limit":
            hc["PidsLimit"] = int(v)
        elif flag == "--log-driver":
            hc["LogConfig"]["Type"] = v
        elif flag == "--log-opt":
            key, _, val = v.partition("=")
            hc["LogConfig"]["Config"][key] = val
        elif flag in ("-v", "--volume"):
            # opts.go: a lone container path is an anonymous volume (Config.Volumes); src:dst is a bind
            # (absolute src) or a named volume, in HostConfig.Binds.
            if ":" in v:
                hc["Binds"] = (hc["Binds"] or []) + [v]
            else:
                anonymous.append(v)
        elif flag == "--mount":
            fields = dict(item.split("=", 1) for item in v.split(","))
            hc["Mounts"] = (hc["Mounts"] or []) + [{"Type": fields.get("type", "volume"), "Source": fields.get("src", ""),
                                                     "Target": fields.get("dst", fields.get("target", ""))}]
        elif flag == "--volumes-from":
            hc["VolumesFrom"] = (hc["VolumesFrom"] or []) + [v]
        elif flag == "--cap-add":
            hc["CapAdd"] = (hc["CapAdd"] or []) + [v]
        elif flag in ("-u", "--user"):
            user = v
        elif flag in ("-w", "--workdir"):
            workdir = v
        elif flag == "--entrypoint":
            entrypoint = [v] if v else []
        else:
            die("unknown flag: " + flag, 125)
    if i >= len(opts):
        die("no image", 125)
    ref, cmd = opts[i], opts[i + 1:]
    img = image(ref)
    if name and any(c["Name"] == "/" + name for c in state["containers"]):
        die("name in use: " + name, 125)
    merged = list(img["Config"]["Env"])
    for entry in env:
        key = entry.split("=", 1)[0]
        merged = [x for x in merged if x.split("=", 1)[0] != key] + [entry]
    state["serial"] += 1
    serial = state["serial"]
    volumes = dict(img["Config"].get("Volumes") or {})
    volumes.update({dest: {} for dest in anonymous})
    mounts = []
    covered = set()
    for bind in hc["Binds"] or []:
        src, dest, *rest = bind.split(":")
        covered.add(dest)
        if src.startswith("/"):
            mounts.append({"Type": "bind", "Name": "", "Source": src, "Destination": dest, "Mode": rest[0] if rest else "",
                           "RW": True, "Propagation": "rprivate"})
        else:
            mounts.append({"Type": "volume", "Name": src, "Source": "/var/lib/docker/volumes/%s/_data" % src,
                           "Destination": dest, "Driver": "local", "Mode": "z", "RW": True, "Propagation": ""})
    for m in hc["Mounts"] or []:
        covered.add(m["Target"])
        mounts.append({"Type": m["Type"], "Name": "", "Source": m["Source"], "Destination": m["Target"], "Mode": "",
                       "RW": True, "Propagation": "rprivate"})
    for dest in sorted(volumes):
        if dest not in covered:
            anon = "%064x" % (0xA000 + serial)
            mounts.append({"Type": "volume", "Name": anon, "Source": "/var/lib/docker/volumes/%s/_data" % anon,
                           "Destination": dest, "Driver": "local", "Mode": "", "RW": True, "Propagation": ""})
    if entrypoint is not None:
        command = cmd or None
    else:
        command = cmd or img["Config"]["Cmd"]
    c = {"Id": "%064x" % serial, "Name": "/" + (name or "c%d" % serial), "Image": img["Id"],
         "Config": {"Image": ref, "Cmd": command,
                    "Entrypoint": img["Config"]["Entrypoint"] if entrypoint is None else (entrypoint or None),
                    "Env": merged,
                    "User": img["Config"]["User"] if user is None else user,
                    "WorkingDir": img["Config"]["WorkingDir"] if workdir is None else workdir,
                    "Volumes": volumes or None, "Tty": False, "OpenStdin": False, "StdinOnce": False},
         "HostConfig": hc, "Mounts": mounts, "RestartCount": 0, "Logs": "",
         "NetworkSettings": {"Networks": {hc["NetworkMode"]: {"IPAMConfig": None, "Aliases": None}}},
         "State": {"Running": False, "Status": "created", "Health": None}}
    state["containers"].append(c)
    return c


def come_up(c):
    """The container starts. The new image's container follows DOCKER_MODE's health story."""
    c["State"].update(Running=True, Status="running")
    ready = "room-memory ready: publisher_generation 1760400000000\n"
    if not new_image(c) or mode not in ("health-exit", "health-never-ready"):
        c["Logs"] = ready
    if not new_image(c):
        return
    if mode == "health-exit":
        c["State"].update(Running=False, Status="exited")
    elif mode == "health-restart":
        c["RestartCount"] = 2
    elif mode == "health-unhealthy":
        c["State"]["Health"] = {"Status": "unhealthy"}
    elif mode in ("health-late-exit", "health-node-died"):
        c["Countdown"] = 3  # State inspections before it exits or its node dies: after the readiness line


def host_path(c, dest):
    for m in c["Mounts"]:
        if m["Destination"] == dest:
            if m["Type"] == "bind":
                return m["Source"]
            target = os.path.join(volumes_dir, m["Name"])
            os.makedirs(target, exist_ok=True)
            return target
    die("no mount at " + dest)


command, rest = (args[0], args[1:]) if args else ("", [])
if command == "build":
    if mode == "build-fail":
        die("build failed")
    tag = rest[rest.index("-t") + 1]
    config = {"Cmd": ["ros2", "launch", "room_memory_cpp", "room_memory_cpp.launch.py"],
              "Entrypoint": ["/ws/src/room_memory_cpp/docker-entrypoint.sh"],
              "Env": ["PATH=/opt/ros/jazzy/bin:/usr/bin", "ROS_DISTRO=jazzy"], "User": "", "WorkingDir": "/ws",
              "Volumes": None}
    if tag.endswith(":" + expected):
        if mode == "image-env":
            config["Env"].append("NEW_ONLY=1")
        elif mode == "image-volume":
            config["Volumes"] = {"/data": {}}
        elif mode == "image-entrypoint":
            config["Entrypoint"] = ["/ws/new-entrypoint.sh"]
        elif mode == "image-workdir":
            config["WorkingDir"] = "/opt"
    state["images"][tag] = {"Id": "sha256:" + tag.encode().hex()[:64], "Config": config}
    save()
elif command == "ps":
    if mode == "ps-fail":
        die("ps failed")
    for c in state["containers"]:
        print(c["Name"][1:])
elif command == "inspect" and rest[0] == "-f":
    c = container(rest[2])
    if ".State." in rest[1] and "Countdown" in c:
        c["Countdown"] -= 1
        if c["Countdown"] <= 0:
            del c["Countdown"]
            if mode == "health-node-died":
                # A launch without a shutdown on exit: the bridge keeps the container running.
                c["Logs"] += ("[ERROR] [room_memory_node-1]: process has died [pid 42, exit code -11, "
                              "cmd '/ws/install/room_memory_cpp/lib/room_memory_cpp/room_memory_node'].\n")
            else:
                c["State"].update(Running=False, Status="exited")
        save()
    print(render(rest[1], c))
elif command == "image" and rest[:2] == ["inspect", "-f"]:
    print(render(rest[2], image(rest[3])))
elif command == "create":
    if mode == "create-fail":
        die("create failed")
    c = create(rest)
    save()
    print(c["Id"])
elif command == "run":
    # Scratch cleanup is a one-shot `docker run --user 0` of find over the state mount. Root can
    # unlink entries the ssh user cannot; this stub is not root, so it restores write permission and
    # then runs the same find against the host path behind /root/room-memory.
    if "--user" in rest and "find /root/room-memory" in rest[-1]:
        if mode in ("cleanup-perm", "rollback-rm-before-rename"):
            die("cleanup refused")
        if "--volumes-from" in rest:
            source = host_path(container(rest[rest.index("--volumes-from") + 1]), "/root/room-memory")
        else:
            bind = rest[rest.index("-v") + 1]
            source = bind.split(":")[0]
        subprocess.run(["find", source, "-type", "d", "-exec", "chmod", "u+w", "{}", "+"], check=True)
        os.execvp("bash", ["bash", "-c", rest[-1].replace("/root/room-memory", source)])
    c = create(rest)
    save()
    if mode in ("run-fail-fresh", "fresh-rm-fail"):
        die("run failed")
    come_up(c)
    save()
    print(c["Id"])
elif command == "start":
    c = container(rest[0])
    if mode == "rollback-start-fail":
        die("start failed")
    if mode == "start-fail" and new_image(c):
        die("start failed")
    # The replacement has to fail so rollback runs; the original's start must still succeed.
    if mode in ("rollback-rm-fail", "rollback-rename-fail") and new_image(c):
        die("start failed")
    come_up(c)
    save()
elif command == "logs":
    sys.stdout.write(container(rest[0])["Logs"])
elif command == "stop":
    if mode == "stop-fail":
        die("stop failed")
    container(rest[0])["State"].update(Running=False, Status="exited")
    save()
elif command == "rename":
    c, new = container(rest[0]), rest[1]
    if mode == "rename-fail" and new == "room-memory-previous":
        die("rename failed")
    # The forward rename is the new image taking the name. Rollback renames the original back.
    if mode == "rollback-rename-fail" and new == "room-memory" and not new_image(c):
        die("rename failed")
    if any(x["Name"] == "/" + new for x in state["containers"]):
        die("name in use: " + new)
    c["Name"] = "/" + new
    save()
elif command == "rm":
    if any(flag.startswith("-") and "v" in flag for flag in rest[:-1]):
        die("this deploy never removes a container's volumes", 3)
    if mode in ("rollback-rm-fail", "rollback-rm-before-rename", "fresh-rm-fail") and rest and rest[0] == "-f":
        die("rm failed")
    c = container(rest[-1])
    if c["State"]["Running"] and rest[0] != "-f":
        die("container is running")
    state["containers"].remove(c)
    save()
elif command == "images":
    if mode == "images-fail":
        die("images failed")
    for tag in sorted(state["images"]):
        print(tag)
elif command == "rmi":
    img = image(rest[0])
    if any(c["Image"] == img["Id"] for c in state["containers"]):
        die("conflict: " + rest[0] + " is used by a container")
    del state["images"][rest[0]]
    save()
else:
    die("unexpected: " + " ".join(args), 3)
EOS
  chmod +x "$bin/ssh" "$bin/docker"
  [[ "$(PATH="$bin:$PATH"; command -v ssh)" == "$bin/ssh" ]] || fail "the ssh stub does not shadow ssh"
  [[ "$(PATH="$bin:$PATH"; command -v docker)" == "$bin/docker" ]] || fail "the docker stub does not shadow docker"

  # The stub joins like OpenSSH. Three words — bash, -c, and an unquoted script —
  # become `bash -c set -euo pipefail` plus the rest of the script, with no errexit.
  # A stub that executes only the last argument would stop at `false` and fail this.
  local probe_script probe_out probe_code
  probe_script=$(printf '%s\n' 'set -euo pipefail' 'false' 'echo reached-probe')
  : >"$ssh_log"
  set +e
  PATH="$bin:$PATH" SSH_LOG="$ssh_log" "$bin/ssh" "$host" bash -c "$probe_script" >"$work/probe.out" 2>"$work/probe.err"
  probe_code=$?
  set -e
  probe_out=$(cat "$work/probe.out")
  [[ "$probe_code" -eq 0 ]] || fail "unquoted ssh join kept errexit (exit ${probe_code}); the stub is not OpenSSH"
  [[ "$probe_out" == *reached-probe* ]] || fail "unquoted ssh join did not run past a failure"
  [[ "$(head -n 1 "$ssh_log")" == "bash -c set -euo pipefail" ]] || fail "unquoted ssh join was not logged as the remote shell sees it"

  local new_tag=labcommand/room-memory-cpp:$first_short
  docker_stub() {
    PATH="$bin:$PATH" HOME="$work/home" DOCKER_LOG="$docker_log" DOCKER_STATE="$state" DOCKER_VOLUMES="$work/volumes" \
      DOCKER_MODE=ok "$bin/docker" "$@" >/dev/null
  }
  # A Python expression over the stub's state: `named` maps container names to containers.
  q() {
    python3 - "$state" "$1" <<'PY'
import json, sys
state = json.load(open(sys.argv[1]))
named = {c["Name"][1:]: c for c in state["containers"]}
print(eval(sys.argv[2], {"named": named, "state": state, "json": json}))
PY
  }
  # Sets a field of the stub's state: `edit 'named["room-memory"]["Config"]["Entrypoint"]' '["a", "b"]'`.
  edit() {
    python3 - "$state" "$1" "$2" <<'PY'
import json, sys
path, target, value = sys.argv[1:]
state = json.load(open(path))
named = {c["Name"][1:]: c for c in state["containers"]}
images = state["images"]
exec(target + " = json.loads(value)", {"named": named, "images": images, "json": json, "value": value})
json.dump(state, open(path, "w"))
PY
  }
  # The images a host has before the deploy and, given arguments, a room-memory started from them.
  seed_host() {
    rm -f "$state"
    docker_stub build -f Dockerfile -t labcommand/room-memory-cpp:previous .
    docker_stub build -f Dockerfile -t labcommand/room-memory-cpp:oldtag .
    if [[ $# -gt 0 ]]; then
      docker_stub run -d --name room-memory "$@"
    fi
  }
  # The service as the operator runs it: the documented settings, and the rig guard on.
  local state_bind=(-v "$work/home/room-memory:/root/room-memory")
  local limits=(--restart unless-stopped --net host -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 --cpus 4
    --memory 2g --memory-swap 2g --pids-limit 256 --log-opt max-size=10m --log-opt max-file=3)
  local original_settings=("${limits[@]}" "${state_bind[@]}")
  local original_command=(labcommand/room-memory-cpp:previous ros2 launch room_memory_cpp room_memory_cpp.launch.py
    rig_guard:=true)
  local guarded_cmd="['ros2', 'launch', 'room_memory_cpp', 'room_memory_cpp.launch.py', 'rig_guard:=true']"

  run_mode() {
    : >"$ssh_log"
    : >"$docker_log"
    set +e
    PATH="$bin:$PATH" HOME="$work/home" ROOM_MEMORY_REPO="$repo" \
      SSH_LOG="$ssh_log" DOCKER_LOG="$docker_log" DOCKER_STATE="$state" DOCKER_VOLUMES="$work/volumes" \
      DOCKER_MODE="$1" EXPECTED_TAG="$first_short" \
      "$script" --yes --sha "$first" --host "$host" >/dev/null 2>"$work/deploy.err"
    code=$?
    set -e
  }
  has() { grep -q "^$1 " "$docker_log"; }
  lacks() { ! grep -q "^$1 " "$docker_log"; }
  service_runs() { grep '^run ' "$docker_log" | grep -- '--name room-memory' || true; }
  no_volume_removed() { ! grep -Eq '^rm (-[a-zA-Z]+ )*-[a-zA-Z]*v' "$docker_log" || fail "$1: a container's volumes were removed"; }
  # The one recovery command a failed rollback printed. Stub diagnostics also start with "docker ".
  recovery_line() {
    local line
    line=$(grep -E '^docker (start|rm|rename) ' "$work/deploy.err" || true)
    [[ -n "$line" ]] || fail "$1: no recovery command in: $(cat "$work/deploy.err")"
    [[ "$line" != *$'\n'* ]] || fail "$1: more than one recovery command: $line"
    printf '%s\n' "$line"
  }
  # Run that command the way an operator pastes it: no set -e, so a leading rm of an
  # already-removed id does not stop the rename and start. The failure mode was the deploy's.
  # A later manual command has a working docker, so the stub is back in DOCKER_MODE=ok.
  apply_recovery() {
    local failure=$1 line=$2 orig=$3
    PATH="$bin:$PATH" HOME="$work/home" DOCKER_LOG="$docker_log" DOCKER_STATE="$state" \
      DOCKER_VOLUMES="$work/volumes" DOCKER_MODE=ok EXPECTED_TAG="$first_short" \
      bash -c "$line" >"$work/recovery.out" 2>"$work/recovery.err" \
      || fail "${failure}: recovery command failed (${line}): $(cat "$work/recovery.err")"
    [[ "$(q 'named["room-memory"]["Id"]')" == "$orig" ]] \
      || fail "${failure}: recovery left $(q 'sorted(named)') instead of the original"
    [[ "$(q 'named["room-memory"]["State"]["Running"]')" == True ]] \
      || fail "${failure}: recovery did not leave the original running"
    [[ "$(q 'named["room-memory"]["Config"]["Cmd"]')" == "$guarded_cmd" ]] \
      || fail "${failure}: recovery lost rig_guard:=true"
    no_volume_removed "${failure} recovery"
  }
  # After a failure: the original container is room-memory again, running, with its own command.
  # Nothing of the new image is left, and no new container was made from the previous image.
  original_restored() {
    [[ "$(q 'named["room-memory"]["Id"]')" == "$1" ]] || fail "$2: room-memory is not the original container"
    [[ "$(q 'named["room-memory"]["State"]["Running"]')" == True ]] || fail "$2: the original is not running"
    [[ "$(q 'named["room-memory"]["Config"]["Cmd"]')" == "$guarded_cmd" ]] || fail "$2: the original lost rig_guard:=true"
    [[ "$(q 'sorted(named)')" == "['room-memory']" ]] || fail "$2: left $(q 'sorted(named)')"
    [[ "$(q "[c for c in state['containers'] if c['Config']['Image'] == '$new_tag']")" == "[]" ]] || fail "$2: a container of the new image is left"
    no_volume_removed "$2"
  }
  # A refusal before stop: room-memory is the same running container, and nothing of the new image is left.
  refused_before_stop() {
    [[ "$code" -ne 0 ]] || fail "$2 still deployed"
    lacks stop && lacks rename && lacks start || fail "$2 still stopped the original"
    grep -q -- "$3" "$work/deploy.err" || fail "$2 did not say why: $(cat "$work/deploy.err")"
    [[ "$(q 'named["room-memory"]["Id"]')" == "$1" ]] || fail "$2 replaced room-memory"
    [[ "$(q 'named["room-memory"]["State"]["Running"]')" == True ]] || fail "$2 left the original stopped"
    [[ "$(q "[c for c in state['containers'] if c['Config']['Image'] == '$new_tag']")" == "[]" ]] || fail "$2 left a replacement"
    no_volume_removed "$2"
  }
  # The replacement runs the new image with everything else of the original.
  replaced() {
    [[ "$code" -eq 0 ]] || fail "$2 exited ${code}: $(cat "$work/deploy.err")"
    [[ "$(q 'named["room-memory"]["Id"]')" != "$1" ]] || fail "$2: room-memory is still the original"
    [[ "$(q 'named["room-memory"]["Config"]["Image"]')" == "$new_tag" ]] || fail "$2: room-memory does not run the new image"
    [[ "$(q 'named["room-memory"]["State"]["Running"]')" == True ]] || fail "$2: the replacement is not running"
    [[ "$(q 'named["room-memory-previous"]["Id"]')" == "$1" ]] || fail "$2: the original was not kept as room-memory-previous"
    [[ "$(q 'named["room-memory-previous"]["State"]["Running"]')" == False ]] || fail "$2: the original was left running"
    local field
    for field in '["Config"]["Cmd"]' '["Config"]["Entrypoint"]' '["Config"]["User"]' '["Config"]["WorkingDir"]' \
      '["Config"]["Volumes"]' '["HostConfig"]' '["Mounts"]' '["NetworkSettings"]'; do
      [[ "$(q "named['room-memory']$field == named['room-memory-previous']$field")" == True ]] \
        || fail "$2: the replacement's $field is not the original's"
    done
    [[ "$(q 'sorted(named["room-memory"]["Config"]["Env"]) == sorted(named["room-memory-previous"]["Config"]["Env"])')" == True ]] \
      || fail "$2: the replacement's environment is not the original's"
    no_volume_removed "$2"
  }

  seed_host "${original_settings[@]}" "${original_command[@]}"
  run_mode build-fail
  [[ "$code" -ne 0 ]] || fail "build failure still exited 0"
  has build || fail "build failure never built"
  lacks create && lacks stop && lacks rename && lacks run || fail "build failure still replaced the container"
  [[ "$(head -n 1 "$ssh_log")" == "bash -c 'set -euo pipefail" ]] || fail "deploy ssh command is not one quoted bash -c script"

  run_mode ps-fail
  [[ "$code" -ne 0 ]] || fail "ps failure still exited 0"
  has build && has ps || fail "ps failure skipped listing"
  lacks create && lacks stop && lacks rename && lacks run || fail "ps failure still replaced the container"

  local orig
  orig=$(q 'named["room-memory"]["Id"]')
  run_mode stop-fail
  [[ "$code" -ne 0 ]] || fail "stop failure still exited 0"
  has create && has stop || fail "stop failure did not stop"
  lacks rename && lacks start || fail "stop failure still renamed or started"
  original_restored "$orig" "stop failure"

  # lab.tmp is a valid room id. map_epoch.tmp is too, when it is the room directory.
  # Scratch names inside a room are leftovers, including a directory at that path.
  room=$work/home/room-memory/lab.tmp
  named=$work/home/room-memory/map_epoch.tmp
  mkdir -p "$room/map_epoch.tmp" "$room/scratch.tmp" "$named"
  echo keep-ref >"$room/reference-mid360.cdr"
  echo keep-epoch >"$room/map_epoch"
  echo keep-gen >"$room/publisher_generation"
  echo inside >"$room/scratch.tmp/inside"
  echo blocked >"$room/map_epoch.tmp/blocked"
  echo junk >"$room/reference-mid360.cdr.tmp"
  echo junk >"$room/reference-jt128.cdr.tmp"
  echo junk >"$room/registration-mid360.bin.tmp"
  echo junk >"$room/rig_state.tmp"
  echo junk >"$room/publisher_generation.tmp"
  echo room-ref >"$named/reference-mid360.cdr"
  echo room-epoch >"$named/map_epoch"
  echo stray >"$work/home/room-memory/reference-jt128.cdr.tmp"
  echo keep >"$work/home/room-memory/notes.tmp"
  seed_host
  docker_stub create --name room-memory-next labcommand/room-memory-cpp:oldtag
  run_mode ok
  [[ "$code" -eq 0 ]] || fail "empty ps list failed: $(cat "$work/deploy.err")"
  [[ "$(q 'sorted(named)')" == "['room-memory']" ]] || fail "a fresh host kept a stopped room-memory-next"
  has build && has ps && has run || fail "a fresh host did not run"
  lacks stop && lacks create || fail "a fresh host stopped a container that was not there"
  has logs || fail "a fresh host did not wait for the readiness line"
  grep -q "^rm room-memory-next$" "$docker_log" && ! grep -q "^rm .*room-memory$" "$docker_log" || fail "a fresh host removed the wrong container"
  has rmi || fail "a successful run left old image tags"
  grep -q -- "--name room-memory" "$docker_log" || fail "run missed the container name"
  grep -q -- "--restart unless-stopped" "$docker_log" || fail "run missed the restart policy"
  grep -q -- "$new_tag" "$docker_log" || fail "run used the wrong tag"
  grep -q "rmi labcommand/room-memory-cpp:oldtag" "$docker_log" || fail "old room-memory tag was kept"
  if grep -q "rmi $new_tag" "$docker_log"; then
    fail "the tag just deployed was removed"
  fi
  [[ "$(q 'named["room-memory"]["State"]["Running"]')" == True ]] || fail "a fresh host's room-memory is not running"
  local leftover
  for leftover in reference-mid360.cdr.tmp reference-jt128.cdr.tmp registration-mid360.bin.tmp rig_state.tmp \
    publisher_generation.tmp map_epoch.tmp; do
    [[ ! -e "$room/$leftover" ]] || fail "${leftover} was left inside the room"
  done
  [[ "$(cat "$room/reference-mid360.cdr")" == "keep-ref" ]] || fail "cleanup removed the room reference"
  [[ "$(cat "$room/map_epoch")" == "keep-epoch" ]] || fail "cleanup removed the room epoch"
  [[ "$(cat "$room/publisher_generation")" == "keep-gen" ]] || fail "cleanup removed the generation"
  [[ "$(cat "$room/scratch.tmp/inside")" == "inside" ]] || fail "cleanup removed a directory that is not a scratch name"
  [[ "$(cat "$named/reference-mid360.cdr")" == "room-ref" ]] || fail "cleanup removed a room named map_epoch.tmp"
  [[ "$(cat "$named/map_epoch")" == "room-epoch" ]] || fail "cleanup removed the epoch of a room named map_epoch.tmp"
  [[ ! -e "$work/home/room-memory/reference-jt128.cdr.tmp" ]] || fail "a stray scratch file was left in the state dir"
  [[ "$(cat "$work/home/room-memory/notes.tmp")" == "keep" ]] || fail "cleanup removed a file that is not a scratch name"

  # Codex r3: room-memory runs with rig_guard:=true. The replacement keeps that argument and every
  # runtime setting, on the new image, and the original is kept stopped for a rollback. A
  # room-memory-previous left stopped by the last deploy goes first. The ssh user cannot unlink a
  # scratch file in a room directory it does not own: root cleanup still removes it.
  locked=$work/home/room-memory/locked
  mkdir -p "$locked"
  echo keep-ref >"$locked/reference-mid360.cdr"
  echo junk >"$locked/reference-mid360.cdr.tmp"
  chmod 555 "$locked"
  seed_host "${original_settings[@]}" "${original_command[@]}"
  docker_stub create --name room-memory-previous labcommand/room-memory-cpp:oldtag
  orig=$(q 'named["room-memory"]["Id"]')
  run_mode replace
  chmod u+w "$locked" || true
  replaced "$orig" "replacing a guarded room-memory"
  has create && has stop && has rename && has start && has logs || fail "replacing a container did not create, stop, rename, start and wait"
  [[ "$(service_runs)" == "" ]] || fail "replacing a container ran a default room-memory"
  [[ "$(q 'named["room-memory"]["Config"]["Cmd"]')" == "$guarded_cmd" ]] || fail "the replacement lost rig_guard:=true"
  [[ "$(q '"FASTDDS_BUILTIN_TRANSPORTS=UDPv4" in named["room-memory"]["Config"]["Env"]')" == True ]] || fail "the replacement lost the transport"
  [[ "$(q 'sorted(named)')" == "['room-memory', 'room-memory-previous']" ]] || fail "replace left $(q 'sorted(named)')"
  grep -q "rmi labcommand/room-memory-cpp:oldtag" "$docker_log" || fail "old room-memory tag was kept"
  if grep -q "rmi labcommand/room-memory-cpp:previous" "$docker_log"; then
    fail "the tag room-memory-previous runs was removed"
  fi
  [[ ! -e "$locked/reference-mid360.cdr.tmp" ]] || fail "root cleanup left a scratch file the ssh user could not remove"
  [[ "$(cat "$locked/reference-mid360.cdr")" == "keep-ref" ]] || fail "root cleanup removed the room reference"

  # The new container is up. `docker images` then fails. Pruning is best-effort: the deploy exits 0
  # and leaves the old tags in place.
  seed_host "${original_settings[@]}" "${original_command[@]}"
  orig=$(q 'named["room-memory"]["Id"]')
  run_mode images-fail
  replaced "$orig" "a deploy whose image list failed"
  has images || fail "a failed image list never listed images"
  lacks rmi || fail "a failed image list still removed tags"
  grep -q "could not list image tags to prune" "$work/deploy.err" || fail "a failed image list did not say it left them: $(cat "$work/deploy.err")"
  [[ "$(q '"labcommand/room-memory-cpp:oldtag" in state["images"]')" == True ]] || fail "a failed image list removed an old tag"

  # Every failure after stop, and every way the replacement fails to come up (it exits, restarts,
  # turns unhealthy, never logs its readiness line, or exits or loses its node within the stability
  # window while Docker still reports it running), starts
  # the same original container: renamed back if it was renamed, with its own command, settings and
  # volumes. Not a new container of the previous image. That rollback succeeded: it says so, exits
  # with the deploy's own failure (not 3), and docker inspect saw room-memory running.
  local failure
  for failure in start-fail rename-fail cleanup-perm health-exit health-restart health-unhealthy health-never-ready \
    health-late-exit health-node-died; do
    echo junk >"$locked/reference-mid360.cdr.tmp"
    chmod 555 "$locked"
    seed_host "${original_settings[@]}" "${original_command[@]}"
    orig=$(q 'named["room-memory"]["Id"]')
    run_mode "$failure"
    chmod u+w "$locked" || true
    [[ "$code" -ne 0 ]] || fail "${failure} still exited 0"
    [[ "$code" -ne 3 ]] || fail "${failure} reported a failed rollback: $(cat "$work/deploy.err")"
    grep -qx "rolled back" "$work/deploy.err" || fail "${failure} did not say it rolled back: $(cat "$work/deploy.err")"
    if grep -q "ROLLBACK FAILED" "$work/deploy.err"; then
      fail "${failure} reported a failed rollback"
    fi
    grep -q "^inspect -f {{.State.Running}} room-memory$" "$docker_log" \
      || fail "${failure} did not confirm room-memory is running"
    has stop || fail "${failure} did not stop the original"
    lacks rmi || fail "${failure} still pruned image tags"
    grep -q "^start ${orig}$" "$docker_log" || fail "${failure} did not start the original container"
    original_restored "$orig" "$failure"
    if [[ "$failure" == cleanup-perm ]]; then
      [[ "$(cat "$locked/reference-mid360.cdr.tmp")" == "junk" ]] || fail "a failed cleanup removed the scratch file"
      [[ "$(cat "$locked/reference-mid360.cdr")" == "keep-ref" ]] || fail "a failed cleanup removed the room reference"
    fi
    if [[ "$failure" == health-* ]]; then
      # An exit, a restart or an unhealthy status is seen before the readiness line is looked for.
      [[ "$failure" != health-never-ready && "$failure" != health-late-exit && "$failure" != health-node-died ]] \
        || has logs || fail "${failure} never looked for the readiness line"
      grep -q "the replacement" "$work/deploy.err" || fail "${failure} did not say why"
    fi
    if [[ "$failure" == health-node-died ]]; then
      grep -q "the replacement lost a process (.*room_memory_node-1.*process has died.*) within 2 s of ready" \
        "$work/deploy.err" || fail "a node that died after readiness was not caught in the stability window"
    fi
  done

  # The rollback's own rm, rename, or start failed. The deploy exits 3 and prints a recovery
  # command for the containers that are left. It does not claim it rolled back. Running that
  # command against the stub leaves the original as room-memory, running, with its own command.
  # Start-fail has already renamed the original back, so the command is `docker start` of its id.
  # `docker rm -f room-memory` there deletes the only remaining copy. Rename-fail has already
  # removed the replacement, so the command's rm of that id fails and the `;` must still rename
  # and start. Rm-fail leaves the replacement holding the name: the command removes it by id.
  local recovery removed
  for failure in rollback-start-fail rollback-rm-fail rollback-rename-fail; do
    seed_host "${original_settings[@]}" "${original_command[@]}"
    orig=$(q 'named["room-memory"]["Id"]')
    run_mode "$failure"
    [[ "$code" -eq 3 ]] || fail "${failure} exited ${code}, not 3: $(cat "$work/deploy.err")"
    grep -qx "ROLLBACK FAILED: room-memory is not running" "$work/deploy.err" \
      || fail "${failure} missing the failure line: $(cat "$work/deploy.err")"
    if grep -q "rolled back" "$work/deploy.err"; then
      fail "${failure} claimed it rolled back"
    fi
    grep -q "^inspect -f {{.State.Running}} room-memory$" "$docker_log" \
      || fail "${failure} did not inspect room-memory"
    if [[ "$(q '"room-memory" in named')" == True ]]; then
      [[ "$(q 'named["room-memory"]["State"]["Running"]')" == False ]] || fail "${failure} left room-memory running"
    fi
    [[ "$(q "any(c['Id'] == '$orig' for c in state['containers'])")" == True ]] \
      || fail "${failure} removed the original container"
    no_volume_removed "$failure"
    recovery=$(recovery_line "$failure")
    [[ "$recovery" != *"docker rm -f room-memory"* && "$recovery" != *"docker rm room-memory"* ]] \
      || fail "${failure}: recovery removes room-memory by name: $recovery"
    case "$failure" in
      rollback-start-fail)
        [[ "$recovery" == "docker start $orig" ]] \
          || fail "${failure}: recovery is not docker start of the original: $recovery"
        ;;
      rollback-rm-fail|rollback-rename-fail)
        removed=${recovery#docker rm -f }
        removed=${removed%%"; docker rename ${orig} room-memory; docker start ${orig}"}
        [[ "$recovery" == "docker rm -f ${removed}; docker rename ${orig} room-memory; docker start ${orig}" ]] \
          || fail "${failure}: recovery is not the id sequence: $recovery"
        [[ "$removed" != "$orig" && ${#removed} -eq 64 && "$removed" != *[!0-9a-f]* ]] \
          || fail "${failure}: recovery does not remove the replacement by id ($removed): $recovery"
        if [[ "$failure" == rollback-rm-fail ]]; then
          [[ "$removed" == "$(q "[c['Id'] for c in state['containers'] if c['Id'] != '$orig'][0]")" ]] \
            || fail "${failure}: recovery does not name the replacement that is still there: $recovery"
        fi
        ;;
    esac
    apply_recovery "$failure" "$recovery" "$orig"
    original_restored "$orig" "${failure} recovery"
  done

  # Rm fails before the replacement takes the name (scratch cleanup failed, then the rollback's rm).
  # The original is renamed back and is running. `docker rm -f room-memory` would delete it.
  seed_host "${original_settings[@]}" "${original_command[@]}"
  orig=$(q 'named["room-memory"]["Id"]')
  run_mode rollback-rm-before-rename
  [[ "$code" -eq 3 ]] || fail "rm before the rename back exited ${code}: $(cat "$work/deploy.err")"
  grep -qx "ROLLBACK FAILED: room-memory is running, but an earlier rollback step failed" "$work/deploy.err" \
    || fail "rm before the rename back missing the failure line: $(cat "$work/deploy.err")"
  if grep -qx "ROLLBACK FAILED: room-memory is not running" "$work/deploy.err"; then
    fail "rm before the rename back said room-memory is not running: $(cat "$work/deploy.err")"
  fi
  if grep -q "rolled back" "$work/deploy.err"; then
    fail "rm before the rename back claimed it rolled back"
  fi
  [[ "$(q 'named["room-memory"]["Id"]')" == "$orig" ]] || fail "rm before the rename back did not put the original back"
  [[ "$(q 'named["room-memory"]["State"]["Running"]')" == True ]] \
    || fail "rm before the rename back left the original stopped"
  [[ "$(q 'named["room-memory"]["Config"]["Cmd"]')" == "$guarded_cmd" ]] \
    || fail "rm before the rename back lost rig_guard:=true"
  [[ "$(q '"room-memory-next" in named')" == True ]] || fail "rm before the rename back removed the replacement"
  recovery=$(recovery_line "rm before the rename back")
  [[ "$recovery" == "docker start $orig" ]] \
    || fail "rm before the rename back recovery is not docker start of the original: $recovery"
  apply_recovery "rm before the rename back" "$recovery" "$orig"
  [[ "$(q '"room-memory-next" in named')" == True ]] \
    || fail "rm before the rename back recovery removed the replacement"
  no_volume_removed "rm before the rename back"

  # A fresh host has no original and no room-memory-previous. A failed removal must not tell the
  # operator to rename a container this host never had. The command removes the failed one.
  seed_host
  run_mode fresh-rm-fail
  [[ "$code" -eq 3 ]] || fail "a fresh host whose removal failed exited ${code}: $(cat "$work/deploy.err")"
  grep -qx "ROLLBACK FAILED: room-memory is not running" "$work/deploy.err" \
    || fail "a fresh host whose removal failed missing the failure line: $(cat "$work/deploy.err")"
  if grep -q "rolled back" "$work/deploy.err"; then
    fail "a fresh host whose removal failed claimed it rolled back"
  fi
  [[ "$(q 'sorted(named)')" == "['room-memory']" ]] \
    || fail "a fresh host whose removal failed left $(q 'sorted(named)')"
  recovery=$(recovery_line "a fresh host whose removal failed")
  [[ "$recovery" == "docker rm -f room-memory" ]] \
    || fail "fresh recovery is not removal of the failed container: $recovery"
  [[ "$recovery" != *room-memory-previous* ]] || fail "fresh recovery names room-memory-previous: $recovery"
  PATH="$bin:$PATH" HOME="$work/home" DOCKER_LOG="$docker_log" DOCKER_STATE="$state" \
    DOCKER_VOLUMES="$work/volumes" DOCKER_MODE=ok EXPECTED_TAG="$first_short" \
    bash -c "$recovery" >"$work/recovery.out" 2>"$work/recovery.err" \
    || fail "fresh recovery command failed (${recovery}): $(cat "$work/recovery.err")"
  [[ "$(q 'sorted(named)')" == "[]" ]] || fail "fresh recovery left $(q 'sorted(named)')"
  no_volume_removed "fresh recovery"

  # A fresh host whose first container never comes up removes only that container.
  seed_host
  run_mode health-exit
  [[ "$code" -ne 0 ]] || fail "a fresh host that exited still deployed"
  grep -q '^rm -f room-memory$' "$docker_log" || fail "the fresh container that exited was not removed"
  [[ "$(q 'sorted(named)')" == "[]" ]] || fail "a failed fresh start left $(q 'sorted(named)')"

  # A setting the deploy does not copy (here a capability) stops it before stop, and the last
  # deploy's room-memory-previous is still there for a manual rollback.
  seed_host "${original_settings[@]}" --cap-add SYS_NICE "${original_command[@]}"
  docker_stub create --name room-memory-previous labcommand/room-memory-cpp:oldtag
  orig=$(q 'named["room-memory"]["Id"]')
  local kept
  kept=$(q 'named["room-memory-previous"]["Id"]')
  run_mode ok
  refused_before_stop "$orig" "a replacement without the original's capability" "does not match room-memory (settings)"
  [[ "$(q 'named["room-memory-previous"]["Id"]')" == "$kept" ]] || fail "a refused deploy removed room-memory-previous"

  # Codex r5: rig_guard:=true is the old image's own CMD, and room-memory was started without a
  # command. The replacement still runs it on the new image, whose default does not have it.
  seed_host
  edit 'images["labcommand/room-memory-cpp:previous"]["Config"]["Cmd"]' \
    '["ros2", "launch", "room_memory_cpp", "room_memory_cpp.launch.py", "rig_guard:=true"]'
  docker_stub run -d --name room-memory "${original_settings[@]}" labcommand/room-memory-cpp:previous
  [[ "$(q 'named["room-memory"]["Config"]["Cmd"]')" == "$guarded_cmd" ]] || fail "the image's CMD was not seeded"
  orig=$(q 'named["room-memory"]["Id"]')
  run_mode replace
  replaced "$orig" "replacing room-memory on its image's CMD"
  [[ "$(q 'named["room-memory"]["Config"]["Cmd"]')" == "$guarded_cmd" ]] || fail "the replacement lost rig_guard:=true from the old image's CMD"

  # The old image's environment and entrypoint are the container's: the replacement keeps both, on a
  # new image with another entrypoint (a single element is --entrypoint).
  seed_host
  edit 'images["labcommand/room-memory-cpp:previous"]["Config"]["Env"]' \
    '["PATH=/opt/ros/jazzy/bin:/usr/bin", "ROS_DISTRO=jazzy", "ROOM_MEMORY_OLD=1"]'
  docker_stub run -d --name room-memory "${original_settings[@]}" "${original_command[@]}"
  orig=$(q 'named["room-memory"]["Id"]')
  run_mode image-entrypoint
  replaced "$orig" "an inherited environment and entrypoint"
  [[ "$(q '"ROOM_MEMORY_OLD=1" in named["room-memory"]["Config"]["Env"]')" == True ]] || fail "the old image's variable was dropped"
  [[ "$(q 'named["room-memory"]["Config"]["Entrypoint"]')" == "['/ws/src/room_memory_cpp/docker-entrypoint.sh']" ]] \
    || fail "the replacement took the new image's entrypoint"
  grep -q -- "--entrypoint /ws/src/room_memory_cpp/docker-entrypoint.sh" "$docker_log" || fail "the entrypoint was not given"

  # An entrypoint override, a user and a working directory of its own, another restart policy: all kept.
  seed_host "${limits[@]/unless-stopped/always}" "${state_bind[@]}" --entrypoint /bin/sh -u 1000 -w /tmp \
    "${original_command[@]}"
  orig=$(q 'named["room-memory"]["Id"]')
  run_mode image-workdir
  replaced "$orig" "an entrypoint, user and working directory of its own"
  [[ "$(q 'named["room-memory"]["HostConfig"]["RestartPolicy"]["Name"]')" == always ]] || fail "the restart policy was not kept"

  # A named volume holds the state: the replacement mounts the same volume, and cleanup reaches it.
  mkdir -p "$work/volumes/roomstate/lab"
  echo junk >"$work/volumes/roomstate/lab/rig_state.tmp"
  echo keep >"$work/volumes/roomstate/lab/rig_state"
  seed_host "${limits[@]}" -v roomstate:/root/room-memory "${original_command[@]}"
  orig=$(q 'named["room-memory"]["Id"]')
  run_mode replace
  replaced "$orig" "a named state volume"
  [[ "$(q '[m["Name"] for m in named["room-memory"]["Mounts"]]')" == "['roomstate']" ]] || fail "the replacement lost the named volume"
  [[ ! -e "$work/volumes/roomstate/lab/rig_state.tmp" && "$(cat "$work/volumes/roomstate/lab/rig_state")" == keep ]] \
    || fail "cleanup did not reach the named volume, or removed a room file"

  # Refused before anything stops: what a replacement cannot reproduce exactly.
  seed_host "${limits[@]}" -v /root/room-memory "${original_command[@]}"
  orig=$(q 'named["room-memory"]["Id"]')
  run_mode ok
  refused_before_stop "$orig" "an anonymous state volume" "anonymous volume"
  lacks create || fail "an anonymous state volume still created a replacement"

  seed_host "${original_settings[@]}" -v /root/scratch "${original_command[@]}"
  orig=$(q 'named["room-memory"]["Id"]')
  run_mode ok
  refused_before_stop "$orig" "an anonymous volume beside the state" "anonymous volume"

  seed_host "${original_settings[@]}" "${original_command[@]}"
  orig=$(q 'named["room-memory"]["Id"]')
  run_mode image-volume
  refused_before_stop "$orig" "a volume only the new image declares" "does not match room-memory (settings)"

  seed_host "${original_settings[@]}" "${original_command[@]}"
  orig=$(q 'named["room-memory"]["Id"]')
  run_mode image-env
  refused_before_stop "$orig" "a variable only the new image has" "does not match room-memory (environment)"

  seed_host "${limits[@]}" "${original_command[@]}"
  orig=$(q 'named["room-memory"]["Id"]')
  run_mode ok
  refused_before_stop "$orig" "a room-memory without a state mount" "no mount at /root/room-memory"
  lacks create || fail "a room-memory without a state mount still created a replacement"

  seed_host "${limits[@]}" --mount "type=bind,src=$work/home/room-memory,dst=/root/room-memory" "${original_command[@]}"
  orig=$(q 'named["room-memory"]["Id"]')
  run_mode ok
  refused_before_stop "$orig" "a --mount state" "--mount"

  seed_host "${original_settings[@]}" "${original_command[@]}"
  edit 'named["room-memory"]["Config"]["Entrypoint"]' '["/bin/sh", "-c"]'
  orig=$(q 'named["room-memory"]["Id"]')
  run_mode ok
  refused_before_stop "$orig" "a two-element entrypoint" "entrypoint"
  lacks create || fail "a two-element entrypoint still created a replacement"

  seed_host "${original_settings[@]}" "${original_command[@]}"
  edit 'named["room-memory"]["NetworkSettings"]["Networks"]["lab-net"]' '{"IPAMConfig": null, "Aliases": ["room"]}'
  orig=$(q 'named["room-memory"]["Id"]')
  run_mode ok
  refused_before_stop "$orig" "another network attachment" "does not match room-memory (networks)"

  # No command at all (an image without a CMD): the replacement would take the new image's
  # default. The command is compared, so that stops the deploy before stop.
  seed_host
  edit 'images["labcommand/room-memory-cpp:previous"]["Config"]["Cmd"]' null
  docker_stub run -d --name room-memory "${original_settings[@]}" labcommand/room-memory-cpp:previous
  orig=$(q 'named["room-memory"]["Id"]')
  run_mode ok
  refused_before_stop "$orig" "a replacement with another command" "does not match room-memory (settings)"
  [[ "$(q 'sorted(named)')" == "['room-memory']" ]] || fail "a refused replacement was left: $(q 'sorted(named)')"

  # A running room-memory-next is not this deploy's to remove.
  seed_host "${original_settings[@]}" "${original_command[@]}"
  docker_stub run -d --name room-memory-next labcommand/room-memory-cpp:oldtag
  run_mode ok
  [[ "$code" -ne 0 ]] || fail "a running room-memory-next still deployed"
  lacks create && lacks stop && lacks rm || fail "a running room-memory-next was replaced or removed"
  grep -q "room-memory-next is running" "$work/deploy.err" || fail "a running room-memory-next did not say why"

  # room-memory missing and room-memory-previous kept: a fresh run would drop its command. Refuse.
  seed_host
  docker_stub create --name room-memory-previous "${original_settings[@]}" "${original_command[@]}"
  run_mode ok
  [[ "$code" -ne 0 ]] || fail "a kept room-memory-previous without room-memory still deployed"
  [[ "$(service_runs)" == "" ]] && lacks create && lacks rm || fail "a kept room-memory-previous was replaced"
  grep -q "room-memory-previous is kept" "$work/deploy.err" || fail "a kept room-memory-previous did not say why"

  seed_host
  run_mode run-fail-fresh
  [[ "$code" -ne 0 ]] || fail "a failed first run still exited 0"
  lacks stop || fail "a fresh host stopped a container that was not there"
  grep -q '^rm -f room-memory$' "$docker_log" || fail "the failed container was not removed"
  [[ "$(service_runs | wc -l | tr -d ' ')" -eq 1 ]] || fail "a fresh host ran more than the failed container"
  [[ "$(service_runs)" == *"$new_tag" ]] || fail "a fresh host ran the wrong image"
  [[ "$(service_runs)" != *previous* ]] || fail "a fresh host rolled back to an image it did not have"
  [[ "$(q 'sorted(named)')" == "[]" ]] || fail "a failed fresh run left $(q 'sorted(named)')"

  : >"$ssh_log"
  : >"$docker_log"
  set +e
  PATH="$bin:$PATH" HOME="$work/home" ROOM_MEMORY_REPO="$repo" \
    SSH_LOG="$ssh_log" DOCKER_LOG="$docker_log" DOCKER_STATE="$state" DOCKER_MODE=ok \
    "$script" --sha "$first" --host "$host" </dev/null >"$work/notty.out" 2>"$work/notty.err"
  code=$?
  set -e
  [[ "$code" -eq 2 ]] || fail "a non-tty deploy without --yes exited ${code}"
  [[ ! -s "$ssh_log" && ! -s "$docker_log" ]] || fail "a non-tty deploy without --yes invoked ssh or docker"
  grep -q "without --yes" "$work/notty.err" || fail "a non-tty deploy did not say why it stopped"

  confirm_with() {
    : >"$ssh_log"
    : >"$docker_log"
    python3 - "$script" "$first" "$host" "$1" <<'PY'
import os, pty, select, subprocess, sys, time
script, sha, host, answer = sys.argv[1:]
master, slave = pty.openpty()
proc = subprocess.Popen(
    [script, "--sha", sha, "--host", host],
    stdin=slave, stdout=slave, stderr=slave, env=os.environ.copy(),
)
os.close(slave)
buf = b""
deadline = time.time() + 8
while time.time() < deadline and proc.poll() is None and b"Type room-memory" not in buf:
    ready, _, _ = select.select([master], [], [], 0.2)
    if ready:
        try:
            buf += os.read(master, 4096)
        except OSError:
            break
if b"Type room-memory" not in buf:
    sys.stderr.buffer.write(buf)
    print("confirm: prompt was not shown", file=sys.stderr)
    proc.kill()
    sys.exit(3)
os.write(master, answer.encode() + b"\n")
try:
    code = proc.wait(timeout=20)
except subprocess.TimeoutExpired:
    proc.kill()
    print("confirm: timed out", file=sys.stderr)
    sys.exit(3)
sys.exit(code)
PY
  }

  seed_host
  set +e
  PATH="$bin:$PATH" HOME="$work/home" ROOM_MEMORY_REPO="$repo" \
    SSH_LOG="$ssh_log" DOCKER_LOG="$docker_log" DOCKER_STATE="$state" DOCKER_VOLUMES="$work/volumes" DOCKER_MODE=ok \
    EXPECTED_TAG="$first_short" confirm_with "nope"
  code=$?
  set -e
  [[ "$code" -eq 2 ]] || fail "a wrong confirmation exited ${code}"
  [[ ! -s "$ssh_log" ]] || fail "a wrong confirmation invoked ssh"

  set +e
  PATH="$bin:$PATH" HOME="$work/home" ROOM_MEMORY_REPO="$repo" \
    SSH_LOG="$ssh_log" DOCKER_LOG="$docker_log" DOCKER_STATE="$state" DOCKER_VOLUMES="$work/volumes" DOCKER_MODE=ok \
    EXPECTED_TAG="$first_short" confirm_with "room-memory"
  code=$?
  set -e
  [[ "$code" -eq 0 ]] || fail "typing room-memory exited ${code}"
  [[ -s "$ssh_log" ]] || fail "typing room-memory did not deploy"
  grep -q '^run ' "$docker_log" || fail "typing room-memory did not run the container"

  : >"$ssh_log"
  : >"$docker_log"
  PATH="$bin:$PATH" HOME="$work/home" ROOM_MEMORY_REPO="$repo" \
    SSH_LOG="$ssh_log" DOCKER_LOG="$docker_log" DOCKER_STATE="$state" DOCKER_MODE=ok \
    "$script" --dry-run --sha "$first" --host "$host" >/dev/null
  [[ ! -s "$ssh_log" && ! -s "$docker_log" ]] || fail "dry-run invoked ssh or docker"
  echo "self-test ok"
}

if [[ "$SELF" -eq 1 ]]; then
  self_test
  exit 0
fi

if [[ -n ${ROOM_MEMORY_REPO:-} ]]; then
  cd "$ROOM_MEMORY_REPO"
else
  cd "$(git -C "$(dirname "$0")" rev-parse --show-toplevel)"
fi

if [[ -n "$SHA" ]]; then
  REV=$(git rev-parse --verify "${SHA}^{commit}")
else
  if [[ -n "$(git status --porcelain)" ]]; then
    echo "refusing to archive a dirty worktree" >&2
    exit 1
  fi
  REV=$(git rev-parse --verify HEAD)
fi
SHORT=$(git rev-parse --short=7 "$REV")
remote=$(render_remote "$SHORT")
reject_forbidden "$remote"
# Quote for the remote shell. ssh joins `bash -c` and this word with a space;
# the quotes put the whole script in bash -c's command string, where errexit
# stops a failed image build before stop/rm/run.
quoted=$(shell_quote "$remote")
reject_forbidden "$quoted"

if [[ "$DRY" -eq 1 ]]; then
  printf 'git archive --format=tar.gz %s | ssh -- %s bash -c %s\n' "$REV" "$HOST" "$quoted"
  exit 0
fi

if [[ "$YES" -ne 1 ]]; then
  if [[ ! -t 0 ]]; then
    echo "refusing to deploy without --yes when stdin is not a terminal" >&2
    exit 2
  fi
  echo "This replaces the room-memory container on ${HOST}." >&2
  printf 'Type room-memory to continue: ' >&2
  IFS= read -r answer || true
  if [[ "$answer" != "room-memory" ]]; then
    echo "not confirmed" >&2
    exit 2
  fi
fi

git archive --format=tar.gz "$REV" | ssh -- "$HOST" bash -c "$quoted"
