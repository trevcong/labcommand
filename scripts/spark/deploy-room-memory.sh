#!/usr/bin/env bash
# Build and replace only the room-memory container on the Spark.
# Reads like ~/Developer/lc-tasks/room-memory-deploy.md. Does not stop lidar-center,
# lidar-dual, or ptp-master, and does not delete ~/room-memory.
# --dry-run prints the archive pipeline and the remote script. --self-test checks the
# guard and failure order with local stand-ins. Neither talks to the Spark.
# OpenSSH joins its command arguments and the remote login shell parses that
# string. The script is one single-quoted word after `bash -c`, so `set -euo pipefail`
# applies to the whole remote script. An unquoted `bash -c "$remote"` does not.
set -euo pipefail

HOST="spark"
SHA=""
DRY=0
SELF=0

usage() {
  echo "usage: deploy-room-memory.sh [--host HOST] [--sha COMMIT] [--dry-run] [--self-test]" >&2
  exit 2
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
  local template
  template=$(cat <<'END'
set -euo pipefail
D=$(mktemp -d /tmp/rmbuild.XXXX)
trap 'rm -rf "$D"' EXIT
tar -xzf - -C "$D"
cd "$D"
docker build -f spark/room_memory_cpp/Dockerfile -t labcommand/room-memory-cpp:__SHORT__ .
rm -rf "$D"
trap - EXIT
names=$(docker ps -a --format '{{.Names}}')
found=0
while IFS= read -r name; do
  if [ "$name" = "room-memory" ]; then
    found=1
  fi
done <<ENDNAMES
$names
ENDNAMES
if [ "$found" -eq 1 ]; then
  docker stop room-memory
  docker rm room-memory
fi
mkdir -p "$HOME/room-memory"
docker run -d --name room-memory --restart unless-stopped --net host \
  -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 --cpus 4 --memory 2g --memory-swap 2g --pids-limit 256 \
  --log-opt max-size=10m --log-opt max-file=3 \
  -v "$HOME/room-memory:/root/room-memory" \
  labcommand/room-memory-cpp:__SHORT__
END
)
  printf '%s\n' "${template//__SHORT__/$short}"
}

self_test() {
  local script work repo bin ssh_log docker_log
  script=$(cd "$(dirname "$0")" && pwd)/$(basename "$0")
  work=$(mktemp -d "${TMPDIR:-/tmp}/room-memory-deploy-test.XXXXXX")
  # Expand the path now. `work` is local, and the EXIT trap runs after that scope is gone.
  trap "rm -rf $(printf '%q' "$work")" EXIT
  mkdir -p "$work/home" "$work/bin"
  bin=$work/bin
  ssh_log=$work/ssh.log
  docker_log=$work/docker.log
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
  [[ "$sample" != *"docker rmi"* && "$sample" != *"docker compose"* && "$sample" != *"docker system prune"* ]] || fail "rendered script prunes or removes images"
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
  out=$(ROOM_MEMORY_REPO="$repo" "$script" --dry-run --sha "$first" --host spark)
  expected_dry=$(printf 'git archive --format=tar.gz %s | ssh spark bash -c %s\n' \
    "$first" "$(shell_quote "$(render_remote "$first_short")")")
  if [[ "$out" != "$expected_dry" ]]; then
    printf 'dry-run mismatch\n--- got ---\n%s\n--- expected ---\n%s\n' "$out" "$expected_dry" >&2
    fail "dry-run is not the quoted archive pipeline"
  fi
  [[ "$out" == *"labcommand/room-memory-cpp:${first_short}"* ]] || fail "image tag is not --short=7 of the --sha commit"
  [[ "$out" != *"labcommand/room-memory-cpp:${head_short}"* ]] || fail "image tag followed HEAD instead of --sha"
  [[ "$out" == "git archive --format=tar.gz ${first} | ssh spark bash -c '"* ]] || fail "remote script is not one quoted bash -c argument"

  echo dirty >>"$repo/file"
  cat >"$bin/ssh" <<EOS
#!/bin/bash
printf '%s\n' "\$*" >>"$ssh_log"
echo invoked >>"$work/marker"
exit 99
EOS
  chmod +x "$bin/ssh"
  set +e
  PATH="$bin:$PATH" ROOM_MEMORY_REPO="$repo" "$script" --dry-run --host spark >/dev/null 2>&1
  code=$?
  set -e
  [[ "$code" -ne 0 ]] || fail "a dirty worktree was archived"
  [[ ! -e "$work/marker" ]] || fail "dirty --dry-run invoked ssh"

  # OpenSSH sends one string: the command arguments joined by a single space.
  # The remote login shell then evaluates that string. Running only the last
  # argument would hide `bash -c` losing every word after the first.
  cat >"$bin/ssh" <<'EOS'
#!/bin/bash
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
  cat >"$bin/docker" <<'EOS'
#!/bin/bash
printf '%s\n' "$*" >>"${DOCKER_LOG:?}"
cmd=$1
case "${DOCKER_MODE:-ok}" in
  build-fail)
    [[ "$cmd" == "build" ]] && exit 1
    ;;
  ps-fail)
    [[ "$cmd" == "ps" ]] && exit 1
    ;;
  stop-fail)
    if [[ "$cmd" == "ps" ]]; then
      printf '%s\n' "room-memory"
      exit 0
    fi
    [[ "$cmd" == "stop" ]] && exit 1
    ;;
  rm-fail)
    if [[ "$cmd" == "ps" ]]; then
      printf '%s\n' "room-memory"
      exit 0
    fi
    [[ "$cmd" == "stop" ]] && exit 0
    [[ "$cmd" == "rm" ]] && exit 1
    ;;
  ok)
    exit 0
    ;;
esac
exit 0
EOS
  chmod +x "$bin/ssh" "$bin/docker"

  # The stub joins like OpenSSH. Three words — bash, -c, and an unquoted script —
  # become `bash -c set -euo pipefail` plus the rest of the script, with no errexit.
  # A stub that executes only the last argument would stop at `false` and fail this.
  local probe_script probe_out probe_code
  probe_script=$(printf '%s\n' 'set -euo pipefail' 'false' 'echo reached-probe')
  : >"$ssh_log"
  set +e
  PATH="$bin:$PATH" SSH_LOG="$ssh_log" "$bin/ssh" spark bash -c "$probe_script" >"$work/probe.out" 2>"$work/probe.err"
  probe_code=$?
  set -e
  probe_out=$(cat "$work/probe.out")
  [[ "$probe_code" -eq 0 ]] || fail "unquoted ssh join kept errexit (exit ${probe_code}); the stub is not OpenSSH"
  [[ "$probe_out" == *reached-probe* ]] || fail "unquoted ssh join did not run past a failure"
  [[ "$(head -n 1 "$ssh_log")" == "bash -c set -euo pipefail" ]] || fail "unquoted ssh join was not logged as the remote shell sees it"

  run_mode() {
    : >"$ssh_log"
    : >"$docker_log"
    set +e
    PATH="$bin:$PATH" HOME="$work/home" ROOM_MEMORY_REPO="$repo" \
      SSH_LOG="$ssh_log" DOCKER_LOG="$docker_log" DOCKER_MODE="$1" \
      "$script" --sha "$first" --host spark >/dev/null
    code=$?
    set -e
  }
  has() { grep -q "^$1 " "$docker_log"; }
  lacks() { ! grep -q "^$1 " "$docker_log"; }

  run_mode build-fail
  [[ "$code" -ne 0 ]] || fail "build failure still exited 0"
  has build || fail "build failure never built"
  lacks stop && lacks rm && lacks run || fail "build failure still replaced the container"
  [[ "$(head -n 1 "$ssh_log")" == "bash -c 'set -euo pipefail" ]] || fail "deploy ssh command is not one quoted bash -c script"

  run_mode ps-fail
  [[ "$code" -ne 0 ]] || fail "ps failure still exited 0"
  has build && has ps || fail "ps failure skipped listing"
  lacks stop && lacks rm && lacks run || fail "ps failure still replaced the container"

  run_mode stop-fail
  [[ "$code" -ne 0 ]] || fail "stop failure still exited 0"
  has build && has ps && has stop || fail "stop failure did not stop"
  lacks rm && lacks run || fail "stop failure still removed or ran"

  run_mode rm-fail
  [[ "$code" -ne 0 ]] || fail "rm failure still exited 0"
  has build && has stop && has rm || fail "rm failure did not remove"
  lacks run || fail "rm failure still ran"

  run_mode ok
  [[ "$code" -eq 0 ]] || fail "empty ps list failed"
  has build && has ps && has run || fail "a fresh host did not run"
  lacks stop && lacks rm || fail "a fresh host stopped a container that was not there"
  grep -q -- "--name room-memory" "$docker_log" || fail "run missed the container name"
  grep -q -- "--restart unless-stopped" "$docker_log" || fail "run missed the restart policy"
  grep -q -- "labcommand/room-memory-cpp:${first_short}" "$docker_log" || fail "run used the wrong tag"

  : >"$ssh_log"
  : >"$docker_log"
  PATH="$bin:$PATH" HOME="$work/home" ROOM_MEMORY_REPO="$repo" \
    SSH_LOG="$ssh_log" DOCKER_LOG="$docker_log" DOCKER_MODE=ok \
    "$script" --dry-run --sha "$first" --host spark >/dev/null
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
  printf 'git archive --format=tar.gz %s | ssh %s bash -c %s\n' "$REV" "$HOST" "$quoted"
  exit 0
fi

git archive --format=tar.gz "$REV" | ssh "$HOST" bash -c "$quoted"
