#!/bin/bash
# Capture one Live Room window from the build-output app.
# Usage: scripts/live-room-shots.sh live|map|status|stale|lost|restart|legacy|unavailable
# Quits that process by PID. Does not install, and does not touch ~/Applications.
set -euo pipefail

MODE="${1:-}"
case "$MODE" in
  live|map|status|stale|lost|restart|legacy|unavailable) ;;
  *) echo "usage: $0 live|map|status|stale|lost|restart|legacy|unavailable" >&2; exit 2 ;;
esac

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BIN="${ROOT}/build/LabCommand.app/Contents/MacOS/LabCommand"
if [[ ! -x "$BIN" ]]; then
  echo "missing $BIN — run ./build.sh --no-install first" >&2
  exit 1
fi

OUT="${ROOT}/build/screenshots"
mkdir -p "$OUT" "${HOME}/Developer/lc-tasks/liveroom"
NAME="live-room-${MODE}"
if [[ "$MODE" == "unavailable" ]]; then
  NAME="live-room-unavailable"
fi
PNG="${OUT}/${NAME}.png"
SIDECAR="${OUT}/${NAME}.txt"
rm -f "$PNG" "$SIDECAR"

export LABCOMMAND_LIVE_ROOM="$MODE"
export LABCOMMAND_ROOM_SIDECAR="$SIDECAR"
"$BIN" >/tmp/lc-liveroom-shot-${MODE}.log 2>&1 &
PID=$!
echo "pid ${PID}"

cleanup() {
  if kill -0 "$PID" 2>/dev/null; then
    kill "$PID" 2>/dev/null || true
    for _ in 1 2 3 4 5 6 7 8 9 10; do
      kill -0 "$PID" 2>/dev/null || return 0
      sleep 0.2
    done
    kill -9 "$PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT

# Live waits up to 30s for the socket. Fixtures should already be on screen.
LIMIT=8
if [[ "$MODE" == "live" ]]; then
  LIMIT=30
fi
START=$SECONDS
WID=""
ready() {
  [[ -f "$SIDECAR" ]] || return 1
  if [[ "$MODE" == "live" ]]; then
    grep -q 'connection .connected' "$SIDECAR" || return 1
    grep -q 'url ws://100.118.187.99:8767' "$SIDECAR" || return 1
  fi
  if [[ "$MODE" == "unavailable" ]]; then
    grep -q 'Room memory unavailable' "$SIDECAR" || return 1
  else
    grep -q '^canvas ' "$SIDECAR" || return 1
  fi
  return 0
}

while (( SECONDS - START < LIMIT )); do
  if ! kill -0 "$PID" 2>/dev/null; then
    echo "app exited before the shot" >&2
    command cat /tmp/lc-liveroom-shot-${MODE}.log >&2 || true
    exit 1
  fi
  if ready; then
    line="$(/tmp/lc-windowid "$PID" | head -1 || true)"
    WID="${line%% *}"
    if [[ -n "$WID" && "$WID" != "$line" || "$WID" =~ ^[0-9]+$ ]]; then
      # A numeric window id is enough. Give Metal a frame to draw the fit.
      if [[ "$MODE" != "stale" ]]; then
        sleep 0.4
      fi
      break
    fi
  fi
  sleep 0.25
done

if ! ready; then
  echo "sidecar was not ready after ${LIMIT}s" >&2
  command cat "$SIDECAR" >&2 || true
  command cat /tmp/lc-liveroom-shot-${MODE}.log >&2 || true
  exit 1
fi

line="$(/tmp/lc-windowid "$PID" | head -1 || true)"
WID="${line%% *}"
if [[ ! "$WID" =~ ^[0-9]+$ ]]; then
  echo "no layer-0 window for pid ${PID}" >&2
  /tmp/lc-windowid "$PID" >&2 || true
  exit 1
fi

screencapture -l "$WID" -o -x "$PNG"
if [[ "$MODE" == "live" ]]; then
  cp -f "$PNG" "${HOME}/Developer/lc-tasks/liveroom/live-room.png"
fi
echo "captured ${PNG} window ${WID}"
command cat "$SIDECAR"
