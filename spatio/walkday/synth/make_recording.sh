#!/usr/bin/env bash
# Generate a synthetic marked-loop recording (bag/ + SYNTHETIC.json + its own
# FAST-LIO config) for dry-running the walk-day pipeline end to end.
#
#   spatio/walkday/synth/make_recording.sh <out-dir> <end-offset-m> [prepare.py options]
#
# <out-dir> must not exist. Keep synthetic recordings out of
# ~/Developer/spatio-data/recordings (that tree holds real copies only).
# The bag is written inside the spatio/replay image (rclpy + livox msgs), with
# no network and no host mounts; the result is docker-cp'd back.
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
DCTX="${DOCKER_CONTEXT:-colima}"
[[ $# -ge 2 ]] || { sed -n '2,11p' "$0" | sed 's/^# \{0,1\}//'; exit 2; }
OUT=$1 OFFSET=$2; shift 2
case $OUT in "$HOME/Developer/spatio-data/recordings"/*) echo "refusing: $OUT is under recordings/" >&2; exit 2 ;; esac
[[ -e $OUT ]] && { echo "exists: $OUT" >&2; exit 2; }

python3 "$HERE/prepare.py" "$OUT" --end-offset-m "$OFFSET" "$@"
OUT="$(cd "$OUT" && pwd)"
read -r LT IT LF IF <<<"$(python3 -c 'import json,sys; m=json.load(open(sys.argv[1])); print(m["topics"]["lidar"], m["topics"]["imu"], m["frames"]["lidar"], m["frames"]["imu"])' "$OUT/SYNTHETIC.json")"
LINES=$(sed -n 's/^ *scan_line: *\([0-9]*\).*/\1/p' "$OUT/fastlio_synthetic.yaml")

SRC_REV="$(git -C "${SPATIO_SRC:-$HOME/Developer/spatio-data/spark-src}/FAST_LIO_ROS2" rev-parse --short=7 HEAD)"
IMAGE="spatio/replay:$SRC_REV"
NAME="spatio-synth-$(basename "$OUT" | tr -c 'A-Za-z0-9_.-\n' '_')"
dk() { docker --context "$DCTX" "$@"; }
dk create --name "$NAME" --network none --cpus 4 --memory 6g --cap-drop ALL \
  --security-opt no-new-privileges "$IMAGE" bash -c \
  "mkdir -p /tmp/rec && source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && \
   python3 /opt/synth/write_bag.py /tmp/rec/bag --params /opt/synth/synth_params.json \
     --lidar-topic '$LT' --imu-topic '$IT' --lidar-frame '$LF' --imu-frame '$IF' --scan-lines '$LINES'" >/dev/null
trap 'dk rm -f "$NAME" >/dev/null 2>&1 || true' EXIT
STAGE="$(mktemp -d)"; cp "$HERE/scene.py" "$HERE/write_bag.py" "$OUT/synth_params.json" "$STAGE/"
chmod 755 "$STAGE"; chmod 644 "$STAGE"/*   # cap-drop ALL: root in the container cannot bypass file modes
dk cp "$STAGE/." "$NAME:/opt/synth/"
rm -rf "$STAGE"
dk start -a "$NAME"
dk cp "$NAME:/tmp/rec/bag" "$OUT/bag"
echo "synthetic recording: $OUT"
ls -la "$OUT" "$OUT/bag"
