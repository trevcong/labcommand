#!/usr/bin/env bash
# Build the Spatio replay image from the read-only spark-src copies.
#
#   spatio/replay/build_image.sh            # tag spatio/replay:<fastlio-rev>
#
# Env: SPATIO_SRC (default ~/Developer/spatio-data/spark-src)
#      DOCKER_CONTEXT (default colima)
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
SRC="${SPATIO_SRC:-$HOME/Developer/spatio-data/spark-src}"
DCTX="${DOCKER_CONTEXT:-colima}"
FL="$SRC/FAST_LIO_ROS2"
LV="$SRC/livox_ros_driver2"
MS="$SRC/launchers/map_saver.py"

for p in "$FL/src/laserMapping.cpp" "$FL/include/ikd-Tree/ikd_Tree.cpp" \
         "$LV/msg/CustomMsg.msg" "$LV/msg/CustomPoint.msg" "$MS"; do
  [[ -s "$p" ]] || { echo "missing or empty: $p" >&2; exit 2; }
done

rev()   { git -C "$1" rev-parse HEAD; }
dirty() { [[ -n "$(git -C "$1" status --porcelain)" ]] && echo true || echo false; }
diffsha() { git -C "$1" diff HEAD | shasum -a 256 | cut -d' ' -f1; }

FL_REV="$(rev "$FL")"
CTX="$(mktemp -d "${TMPDIR:-/tmp}/spatio-replay-ctx.XXXXXX")"
trap 'rm -rf "$CTX"' EXIT

mkdir -p "$CTX/src/livox_ros_driver2/msg" "$CTX/tools"
rsync -a --exclude .git --exclude 'PCD/*' --exclude 'Log/*' "$FL/" "$CTX/src/FAST_LIO_ROS2/"
cp "$HERE/livox_msgs_pkg/package.xml" "$HERE/livox_msgs_pkg/CMakeLists.txt" "$CTX/src/livox_ros_driver2/"
cp "$LV/msg/CustomMsg.msg" "$LV/msg/CustomPoint.msg" "$CTX/src/livox_ros_driver2/msg/"
cp "$MS" "$CTX/tools/map_saver.py"
cp "$HERE/Dockerfile" "$CTX/Dockerfile"

sha() { shasum -a 256 "$1" | cut -d' ' -f1; }
cat > "$CTX/provenance.json" <<EOF
{
  "fast_lio_ros2": {"revision": "$FL_REV", "dirty": $(dirty "$FL"),
                    "status": $(git -C "$FL" status --porcelain | python3 -c 'import json,sys; print(json.dumps(sys.stdin.read().splitlines()))'),
                    "diff_sha256": "$(diffsha "$FL")"},
  "livox_ros_driver2": {"revision": "$(rev "$LV")", "dirty": $(dirty "$LV"),
                        "status": $(git -C "$LV" status --porcelain | python3 -c 'import json,sys; print(json.dumps(sys.stdin.read().splitlines()))'),
                        "diff_sha256": "$(diffsha "$LV")",
                        "used": "msg/CustomMsg.msg + msg/CustomPoint.msg only",
                        "custom_msg_sha256": "$(sha "$LV/msg/CustomMsg.msg")",
                        "custom_point_sha256": "$(sha "$LV/msg/CustomPoint.msg")"},
  "map_saver_py_sha256": "$(sha "$MS")",
  "built_at": "$(date -u +%Y-%m-%dT%H:%M:%SZ)"
}
EOF
python3 -m json.tool "$CTX/provenance.json" >/dev/null

TAG="spatio/replay:${FL_REV:0:7}"
docker --context "$DCTX" build --build-arg "FASTLIO_REVISION=$FL_REV" -t "$TAG" "$CTX"
echo "built $TAG ($(docker --context "$DCTX" image inspect -f '{{.Id}}' "$TAG"))"
