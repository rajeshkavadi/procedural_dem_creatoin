#!/usr/bin/env bash
# Start the snapshot service in the background, optionally replaying a bag.
#
# Usage:
#   ./start_all.sh                          # service only (live ROS 1 topic)
#   ./start_all.sh /path/to/recording.bag   # service + direct bag replay
#
# Env overrides:
#   PCSNAP_TOPIC        PointCloud2 topic (default /mid360/livox/lidar)
#   PCSNAP_PORT         HTTP port (default 8085)
#   PCSNAP_CLIP_RANGE_M radial clip in meters (default 60)
#   PCSNAP_BAG_RATE     bag speed factor (default 1.0)
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJ_DIR="$(dirname "$SCRIPT_DIR")"
RUN_DIR="$PROJ_DIR/.run"
mkdir -p "$RUN_DIR"

# Clean up any previous run first so repeated starts can't stack orphaned
# processes that hold port 8085 and answer health checks with stale data.
"$SCRIPT_DIR/stop_all.sh" >/dev/null 2>&1 || true

BAG="${1:-}"
TOPIC="${PCSNAP_TOPIC:-/mid360/livox/lidar}"
PORT="${PCSNAP_PORT:-8085}"
CLIP="${PCSNAP_CLIP_RANGE_M:-60}"
RATE="${PCSNAP_BAG_RATE:-1.0}"

ARGS=(--topic "$TOPIC" --port "$PORT" --clip-range "$CLIP")
if [ -n "$BAG" ]; then
  ARGS+=(--bag "$BAG" --bag-rate "$RATE")
  echo "Starting snapshot service (bag=$BAG, topic=$TOPIC, port=$PORT, clip=${CLIP}m)..."
else
  echo "Starting snapshot service (live, topic=$TOPIC, port=$PORT, clip=${CLIP}m)..."
fi

"$SCRIPT_DIR/run_service.sh" "${ARGS[@]}" \
  > "$RUN_DIR/service.log" 2>&1 &
echo $! > "$RUN_DIR/service.pid"
echo "  pid $(cat "$RUN_DIR/service.pid"), log $RUN_DIR/service.log"

sleep 2
echo
echo "Health check:"
curl -s "http://127.0.0.1:$PORT/api/health" | python3 -m json.tool || \
  echo "service not up yet; check $RUN_DIR/service.log"
echo
echo "Open http://127.0.0.1:$PORT/ in a browser. Stop with scripts/stop_all.sh"
