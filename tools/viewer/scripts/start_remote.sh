#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJ_DIR="$(dirname "$SCRIPT_DIR")"
RUN_DIR="$PROJ_DIR/.run"
mkdir -p "$RUN_DIR"

BAG="${1:?usage: start_remote.sh /path/to/recording.bag [topic]}"
TOPIC="${2:-/livox/lidar}"
PORT="${PCSNAP_PORT:-8085}"
CLIP="${PCSNAP_CLIP_RANGE_M:-60}"
RATE="${PCSNAP_BAG_RATE:-1.0}"
TOKEN="${PCSNAP_API_TOKEN:-lidar-snap-2026}"

[ ! -e "$BAG" ] && { echo "ERROR: bag path not found: $BAG" >&2; exit 1; }

"$SCRIPT_DIR/stop_all.sh" >/dev/null 2>&1 || true

echo "[1/2] Starting snapshot service + bag replay (topic=$TOPIC, port=$PORT, clip=${CLIP}m)..."
PCSNAP_API_TOKEN="$TOKEN" "$SCRIPT_DIR/run_service.sh" \
  --bag "$BAG" --bag-rate "$RATE" --topic "$TOPIC" --port "$PORT" \
  --clip-range "$CLIP" \
  > "$RUN_DIR/service.log" 2>&1 &
echo $! > "$RUN_DIR/service.pid"

echo "[2/2] Starting Cloudflare tunnel..."
CF="$(command -v cloudflared 2>/dev/null || echo "$HOME/.local/bin/cloudflared" || echo "$PROJ_DIR/cloudflared" || echo "/home/rajesh/cloudflared")"
if [ ! -x "$CF" ]; then
  echo "ERROR: cloudflared not found. Install it or put binary in PATH." >&2
  exit 1
fi
nohup "$CF" tunnel --url "http://localhost:$PORT" > "$RUN_DIR/tunnel.log" 2>&1 &
echo $! > "$RUN_DIR/tunnel.pid"

URL=""
for _ in $(seq 1 40); do
  URL="$(grep -oE 'https://[a-z0-9-]+\.trycloudflare\.com' "$RUN_DIR/tunnel.log" 2>/dev/null | head -1 || true)"
  [ -n "$URL" ] && break
  sleep 1
done

sleep 3

echo
if [ -n "$URL" ]; then
  echo "=========================================================="
  echo "  OPEN IN BROWSER (remote OK):"
  echo "  $URL"
  echo
  echo "  API token (paste in the UI's token box): $TOKEN"
  echo "  Local-only URL: http://127.0.0.1:$PORT/"
  echo "=========================================================="
else
  echo "Tunnel may still be starting. Check: $RUN_DIR/tunnel.log"
fi

echo "
Health check:"
curl -s "http://127.0.0.1:$PORT/" | head -c 60 || echo "service not up yet; check $RUN_DIR/service.log"
echo "
Stop everything with:  $SCRIPT_DIR/stop_all.sh
Logs:  $RUN_DIR/service.log  $RUN_DIR/tunnel.log"
