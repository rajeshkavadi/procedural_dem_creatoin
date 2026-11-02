#!/usr/bin/env bash
# One-command end-to-end run of the point-cloud viewer behind Cloudflare.
#
#   ./scripts/start_cloudflare.sh                  # repo dataset + tunnel
#   ./scripts/start_cloudflare.sh --no-tunnel      # local only
#   ./scripts/start_cloudflare.sh --bag rec.bag --gnss gps.txt \
#                                 --topic /livox/lidar
#
# What it does, in order:
#   1. checks the bag (and catches an unfetched Git LFS pointer),
#   2. creates/repairs the Python virtualenv when needed,
#   3. ensures the `cloudflared` binary is present (downloads it once),
#   4. starts the snapshot service replaying the bag,
#   5. waits until a point cloud is actually arriving,
#   6. opens a Cloudflare quick tunnel and waits for the public URL,
#   7. proves snapshot + DEM + live stream work, locally and remotely.
#
# Stop everything with ./scripts/stop_all.sh
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJ_DIR="$(dirname "$SCRIPT_DIR")"
ROOT_DIR="$(dirname "$(dirname "$PROJ_DIR")")"
RUN_DIR="$PROJ_DIR/.run"
mkdir -p "$RUN_DIR"

BAG="$ROOT_DIR/20261030_221123_mid360s.bag"
GNSS="$ROOT_DIR/ublox7_raw.txt"
TOPIC="/livox/lidar"
PORT="${PCSNAP_PORT:-8085}"
RATE="${PCSNAP_BAG_RATE:-1.0}"
TOKEN="${PCSNAP_API_TOKEN:-lidar-snap-2026}"
USE_TUNNEL=1
RUN_TEST=1
BOOTSTRAP=1

usage() {
  sed -n '2,20p' "$0" | sed 's/^# \{0,1\}//'
  cat <<'EOF'

Options:
  --bag PATH       ROS 1 .bag to replay        (default: repo recording)
  --gnss PATH      raw NMEA log for the anchor (default: repo ublox7_raw.txt)
  --topic NAME     PointCloud2 topic            (default: /livox/lidar)
  --port N         HTTP port                    (default: 8085)
  --rate N         playback speed               (default: 1.0)
  --token TOKEN    API token for snapshots/DEM  (default: lidar-snap-2026)
  --no-tunnel      do not start cloudflared
  --no-test        skip the snapshot/DEM/stream self-test
  --no-bootstrap   never create a venv or download cloudflared
EOF
}

while [ $# -gt 0 ]; do
  case "$1" in
    --bag)          BAG="$2"; shift 2 ;;
    --gnss)         GNSS="$2"; shift 2 ;;
    --topic)        TOPIC="$2"; shift 2 ;;
    --port)         PORT="$2"; shift 2 ;;
    --rate)         RATE="$2"; shift 2 ;;
    --token)        TOKEN="$2"; shift 2 ;;
    --no-tunnel)    USE_TUNNEL=0; shift ;;
    --no-test)      RUN_TEST=0; shift ;;
    --no-bootstrap) BOOTSTRAP=0; shift ;;
    -h|--help)      usage; exit 0 ;;
    *) echo "unknown option: $1 (try --help)" >&2; exit 2 ;;
  esac
done

# --------------------------------------------------------------- 0. checks
if [ ! -f "$BAG" ]; then
  echo "ERROR: bag not found: $BAG" >&2
  exit 1
fi
if [ "$(stat -c %s "$BAG")" -lt 4096 ] && head -c 40 "$BAG" | grep -q "git-lfs"; then
  echo "ERROR: $BAG is an unfetched Git LFS pointer, not the recording." >&2
  echo "       Run: git lfs pull --include=\"$(basename "$BAG")\"" >&2
  exit 1
fi
if [ ! -f "$GNSS" ]; then
  echo "WARNING: GNSS log not found: $GNSS (continuing without an anchor)" >&2
  GNSS=""
fi

echo "Bag   : $BAG"
echo "GNSS  : ${GNSS:-<none>}"
echo "Topic : $TOPIC"
echo "Port  : $PORT"

# Free the port / kill any previous service before starting.
"$SCRIPT_DIR/stop_all.sh" >/dev/null 2>&1 || true

# ------------------------------------------------------------ 1. env/venv
PY="$PROJ_DIR/.venv/bin/python"
if ! "$PY" -c "import fastapi, uvicorn, rosbags, numpy, matplotlib" >/dev/null 2>&1; then
  if [ "$BOOTSTRAP" = "1" ]; then
    echo "Setting up the Python environment (first run only)…"
    "$SCRIPT_DIR/bootstrap_env.sh" > "$RUN_DIR/bootstrap.log" 2>&1 || {
      echo "ERROR: environment setup failed; see $RUN_DIR/bootstrap.log" >&2
      tail -20 "$RUN_DIR/bootstrap.log" >&2
      exit 1
    }
  else
    echo "ERROR: $PY is missing dependencies and --no-bootstrap was given." >&2
    exit 1
  fi
fi
"$PY" -c "import fastapi, uvicorn, rosbags, numpy, matplotlib" >/dev/null 2>&1 || {
  echo "ERROR: Python environment is still incomplete: $PY" >&2
  exit 1
}

# ---------------------------------------------------- 2. cloudflared binary
CF=""
if [ "$USE_TUNNEL" = "1" ]; then
  if [ -n "${CLOUDFLARED:-}" ]; then
    CF="$CLOUDFLARED"
  elif command -v cloudflared >/dev/null 2>&1; then
    CF="$(command -v cloudflared)"
  elif [ -x "$PROJ_DIR/cloudflared" ]; then
    # The repo ships a symlink here; it only counts when it resolves.
    CF="$PROJ_DIR/cloudflared"
  elif [ -x "$RUN_DIR/cloudflared" ]; then
    CF="$RUN_DIR/cloudflared"
  elif [ -x "$HOME/.local/bin/cloudflared" ]; then
    CF="$HOME/.local/bin/cloudflared"
  fi

  if [ -z "$CF" ] || [ ! -x "$CF" ]; then
    if [ "$BOOTSTRAP" != "1" ]; then
      echo "ERROR: cloudflared not found and --no-bootstrap was given." >&2
      exit 1
    fi
    case "$(uname -m)" in
      x86_64|amd64)  ASSET="cloudflared-linux-amd64" ;;
      aarch64|arm64) ASSET="cloudflared-linux-arm64" ;;
      armv7l|armv6l) ASSET="cloudflared-linux-arm" ;;
      *) echo "ERROR: unsupported architecture $(uname -m) for cloudflared" >&2; exit 1 ;;
    esac
    echo "Downloading cloudflared ($ASSET)…"
    # Keep the download in the gitignored .run/ directory: the tracked
    # tools/viewer/cloudflared is a symlink and must not be overwritten.
    curl -fsSL --retry 3 -o "$RUN_DIR/cloudflared.tmp" \
      "https://github.com/cloudflare/cloudflared/releases/latest/download/$ASSET"
    mv "$RUN_DIR/cloudflared.tmp" "$RUN_DIR/cloudflared"
    chmod +x "$RUN_DIR/cloudflared"
    CF="$RUN_DIR/cloudflared"
  fi
  echo "Tunnel: $("$CF" --version 2>/dev/null | head -1)"
fi

# ------------------------------------------------------------- 3. service
echo
echo "[1/3] Starting the snapshot service…"
ARGS=(--bag "$BAG" --topic "$TOPIC" --port "$PORT" --bag-rate "$RATE")
[ -n "$GNSS" ] && ARGS+=(--gnss "$GNSS")

PCSNAP_API_TOKEN="$TOKEN" nohup "$SCRIPT_DIR/run_service.sh" "${ARGS[@]}" \
  > "$RUN_DIR/service.log" 2>&1 &
SERVICE_PID=$!
echo "$SERVICE_PID" > "$RUN_DIR/service.pid"

health_field() {
  # health_field <dotted.path> [base_url] -> value or empty
  curl -fsS "${2:-http://127.0.0.1:$PORT}/api/health" 2>/dev/null \
    | "$PY" -c "import json,sys; d=json.load(sys.stdin); \
k='$1'.split('.'); v=d
for p in k:
    v = v.get(p) if isinstance(v, dict) else None
print('' if v is None else v)" 2>/dev/null || true
}

READY=0
for _ in $(seq 1 90); do
  if ! kill -0 "$SERVICE_PID" 2>/dev/null; then
    echo "ERROR: the service exited during start-up." >&2
    tail -25 "$RUN_DIR/service.log" >&2
    exit 1
  fi
  connected="$(health_field source.connected)"
  received="$(health_field point_cloud.message_received)"
  if [ "$connected" = "True" ] && [ "$received" = "True" ]; then
    READY=1
    break
  fi
  sleep 1
done
if [ "$READY" != "1" ]; then
  echo "ERROR: no point cloud arrived within 90 s." >&2
  tail -25 "$RUN_DIR/service.log" >&2
  exit 1
fi

SRC_TOPIC="$(health_field point_cloud.topic)"
POINTS="$(health_field point_cloud.latest_point_count)"
echo "      cloud on $SRC_TOPIC: $POINTS points"
GNSS_OK="$(health_field gnss.available)"
if [ "$GNSS_OK" = "True" ]; then
  echo "      GNSS: $(health_field gnss.fixes_valid) fixes, anchor $(health_field gnss.anchor.lat), $(health_field gnss.anchor.lon)"
elif [ -n "$GNSS" ]; then
  echo "      GNSS: unavailable ($(health_field gnss.reason))"
fi

# -------------------------------------------------------------- 4. tunnel
PUBLIC_URL=""
if [ "$USE_TUNNEL" = "1" ]; then
  echo
  echo "[2/3] Opening the Cloudflare quick tunnel…"
  nohup "$CF" tunnel --no-autoupdate --url "http://127.0.0.1:$PORT" \
    > "$RUN_DIR/tunnel.log" 2>&1 &
  echo $! > "$RUN_DIR/tunnel.pid"

  for _ in $(seq 1 75); do
    PUBLIC_URL="$(grep -oE 'https://[a-z0-9-]+\.trycloudflare\.com' \
      "$RUN_DIR/tunnel.log" 2>/dev/null | head -1 || true)"
    [ -n "$PUBLIC_URL" ] && break
    sleep 1
  done
  if [ -z "$PUBLIC_URL" ]; then
    echo "ERROR: cloudflared did not report a URL; see $RUN_DIR/tunnel.log" >&2
    tail -20 "$RUN_DIR/tunnel.log" >&2
    exit 1
  fi
  # Cloudflare needs a moment before it will route to the origin.
  REMOTE_OK=0
  for _ in $(seq 1 60); do
    code="$(curl -s -o /dev/null -w '%{http_code}' "$PUBLIC_URL/api/health" || true)"
    if [ "$code" = "200" ]; then REMOTE_OK=1; break; fi
    sleep 1
  done
  if [ "$REMOTE_OK" != "1" ]; then
    echo "WARNING: the tunnel URL is not answering yet (last HTTP ${code:-none})." >&2
  fi
else
  echo
  echo "[2/3] Tunnel disabled (--no-tunnel)."
fi

# ------------------------------------------------------------ 5. self-test
FAILED=0

check_png() {
  # check_png <file> -> 0 when it is a real PNG
  "$PY" - "$1" <<'PY'
import sys
data = open(sys.argv[1], 'rb').read(8)
raise SystemExit(0 if data[:4] == b'\x89PNG' else 1)
PY
}

run_suite() {
  local base="$1" label="$2" post image dem_img stream_frames
  echo
  echo "      --- $label ---"

  if ! post="$(curl -fsS -X POST -H "X-API-Token: $TOKEN" "$base/api/snapshot")"; then
    echo "      FAIL point-cloud snapshot (POST /api/snapshot)"; FAILED=1; return
  fi
  image="$("$PY" -c "import json,sys; print(json.load(sys.stdin)['images']['top'])" <<<"$post" 2>/dev/null || true)"
  if [ -n "$image" ] && curl -fsS "$image" -o "$RUN_DIR/test_top.png" \
       && check_png "$RUN_DIR/test_top.png"; then
    echo "      PASS point-cloud snapshot (5 views rendered)"
  else
    echo "      FAIL point-cloud snapshot image"; FAILED=1
  fi

  if ! post="$(curl -fsS -X POST -H "X-API-Token: $TOKEN" "$base/api/dem")"; then
    echo "      FAIL DEM snapshot (POST /api/dem)"; FAILED=1; return
  fi
  dem_img="$("$PY" -c "import json,sys; print(json.load(sys.stdin)['image'])" <<<"$post" 2>/dev/null || true)"
  if [ -n "$dem_img" ] && curl -fsS "$dem_img" -o "$RUN_DIR/test_dem.png" \
       && check_png "$RUN_DIR/test_dem.png"; then
    echo "      PASS DEM snapshot ($(echo "$post" | "$PY" -c "import json,sys; d=json.load(sys.stdin); print(d['scans_accumulated'], 'scans,', d['grid']['valid_cells'], 'cells')" 2>/dev/null))"
  else
    echo "      FAIL DEM snapshot image"; FAILED=1
  fi

  timeout 12 curl -sN "$base/api/stream?view=front&token=$TOKEN" \
    -o "$RUN_DIR/test_stream.bin" || true
  stream_frames="$("$PY" - "$RUN_DIR/test_stream.bin" <<'PY'
import sys
data = open(sys.argv[1], 'rb').read()
print(data.count(b'\xff\xd8\xff'))
PY
)"
  if [ "${stream_frames:-0}" -ge 2 ]; then
    echo "      PASS live stream ($stream_frames JPEG frames in ~10 s)"
  else
    echo "      FAIL live stream (only ${stream_frames:-0} frames)"; FAILED=1
  fi
}

if [ "$RUN_TEST" = "1" ]; then
  echo
  echo "[3/3] Self-test."
  run_suite "http://127.0.0.1:$PORT" "local"
  if [ -n "$PUBLIC_URL" ]; then
    run_suite "$PUBLIC_URL" "through Cloudflare ($PUBLIC_URL)"
  fi
else
  echo
  echo "[3/3] Self-test skipped (--no-test)."
fi

# -------------------------------------------------------------- 6. summary
cat > "$RUN_DIR/session.env" <<EOF
BAG=$BAG
GNSS=$GNSS
TOPIC=$SRC_TOPIC
PORT=$PORT
TOKEN=$TOKEN
PUBLIC_URL=$PUBLIC_URL
EOF

echo
echo "=========================================================================="
echo "  Local URL : http://127.0.0.1:$PORT/"
if [ -n "$PUBLIC_URL" ]; then
  echo "  Public URL: $PUBLIC_URL"
fi
echo "  API token : $TOKEN"
echo
echo "  Open the URL, paste the token, then use Fetch Point Cloud / Get DEM /"
echo "  Start Live View."
echo
echo "  Logs  : $RUN_DIR/service.log  $RUN_DIR/tunnel.log"
echo "  Stop  : $SCRIPT_DIR/stop_all.sh"
echo "=========================================================================="

if [ "$RUN_TEST" != "1" ]; then
  echo
  echo "Service is running (self-test skipped)."
elif [ "$FAILED" != "0" ]; then
  echo
  echo "One or more self-tests FAILED (see above); the service is left running."
  exit 1
else
  echo
  echo "All self-tests passed."
fi
