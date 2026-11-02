#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJ_DIR="$(dirname "$SCRIPT_DIR")"
VENV="$PROJ_DIR/.venv"

PYTHON="python3"
if [ -f "$VENV/bin/python" ]; then
  PYTHON="$VENV/bin/python"
fi

# Defaults
BAG=""
TOPIC="/livox/lidar"
PORT=8085
RATE=1.0
CLIP=60
HOST="127.0.0.1"

while [ $# -gt 0 ]; do
  case "$1" in
    --bag) BAG="$2"; shift 2;;
    --topic) TOPIC="$2"; shift 2;;
    --port) PORT="$2"; shift 2;;
    --bag-rate) RATE="$2"; shift 2;;
    --clip-range) CLIP="$2"; shift 2;;
    --host) HOST="$2"; shift 2;;
    *) shift;;
  esac
done

[ -z "$BAG" ] && { echo "ERROR: --bag required"; exit 1; }

export PCSNAP_BAG_PATH="$BAG"
export PCSNAP_BAG_TOPIC="$TOPIC"
export PCSNAP_BAG_RATE="$RATE"
export PCSNAP_CLIP_RANGE_M="$CLIP"
export PCSNAP_PORT="$PORT"
export PCSNAP_HOST="$HOST"
export PCSNAP_STATIC_DIR="$PROJ_DIR/static"
export PCSNAP_API_TOKEN="${PCSNAP_API_TOKEN:-lidar-snap-2026}"
export PYTHONPATH="/home/rajesh/.local/lib/python3.6/site-packages:$PROJ_DIR:$PYTHONPATH"
export PATH="/home/rajesh/.local/bin:$PATH"

cd "$PROJ_DIR"
exec python3 -m uvicorn pc_snapshot.api:app --host "$HOST" --port "$PORT"
