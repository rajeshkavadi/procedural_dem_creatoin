#!/usr/bin/env bash
# Start the snapshot HTTP service (foreground).
#
# Usage:
#   ./run_service.sh --bag /path/to/recording.bag [options]
#   ./run_service.sh --topic /livox/lidar --port 8085
#
# All arguments are passed straight to `python -m pc_snapshot`, so every CLI
# option is available here (--bag, --topic, --port, --clip-range, --no-loop,
# --dem-*, --api-token, ...).
#
# No ROS environment is needed for --bag: the .bag file is read directly with
# the pure-Python `rosbags` library.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJ_DIR="$(dirname "$SCRIPT_DIR")"
REPO_ROOT="$(cd "$PROJ_DIR/../.." && pwd)"

# Prefer the project virtualenv (Python 3.8 + FastAPI + rosbags).
PY="${PCSNAP_PYTHON:-}"
if [ -z "$PY" ]; then
  for cand in "$PROJ_DIR/.venv/bin/python" "$REPO_ROOT/.venv/bin/python"; do
    if [ -x "$cand" ]; then PY="$cand"; break; fi
  done
fi
if [ -z "$PY" ]; then
  PY="$(command -v python3)"
fi

echo "Using interpreter: $PY"
cd "$PROJ_DIR"
exec "$PY" -m pc_snapshot "$@"
