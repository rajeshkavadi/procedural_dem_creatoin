#!/usr/bin/env bash
# Run the snapshot service in the foreground.
#
# All arguments are passed straight to `python -m pc_snapshot` (see
# `python -m pc_snapshot --help`), so the service owns the option list:
#
#   ./run_service.sh --bag rec.bag --topic /livox/lidar --port 8085 \
#                    --gnss ublox7_raw.txt
#
# The interpreter is the project virtualenv when present; override with
# PCSNAP_PYTHON.  Use scripts/bootstrap_env.sh to create the venv.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJ_DIR="$(dirname "$SCRIPT_DIR")"
VENV="$PROJ_DIR/.venv"

PYTHON="${PCSNAP_PYTHON:-}"
if [ -z "$PYTHON" ]; then
  if [ -x "$VENV/bin/python" ]; then
    PYTHON="$VENV/bin/python"
  else
    PYTHON="$(command -v python3.8 || command -v python3.9 \
              || command -v python3.10 || command -v python3.11 \
              || command -v python3.12 || command -v python3)"
  fi
fi
[ -n "$PYTHON" ] || { echo "ERROR: no Python interpreter found (see scripts/bootstrap_env.sh)" >&2; exit 1; }

# The package lives in the project root; dem_mapping is found by the service
# itself (ws/src/dem_mapping).  Never inject foreign site-packages here.
export PCSNAP_STATIC_DIR="${PCSNAP_STATIC_DIR:-$PROJ_DIR/static}"
export PYTHONPATH="$PROJ_DIR${PYTHONPATH:+:$PYTHONPATH}"
export PCSNAP_API_TOKEN="${PCSNAP_API_TOKEN:-}"

cd "$PROJ_DIR"
exec "$PYTHON" -m pc_snapshot "$@"
