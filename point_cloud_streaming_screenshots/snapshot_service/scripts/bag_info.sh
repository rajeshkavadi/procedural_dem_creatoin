#!/usr/bin/env bash
# Print the topics, message counts and duration of a ROS 1 .bag.
#
# Usage:
#   ./scripts/bag_info.sh /path/to/recording.bag
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJ_DIR="$(dirname "$SCRIPT_DIR")"
REPO_ROOT="$(cd "$PROJ_DIR/../.." && pwd)"
BAG="${1:?usage: bag_info.sh /path/to/recording.bag}"

PY="${PCSNAP_PYTHON:-}"
if [ -z "$PY" ]; then
  for cand in "$PROJ_DIR/.venv/bin/python" "$REPO_ROOT/.venv/bin/python"; do
    if [ -x "$cand" ]; then PY="$cand"; break; fi
  done
fi
if [ -z "$PY" ]; then PY="$(command -v python3)"; fi

cd "$PROJ_DIR"
exec "$PY" -m pc_snapshot --bag "$BAG" --inspect
