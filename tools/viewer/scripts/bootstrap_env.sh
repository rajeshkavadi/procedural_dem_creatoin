#!/usr/bin/env bash
# Create/refresh the Python 3 virtualenv used by the snapshot service.
#
#   ./scripts/bootstrap_env.sh            # create or update .venv
#   ./scripts/bootstrap_env.sh --force    # recreate it from scratch
#
# Why this exists: the service needs Python 3.8+ (FastAPI) and `rosbags` for
# ROS 1 .bag replay.  On Ubuntu 18.04 the packaged interpreter is Python
# 3.8.0, which is older than rosbags' declared 3.8.2 floor even though the
# wheel runs fine; this script installs the compatible wheels explicitly
# instead of failing on that metadata.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJ_DIR="$(dirname "$SCRIPT_DIR")"
VENV="$PROJ_DIR/.venv"
FORCE=0
[ "${1:-}" = "--force" ] && FORCE=1

pick_python() {
  if [ -n "${PCSNAP_PYTHON:-}" ]; then echo "$PCSNAP_PYTHON"; return; fi
  for candidate in python3.8 python3.9 python3.10 python3.11 python3.12 python3; do
    if command -v "$candidate" >/dev/null 2>&1; then
      if "$candidate" -c 'import sys; raise SystemExit(0 if sys.version_info >= (3, 8) else 1)' 2>/dev/null; then
        echo "$candidate"; return
      fi
    fi
  done
}

PY="$(pick_python)"
[ -n "$PY" ] || {
  echo "ERROR: Python 3.8+ is required and none was found." >&2
  echo "       Install it (e.g. sudo apt install python3.8-venv) or set PCSNAP_PYTHON." >&2
  exit 1
}
echo "Using interpreter: $PY ($($PY -c 'import platform; print(platform.python_version())'))"

if [ "$FORCE" = "1" ] && [ -d "$VENV" ]; then
  echo "Removing $VENV"
  rm -rf "$VENV"
fi
if [ ! -d "$VENV" ]; then
  echo "Creating $VENV"
  "$PY" -m venv "$VENV"
fi

PIP="$VENV/bin/pip"
"$PIP" install --quiet --upgrade pip setuptools wheel

# Runtime + tests, without rosbags (installed separately below).
"$PIP" install --quiet \
  "numpy>=1.21,<2" \
  "Pillow>=9.0" \
  "matplotlib>=3.5" \
  "fastapi>=0.110" \
  "uvicorn>=0.27" \
  "pytest>=7.0" \
  "httpx>=0.24"

# rosbags' compression extras: pin versions that have wheels for Python 3.8.
"$PIP" install --quiet \
  "lz4>=4.3,<5" \
  "zstandard>=0.22,<0.23" \
  "ruamel.yaml>=0.17"

# The wheel is pure Python and runs on 3.8.0; ignore the 3.8.2 metadata floor.
"$PIP" install --quiet --no-deps --ignore-requires-python "rosbags>=0.9.20,<0.10"

"$VENV/bin/python" - <<'PY'
import fastapi, matplotlib, numpy, rosbags, uvicorn
print("environment OK: python %s, numpy %s" % (__import__("platform").python_version(), numpy.__version__))
PY
