#!/usr/bin/env bash
# PointPillars ROS inference - dependency installer for NVIDIA Jetson.
#
# Tested target: Jetson Nano, JetPack 4.6.x (L4T 32.6/32.7), Ubuntu 18.04,
# ROS Melodic, Python 3.6.
#
# Why this is simple: the detector uses ONLY standard PyTorch ops. There is no
# OpenPCDet / spconv / CUDA extension to compile, so no 4 GB swap + 1 h build.
#
# Usage:
#   ./install_deps.sh                 # apt + pip + checks
#   ./install_deps.sh --no-apt        # skip apt (already provisioned image)
#   INSTALL_TORCH=1 TORCH_WHEEL_URL=... ./install_deps.sh
#
# If PyTorch is missing it is NOT downloaded by default: Jetson wheels are
# tied to the exact JetPack release. The script prints the matching source.

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON="${PYTHON:-python3}"
DO_APT=1
for arg in "$@"; do
  case "$arg" in
    --no-apt) DO_APT=0 ;;
    -h|--help) sed -n '2,25p' "$0"; exit 0 ;;
    *) echo "unknown argument: $arg"; exit 1 ;;
  esac
done

say() { printf "\n== %s\n" "$*"; }
warn() { printf "!! %s\n" "$*"; }

say "System"
if [ -f /etc/nv_tegra_release ]; then
  head -n 1 /etc/nv_tegra_release
else
  warn "no /etc/nv_tegra_release - this does not look like a Jetson (continuing anyway)"
fi
. /etc/os-release 2>/dev/null || true
echo "os: ${PRETTY_NAME:-unknown}"
echo "python: $($PYTHON --version 2>&1)"
command -v nvcc >/dev/null 2>&1 && nvcc --version | tail -n 1 || warn "nvcc not on PATH (not needed for inference)"

if [ -n "${ROS_DISTRO:-}" ]; then
  echo "ros: $ROS_DISTRO"
elif [ -d /opt/ros/melodic ]; then
  echo "ros: melodic found at /opt/ros/melodic (source /opt/ros/melodic/setup.bash)"
elif [ -d /opt/ros/noetic ]; then
  echo "ros: noetic found at /opt/ros/noetic (source /opt/ros/noetic/setup.bash)"
else
  warn "ROS not found in /opt/ros - install ROS first (Melodic on JetPack 4.x)"
fi

if [ "$DO_APT" = "1" ]; then
  if command -v apt-get >/dev/null 2>&1; then
    say "apt packages"
    SUDO=""
    [ "$(id -u)" -ne 0 ] && SUDO="sudo"
    $SUDO apt-get update
    $SUDO apt-get install -y --no-install-recommends \
      python3-pip python3-dev python3-numpy python3-yaml \
      build-essential git ca-certificates

    ROS_DIST="${ROS_DISTRO:-}"
    if [ -z "$ROS_DIST" ] && [ -d /opt/ros/melodic ]; then ROS_DIST=melodic; fi
    if [ -z "$ROS_DIST" ] && [ -d /opt/ros/noetic ]; then ROS_DIST=noetic; fi
    if [ -n "$ROS_DIST" ]; then
      say "ROS packages ($ROS_DIST)"
      $SUDO apt-get install -y --no-install-recommends \
        "ros-$ROS_DIST-rosbag" \
        "ros-$ROS_DIST-sensor-msgs" \
        "ros-$ROS_DIST-visualization-msgs" \
        "ros-$ROS_DIST-geometry-msgs" \
        "ros-$ROS_DIST-rviz" || warn "could not install some ROS packages (rosbag/rviz)"
    else
      warn "skipping ROS apt packages (no distro detected)"
    fi
  else
    warn "apt-get not found - skipping system packages"
  fi
fi

say "pip packages"
PIP_USER=""
[ "$(id -u)" -ne 0 ] && PIP_USER="--user"
$PYTHON -m pip install $PIP_USER --upgrade "pip<22" >/dev/null 2>&1 || true
$PYTHON -m pip install $PIP_USER -r "$ROOT/requirements.txt" || {
  warn "pip install failed - if pip is old on Python 3.6 try:"
  warn "  python3 -m pip install --upgrade 'pip<22'"
  exit 1
}

say "PyTorch"
if $PYTHON -c "import torch" >/dev/null 2>&1; then
  $PYTHON - <<'PY'
import torch
print("torch:", torch.__version__)
print("cuda build:", torch.version.cuda, "| available:", torch.cuda.is_available())
if torch.cuda.is_available():
    print("device:", torch.cuda.get_device_name(0))
else:
    print("WARNING: GPU not visible. CPU inference works but is very slow on a Nano.")
PY
else
  warn "PyTorch is not installed."
  warn "Jetson wheels are JetPack-specific - install the one matching your L4T release:"
  warn "  https://developer.nvidia.com/embedded/pytorch"
  warn "  https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048"
  warn ""
  warn "JetPack 4.6.x example (adjust the exact wheel name from the forum thread):"
  warn "  pip3 install --no-cache https://developer.download.nvidia.com/compute/redist/jp/v461/pytorch/<wheel>.whl"
  warn ""
  warn "Then re-run this script. To let it download automatically:"
  warn "  INSTALL_TORCH=1 TORCH_WHEEL_URL=<url> ./install_deps.sh"
  if [ "${INSTALL_TORCH:-0}" = "1" ] && [ -n "${TORCH_WHEEL_URL:-}" ]; then
    $PYTHON -m pip install $PIP_USER --no-cache-dir "$TORCH_WHEEL_URL"
  else
    exit 1
  fi
fi

say "Environment check"
$PYTHON - <<'PY'
import numpy, yaml, torch
print("numpy:", numpy.__version__)
print("pyyaml:", yaml.__version__)
print("torch:", torch.__version__)
PY

CONFIG_STATE=$($PYTHON - "$ROOT/config/pointpillar_car.yaml" <<'PY'
import sys
from pathlib import Path
import yaml
p = Path(sys.argv[1])
cfg = yaml.safe_load(p.read_text()) if p.is_file() else {}
missing = [k for k in ("point_cloud_range", "anchor_sizes", "anchor_bottom_height")
           if cfg.get(k) in (None, "", [])]
print("MISSING:" + ",".join(missing) if missing else "OK")
PY
)
if [ "$CONFIG_STATE" = "OK" ]; then
  echo "config: required values present"
else
  warn "config ${CONFIG_STATE#MISSING:} still empty in config/pointpillar_car.yaml (see step 2)"
fi

say "Next"
cat <<EOF
1. source your ROS environment:
     source /opt/ros/${ROS_DISTRO:-melodic}/setup.bash
2. Fill config/pointpillar_car.yaml (point_cloud_range, anchor_sizes,
   anchor_bottom_height) from the training notebook, or run:
     $PYTHON tools/derive_config_from_dataset.py --data-root /path/to/custom_livox_data
3. Verify the checkpoint:
     $PYTHON tools/check_setup.py
4. Run on a bag:
     $PYTHON tools/bag_inference.py --bag /path/to/recording.bag
   or live:
     $PYTHON tools/bag_inference.py --topic /livox/lidar
   and open tools/rviz.rviz.
EOF
