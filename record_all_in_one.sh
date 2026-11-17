#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# record_all_in_one.sh
# Do everything from commANDS in one go.
# Run with sudo: sudo ./record_all_in_one.sh
# Press Ctrl+C to stop all recording (GPS raw + ROS bag)
# ---------------------------------------------------------------------------
set -uo pipefail

IFACE="enp0s31f6"
LIDAR_IP="192.168.1.193"
LAPTOP_IP="192.168.1.50"
GPS_PORT="/dev/ttyACM0"
GPS_WAIT=60
GPSDIR="$HOME/gps_raw"
BAGDIR="$HOME/livox_bags"
LIVOX_WS="$HOME/ws_livox"

STAMP="$(date +%Y%m%d_%H%M%S)"
mkdir -p "$GPSDIR" "$BAGDIR"
RAWGPS="$GPSDIR/ublox7_${STAMP}.txt"
BAG="$BAGDIR/mid360s_${STAMP}.bag"
LOGDIR="$BAGDIR/logs_sync_${STAMP}"
mkdir -p "$LOGDIR"

# Track PIDs
GPSD_PID=""
GPSPIPE_PID=""
LIVOX_PID=""
BAG_PID=""

cleanup() {
  echo -e "\n[INFO] Stopping all recording..."
  # Stop bag first to flush
  if [[ -n "${BAG_PID:-}" ]] && kill -0 "$BAG_PID" 2>/dev/null; then
    kill -INT "$BAG_PID" 2>/dev/null
    sleep 3
    kill -TERM "$BAG_PID" 2>/dev/null
    sleep 2
    kill -KILL "$BAG_PID" 2>/dev/null
  fi
  # Stop Livox
  if [[ -n "${LIVOX_PID:-}" ]] && kill -TERM "$LIVOX_PID" 2>/dev/null; then
    sleep 3
  fi
  # Stop GPS pipe
  if [[ -n "${GPSPIPE_PID:-}" ]] && kill -INT "$GPSPIPE_PID" 2>/dev/null; then
    sleep 1
  fi
  # Stop gpsd
  if [[ -n "${GPSD_PID:-}" ]] && kill -TERM "$GPSD_PID" 2>/dev/null; then
    sleep 1
  fi
  pkill -f "rviz_MID360s" 2>/dev/null || true
  pkill -f "livox_ros_driver2" 2>/dev/null || true
  pkill -f "gpspipe.*$STAMP" 2>/dev/null || true
  echo "[INFO] Done."
  echo "[INFO] Raw GPS: $RAWGPS"
  echo "[INFO] ROS Bag:  $BAG"
}
trap cleanup EXIT INT TERM

echo "=========================================="
echo "  Sync recording: GPS + Livox LiDAR + IMU"
echo "=========================================="
echo "[INFO] GPS port:  $GPS_PORT"
echo "[INFO] IFACE:      $IFACE"
echo "[INFO] LiDAR IP:   $LIDAR_IP"
echo "[INFO] Laptop IP:  $LAPTOP_IP"
echo "[INFO] GPS wait:   ${GPS_WAIT}s"
echo "[INFO] Raw GPS:    $RAWGPS"
echo "[INFO] Bag:        $BAG"
echo "[INFO] Press Ctrl+C to stop everything"
echo ""

# PART 1: GPS SETUP AND RECORDING
echo "[PART 1] GPS setup..."
echo "[1/7] Stopping gpsd/gpsd.socket..."
systemctl stop gpsd 2>/dev/null || true
systemctl stop gpsd.socket 2>/dev/null || true
pkill gpsd 2>/dev/null || true
sleep 1

echo "[2/7] Starting gpsd on $GPS_PORT..."
gpsd -N -n "$GPS_PORT" > "$LOGDIR/gpsd.log" 2>&1 &
GPSD_PID=$!
sleep 2

echo "[3/7] Recording raw NMEA to $RAWGPS..."
gpspipe -r | tee "$RAWGPS" > "$LOGDIR/gpspipe.log" 2>&1 &
GPSPIPE_PID=$!
sleep 2

echo "[4/7] Waiting ${GPS_WAIT}s for GPS warmup (put antenna in clear view)..."
for ((i=GPS_WAIT; i>0; i--)); do
  printf "\r   %3ds remaining..." "$i"
  sleep 1
done
echo -e "\r   GPS warmup done.                "

# PART 2: LIVox SETUP
echo "[PART 2] LiDAR network setup..."
echo "[5/7] Configuring $IFACE to $LAPTOP_IP/24..."
ip link set "$IFACE" up 2>&1 | tee "$LOGDIR/net.log"
ip addr flush dev "$IFACE" 2>&1 | tee -a "$LOGDIR/net.log"
ip addr add "${LAPTOP_IP}/24" dev "$IFACE" 2>&1 | tee -a "$LOGDIR/net.log"
echo "---"
ip -br addr | grep "$IFACE" | tee -a "$LOGDIR/net.log"
echo "---"

echo "[6/7] Pinging LiDAR $LIDAR_IP..."
ping -c 4 "$LIDAR_IP" 2>&1 | tee "$LOGDIR/ping.log" || echo "[WARN] Ping failed - check cable/IP"

# Source ROS
echo "[7/7] Starting Livox driver..."
# Source ROS as the original user context if running under sudo? But script says run with sudo.
# Try to source for current user env
if [ -n "${SUDO_USER:-}" ]; then
  REAL_USER="$SUDO_USER"
  REAL_HOME=$(eval echo "~$REAL_USER")
else
  REAL_USER="$USER"
  REAL_HOME="$HOME"
fi

source /opt/ros/melodic/setup.bash
if [ -f "${LIVOX_WS}/devel/setup.bash" ]; then
  source "${LIVOX_WS}/devel/setup.bash"
elif [ -f "$REAL_HOME/ws_livox/devel/setup.bash" ]; then
  source "$REAL_HOME/ws_livox/devel/setup.bash"
else
  echo "[ERR] ws_livox/devel/setup.bash not found"
  exit 1
fi

# Launch Livox driver (rviz_MID360s.launch as per commANDS)
roslaunch livox_ros_driver2 rviz_MID360s.launch > "$LOGDIR/livox.launch.log" 2>&1 &
LIVOX_PID=$!
echo "[INFO] Waiting for Livox driver to start (10s)..."
sleep 10

# PART 3: VERIFY & RECORD
echo "[INFO] Checking topics..."
rostopic list 2>&1 | grep -E "/livox/(lidar|imu)" | tee "$LOGDIR/topics.log" || true
rostopic hz /livox/imu 2>&1 | tail -n 2 | tee -a "$LOGDIR/topics.log" || true
rostopic hz /livox/lidar 2>&1 | tail -n 2 | tee -a "$LOGDIR/topics.log" || true

echo ""
echo "=========================================="
echo "  STARTING RECORDING"
echo "=========================================="
echo "Recording: /livox/lidar /livox/imu"
echo "Bag: $BAG"
echo "Raw GPS: $RAWGPS (already recording)"
echo ""
echo "Press Ctrl+C to stop and flush everything"
echo "=========================================="

rosbag record -O "$BAG" /livox/lidar /livox/imu > "$LOGDIR/rosbag.log" 2>&1 &
BAG_PID=$!

wait "$BAG_PID"
cleanup
