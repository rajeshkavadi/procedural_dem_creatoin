#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# run_pipeline.sh - one command from ROS 2 bag to georeferenced DEM + PCD
#
#   ./run_pipeline.sh [options]
#
#     --bag PATH      rosbag2 directory to replay
#                     (default: /home/rajesh/proc_dem_creation/OutdoorRoad_dataset_ros2)
#     --out DIR       output directory           (default: <repo>/out)
#     --prefix NAME   output file prefix         (default: outdoorroad)
#     --rate N        playback speed             (default: 1.0)
#     --start N       seconds into the bag       (default: 0)
#     --rviz          also open RViz2
#     --no-georef     stop after saving the local DEM/PCD
#
# What it runs, in order:
#   1. dem_replay record_odom   - stores the live FAST-LIO trajectory to CSV
#   2. dem_bringup pipeline     - replay -> imu_relay -> FAST-LIO2 -> dem_node
#   3. waits until /replay/status reports state "finished"
#   4. /map_save  (std_srvs/Trigger) -> out/map_fastlio.pcd
#   5. /dem_node/save             -> out/dem_<stamp>.npz
#   6. dem_mapping georef_export  -> *_dem.tif, *_dem.pcd, *_map_georef.pcd,
#                                    *_georef.json  (UTM / EPSG auto)
#
# Everything is logged under <out>/logs/.
# ---------------------------------------------------------------------------
set -uo pipefail

ROOT="/home/rajesh/proc_dem_creation"
WS="$ROOT/ws"
BAG="$ROOT/OutdoorRoad_dataset_ros2"
OUT="$ROOT/out"
PREFIX="outdoorroad"
RATE="1.0"
START="0.0"
RVIZ="false"
GEOREF="1"

while [ $# -gt 0 ]; do
  case "$1" in
    --bag)       BAG="$2"; shift 2 ;;
    --out)       OUT="$2"; shift 2 ;;
    --prefix)    PREFIX="$2"; shift 2 ;;
    --rate)      RATE="$2"; shift 2 ;;
    --start)     START="$2"; shift 2 ;;
    --rviz)      RVIZ="true"; shift ;;
    --no-georef) GEOREF="0"; shift ;;
    -h|--help)   sed -n '2,30p' "$0"; exit 0 ;;
    *) echo "unknown option: $1"; exit 1 ;;
  esac
done

LOGDIR="$OUT/logs"
mkdir -p "$OUT" "$LOGDIR"
MAIN_LOG="$LOGDIR/pipeline_$(date +%Y%m%d_%H%M%S).log"
MEM_LOG="$LOGDIR/mem_$(date +%Y%m%d_%H%M%S).csv"

[ -d "$BAG" ] || {
  echo "ERROR: ROS 2 bag '$BAG' not found."
  echo "       convert it first:"
  echo "         pip install --user rosbags"
  echo "         ros2 run dem_replay bag_convert -- \\"
  echo "             $ROOT/OutdoorRoad_dataset.bag -o $BAG"
  exit 1
}

set +u                                     # ROS setup files are not nounset-clean
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u

# Wait up to $2 seconds for $1 to exit; escalate TERM -> KILL.  Plain
# `wait` is dangerous here: async commands inherit SIG_IGN for SIGINT in
# non-interactive shells (so `ros2 run` swallows it) and an idle rclpy
# node blocked in rcl_wait() does not process signals either - a run once
# hung for 1.5 h at such a wait.
stop_pid() {
  local pid="$1" grace="${2:-15}" i
  [ -n "$pid" ] || return 0
  for ((i = 0; i < grace; i++)); do
    kill -0 "$pid" 2>/dev/null || return 0
    sleep 1
  done
  kill -TERM "$pid" 2>/dev/null
  for ((i = 0; i < 5; i++)); do
    kill -0 "$pid" 2>/dev/null || return 0
    sleep 1
  done
  kill -KILL "$pid" 2>/dev/null
  return 0
}

# Stop every process this script started (never unrelated ones)
stop_pipeline_nodes() {
  pkill -INT  -f 'lib/dem_replay/record_odom --topic' 2>/dev/null
  pkill -TERM -f 'fast_lio/lib/fast_lio/fastlio_mapping' 2>/dev/null
  pkill -TERM -f 'dem_mapping/lib/dem_mapping/dem_node' 2>/dev/null
  pkill -TERM -f 'dem_replay/lib/dem_replay/replay --ros-args' 2>/dev/null
  pkill -TERM -f 'dem_replay/lib/dem_replay/imu_relay' 2>/dev/null
  pkill -TERM -f 'dem_replay/lib/dem_replay/stream_monitor' 2>/dev/null
  sleep 2
  pkill -KILL -f 'fast_lio/lib/fast_lio/fastlio_mapping' 2>/dev/null
  pkill -KILL -f 'dem_mapping/lib/dem_mapping/dem_node' 2>/dev/null
  pkill -KILL -f 'dem_replay/lib/dem_replay/(replay|imu_relay|stream_monitor)' 2>/dev/null
  pkill -KILL -f 'lib/dem_replay/record_odom --topic' 2>/dev/null
  return 0
}

cleanup() {
  # stop everything this script started, never touch unrelated processes
  [ -n "${MEM_PID:-}" ]  && kill "$MEM_PID" 2>/dev/null
  if [ -n "${REC_PID:-}" ]; then
    kill -INT "$REC_PID" 2>/dev/null
    pkill -INT -f 'lib/dem_replay/record_odom --topic' 2>/dev/null
    stop_pid "$REC_PID" 10
    REC_PID=""
  fi
  if [ -n "${LAUNCH_PID:-}" ]; then
    kill -INT "$LAUNCH_PID" 2>/dev/null
    stop_pid "$LAUNCH_PID" 10
    LAUNCH_PID=""
  fi
  stop_pipeline_nodes          # ros2 launch may have died without children
  MEM_PID=""
}
trap cleanup EXIT

# never run on top of a previous session
pkill -f 'ros2 [l]aunch dem_bringup' 2>/dev/null
pkill -f 'fastlio_[m]apping' 2>/dev/null
pkill -f 'dem_[n]ode' 2>/dev/null
pkill -f 'record_[o]dom' 2>/dev/null
pkill -f '[r]viz2' 2>/dev/null
sleep 2

BAG_DURATION=$(python3 - "$BAG" <<'EOF'
import sys, yaml
print(yaml.safe_load(open(sys.argv[1] + '/metadata.yaml'))
      ['rosbag2_bagfile_information']['duration']['nanoseconds'] / 1e9)
EOF
)
MAX_WAIT=$(python3 -c "print(int($BAG_DURATION / $RATE) + 180)")
echo "bag duration ${BAG_DURATION}s at ${RATE}x -> wait budget ${MAX_WAIT}s"

(
  echo "epoch,used_gb,free_gb,available_gb"
  while true; do
    echo "$(date +%s),$(free -m | awk 'NR==2{print $3/1024,$4/1024,$7/1024}' | tr ' ' ',')"
    sleep 10
  done
) > "$MEM_LOG" 2>/dev/null &
MEM_PID=$!

: > "$MAIN_LOG"
echo "== 1/6 trajectory recorder -> $OUT/odom.csv"
ros2 run dem_replay record_odom -- --topic /Odometry \
      --out "$OUT/odom.csv" >> "$LOGDIR/record_odom.log" 2>&1 &
REC_PID=$!
sleep 2
kill -0 "$REC_PID" 2>/dev/null || { echo "record_odom died (see $LOGDIR/record_odom.log)"; exit 1; }

echo "== 2/6 pipeline (replay + FAST-LIO2 + DEM) -> $MAIN_LOG"
ros2 launch dem_bringup pipeline.launch.py \
      bag:="$BAG" rate:="$RATE" use_rviz:="$RVIZ" use_viz:=false \
      use_monitor:=true player:=custom use_fastlio:=true use_gnss:=true \
      >> "$MAIN_LOG" 2>&1 &
LAUNCH_PID=$!

for _ in $(seq 1 60); do
  grep -q 'Node init finished' "$MAIN_LOG" && break
  kill -0 "$LAUNCH_PID" 2>/dev/null || break
  sleep 1
done
grep -q 'Node init finished' "$MAIN_LOG" \
  || { echo "FAST-LIO did not start (see $MAIN_LOG)"; tail -20 "$MAIN_LOG"; exit 1; }
grep -q 'p_pre->lidar_type 4' "$MAIN_LOG" \
  || { echo "ERROR: lidar_type is not 4 - see $MAIN_LOG"; exit 1; }

if [ "$START" != "0.0" ] && [ "$START" != "0" ]; then
  echo "   (starting ${START}s into the bag)"
  ros2 topic pub --once /replay/rate std_msgs/msg/Float64 "{data: $RATE}" >/dev/null 2>&1
fi

echo "== 3/6 waiting for playback to finish (up to ${MAX_WAIT}s)"
finished=0
deadline=$(( $(date +%s) + MAX_WAIT ))
while [ "$(date +%s)" -lt "$deadline" ]; do
  kill -0 "$LAUNCH_PID" 2>/dev/null || { echo "pipeline exited early (see $MAIN_LOG)"; break; }
  st=$(timeout 25 ros2 topic echo /replay/status --once 2>/dev/null | tr -d '\n' || true)
  if echo "$st" | grep -q '"state": "finished"'; then
    finished=1
    echo "   $st"
    break
  fi
  sleep 5
done
[ "$finished" = "1" ] || echo "WARNING: replay 'finished' not observed - continuing anyway"

sleep 5                                   # let the last scans propagate
echo "== 4/6 saving the FAST-LIO map -> $OUT/map_fastlio.pcd"
ros2 service call /map_save std_srvs/srv/Trigger {} 2>&1 | tee -a "$LOGDIR/services.log"

echo "== 5/6 saving the DEM grid -> $OUT/dem_<stamp>.npz"
ros2 service call /dem_node/save std_srvs/srv/Trigger {} 2>&1 | tee -a "$LOGDIR/services.log"

sleep 2
kill -INT "$REC_PID" 2>/dev/null
pkill -INT -f 'lib/dem_replay/record_odom --topic' 2>/dev/null
stop_pid "$REC_PID" 10; REC_PID=""        # bounded - never hang on wait
echo "   odom rows: $(wc -l < "$OUT/odom.csv" 2>/dev/null || echo 0)"

cleanup; trap - EXIT                       # pipeline finished, stop the nodes
sleep 2

DEM=$(ls -t "$OUT"/dem_*.npz 2>/dev/null | head -1 || true)
echo
echo "DEM : ${DEM:-MISSING}"
echo "odom: $OUT/odom.csv ($(wc -l < "$OUT/odom.csv" 2>/dev/null || echo 0) rows)"
echo "map : $OUT/map_fastlio.pcd $( [ -f "$OUT/map_fastlio.pcd" ] && stat -c%s "$OUT/map_fastlio.pcd" || echo '(missing)' ) bytes"

if [ "$GEOREF" = "1" ]; then
  echo
  echo "== 6/6 georeferencing (local map frame -> WGS84/UTM) + PCD export"
  [ -n "$DEM" ] || { echo "ERROR: no DEM file to georeference"; exit 1; }
  ros2 run dem_mapping georef_export -- \
      --dem "$DEM" \
      --odom "$OUT/odom.csv" \
      --bag "$BAG" \
      --gnss-topic /gnss_pose \
      --map "$OUT/map_fastlio.pcd" \
      --out-dir "$OUT" --prefix "$PREFIX"
  rc=$?
  echo
  echo "exit code: $rc"
fi

echo
echo "outputs in $OUT:"
ls -lh "$OUT" | sed 's/^/  /'
