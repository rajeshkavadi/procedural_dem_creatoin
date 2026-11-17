#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# run_pipeline.sh - one command from a ROS 1 bag to georeferenced DEM + PCD
#
#   ./run_pipeline.sh [options]
#
#     --bag PATH      ROS 1 .bag to replay     (default: <repo>/OutdoorRoad_dataset.bag)
#     --out DIR       output directory         (default: <repo>/out)
#     --prefix NAME   output file prefix       (default: outdoorroad)
#     --rate N        playback speed           (default: 1.0)
#     --start N       seconds into the bag     (default: 0)
#     --lidar-topic T point cloud (sensor_msgs/PointCloud2) topic
#     --imu-topic T   IMU (sensor_msgs/Imu) topic
#     --gnss-topic T  GNSS pose topic
#     --no-gnss       do not replay GNSS (also skips georeferencing)
#     --rviz          also open RViz
#     --no-georef     stop after saving the local DEM/PCD
#
# Topic selection (bags recorded from several drivers keep each sensor in
# its own namespace, e.g. /mid360/livox/lidar next to /ouster/points):
#   1. an explicit --lidar-topic / --imu-topic wins when the bag has it;
#   2. otherwise the usual defaults (/mid360/livox/lidar, /mid360/livox/imu)
#      are used when the bag carries them;
#   3. otherwise the bag is searched for another sensor_msgs/PointCloud2 and
#      sensor_msgs/Imu stream, preferring a lidar/IMU pair from one namespace;
#   4. if the bag has no point cloud or no IMU at all, the run stops with the
#      list of topics it did find instead of replaying nothing.
#
# What it runs, in order:
#   1. dem_replay record_odom     - stores the live FAST-LIO trajectory to CSV
#   2. dem_bringup pipeline.launch - replay -> imu_relay -> FAST-LIO2 -> dem_node
#   3. waits until /replay/status reports state "finished"
#   4. /map_save  (std_srvs/Trigger) -> out/map_fastlio.pcd  (if FAST-LIO offers it)
#   5. /dem_node/save                -> out/dem_<stamp>.npz
#   6. dem_mapping georef_export     -> *_dem.tif, *_dem.pcd, *_map_georef.pcd,
#                                       *_georef.json  (UTM / EPSG auto)
#
# Requires ROS 1 Melodic and a built workspace:
#   cd ws && catkin_make && source devel/setup.bash
#
# Everything is logged under <out>/logs/.
# ---------------------------------------------------------------------------
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$ROOT/ws"
BAG="$ROOT/OutdoorRoad_dataset.bag"
OUT="$ROOT/out"
PREFIX="outdoorroad"
RATE="1.0"
START="0.0"
RVIZ="false"
GEOREF="1"
WANT_GNSS="1"
LIDAR_TOPIC=""
IMU_TOPIC=""
GNSS_TOPIC=""

while [ $# -gt 0 ]; do
  case "$1" in
    --bag)         BAG="$2"; shift 2 ;;
    --out)         OUT="$2"; shift 2 ;;
    --prefix)      PREFIX="$2"; shift 2 ;;
    --rate)        RATE="$2"; shift 2 ;;
    --start)       START="$2"; shift 2 ;;
    --lidar-topic) LIDAR_TOPIC="$2"; shift 2 ;;
    --imu-topic)   IMU_TOPIC="$2"; shift 2 ;;
    --gnss-topic)  GNSS_TOPIC="$2"; shift 2 ;;
    --no-gnss)     WANT_GNSS="0"; GEOREF="0"; shift ;;
    --rviz)        RVIZ="true"; shift ;;
    --no-georef)   GEOREF="0"; shift ;;
    -h|--help)     sed -n '2,40p' "$0"; exit 0 ;;
    *) echo "unknown option: $1"; exit 1 ;;
  esac
done

LOGDIR="$OUT/logs"
mkdir -p "$OUT" "$LOGDIR"
MAIN_LOG="$LOGDIR/pipeline_$(date +%Y%m%d_%H%M%S).log"
MEM_LOG="$LOGDIR/mem_$(date +%Y%m%d_%H%M%S).csv"

[ -f "$BAG" ] || {
  echo "ERROR: ROS 1 bag '$BAG' not found."
  echo "       pass a recording with --bag /path/to/recording.bag"
  exit 1
}

set +u                                     # ROS setup files are not nounset-clean
source /opt/ros/melodic/setup.bash
source "$WS/devel/setup.bash"
set -u

rospack find fast_lio >/dev/null 2>&1 || {
  echo "ERROR: the 'fast_lio' package is not on the ROS package path."
  echo "       clone https://github.com/hku-mars/FAST_LIO into $WS/src"
  echo "       (git clone --recursive), run catkin_make, and source $WS/devel/setup.bash"
  exit 1
}

# --------------------------------------------------------------- topics ---
# Which lidar / IMU / GNSS streams to replay.  A bag merged from several
# drivers holds one namespace per sensor (e.g. /mid360/livox/lidar next to
# /ouster/points), so the names are read from the bag instead of assumed.
DEFAULT_LIDAR_TOPIC="/mid360/livox/lidar"
DEFAULT_IMU_TOPIC="/mid360/livox/imu"
DEFAULT_GNSS_TOPIC="/gnss_pose"

# "<topic> <type>" per topic, from the bag's index.  'rosbag info' prints the
# FIRST topic on the 'topics:' line itself and the rest indented below it.
topic_table() {
  rosbag info "$BAG" 2>/dev/null | awk '
    /^topics:/ {
      in_topics = 1
      sub(/^topics:[ \t]*/, "")
      if ($1 ~ /^\//) print $1, $NF
      next
    }
    in_topics && /^[^ \t]/ { in_topics = 0 }
    in_topics && $1 ~ /^\//   { print $1, $NF }'
}

# true when "$1" (newline separated) contains the exact line "$2"
list_has() {
  [ -n "$1" ] && printf '%s\n' "$1" | grep -qxF -- "$2"
}

# Best IMU for a given lidar topic: the one sharing the longest namespace
# prefix, so /ouster/points goes with /ouster/imu and not /avia/livox/imu.
best_imu_for() {
  awk -v l="$1" -v m="$2" 'BEGIN {
    nm = split(m, M, "\n"); best = ""; best_score = 0
    sl = split(l, a, "/")
    for (j = 1; j <= nm; j++) {
      if (M[j] == "") continue
      sm = split(M[j], b, "/")
      score = 0
      for (k = 1; k <= sl && k <= sm; k++) {
        if (a[k] != b[k]) break
        score++
      }
      if (score > best_score) { best_score = score; best = M[j] }
    }
    if (best != "") print best
  }'
}

# Best lidar/IMU pair by shared namespace prefix (used when no default or
# explicit topic applies).
pick_pair() {
  awk -v l="$1" -v m="$2" 'BEGIN {
    nl = split(l, L, "\n"); nm = split(m, M, "\n")
    best = ""; best_score = 0
    for (i = 1; i <= nl; i++) {
      if (L[i] == "") continue
      for (j = 1; j <= nm; j++) {
        if (M[j] == "") continue
        sl = split(L[i], a, "/"); sm = split(M[j], b, "/")
        score = 0
        for (k = 1; k <= sl && k <= sm; k++) {
          if (a[k] != b[k]) break
          score++
        }
        if (score > best_score) { best_score = score; best = L[i] " " M[j] }
      }
    }
    if (best != "") print best
  }'
}

TOPIC_TABLE="$(topic_table)"
if [ -z "$TOPIC_TABLE" ]; then
  echo "WARNING: 'rosbag info' returned no topic list for $BAG"
  echo "         (unindexed/truncated bag?) - falling back to the default topics"
  LIDAR_CANDIDATES=""; IMU_CANDIDATES=""; GNSS_CANDIDATES=""
else
  LIDAR_CANDIDATES=$(printf '%s\n' "$TOPIC_TABLE" \
      | awk '$2 == "sensor_msgs/PointCloud2" { print $1 }')
  IMU_CANDIDATES=$(printf '%s\n' "$TOPIC_TABLE" \
      | awk '$2 == "sensor_msgs/Imu" { print $1 }')
  GNSS_CANDIDATES=$(printf '%s\n' "$TOPIC_TABLE" \
      | awk '$2 == "geometry_msgs/PoseStamped" \
          || $2 == "geometry_msgs/PoseWithCovarianceStamped" \
          || $2 == "sensor_msgs/NavSatFix" \
          || $2 == "gps_common/GPSFix" { print $1 }')
fi

PAIR="$(pick_pair "$LIDAR_CANDIDATES" "$IMU_CANDIDATES")"
PAIR_LIDAR="${PAIR%% *}"
PAIR_IMU="${PAIR#* }"
[ "$PAIR_LIDAR" = "$PAIR" ] && PAIR_LIDAR=""      # no pair available

# ---- lidar
if [ -n "$LIDAR_TOPIC" ]; then
  if [ -n "$TOPIC_TABLE" ] && ! list_has "$LIDAR_CANDIDATES" "$LIDAR_TOPIC"; then
    echo "WARNING: --lidar-topic $LIDAR_TOPIC is not a sensor_msgs/PointCloud2"
    echo "         topic in this bag - searching the bag for another one"
    LIDAR_TOPIC=""
  fi
fi
[ -n "$LIDAR_TOPIC" ] || {
  if list_has "$LIDAR_CANDIDATES" "$DEFAULT_LIDAR_TOPIC"; then
    LIDAR_TOPIC="$DEFAULT_LIDAR_TOPIC"
  else
    LIDAR_TOPIC="$PAIR_LIDAR"
  fi
}
[ -n "$LIDAR_TOPIC" ] || LIDAR_TOPIC="$(printf '%s\n' "$LIDAR_CANDIDATES" | head -1)"

# ---- IMU
if [ -n "$IMU_TOPIC" ]; then
  if [ -n "$TOPIC_TABLE" ] && ! list_has "$IMU_CANDIDATES" "$IMU_TOPIC"; then
    echo "WARNING: --imu-topic $IMU_TOPIC is not a sensor_msgs/Imu topic in"
    echo "         this bag - searching the bag for another one"
    IMU_TOPIC=""
  fi
fi
[ -n "$IMU_TOPIC" ] || {
  if list_has "$IMU_CANDIDATES" "$DEFAULT_IMU_TOPIC"; then
    IMU_TOPIC="$DEFAULT_IMU_TOPIC"
  else
    # whichever IMU belongs to the lidar's own namespace
    IMU_TOPIC="$(best_imu_for "$LIDAR_TOPIC" "$IMU_CANDIDATES")"
    [ -n "$IMU_TOPIC" ] || IMU_TOPIC="$PAIR_IMU"
  fi
}
[ -n "$IMU_TOPIC" ] || IMU_TOPIC="$(printf '%s\n' "$IMU_CANDIDATES" | head -1)"

# ---- GNSS
if [ "$WANT_GNSS" = "1" ]; then
  [ -n "$GNSS_TOPIC" ] || {
    if list_has "$GNSS_CANDIDATES" "$DEFAULT_GNSS_TOPIC"; then
      GNSS_TOPIC="$DEFAULT_GNSS_TOPIC"
    else
      GNSS_TOPIC="$(printf '%s\n' "$GNSS_CANDIDATES" | head -1)"
    fi
  }
  if [ -z "$GNSS_TOPIC" ]; then
    echo "NOTE: no GNSS pose topic in this bag - replaying without GNSS"
    echo "      (local DEM/PCD only, georeferencing is skipped)"
    WANT_GNSS="0"; GEOREF="0"
  fi
else
  GNSS_TOPIC=""
fi

if [ -z "$LIDAR_TOPIC" ] || [ -z "$IMU_TOPIC" ]; then
  echo "ERROR: could not find a usable sensor pair in '$BAG'."
  echo "       point clouds (sensor_msgs/PointCloud2): ${LIDAR_CANDIDATES:-<none>}"
  echo "       IMUs         (sensor_msgs/Imu)        : ${IMU_CANDIDATES:-<none>}"
  echo "       topics in the bag:"
  if [ -n "$TOPIC_TABLE" ]; then
    printf '%s\n' "$TOPIC_TABLE" | sed 's/^/         /'
  else
    echo "         (could not read the bag index with 'rosbag info')"
  fi
  echo "       pick one explicitly:"
  echo "         --lidar-topic /namespace/points --imu-topic /namespace/imu"
  exit 1
fi

echo "topics: lidar=$LIDAR_TOPIC  imu=$IMU_TOPIC  gnss=${GNSS_TOPIC:-<none>}"
if [ -n "$LIDAR_CANDIDATES" ]; then
  echo "        (other point clouds in the bag: $(printf '%s\n' "$LIDAR_CANDIDATES" | tr '\n' ' '))"
  echo "        (other IMUs in the bag:         $(printf '%s\n' "$IMU_CANDIDATES" | tr '\n' ' '))"
fi
case "$LIDAR_TOPIC" in
  *ouster*|*OS1*|*OS0*|*OS2*)
    echo "WARNING: $LIDAR_TOPIC looks like an Ouster stream, but the pipeline"
    echo "         loads fastlio_mid360.yaml (preprocess/lidar_type 4 = Livox PointCloud2)."
    echo "         Set lidar_type 3 in that config for Ouster data."
    ;;
esac

# Nodes this pipeline may leave behind (matched by name, never by command line).
PIPELINE_NODES=(/replay /fastlio_mapping /laserMapping /dem_node /imu_relay
                /stream_monitor /record_odom /dem_heatmap)

stop_ros_nodes() {
  local running name
  running="$(timeout 10 rosnode list 2>/dev/null || true)"
  [ -n "$running" ] || return 0
  for name in "${PIPELINE_NODES[@]}"; do
    if printf '%s\n' "$running" | grep -qx "$name"; then
      timeout 10 rosnode kill "$name" >/dev/null 2>&1 || true
    fi
  done
  sleep 1
}

# Wait up to $2 seconds for $1 to exit; escalate INT -> TERM -> KILL.  Plain
# `wait` is dangerous here: async commands inherit SIG_IGN for SIGINT in
# non-interactive shells, and an idle rospy node blocked in rospy.spin() does
# not process signals either.
stop_pid() {
  local pid="$1" grace="${2:-15}" i
  [ -n "$pid" ] || return 0
  kill -INT "$pid" 2>/dev/null
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

cleanup() {
  [ -n "${MEM_PID:-}" ] && kill "$MEM_PID" 2>/dev/null
  if [ -n "${REC_PID:-}" ]; then
    stop_pid "$REC_PID" 10
    REC_PID=""
  fi
  if [ -n "${LAUNCH_PID:-}" ]; then
    stop_pid "$LAUNCH_PID" 15
    LAUNCH_PID=""
  fi
  stop_ros_nodes                      # roslaunch may have died without children
  MEM_PID=""
}
trap cleanup EXIT

# never run on top of a previous session
stop_ros_nodes
sleep 1

BAG_DURATION=$(rosbag info "$BAG" 2>/dev/null | awk '/^duration:/ {gsub("s","",$2); print $2}')
[ -n "$BAG_DURATION" ] || BAG_DURATION=0
MAX_WAIT=$(awk -v d="$BAG_DURATION" -v r="$RATE" 'BEGIN {printf "%d", d / r + 180}')
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
rosrun dem_replay record_odom --topic /Odometry --out "$OUT/odom.csv" \
      >> "$LOGDIR/record_odom.log" 2>&1 &
REC_PID=$!
sleep 2
kill -0 "$REC_PID" 2>/dev/null || { echo "record_odom died (see $LOGDIR/record_odom.log)"; exit 1; }

echo "== 2/6 pipeline (replay + FAST-LIO2 + DEM) -> $MAIN_LOG"
GNSS_ARG=""; USE_GNSS="false"
if [ "$WANT_GNSS" = "1" ]; then
  GNSS_ARG="gnss_topic:=$GNSS_TOPIC"; USE_GNSS="true"
fi
roslaunch dem_bringup pipeline.launch \
      bag:="$BAG" rate:="$RATE" start_offset:="$START" \
      lidar_topic:="$LIDAR_TOPIC" imu_topic:="$IMU_TOPIC" $GNSS_ARG \
      use_rviz:="$RVIZ" use_viz:=false use_monitor:=true \
      player:=custom use_fastlio:=true use_gnss:="$USE_GNSS" \
      save_dir:="$OUT" \
      >> "$MAIN_LOG" 2>&1 &
LAUNCH_PID=$!

for _ in $(seq 1 60); do
  grep -q 'Node init finished' "$MAIN_LOG" && break
  kill -0 "$LAUNCH_PID" 2>/dev/null || break
  sleep 1
done
if grep -q 'Node init finished' "$MAIN_LOG"; then
  grep -q 'p_pre->lidar_type 4' "$MAIN_LOG" \
    || echo "WARNING: lidar_type is not 4 - check fastlio_mid360.yaml in $MAIN_LOG"
else
  echo "WARNING: FAST-LIO did not report startup - see $MAIN_LOG"
fi

echo "== 3/6 waiting for playback to finish (up to ${MAX_WAIT}s)"
finished=0
deadline=$(( $(date +%s) + MAX_WAIT ))
while [ "$(date +%s)" -lt "$deadline" ]; do
  kill -0 "$LAUNCH_PID" 2>/dev/null || { echo "pipeline exited early (see $MAIN_LOG)"; break; }
  st=$(timeout 25 rostopic echo -n 1 /replay/status 2>/dev/null | tr -d '\n' || true)
  if echo "$st" | grep -q '"state": *"finished"'; then
    finished=1
    echo "   $(echo "$st" | tail -c 200)"
    break
  fi
  sleep 5
done
[ "$finished" = "1" ] || echo "WARNING: replay 'finished' not observed - continuing anyway"

sleep 5                                   # let the last scans propagate
echo "== 4/6 saving the FAST-LIO map -> $OUT/map_fastlio.pcd"
if rosservice list 2>/dev/null | grep -qx '/map_save'; then
  rosservice call /map_save 2>&1 | tee -a "$LOGDIR/services.log"
else
  echo "   /map_save not offered by this FAST-LIO build - skipping the map PCD"
  echo "   (the DEM, trajectory and georeferencing are unaffected)" \
    | tee -a "$LOGDIR/services.log"
fi

echo "== 5/6 saving the DEM grid -> $OUT/dem_<stamp>.npz"
rosservice call /dem_node/save 2>&1 | tee -a "$LOGDIR/services.log"

sleep 2
stop_pid "${REC_PID:-}" 10; REC_PID=""
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
  MAP_ARG=""
  [ -f "$OUT/map_fastlio.pcd" ] && MAP_ARG="--map=$OUT/map_fastlio.pcd"
  rosrun dem_mapping georef_export \
      --dem "$DEM" \
      --odom "$OUT/odom.csv" \
      --bag "$BAG" \
      --gnss-topic "$GNSS_TOPIC" \
      $MAP_ARG \
      --out-dir "$OUT" --prefix "$PREFIX"
  rc=$?
  echo
  echo "exit code: $rc"
fi

echo
echo "outputs in $OUT:"
ls -lh "$OUT" | sed 's/^/  /'
