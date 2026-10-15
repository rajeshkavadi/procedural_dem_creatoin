# Procedural DEM creation from a ROS bag (ROS 1 → ROS 2, FAST-LIO2, georeferenced output)

---


Converts a recorded road dataset (ROS 1 `.bag`) into ROS 2, replays it through
FAST-LIO2 LiDAR-inertial odometry, builds an elevation grid online, and exports
the result **georeferenced** (WGS84 / UTM) as GeoTIFF + PCD point clouds —
with one command.

```
OutdoorRoad_dataset.bag   (ROS 1, 47 GB)
        │  bag_convert  (one-off, no ROS 1 install needed)
        ▼
OutdoorRoad_dataset_ros2/ (rosbag2, 45 GB)
        │  ./run_pipeline.sh
        ├──► replay ─► imu_relay ─► FAST-LIO2 ─► dem_node ─► DEM grid (.npz)
        │                 │             │            │
        │                 │             └─ /Odometry ─► record_odom (CSV)
        │                 │             └─ /map_save ─► map_fastlio.pcd
        ▼
georef_export  (GNSS track ⟷ LIO trajectory fit)
        │
        ▼
out/outdoorroad_dem.tif   (+ hillshade, preview PNG)
out/outdoorroad_dem.pcd   (DEM cells in UTM metres)
out/outdoorroad_map_georef.pcd (FAST-LIO map in UTM metres)
out/outdoorroad_georef.json    (CRS, transform, RMS provenance)
```

---

## Contents

1. [Requirements](#1-requirements)
2. [Quick start](#2-quick-start)
3. [ROS 1 → ROS 2 bag conversion](#3-ros-1--ros-2-bag-conversion)
4. [The pipeline, stage by stage](#4-the-pipeline-stage-by-stage)
5. [Georeferencing — how the DEM gets real coordinates](#5-georeferencing--how-the-dem-gets-real-coordinates)
6. [Outputs](#6-outputs)
7. [Configuration & tuning](#7-configuration--tuning)
8. [Running the stages manually](#8-running-the-stages-manually)
9. [Errors and how they were fixed](#9-errors-and-how-they-were-fixed)
10. [Switching the pipeline ROS 1 → ROS 2 (and back)](#10-switching-the-pipeline-ros-1--ros-2-and-back)
11. [Development: build, test, layout](#11-development-build-test-layout)

---

## 1. Requirements

* Ubuntu 22.04 + **ROS 2 Humble** (`/opt/ros/humble`)
* Python 3.10 with:

  ```bash
  sudo apt install python3-numpy python3-scipy python3-matplotlib python3-gdal
  pip install --user rosbags          # ROS 1 bag reading / conversion
  ```

  | package     | used for                                   | tested with |
  |-------------|--------------------------------------------|-------------|
  | `rosbags`   | ROS 1 `.bag` read + ROS 1→2 conversion      | 0.10.x      |
  | `numpy`     | grids, fits                                | 1.26.4      |
  | `scipy`     | `cKDTree` reprojection, KD-tree lookups    | 1.8.0       |
  | `matplotlib`| preview / hillshade PNG                    | 3.5.1       |
  | GDAL (`osgeo`) | GeoTIFF write/read                      | 3.4.1       |

  > The Ubuntu `python3-scipy` package prints a benign
  > `UserWarning: A NumPy version >=1.17.3 and <1.25.0 is required …`
  > on import. It is cosmetic — everything works; see
  > [§9](#9-errors-and-how-they-were-fixed).

* ~65 GB free disk for bag conversion (47 GB in + 45 GB out) and ≥ 8 GB RAM
  recommended (the FAST-LIO map cloud grows to several hundred MB; peak RSS
  observed ≈ 4–5 GB).
* No ROS 1, no `ros1_bridge`, no Docker: the ROS 1 bag is read with the
  pure-Python `rosbags` library.

Environment for every command below:

```bash
source /opt/ros/humble/setup.bash
source /home/rajesh/proc_dem_creation/ws/install/setup.bash
```

## 2. Quick start

```bash
# 1. one-off: convert the ROS 1 bag (≈ 5.5 min for 47 GB)
pip install --user rosbags
ros2 run dem_replay bag_convert -- \
    OutdoorRoad_dataset.bag -o OutdoorRoad_dataset_ros2

# 2. everything else: replay → LIO → DEM → georeference (≈ 13 min at rate 1.0)
./run_pipeline.sh
```

Useful flags of `run_pipeline.sh`:

```
--bag PATH      rosbag2 directory to replay (default OutdoorRoad_dataset_ros2)
--out DIR       output directory            (default <repo>/out)
--prefix NAME   output file prefix          (default outdoorroad)
--rate N        playback speed              (default 1.0)
--rviz          also open RViz2
--no-georef     stop after the local DEM/PCD (skip step 6)
```

All node output is logged under `out/logs/`; a memory sampler writes
`out/logs/mem_*.csv` every 10 s so you can inspect the peak RAM of the run.

## 3. ROS 1 → ROS 2 bag conversion

### Why a custom tool

`ros2 bag convert` and `ros1_bridge` need a ROS 1 environment or a bridge
setup. `rosbags` (`pip install rosbags`) reads the ROS 1 serialization format
directly and we re-serialise into CDR, so **only Python is needed**.

The wrapper `dem_replay/bag_convert.py` adds, over the stock
`rosbags-convert`:

* `--topics` / `--exclude-topics` — keep only what the pipeline consumes,
* `--start` / `--duration` — cut a time window (handy for quick tests),
* `--dry-run` — inspect without converting,
* progress + ETA + per-topic byte statistics,
* **automatic post-conversion validation**: the result is re-read, message
  counts are compared per topic and one payload per topic is
  de-serialised; a JSON report is written.

### What it does to the messages

* `std_msgs/Header`: ROS 1 has a `seq` field, ROS 2 dropped it — it is
  removed (this is the only structural difference for standard messages).
* everything else: byte-copy when the wire formats match, otherwise field
  migration (renames applied, removed fields dropped, added fields defaulted).
* Output: rosbag2 **bagfile version 8** (readable by Humble's rosbag2;
  version 9 would require a newer rosbag2), storage `sqlite3`, no
  compression.

### The conversion used here

```bash
ros2 run dem_replay bag_convert -- \
    OutdoorRoad_dataset.bag -o OutdoorRoad_dataset_ros2
```

Result (`/tmp/opencode/convert_report.json` on this machine):

* 657.3 s, **432,794 messages**, 7 topics, 47.73 GB → 45 GB
* wall time 332 s (≈ 145 MB/s), validation `ok: true`

| topic | type | count | rate |
|---|---|---|---|
| `/mid360/livox/lidar` | `PointCloud2` | 6 573 | 10 Hz |
| `/mid360/livox/imu`   | `Imu`         | 131 462 | 200 Hz |
| `/gnss_pose`          | `PoseStamped` | 65 732 | 100 Hz |
| `/avia/livox/lidar`   | `PointCloud2` | 6 573 | 10 Hz |
| `/avia/livox/imu`     | `Imu`         | 133 719 | 203 Hz |
| `/ouster/points`      | `PointCloud2` | 6 573 | 10 Hz |
| `/ouster/imu`         | `Imu`         | 82 162 | 125 Hz |

All 7 topics were converted (full fidelity). The DEM pipeline itself consumes
only `/mid360/livox/{lidar,imu}` + `/gnss_pose`; to save disk use:

```bash
ros2 run dem_replay bag_convert -- OutdoorRoad_dataset.bag -o small_ros2 \
    --topics /mid360/livox/lidar /mid360/livox/imu /gnss_pose \
    --start 0 --duration 120
```

Check a bag (any bag) without converting:

```bash
ros2 run dem_replay bag_convert -- OutdoorRoad_dataset.bag --dry-run
ros2 run dem_replay inspect_bag -- OutdoorRoad_dataset_ros2   # rates, stamp monotonicity, samples
ros2 bag info OutdoorRoad_dataset_ros2                        # stock ROS 2 view
```

## 4. The pipeline, stage by stage

`./run_pipeline.sh` runs six stages; each can also be run by hand
([§8](#8-running-the-stages-manually)).

1. **`record_odom`** (`dem_replay`) — subscribes to `/Odometry` (FAST-LIO's
   live trajectory) and appends `t,x,y,z,qx,qy,qz,qw` to `out/odom.csv`.
   The trajectory is computed *live* during replay and is **not** in the bag,
   so someone has to write it down for the georeferencing step. `t` is the
   header stamp (bag epoch time) so it matches the GNSS track timestamps.
2. **`pipeline.launch.py`** (`dem_bringup`) starts:
   * `replay` (`dem_replay`) — timestamp-preserving bag player with rate
     control, `/replay/status` JSON, pause/resume/restart services,
   * `imu_relay` — Livox IMU accelerations are recorded in **g**; FAST-LIO
     needs m/s², so `/mid360/livox/imu` is republished as
     `/mid360/livox/imu_si` scaled ×9.80665 (raw topic untouched),
   * `fastlio_mapping` (FAST-LIO2, C++) — LiDAR-inertial odometry,
     publishes `/Odometry`, `/cloud_registered`, and accumulates the map
     cloud for `/map_save`,
   * `dem_node` (`dem_mapping`) — incremental elevation grid from the
     registered clouds (`mode: registered`, output frame `camera_init`),
   * optional: `stream_monitor` (ordering/synchronisation watchdog),
     `rviz`, matplotlib live view.
3. **wait** until `/replay/status` reports `"state": "finished"`.
4. **`/map_save`** (`std_srvs/Trigger`, FAST-LIO2) → `out/map_fastlio.pcd`.
5. **`/dem_node/save`** (`std_srvs/Trigger`) → `out/dem_<stamp>.npz`.
6. **`georef_export`** (`dem_mapping`) — the georeferencing step, see below.

## 5. Georeferencing — how the DEM gets real coordinates

### The problem

FAST-LIO2 and `dem_node` work in the **local mapper frame** `camera_init`:
metres, but an arbitrary origin and an arbitrary yaw (whatever direction the
vehicle happened to start in). A DEM in that frame cannot be put on a map,
compared with satellite imagery, or stored as a GeoTIFF. The `/gnss_pose`
topic in the bag *is* geographic (WGS84 lat/lon in degrees, ~60.44° N /
22.31° E — Turku, Finland; UTM zone 34N), so the two must be tied together.

### The method (`dem_mapping/georef_export.py`)

1. Read `/gnss_pose` from the bag (ROS 1 `.bag` **or** rosbag2 — same code
   path as the converter, no ROS 1 needed).
2. Read the recorded FAST-LIO trajectory (`out/odom.csv`).
3. Match the two tracks by timestamp (default tolerance `--tol 0.2 s`).
4. Estimate the rigid transform `p_utm = R · p_local + t`:
   * **default `--fit yaw`** — one rotation about the gravity axis plus a
     translation. Both frames are gravity-aligned (GNSS/IMU up = LIO up), so
     letting the fit tilt would only absorb sensor errors and make the
     residual look better than reality; an Umeyama full 3-D fit is available
     with `--fit full` and is reported as a diagnostic (`scale` should be
     ≈ 1.0 if the two tracks describe the same motion).
   * convert the GNSS lat/lon to UTM — the zone is derived from longitude
     (here **EPSG:32634**, zone 34N); `--epsg` forces a code.
5. **Lat/lon order auto-detection**: a `PoseStamped` can hold
   `(x=lat, y=lon)` or `(x=lon, y=lat)` and both look plausible. Both
   hypotheses are fitted; they differ by up to 2× in metric scale, so the one
   whose UTM extent matches the metric LiDAR track wins unambiguously. Both
   are reported in the JSON so the decision is auditable.
6. **Quality gate**: RMS of the matched track in metres. If
   `RMS > --rms-threshold` (default 1.0 m) the tool **refuses** to write
   georeferenced output (exit code 2) and explains that the streams do not
   describe the same motion — pass `--allow-poor` to override. A bad
   georeference is worse than none.
7. **Re-project the grids**: every DEM cell centre is transformed with
   `R, t` and injected into a new UTM-aligned grid whose origin sits on a
   multiple of the output resolution (clean GeoTIFF geotransform), filled by
   nearest-neighbour (`scipy.cKDTree`) with a `0.75·res` distance cap so the
   rotated footprint does not smear into empty corners. Fill ratio is
   reported.

### Outputs of the fit (all in `out/`)

* `outdoorroad_dem.tif` — GeoTIFF, EPSG:32634, NaN nodata, elevations in
  metres (transformed into the GNSS vertical),
* `outdoorroad_dem_hillshade.tif`, `outdoorroad_dem_preview.png` — visual
  control,
* `outdoorroad_dem.pcd` — DEM cells as a point cloud in UTM metres,
* `outdoorroad_map_georef.pcd` — the whole FAST-LIO map in UTM metres,
* `outdoorroad_georef.json` — provenance: CRS, `R`, `t`, yaw, RMS, both
  lat/lon hypotheses, scale diagnostic, grid statistics.

Run again with `--origin-lat … --origin-lon …` when your GNSS topic holds a
**local metric** frame (`x=East, y=North` metres) instead of degrees: the
origin anchors it to a WGS84 position (and then `--epsg` is required if no
UTM zone can be derived).

## 6. Outputs

Everything lands in `out/`:

| file | produced by | content |
|---|---|---|
| `dem_<stamp>.npz` | `dem_node` `/dem_node/save` | local elevation grid (z, mask, params) |
| `odom.csv` | `record_odom` | FAST-LIO trajectory, one row per `/Odometry` |
| `map_fastlio.pcd` | FAST-LIO2 `/map_save` | registered map cloud, `camera_init` frame |
| `<prefix>_dem.tif` | `georef_export` | **georeferenced DEM** (GeoTIFF, EPSG auto) |
| `<prefix>_dem_hillshade.tif` | `georef_export` | hillshade rendering |
| `<prefix>_dem_preview.png` | `georef_export` | quick-look image with colour bar |
| `<prefix>_dem.pcd` | `georef_export` | DEM cells, UTM metres |
| `<prefix>_map_georef.pcd` | `georef_export` | FAST-LIO map, UTM metres |
| `<prefix>_georef.json` | `georef_export` | CRS + transform + RMS provenance |
| `logs/` | `run_pipeline.sh` | per-node logs, service calls, memory trace |

## 7. Configuration & tuning

### `ws/src/dem_bringup/config/dem.yaml` (DEM node)

| parameter | value used here | notes |
|---|---|---|
| `resolution` | `0.2` m | cell size |
| `size_x`, `size_y` | `400.0` m | 400×400 m covers the ~230 m GNSS extent; enlarge for longer trajectories |
| `origin_x/y` | `-200.0` | grid origin; shift if the trajectory leaves the grid (watch `/dem_node/trajectory`) |
| `ground_filter` | `height_band` | keep 6 m below … 4 m above the vehicle |
| `min_range` / `max_range` | `1.0` / `60.0` m | blind zone / max useful range |
| `save_dir` | `…/out` | where `~/save` writes; **absolute path** (node cwd is `~/.ros`) |

### `ws/src/dem_bringup/config/fastlio_mid360.yaml` (FAST-LIO2)

| parameter | value | why |
|---|---|---|
| `preprocess.lidar_type` | `4` | Mid-360 = `sensor_msgs/PointCloud2` in Livox format (1 = CustomMsg, 2 = Velodyne, 3 = Ouster) |
| `common.imu_topic` | `/mid360/livox/imu_si` | SI units from `imu_relay` |
| `mapping.extrinsic_est_en` | `false` | never estimate the factory Mid-360 IMU extrinsic online — it diverges (see error #14) |
| `mapping.acc_cov` / `gyr_cov` | `0.5` | this rig vibrates hard (`|acc|` p99 5.2 g); upstream 0.1 makes the EKF over-trust the IMU and drift (error #16) |
| `pcd_save.pcd_save_en` | `true` | gates the `/map_save` service (upstream shutdown-save is dead) |
| `publish.map_en` | `true` | accumulate the map cloud (needed by `/map_save`) |
| `publish.dense_publish_en` | `true` | full undistorted scans for the DEM |
| `map_file_path` | **absolute** | relative paths resolve against `~/.ros` |

`pcd_save_en`/`map_en` cost memory (the map cloud grows continuously); the
`publish_map()` patch ([§9](#9-errors-and-how-they-were-fixed)) makes the
per-second map serialization a no-op while nothing subscribes to
`/Laser_map`, which removes the large periodic copies.

**FAST-LIO diagnostics:** `ws/src/FAST_LIO/Log/mat_pre.txt` is rewritten on
**every run** (no flag needed) with the full filter state, one row per scan:

```
t  euler(3)  pos(3)  ext_euler(3)  offset_T_L_I(3)  vel(3)  bg(3)  ba(3)  grav(3)
```

Suspicious signatures: `ext_euler`/`offset_T` drifting away from the fixed
extrinsic (should stay ≈0° / ≈0.05 m — divergence here meant
`extrinsic_est_en` must be false), `|ba|` growing past ~1 m/s², `vel`
exceeding anything the vehicle can do, or `grav` tilting away from
`(0, 0, −9.81)`. `Log/fast_lio_time_log.csv` (only with
`runtime_pos_log_enable: true`) adds per-iteration timing/ikdtree stats.

### Playback

`--rate 0.5` for slow/paranoid runs, `2.0` for a fast pass; `--start N` to
begin N seconds into the bag (note: FAST-LIO's IMU initialisation
`MAX_INI_COUNT` needs the vehicle stationary at the start — keep the first
few seconds of the bag motionless or raise it in
`ws/src/FAST_LIO/include/utility.h`).

## 8. Running the stages manually

```bash
source /opt/ros/humble/setup.bash && source ws/install/setup.bash
OUT=/home/rajesh/proc_dem_creation/out && mkdir -p "$OUT"

# 1. trajectory recorder (terminal A)
ros2 run dem_replay record_odom -- --topic /Odometry --out "$OUT/odom.csv"

# 2. pipeline (terminal B)
ros2 launch dem_bringup pipeline.launch.py \
    bag:=/home/rajesh/proc_dem_creation/OutdoorRoad_dataset_ros2 \
    rate:=1.0 use_rviz:=false

# 3. when /replay/status says "finished" (terminal C):
ros2 topic echo /replay/status --once
ros2 service call /map_save   std_srvs/srv/Trigger {}
ros2 service call /dem_node/save std_srvs/srv/Trigger {}
# Ctrl-C the recorder in terminal A -> flushes odom.csv

# 4. georeference
ros2 run dem_mapping georef_export -- \
    --dem "$OUT"/dem_*.npz --odom "$OUT/odom.csv" \
    --bag /home/rajesh/proc_dem_creation/OutdoorRoad_dataset_ros2 \
    --gnss-topic /gnss_pose --map "$OUT/map_fastlio.pcd" \
    --out-dir "$OUT" --prefix outdoorroad
```

Replay control while running:

```bash
ros2 service call /replay/pause  std_srvs/srv/Trigger {}
ros2 service call /replay/resume std_srvs/srv/Trigger {}
ros2 topic pub --once /replay/rate std_msgs/msg/Float64 "{data: 2.0}"
```

## 9. Errors and how they were fixed

| # | Symptom / error | Cause | Fix |
|---|---|---|---|
| 1 | `colcon build` fails: `install/dem_replay ... File exists` / stale overlay entries after adding a new executable | leftover `build/` + `install/` dirs from before `--symlink-install`, mixing copy- and symlink-installed layouts | delete the affected package's `build/<pkg>` **and** `install/<pkg>`, rebuild: `rm -rf ws/build/fast_lio ws/install/fast_lio && colcon build --packages-select fast_lio` |
| 2 | `/map_save` returns failure / no `map_fastlio.pcd` | upstream FAST-LIO only saves on shutdown and only when `pcd_save_en` is true | set `pcd_save.pcd_save_en: true` **and** `publish.map_en: true` in `fastlio_mid360.yaml`; call the service explicitly |
| 3 | FAST-LIO diverges / "Initialize error" at replay start, or a systematically tilted trajectory (gravity init error) | IMU initialisation (`MAX_INI_COUNT`) averages only the first samples; the upstream default of **10 samples ≈ 0.05 s** is far too short for a vibrating platform (here it produced a ~1.5° tilted gravity estimate that the whole trajectory inherits) | keep the first seconds of the recording stationary (verified: static for ~4 s here) **and** raise `MAX_INI_COUNT` — this repo uses **400** (2 s @ 200 Hz) in `src/FAST_LIO/src/IMU_Processing.hpp` |
| 4 | FAST-LIO drifts immediately, gravity never converges | Livox driver records acceleration in **g**; the estimator expects m/s² | `imu_relay` node republishes the topic with `accel_scale 9.80665` and FAST-LIO subscribes to `/mid360/livox/imu_si` |
| 5 | No data reaches `dem_node` although topics publish | QoS mismatch: sensor data is `BEST_EFFORT`, subscribers requested `RELIABLE` (or vice versa) | use sensor-data `BEST_EFFORT` QoS for cloud/IMU subscriptions (`dem_node`, `stream_monitor`, `replay` publishers aligned) |
| 6 | FAST-LIO errors on the point cloud / empty preprocess | wrong `lidar_type`: default is Livox `CustomMsg`, the Mid-360 records `PointCloud2` | `preprocess.lidar_type: 4`, `timestamp_unit: 3` (ns), `scan_rate: 10` |
| 7 | Map never saved although the service exists | `map_file_path` was relative (`./test.pcd`) → resolved against the node's cwd `~/.ros` | use an **absolute** `map_file_path` in `fastlio_mid360.yaml` |
| 8 | `UserWarning: A NumPy version >=1.17.3 and <1.25.0 is required …` on import | Ubuntu's `python3-scipy` 1.8.0 built against numpy < 1.25, system numpy is 1.26.4 | benign — ignore, or `pip install --user 'scipy>=1.11'` if it ever becomes an error |
| 9 | Georeferencing impossible: `/gnss_pose` is not degrees | some datasets ship a **local metric** GNSS (`x=East, y=North`) | pass `--origin-lat/--origin-lon [--origin-alt]` to anchor the local frame to WGS84 (and `--epsg`) |
| 10 | `AttributeError: 'str' object has no attribute '__name__'` — `record_odom` dies at startup | `msgtype` passed to the log line is the type *string*, not the class | log `cls.__name__` (the class from `get_message()`); also handle `ExternalShutdownException` so SIGINT stops the node without a traceback |
| 11 | DEM ends up shifted against the GNSS track / RMS ≈ several m | (a) lat/lon order swapped, (b) trajectory and GNSS clock mismatch, (c) grid too small so part of the trajectory fell outside | (a) both orders are now auto-fitted and the winner reported; (b) `--tol` + RMS gate refuses bad fits; (c) 400×400 m grid covers the full ~230 m extent |
| 12 | `ros2` / `ros2 bag info` → `command not found` | environment not sourced | `source /opt/ros/humble/setup.bash && source ws/install/setup.bash` (rosbags are **not** on the default `PATH`) |
| 13 | Big RAM spikes / near-OOM on long recordings | FAST-LIO serialized the whole growing map into a `PointCloud2` every second even with no subscriber | patched `publish_map()` in `laserMapping.cpp`: `reserve()` up front and return early when `/Laser_map` has 0 subscribers (only the internal accumulation continues) |
| 14 | **FAST-LIO track diverges mid-run** (here: healthy for ~280 s, then velocity spikes to 14 m/s while GNSS says 1.2 m/s, z collapses by 200 m; the georef fit then correctly refuses with RMS ≫ 1 m) | `extrinsic_est_en: true` lets the EKF estimate the IMU↔LiDAR extrinsic online. On this (vibrating, slowly rotating) platform that quantity is unobservable: the estimate random-walks away from the factory value and drags accel bias + attitude with it until the whole state blows up | set `extrinsic_est_en: false` in `fastlio_mid360.yaml` (the Mid-360 built-in IMU extrinsic is factory-known and already listed as `extrinsic_T/R`). **Diagnosis:** `ws/src/FAST_LIO/Log/mat_pre.txt` is written *every run, unconditionally*, one row per scan: `t, euler(3), pos(3), ext_euler(3), offset_T(3), vel(3), bg(3), ba(3), grav(3)` — plot `ext_euler`/`vel` and the culprit state component is immediately visible (`ext_euler` grew 0° → 599° over the run while the truth is 0°) |
| 15 | `./run_pipeline.sh` hangs forever after the save services (observed: stuck 1.5 h between stage 5 and georeferencing) | (a) in non-interactive shells backgrounded commands **inherit SIGINT as ignored** (POSIX), so `ros2 run … &` swallowed `kill -INT $REC_PID`; (b) the idle recorder node was blocked in `rcl_wait()`, rmw retries on EINTR and the Python signal handler never ran — SIGINT is a no-op while a node has no traffic | `run_pipeline.sh` now has `stop_pid()` (bounded poll, escalates INT→TERM→KILL, never a bare `wait`) and `stop_pipeline_nodes()` (reaps any children `ros2 launch` left behind); `record_odom` spins with `timeout_sec=0.5` so it processes pending signals within half a second even when idle |
| 16 | Slow degradation over a long run: LIO-vs-GNSS RMS grows from 0.15 m (first minute) to metres after ~8 min; `mat_pre.txt` shows accel bias growing and the gravity estimate tilting while `ext_euler` stays 0 | IMU noise parameters model a much quieter sensor than this rig produces (raw `|acc|` p99 = 5.2 g of vibration): the EKF over-trusts the accelerometer and slowly trades bias + gravity tilt for position (z) drift | raise `mapping.acc_cov` / `gyr_cov` from the upstream `0.1` to `0.5` so the filter leans on the LiDAR; measure the residual drift with prefix fits (see §7) and document it |

Checklist when a run misbehaves: `out/logs/pipeline_*.log` (all node output),
`out/logs/record_odom.log`, `out/logs/services.log` (service responses),
`out/logs/mem_*.csv` (RAM trace), `ws/src/FAST_LIO/Log/mat_pre.txt` (full
FAST-LIO state history — see [§7](#7-configuration--tuning)), and
`ros2 run dem_replay inspect_bag -- <bag>` (topic rates, stamp monotonicity).

## 10. Switching the pipeline ROS 1 → ROS 2 (and back)

The dataset arrived as **ROS 1**; the pipeline is **ROS 2 (Humble)**. The
hinge is the bag — everything downstream is ROS 2 native.

**What changed, conceptually**

| layer | ROS 1 input | ROS 2 here |
|---|---|---|
| bag format | `OutdoorRoad_dataset.bag` (ROS 1 V2.0, 47 GB) | `OutdoorRoad_dataset_ros2/` (rosbag2 v8, sqlite3, 45 GB) |
| conversion | — | `ros2 run dem_replay bag_convert -- in.bag -o out_dir` (pure Python, `rosbags`; no ROS 1/bridge/Docker) |
| message formats | `Header.seq` exists, older field names | `seq` dropped, fields migrated where needed (done by the converter) |
| transport | topics + `rosbag play` | `dem_replay/replay` (timestamp-preserving, rate control, status topic) |
| nodes | `ros::NodeHandle`, catkin | `rclpy` nodes + `colcon` packages (`dem_replay`, `dem_mapping`, `dem_bringup`) |
| services | `std_srvs/Trigger` via `rosservice` | same srv type, `ros2 service call` |
| launch | `.launch` XML | `pipeline.launch.py` |
| QoS | queue size only | explicit reliability/history — must match publisher and subscriber |

**To bring up a *different* ROS 1 dataset**

1. convert: `ros2 run dem_replay bag_convert -- new.bag -o new_ros2`
   (add `--topics … --duration …` for a reduced bag),
2. inspect: `ros2 run dem_replay inspect_bag -- new_ros2` — check rates,
   stamp ordering, and whether the IMU is in g (`mean |acc| ≈ 1`) — if so,
   `imu_relay` handles it,
3. point the launch at it: `--bag new_ros2` or edit the default in
   `ws/src/dem_bringup/launch/pipeline.launch.py`,
4. fix topic names / `lidar_type` / extrinsics in
   `fastlio_mid360.yaml` and `dem.yaml` for the new sensor,
5. run `./run_pipeline.sh --bag … --prefix …`.

**To go back to ROS 1**: the original `.bag` is untouched, and the converter
reports per-topic byte counts on both sides; `rosbags` can also read the
converted data at any time (`--dry-run`, `inspect_bag`). Nothing in `ws/`
requires ROS 1 to be installed.

## 11. Development: build, test, layout

```bash
source /opt/ros/humble/setup.bash
cd /home/rajesh/proc_dem_creation/ws
colcon build --symlink-install                 # python packages: edits take effect without rebuild
colcon build --packages-select fast_lio        # C++ changes (laserMapping.cpp) need this
python3 -m pytest src/dem_mapping/test -q      # 11 tests (fits, regridding, PCD I/O, guards)
```

`dem_replay` and `dem_mapping` are installed with `--symlink-install`
(hard-linked into `build/`), so editing a `.py` file needs **no rebuild**;
editing FAST-LIO C++ does.

```
proc_dem_creation/
├── OutdoorRoad_dataset.bag          # source ROS 1 bag (47 GB)
├── OutdoorRoad_dataset_ros2/        # converted rosbag2 (45 GB)
├── run_pipeline.sh                  # one-command end-to-end runner
├── out/                             # all results (DEM, PCD, logs)
└── ws/
    ├── src/
    │   ├── FAST_LIO/                # FAST-LIO2 (C++, patched: /map_save + publish_map)
    │   ├── livox_ros_driver2/       # Livox driver (reference)
    │   ├── dem_bringup/             # launch + config (pipeline.launch.py, dem.yaml, fastlio_mid360.yaml)
    │   ├── dem_replay/              # bag_convert, inspect_bag, replay, imu_relay, record_odom, stream_monitor
    │   ├── dem_mapping/             # dem_node, georef_export, pcd_io, ground_filters, georef
    │   └── dem_viz/                 # optional live visualisation
    ├── build/  install/  log/
```

Key entry points:

| command | purpose |
|---|---|
| `ros2 run dem_replay bag_convert -- <bag> -o <dir>` | ROS 1 → ROS 2 conversion (+ validation) |
| `ros2 run dem_replay inspect_bag -- <bag>` | topic/rate/stamp report for either format |
| `ros2 run dem_replay record_odom -- --out odom.csv` | record the live LIO trajectory |
| `ros2 launch dem_bringup pipeline.launch.py` | replay + LIO + DEM |
| `ros2 run dem_mapping georef_export -- --dem … --odom … --bag …` | local → WGS84/UTM export |
| `./run_pipeline.sh` | all of the above, end to end |

---

### Reference data for this dataset

* recording: 657.3 s, 432 794 messages, 7 topics (Mid-360 + Avia + Ouster + GNSS)
* GNSS: `/gnss_pose`, `PoseStamped`, 100 Hz, WGS84 degrees → Turku/Finland,
  UTM zone 34N → **EPSG:32634**
* lidar used: Mid-360, 10 Hz, 19 968 pts/scan; IMU 200 Hz (g units)
* pipeline: rate 1.0, DEM 400×400 m @ 0.2 m, run time ≈ 13 min on 8 cores
