# Procedural DEM creation from a ROS 1 bag (Melodic, FAST-LIO2, georeferenced output)

Converts a recorded road dataset (ROS 1 `.bag`) into a **georeferenced**
elevation model (WGS84 / UTM) as GeoTIFF + PCD point clouds, with one command:

```
recording.bag                 (ROS 1, topics: Mid-360 LiDAR + IMU + GNSS pose)
        │  ./run_pipeline.sh
        ├──► replay ─► imu_relay ─► FAST-LIO2 ─► dem_node ─► DEM grid (.npz)
        │                 │             │            │
        │                 │             └─ /Odometry ─► record_odom (CSV)
        │                 │             └─ /map_save ─► map_fastlio.pcd (optional)
        ▼
georef_export  (GNSS track ⟷ LIO trajectory fit)
        │
        ▼
out/outdoorroad_dem.tif          (+ hillshade, preview PNG)
out/outdoorroad_dem.pcd          (DEM cells in UTM metres)
out/outdoorroad_map_georef.pcd   (FAST-LIO map in UTM metres)
out/outdoorroad_georef.json      (CRS, transform, RMS provenance)
```

The whole toolchain is **ROS 1 Melodic**: `rospy` nodes + `catkin` packages,
`pipeline.launch` (XML), `rosrun` / `roslaunch` / `rosservice`, and bag I/O
through the Melodic `rosbag` Python module. No ROS 2 component is required.

---

## Contents

1. [Requirements](#1-requirements)
2. [Build the workspace](#2-build-the-workspace)
3. [Quick start](#3-quick-start)
4. [The pipeline, stage by stage](#4-the-pipeline-stage-by-stage)
5. [Georeferencing — how the DEM gets real coordinates](#5-georeferencing--how-the-dem-gets-real-coordinates)
6. [Outputs](#6-outputs)
7. [Configuration & tuning](#7-configuration--tuning)
8. [Running the stages manually](#8-running-the-stages-manually)
9. [Errors and how they were fixed](#9-errors-and-how-they-were-fixed)
10. [Using a different recording](#10-using-a-different-recording)
11. [Development: layout and tests](#11-development-layout-and-tests)

---

## 1. Requirements

* Ubuntu 18.04 + **ROS 1 Melodic** (`/opt/ros/melodic`)
* Python 2.7 with the ROS packages (`rospy`, `rosbag`, `roslib`) and:

  ```bash
  sudo apt install python-numpy python-scipy python-matplotlib python-gdal
  ```

  | package     | used for                                | Melodic build |
  |-------------|-----------------------------------------|---------------|
  | `rospy`     | every node                              | 1.14.x        |
  | `rosbag`    | replay, GNSS read-back, bag inspection  | 1.14.x        |
  | `numpy`     | grids, fits                             | 1.13+         |
  | `scipy`     | `cKDTree` reprojection, KD-tree lookups | 1.x           |
  | `matplotlib`| preview / hillshade PNG, live heatmap   | 2.x/3.x       |
  | GDAL (`osgeo`) | GeoTIFF write/read                   | 2.x/3.x       |

* ~8 GB RAM recommended (the FAST-LIO map cloud grows to several hundred MB;
  peak RSS observed ≈ 4–5 GB) and enough disk for the outputs.
* **FAST-LIO2** (ROS 1) and **livox_ros_driver** (ROS 1, for the `CustomMsg`
  header FAST-LIO includes) in `ws/src` — see §2. They are *not* vendored here.
* Optional: `rviz` for `--rviz`.

Environment for every command below:

```bash
source /opt/ros/melodic/setup.bash
source <repo>/ws/devel/setup.bash
```

## 2. Build the workspace

```bash
cd <repo>/ws

# LIO + Livox message definitions (ROS 1), once.  --recursive pulls the
# ikd-Tree submodule FAST-LIO links against.
git clone --recursive https://github.com/hku-mars/FAST_LIO.git src/FAST_LIO
git clone https://github.com/Livox-SDK/livox_ros_driver.git src/livox_ros_driver

# FAST-LIO is C++ and needs PCL, Eigen and the Python 2 headers:
sudo apt install ros-melodic-pcl-ros ros-melodic-eigen-conversions \
    libeigen3-dev libpcl-dev python-dev

catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

`livox_ros_driver` is only needed for its `CustomMsg` definition (FAST-LIO
includes that header); building its driver node additionally needs the
[Livox SDK](https://github.com/Livox-SDK/Livox-SDK). The bag already contains
sensor data, so the driver itself is not used by this pipeline.

The repository's own packages (`dem_replay`, `dem_mapping`, `dem_bringup`,
`dem_viz`) build with `catkin_make`/`catkin build` and are plain Python, so
editing them needs no rebuild (they are installed from source paths; rerun
`catkin_make` only after changing a `CMakeLists.txt`, `package.xml` or adding
an executable).

`ws/src/CMakeLists.txt` is the standard catkin `toplevel.cmake` symlink that
`catkin_make` expects. `dem_bringup/config/fastlio_mid360.yaml` targets the
upstream ROS 1 FAST-LIO parameter layout (`common/lid_topic`,
`mapping/acc_cov`, `pcd_save/pcd_save_en`, …), which `pipeline.launch` loads
into the **global** namespace exactly like FAST-LIO's own launch file.

> **`/map_save` is optional.** Upstream FAST-LIO only writes its PCD on
> shutdown; some builds add a `/map_save` service. `run_pipeline.sh` calls it
> when it exists and otherwise skips `map_fastlio.pcd`. DEM generation and
> georeferencing use `/cloud_registered`, `/Odometry` and `/gnss_pose`, so
> they work with a stock build.

## 3. Quick start

```bash
./run_pipeline.sh --bag /path/to/recording.bag
```

Useful flags of `run_pipeline.sh`:

```
--bag PATH      ROS 1 .bag to replay      (default <repo>/OutdoorRoad_dataset.bag)
--out DIR       output directory          (default <repo>/out)
--prefix NAME   output file prefix        (default outdoorroad)
--rate N        playback speed            (default 1.0)
--start N       seconds into the bag      (default 0)
--rviz          also open RViz
--no-georef     stop after the local DEM/PCD (skip step 6)
```

All node output is logged under `out/logs/`; a memory sampler writes
`out/logs/mem_*.csv` every 10 s so you can inspect the peak RAM of the run.

Check a recording before running the pipeline:

```bash
rosrun dem_replay inspect_bag /path/to/recording.bag   # rates, stamps, fields, samples
rosbag info /path/to/recording.bag                     # stock view
```

## 4. The pipeline, stage by stage

`./run_pipeline.sh` runs six stages; each can also be run by hand
([§8](#8-running-the-stages-manually)).

1. **`record_odom`** (`dem_replay`) — subscribes to `/Odometry` (FAST-LIO's
   live trajectory) and appends `t,x,y,z,qx,qy,qz,qw` to `out/odom.csv`.
   The trajectory is computed *live* during replay and is **not** in the bag,
   so someone has to write it down for the georeferencing step. `t` is the
   header stamp (bag epoch time) so it matches the GNSS track timestamps.
2. **`pipeline.launch`** (`dem_bringup`) starts:
   * `replay` (`dem_replay`) — timestamp-preserving bag player with rate
     control, `/replay/status` JSON and pause/resume/restart services
     (`player:=stock` uses plain `rosbag play` instead),
   * `imu_relay` — Livox IMU accelerations are recorded in **g**; FAST-LIO
     needs m/s², so `/mid360/livox/imu` is republished as
     `/mid360/livox/imu_si` scaled ×9.80665 (raw topic untouched),
   * `fastlio_mapping` (FAST-LIO2, C++) — LiDAR-inertial odometry,
     publishes `/Odometry`, `/cloud_registered`, and accumulates the map
     cloud,
   * `dem_node` (`dem_mapping`) — incremental elevation grid from the
     registered clouds (`mode: registered`, output frame `camera_init`),
   * optional: `stream_monitor` (ordering/synchronisation watchdog),
     `rviz`, matplotlib live heatmap (`use_viz:=true`).
3. **wait** until `/replay/status` reports `"state": "finished"`.
4. **`/map_save`** (`std_srvs/Trigger`, when the build offers it) →
   `out/map_fastlio.pcd`.
5. **`/dem_node/save`** (`std_srvs/Trigger`) → `out/dem_<stamp>.npz`
   (`dem_node` also saves on shutdown when `~save_dir` is non-empty).
6. **`georef_export`** (`dem_mapping`) — the georeferencing step, see below.

## 5. Georeferencing — how the DEM gets real coordinates

### The problem

FAST-LIO2 and `dem_node` work in the **local mapper frame** `camera_init`:
metres, but an arbitrary origin and an arbitrary yaw (whatever direction the
vehicle happened to start in). A DEM in that frame cannot be put on a map,
compared with satellite imagery, or stored as a GeoTIFF. The `/gnss_pose`
topic in the bag *is* geographic (WGS84 lat/lon in degrees, e.g. ~60.44° N /
22.31° E — Turku, Finland; UTM zone 34N), so the two must be tied together.

### The method (`dem_mapping/georef_export.py`)

1. Read `/gnss_pose` from the ROS 1 `.bag` (Melodic `rosbag`).
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
     (`--epsg` forces a code, e.g. `--epsg 32634` for zone 34N).
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
* `outdoorroad_map_georef.pcd` — FAST-LIO map in UTM metres (when the map PCD
  exists),
* `outdoorroad_georef.json` — provenance: CRS, `R`, `t`, yaw, RMS, both
  lat/lon hypotheses, scale diagnostic, grid statistics.

Run again with `--origin-lat … --origin-lon …` when your GNSS topic holds a
**local metric** frame (`x=East, y=North` metres) instead of degrees: the
origin anchors it to a WGS84 position (and then `--epsg` is required if no
UTM zone can be derived).

If the LIO trajectory drifts (RMS too large) there are two options:

* `dem_mapping gnss_warp` fits a **time-parameterised** alignment (sliding
  windows) and writes `warped_odom.csv`, then `dem_replay pose_play`
  republishes that track while `dem_node` runs in `mode: pose`, so the grid
  is built directly in UTM; `georef_export --warp-report …` documents it.
* otherwise accept a local DEM only (omit step 6).

## 6. Outputs

Everything lands in `out/`:

| file | produced by | content |
|---|---|---|
| `dem_<stamp>.npz` | `dem_node` `/dem_node/save` | local elevation grid (z, mask, params) |
| `odom.csv` | `record_odom` | FAST-LIO trajectory, one row per `/Odometry` |
| `map_fastlio.pcd` | FAST-LIO `/map_save` (optional) | registered map cloud, `camera_init` frame |
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
| `save_dir` | passed by the launch file | absolute path; a relative one resolves against the node cwd (`~/.ros`) |

### `ws/src/dem_bringup/config/fastlio_mid360.yaml` (FAST-LIO2)

| parameter | value | why |
|---|---|---|
| `preprocess.lidar_type` | `4` | Mid-360 = `sensor_msgs/PointCloud2` in Livox format (1 = CustomMsg, 2 = Velodyne, 3 = Ouster) |
| `common.imu_topic` | `/mid360/livox/imu_si` | SI units from `imu_relay` |
| `mapping.extrinsic_est_en` | `false` | never estimate the factory Mid-360 IMU extrinsic online — it diverges (see error #9) |
| `mapping.acc_cov` / `gyr_cov` | `0.5` | this rig vibrates hard (`|acc|` p99 5.2 g); upstream 0.1 makes the EKF over-trust the IMU and drift (error #10) |
| `pcd_save.pcd_save_en` | `true` | enables map saving (`/map_save` where the build supports it) |
| `publish.map_en` | `true` | accumulate the map cloud |
| `publish.dense_publish_en` | `true` | full undistorted scans for the DEM |
| `map_file_path` | absolute (set by the launch file) | relative paths resolve against `~/.ros` |

`pcd_save_en`/`map_en` cost memory (the map cloud grows continuously).

**FAST-LIO diagnostics:** upstream FAST-LIO's runtime log
(`Log/mat_pre.txt`, written when `runtime_pos_log_enable: true`) has one row
per scan with the full filter state:

```
t  euler(3)  pos(3)  ext_euler(3)  offset_T_L_I(3)  vel(3)  bg(3)  ba(3)  grav(3)
```

Suspicious signatures: `ext_euler`/`offset_T` drifting away from the fixed
extrinsic (should stay ≈0° / ≈0.05 m), `|ba|` growing past ~1 m/s², `vel`
exceeding anything the vehicle can do, or `grav` tilting away from
`(0, 0, −9.81)`.

### Playback

`--rate 0.5` for slow/paranoid runs, `2.0` for a fast pass; `--start N` to
begin N seconds into the bag (note: FAST-LIO's IMU initialisation
`MAX_INI_COUNT` needs the vehicle stationary at the start — keep the first
few seconds of the bag motionless or raise it in FAST-LIO's
`IMU_Processing.hpp`).

## 8. Running the stages manually

```bash
source /opt/ros/melodic/setup.bash && source ws/devel/setup.bash
OUT=/path/to/repo/out && mkdir -p "$OUT"

# 1. trajectory recorder (terminal A)
rosrun dem_replay record_odom --topic /Odometry --out "$OUT/odom.csv"

# 2. pipeline (terminal B; starts a roscore if none is running)
roslaunch dem_bringup pipeline.launch \
    bag:=/path/to/recording.bag rate:=1.0 use_rviz:=false \
    save_dir:="$OUT"

# 3. when /replay/status says "finished" (terminal C):
rostopic echo -n 1 /replay/status
rosservice call /map_save            # optional (if offered)
rosservice call /dem_node/save
# Ctrl-C the recorder in terminal A -> flushes odom.csv

# 4. georeference
rosrun dem_mapping georef_export \
    --dem "$OUT"/dem_*.npz --odom "$OUT/odom.csv" \
    --bag /path/to/recording.bag \
    --gnss-topic /gnss_pose --map "$OUT/map_fastlio.pcd" \
    --out-dir "$OUT" --prefix outdoorroad
```

Replay control while running:

```bash
rosservice call /replay/pause
rosservice call /replay/resume
rostopic pub -1 /replay/rate std_msgs/Float64 "data: 2.0"
```

## 9. Errors and how they were fixed

| # | Symptom / error | Cause | Fix |
|---|---|---|---|
| 1 | `/map_save` returns failure / no `map_fastlio.pcd` | stock FAST-LIO only saves on shutdown | call the service only if `rosservice list` shows `/map_save`; the DEM and georeference do not need the PCD |
| 2 | FAST-LIO diverges / "Initialize error" at replay start, or a systematically tilted trajectory | IMU initialisation (`MAX_INI_COUNT`) averages only the first samples; the upstream default of 10 samples ≈ 0.05 s is far too short for a vibrating platform (here it produced a ~1.5° tilted gravity estimate that the whole trajectory inherits) | keep the first seconds of the recording stationary and raise `MAX_INI_COUNT` (this rig uses 400 ≈ 2 s @ 200 Hz in FAST-LIO's `IMU_Processing.hpp`) |
| 3 | FAST-LIO drifts immediately, gravity never converges | Livox driver records acceleration in **g**; the estimator expects m/s² | `imu_relay` republishes the topic with `accel_scale 9.80665`; FAST-LIO subscribes to `/mid360/livox/imu_si` |
| 4 | FAST-LIO errors on the point cloud / empty preprocess | wrong `lidar_type`: default is Livox `CustomMsg`, the Mid-360 records `PointCloud2` | `preprocess.lidar_type: 4`, `timestamp_unit: 3` (ns), `scan_rate: 10` |
| 5 | Nodes miss messages although topics publish | subscriber `queue_size` too small for 10 Hz clouds + 200 Hz IMU | sensor subscriptions use `queue_size` 100–1000 (`dem_node`, `stream_monitor`, replay publishers) |
| 6 | Map saved to an unexpected place | `map_file_path` was relative (`./test.pcd`) → resolved against the node cwd `~/.ros` | `pipeline.launch` sets an absolute `map_file_path` from `save_dir` |
| 7 | Georeferencing impossible: `/gnss_pose` is not degrees | some datasets ship a **local metric** GNSS (`x=East, y=North`) | pass `--origin-lat/--origin-lon [--origin-alt]` to anchor the local frame to WGS84 (and `--epsg`) |
| 8 | DEM ends up shifted against the GNSS track / RMS ≈ several m | (a) lat/lon order swapped, (b) trajectory and GNSS clock mismatch, (c) grid too small so part of the trajectory fell outside | (a) both orders are auto-fitted and the winner reported; (b) `--tol` + RMS gate refuses bad fits; (c) enlarge the grid in `dem.yaml` |
| 9 | **FAST-LIO track diverges mid-run** (healthy for ~280 s, then velocity spikes to 14 m/s while GNSS says 1.2 m/s, z collapses) | `extrinsic_est_en: true` lets the EKF estimate the IMU↔LiDAR extrinsic online; unobservable on this slowly rotating, vibrating platform, so it random-walks away and drags accel bias + attitude with it | `extrinsic_est_en: false` (the Mid-360 IMU extrinsic is factory-known and listed as `extrinsic_T/R`) |
| 10 | Slow degradation over a long run: LIO-vs-GNSS RMS grows from 0.15 m to metres after ~8 min | IMU noise parameters model a much quieter sensor than this rig produces (raw `|acc|` p99 = 5.2 g of vibration) | raise `mapping.acc_cov` / `gyr_cov` from 0.1 to 0.5 so the filter leans on the LiDAR |
| 11 | `./run_pipeline.sh` hangs after the save services | in non-interactive shells backgrounded commands inherit SIGINT as ignored, and an idle `rospy.spin()` node processes no signals | the script stops nodes by name (`rosnode kill`), uses bounded INT→TERM→KILL escalation and never a bare `wait` |
| 12 | `rosrun` / `roslaunch` → command not found | environment not sourced | `source /opt/ros/melodic/setup.bash && source ws/devel/setup.bash` |
| 13 | Big RAM spikes / near-OOM | FAST-LIO accumulates the whole map (`map_en`) | lower `publish.map_en` / `pcd_save.pcd_save_en` when the map PCD is not needed |
| 14 | `rosbag.Bag` / `import rospy` fails under `python3` | Melodic ships `rospy`/`rosbag` for Python **2.7** only | run nodes with `rosrun`/`roslaunch` (they use `#!/usr/bin/env python`); use Python 3 only for the ROS-free snapshot service |

Checklist when a run misbehaves: `out/logs/pipeline_*.log` (all node output),
`out/logs/record_odom.log`, `out/logs/services.log` (service responses),
`out/logs/mem_*.csv` (RAM trace), FAST-LIO's `Log/` files, and
`rosrun dem_replay inspect_bag <bag>` (topic rates, stamp monotonicity).

## 10. Using a different recording

1. inspect it: `rosrun dem_replay inspect_bag new.bag` — check rates, stamp
   ordering, and whether the IMU is in g (`mean |acc| ≈ 1`); if so, `imu_relay`
   handles it,
2. fix topic names / `lidar_type` / extrinsics in `fastlio_mid360.yaml` and
   `dem.yaml` for the new sensor,
3. run `./run_pipeline.sh --bag new.bag --prefix newname`.

The launch arguments `lidar_topic`, `imu_topic`, `gnss_topic` let you remap
without editing files:

```bash
roslaunch dem_bringup pipeline.launch bag:=new.bag \
    lidar_topic:=/livox/lidar imu_topic:=/livox/imu
```

## 11. Development: layout and tests

```bash
source /opt/ros/melodic/setup.bash
cd ws && catkin_make
python -m pytest src/dem_mapping/test -q      # fits, regridding, PCD I/O, guards
```

The Python packages are installed from `ws/src`, so editing a `.py` file takes
effect without a rebuild; editing FAST-LIO C++ needs `catkin_make`.

```
<repo>/
├── recording.bag                    # source ROS 1 bag
├── run_pipeline.sh                  # one-command end-to-end runner
├── out/                             # all results (DEM, PCD, logs)
└── ws/
    ├── src/
    │   ├── FAST_LIO/                # FAST-LIO2 (cloned; upstream ROS 1)
    │   ├── livox_ros_driver/        # Livox messages (cloned; upstream ROS 1)
    │   ├── dem_bringup/             # launch + config (pipeline.launch, dem.yaml, fastlio_mid360.yaml)
    │   ├── dem_replay/              # replay, imu_relay, record_odom, stream_monitor, inspect_bag, pose_play
    │   ├── dem_mapping/             # dem_node, georef_export, gnss_warp, gnss_align, pcd_io, ground_filters, georef
    │   └── dem_viz/                 # optional live visualisation
    └── build/  devel/  log/
```

Key entry points:

| command | purpose |
|---|---|
| `rosrun dem_replay inspect_bag <bag>` | topic/rate/stamp report |
| `rosrun dem_replay replay _bag_path:=<bag>` | timestamp-preserving replay alone |
| `rosrun dem_replay record_odom --out odom.csv` | record the live LIO trajectory |
| `roslaunch dem_bringup pipeline.launch bag:=…` | replay + LIO + DEM |
| `rosrun dem_mapping georef_export --dem … --odom … --bag …` | local → WGS84/UTM export |
| `./run_pipeline.sh` | all of the above, end to end |

---

### Reference data for this dataset

* recording: 657.3 s, 432 794 messages, 7 topics (Mid-360 + Avia + Ouster + GNSS)
* GNSS: `/gnss_pose`, `PoseStamped`, 100 Hz, WGS84 degrees → Turku/Finland,
  UTM zone 34N → **EPSG:32634**
* lidar used: Mid-360, 10 Hz, 19 968 pts/scan; IMU 200 Hz (g units)
* pipeline: rate 1.0, DEM 400×400 m @ 0.2 m, run time ≈ 13 min on 8 cores
