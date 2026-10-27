# Practical Assessment: Replay Pipeline → Real-Time Drone Mapping

Answers to the six question groups, grounded in the actual source in this
repository. Where a statement is derived from code it is cited as
`file:line`. Where something is *inferred* rather than verified it says so.

Two reading notes:

* **`.db3` vs `.bag`.** The *original* input is a ROS 1 `OutdoorRoad_dataset.bag`
  (47 GB). It is converted once to a rosbag2 directory (bagfile v8, one `.db3`
  inside). Everything downstream reads the rosbag2 form. So the "replay from
  `.db3`" description is correct for the normal path, but the repo also still
  supports reading the ROS 1 `.bag` directly (used by `bag_convert` and
  `georef_export --bag`).
* **Blockers in this checkout.** Several things needed for a live build are
  *not present in this working copy*. They are collected in
  [Appendix A](#appendix-a-blockers-found-in-this-checkout) because they affect
  the answers to §3 and §6 directly.

---

## 1. Understanding the Current Data Processing Pipeline

### 1.1 How is "real-time" simulated?

Not by `ros2 bag play` — by a custom node, `dem_replay/replay`
(`ws/src/dem_replay/dem_replay/replay_node.py`), which fakes a live sensor
publisher:

1. **One merged, time-ordered pass.** A single
   `rosbag2_py.SequentialReader` is opened with a `StorageFilter` over the
   selected topics (`replay_node.py:117-131`). rosbag2's sequential reader
   already returns messages interleaved across topics in stored-timestamp
   order, so there is *no per-topic chunking* — LiDAR, IMU and GNSS come out
   in the exact order they were recorded. The docstring is explicit about
   this: "one merged, time-ordered pass over every selected topic (never
   per-topic chunking)" (`replay_node.py:6-8`).

2. **Wall-clock pacing.** The read loop pulls exactly **one message**, then
   sleeps until real time matches recorded time scaled by `rate`
   (`replay_node.py:295-338`):

   ```
   target = base_wall + (bag_ts - t0_bag) / rate
   sleep(min(0.02, target - now))
   ```

   At `rate=1.0` the bag is consumed in real time (657.3 s of recording ≈
   657.3 s wall). `rate` can be changed at runtime via `/replay/rate`.

3. **Original timestamps preserved verbatim.** The message is deserialized
   and published unmodified (`replay_node.py:306-307`), so
   `header.stamp` still carries the recording epoch. Downstream nodes key all
   synchronisation off those stamps, not off arrival time.

4. **Optional `/clock`.** Every message also publishes a `rosgraph_msgs/Clock`
   (`replay_node.py:312-317`), so `use_sim_time` consumers (and `pose_play`)
   can pace themselves against bag time.

5. **Control surface.** `/replay/{pause,resume,restart,toggle}` services, a
   `/replay/status` JSON topic (5 Hz), and `/replay/rate`. `run_pipeline.sh`
   polls `/replay/status` for `"state": "finished"`.

**QoS:** the replay publishers use `RELIABLE / KEEP_LAST / depth=100`
(`replay_node.py:86-88`).

### 1.2 Per-message, or accumulated into chunks?

**Per-message, strictly.** There is no accumulation stage anywhere:

* Replay publishes one message at a time.
* `dem_node` does all of its work inside a single subscription callback
  (`dem_node.py:263-324`). One `PointCloud2` message in → one grid update out.
* For this dataset one LiDAR message **is one complete 10 Hz scan**
  (~19,968 points, Mid-360). So "one message" ≈ "one 100 ms sweep of the
  sensor" — but that is a property of the sensor, not a configured chunk size.
* FAST-LIO2 likewise consumes one scan message per iteration.

There is **no time-based batching** (no "accumulate 100 ms then process") and
**no count-based batching** (no "process 10 scans together") in the code.

### 1.3 Main stages: sensor message → point cloud + DEM

```
┌ replay ──────────────────────────────────────────────────────────────┐
│ rosbag2 sequential read (merged order)                               │
│   → wall-clock pacing → publish with original header.stamp           │
│   → /clock                                                          │
└──────────────────────────────────────────────────────────────────────┘
        │ /mid360/livox/lidar (PointCloud2, 10 Hz)
        │ /mid360/livox/imu   (Imu, 200 Hz, units = g)
        │ /gnss_pose          (PoseStamped, 100 Hz, WGS84 degrees)
        ▼
┌ imu_relay ───────────────────────────────────────────────────────────┐
│ accel × 9.80665 → /mid360/livox/imu_si   (raw topic untouched)       │
│ (Livox drivers record acceleration in g; FAST-LIO expects m/s²)      │
└──────────────────────────────────────────────────────────────────────┘
        ▼
┌ FAST-LIO2 (C++, fastlio_mapping) ────────────────────────────────────┐
│ preprocess (lidar_type 4 = PointCloud2/Livox format, blind 0.5 m)    │
│   → per-point deskew using IMU                                        │
│   → EKF (iterated error-state) + ikd-tree local map                  │
│   → publishes /Odometry, /cloud_registered (points in camera_init)    │
│   → accumulates full map for /map_save                                │
└──────────────────────────────────────────────────────────────────────┘
        │ /Odometry                     │ /cloud_registered
        ▼                               ▼
┌ record_odom (side channel)      ┌ dem_node ──────────────────────────┐
│ append t,x,y,z,qx,qy,qz,qw      │ 1. pointcloud2_to_xyz  (parse)     │
│ → out/odom.csv                  │ 2. transform to map frame          │
│ (NOT in the bag — computed       │    ('registered': already done,    │
│  live, needed by georeferencing) │     'pose': PoseBuffer lookup)     │
└─────────────────────────────────┘ 3. radial_range filter             │
                                     │ 4. height_band / z_range filter  │
                                     │ 5. max_points subsample          │
                                     │ 6. ElevationGrid.update (in place│
                                     │    NumPy bincount / minimum.at)   │
                                     │ → elevation_image, elevation_     │
                                     │   cloud, occupancy, trajectory,   │
                                     │   stats; ~/save → .npz            │
                                     └───────────────────────────────────┘
```

Two separate artifacts are produced and they are *not* the same thing:

| artifact | producer | nature |
|---|---|---|
| **3D point cloud** | FAST-LIO2 (`/map_save` → `out/map_fastlio.pcd`) | unstructured, full 3D, undistorted, `camera_init` frame |
| **DEM grid** | `dem_node` (`~/save` → `out/dem_*.npz`) | structured 2.5D raster, `min` or `mean` z per cell |

They are built independently from the same `/cloud_registered` stream.

### 1.4 Post-processing (offline, after replay)

`run_pipeline.sh` then calls `georef_export`, which fits the LIO trajectory
(`odom.csv`) to the GNSS track and re-projects every grid cell into UTM. See
§2.6 for how GNSS actually enters the system (spoiler: **offline only**).

---

## 2. Buffering, Queues, and Chunk-Based Processing

### 2.1 Is there a buffer/queue for incoming LiDAR?

There are **three distinct buffering layers**, and it matters which one you
mean:

**(a) DDS / ROS 2 QoS queues — yes, these exist and are the real queue.**

| endpoint | QoS | file |
|---|---|---|
| `replay` publishers | `RELIABLE`, `KEEP_LAST`, **depth = 100** | `replay_node.py:86-88` |
| `dem_node` cloud subscription | **`BEST_EFFORT`**, `KEEP_LAST`, **depth = 10** | `dem_node.py:182-191` |
| `dem_node` pose subscription | `RELIABLE`, `KEEP_LAST`, depth = 10 | `dem_node.py:185-187` |

So the LiDAR queue that actually exists is a **bounded depth-10 BEST_EFFORT
history** in `dem_node`. Ten scans at 10 Hz = **100 ms of slack**. The
producer side has a deeper 100-message history it can hold.

**(b) Application-level queue inside `dem_node` — none.**

The callback processes the message synchronously and returns. There is no
internal work queue, no worker thread doing deferred grid updates, no
double-buffering of scans. `MultiThreadedExecutor(num_threads=4)`
(`dem_node.py:509`) only lets *different* callbacks run concurrently — the
sensor callbacks themselves live in one `MutuallyExclusiveCallbackGroup`
(`dem_node.py:178`), so **two clouds are never processed at once**.

**(c) `PoseBuffer` — a bounded time-indexed pose history, not a work queue.**

`pose_buffer.py:67`:

```python
def __init__(self, max_span_s=10.0, max_size=200_000):
```

`dem_node` constructs it with `max_span_s=20.0` (`dem_node.py:157`). It is
bounded on **both** dimensions: 20 seconds of history, at most 200,000 poses;
whichever bound trips first trims it (`pose_buffer.py:94-110`). It stores
poses only — no point data. Out-of-order poses are **dropped**
(`pose_buffer.py:84-85`, returns `False` if `t <= t[-1]`).

### 2.2 How much data per iteration?

**One LiDAR message = one scan.** No exceptions, no configuration.

Within that single iteration:

| stage | data volume | configured by |
|---|---|---|
| input scan | ~19,968 points (Mid-360, 10 Hz) | sensor |
| after `radial_range` | 1–60 m from sensor origin | `min_range`, `max_range` |
| after `height_band` | pose_z −6 m … pose_z +4 m | `band_below`, `band_above` |
| after `max_points` | **≤ 12,000 points** (random subsample) | `max_points_per_scan` |
| grid cells touched | small subset of 4,000,000 | `resolution`, `size_x/y` |

All in `dem.yaml` (and `dem_utm.yaml`). The final cap —
`max_points_per_scan: 12000` — is explicitly documented as "a CPU budget
guard, not a ground classifier" (`ground_filters.py:99-100`).

### 2.3 Is chunk size configurable?

**There is no chunk size to configure**, because there is no chunking. The
configurable knobs are all *per-scan volume* knobs:

* `max_points_per_scan` — the hard CPU budget per scan (`dem.yaml:39`)
* `min_range` / `max_range` — radial gate (`dem.yaml:37-38`)
* `band_below` / `band_above` — vertical gate (`dem.yaml:33-34`)
* `resolution`, `size_x`, `size_y`, `origin_x/y` — grid geometry
* publish rates `image_rate`, `cloud_rate`, `occupancy_rate`, `stats_rate`

If you wanted time-based chunks (e.g. "accumulate 100 ms, then merge"), that
would be a **new feature**, not a parameter change. It would go in
`DemNode._cloud_cb` — collect messages until `scan_t - window_start >= 0.1`,
concatenate, then run stages 3–6 once. The grid update is already vectorised
over an `(N,3)` array, so it would accept a concatenated batch with no change
to `ElevationGrid`.

### 2.4 What happens if data arrives faster than processing?

**Messages are silently dropped at the DDS layer.** Reasoning from the code:

* The subscription is `BEST_EFFORT / KEEP_LAST / depth=10`
  (`dem_node.py:182-184`). With `BEST_EFFORT`, if the subscriber's history is
  full when a new sample arrives, **the oldest is discarded** and no error is
  raised.
* There is **no backpressure path** to `replay`: the replay node does not
  know or care whether anyone consumed a message. It paces purely against
  wall clock (`replay_node.py:319-338`).
* There is **no drop counter for QoS-level loss.** `dem_node` tracks
  `clouds_dropped_no_pose` (`dem_node.py:284, 459`) — that is a *different*
  failure (no pose available) — but a scan dropped by DDS is invisible to
  the statistics.

**What does get measured** is in `~/stats` (`dem_node.py:456-467`):
`clouds_received`, `input_hz` (arrival rate), and per-stage
`processing_ms` with `last / mean / p95`. That is the signal you would watch
to detect overload: if `input_hz` falls below 10 while the bag is running at
1.0×, you are losing scans.

**Mitigations already present:**

* `MaxPointsFilter` caps per-scan cost at 12,000 points regardless of input.
* Heavy publishers (`elevation_image`, `elevation_cloud`, `occupancy`) are in
  a *separate* mutually-exclusive callback group from sensor callbacks
  (`dem_node.py:177-180`), and the comment explains why: the 16 MB image
  serialisation "must never delay incoming scans, otherwise clouds are
  dropped by QoS".
* Grid snapshot state is copied under lock and evaluated/serialised *outside*
  it (`dem_node.py:331-334`, `elevation_grid.py:131-186`).

**Evidence from the recorded run** (`out/logs/pipeline_20261015_163037.log`,
end of file): at `rate=1.0` the monitor reported exactly
`lidar n=6573 @ 10.00 Hz`, `imu n=131462 @ 200.00 Hz`,
`gnss n=65732 @ 100.00 Hz`, each `monotonic=OK` over 657.2–657.3 s. Those
counts match the bag exactly, so **at 1.0× replay on that machine nothing was
lost**. Note this is a paced producer, so it is not a proof of headroom for
an unpaced live sensor — see §3.5.

Also from that log: `global order check: FAIL (max inversion 786.747 ms
within 250.0 ms tolerance)` — cross-topic interleave ordering has large
inversions. Per-topic monotonicity is fine, and pairwise sync is good
(lidar↔imu median |Δt| = **1.71 ms**, lidar↔gnss median |Δt| = **0.64 ms**),
but you should not assume globally ordered delivery across topics.

### 2.5 How are LiDAR and GPS timestamped / how does the drone position enter?

**Synchronisation is by `header.stamp`, never by arrival time.**

There is **no dedicated synchroniser node** and no `message_filters`
usage anywhere in the repo. Instead:

* **Online (inside `dem_node`)** — `PoseBuffer.pose_at(t, max_age=0.5)`
  (`pose_buffer.py:130-145`) brackets the scan timestamp between the two
  nearest poses, linearly interpolates position and slerps orientation. If
  the scan stamp is more than `pose_timeout = 0.5 s`
  (`dem.yaml:12`) from any pose, the result is `None`.
* **Per-point motion compensation** — if the cloud carries per-point
  timestamps, `PoseBuffer.transform_points` looks up a *separate pose per
  point* (`pose_buffer.py:147-186`). `pointcloud2_point_times`
  (`pointcloud_utils.py:60-103`) auto-detects three encodings: Livox
  `timestamp` (absolute, s/ms/µs/ns by magnitude), Ouster `t` (uint32 ns),
  Velodyne `time` (relative float). If none is found the whole scan is
  treated as stamped at header time.
* **Offline (`georef_export`, `gnss_align`, `gnss_warp`)** — nearest
  timestamp match with a tolerance, default `--tol 0.2 s`
  (`georef_export.py:404`, `gnss_align.py:123`), via `nearest_match`, which
  is a `searchsorted` + nearest-of-two-neighbours lookup.

**How the drone position enters the point cloud** — two modes, chosen by
`mode:` in `dem.yaml`:

| mode | who transforms the points | frame |
|---|---|---|
| `registered` (default) | **FAST-LIO2 already did it.** `dem_node` only bins `(x, y, z)` into cells (`dem_node.py:314-316`); pose is used *only* for the radial/height filters | `camera_init` (arbitrary origin & yaw) |
| `pose` | `dem_node` transforms raw scans itself using `PoseBuffer`, with optional per-point deskew (`dem_node.py:293-301`) | whatever the pose stream is in |

**Critical point for your use case: GNSS is not used online at all.**
`/gnss_pose` is replayed (so it can be recorded/monitored) but no node
subscribes to it for real-time georeferencing. Every use of GNSS in this repo
is **post-hoc**:

* `gnss_align` — offline feasibility check
* `georef_export` — offline rigid fit `p_utm = R·p_local + t`, then regrid
* `gnss_warp` — offline *time-varying* fit R(t), t(t)

The reason is documented in `georef.py:14-17`: without a calibrated
antenna↔IMU↔LiDAR extrinsic, absolute alignment cannot be claimed online.

There *is* a nominal path to feed GNSS into `dem_node` live —
`pipeline.launch.py:59-61` will set `pose_topic = /gnss_pose`
(`PoseStamped`) when `use_fastlio:=false use_gnss:=true`. **This is
semantically wrong for this dataset** because `/gnss_pose` holds WGS84
*degrees*; `dem_node` would treat 60.44 as metres. The `--origin-lat/lon`
local-frame anchoring exists only in `georef_export`
(`georef_export.py:512-537`), not in `dem_node`.

### 2.6 Independent chunks, or incremental state?

**Strictly incremental — the DEM is never rebuilt from historical points.**

`ElevationGrid` (`elevation_grid.py`) holds three flat arrays over
`nx × ny` cells:

```python
self._sum   = np.zeros(n, float64)   # mean aggregation
self._min   = np.full(n, np.inf)     # min aggregation
self._count = np.zeros(n, int32)     # observation count
```

`update()` (`elevation_grid.py:69-110`) does a `np.bincount` for counts (and
sums, or `np.minimum.at` for min) over *only the touched index span*
(`lo..hi`), then adds into the arrays. Nothing is stored per point — once a
batch is merged its points are gone. Validity is `count > 0`; unknown cells
stay `NaN`.

Consequences worth stating explicitly:

* Memory is **fixed** at grid size regardless of flight duration
  (2000×2000 cells × 20 bytes ≈ **80 MB** for the 400 m @ 0.2 m grid).
* History is **lossy by design** — `min` keeps only the lowest observed z,
  `mean` keeps only the running sum/count. You cannot recover the individual
  observations.
* It **cannot grow** — points outside `origin_x/y .. origin+size` are counted
  in `updates_out_of_bounds` and thrown away (`elevation_grid.py:86-88`).

The **3D point cloud side is the opposite**: FAST-LIO2 accumulates every
registered scan into an in-memory map (and an ikd-tree) that grows
continuously for the whole run, bounded only by `cube_side_length: 1000.0`
and `det_range: 100.0` (`fastlio_mid360.yaml:8,40`). That is the structure
that causes the memory growth noted in the README (peak RSS ≈ 4–5 GB).

---

## 3. Compatibility with Live Livox Mid-360 and GPS Inputs

### 3.1 Can it process live LiDAR instead of replaying a bag?

**Conceptually yes — that was clearly a design goal. Nothing downstream of
`replay` is bag-specific.** All five consumers take topic names as ROS
parameters, not as bag references:

| component | bag-coupled? | notes |
|---|---|---|
| `replay` | yes — this *is* the bag reader | remove/replace for live |
| `imu_relay` | no | pure topic in → topic out, `input_topic`/`output_topic` params |
| `fastlio_mapping` | no | subscribes `common.lid_topic` / `common.imu_topic` from YAML |
| `dem_node` | no | `cloud_topic`, `pose_topic`, `pose_type` are parameters |
| `stream_monitor` | no | takes a `topics:` list |
| `pose_play` | no (CSV, but paced by `/clock`) | needs a `/clock` source — falls back to wall clock after 15 s (`pose_play.py:89-96`) |
| `record_odom` | no | generic `/Odometry` subscriber |
| `georef_export` / `gnss_warp` | yes — reads `/gnss_pose` from a bag | see §3.3 |

**The launch file would need a live variant.** `pipeline.launch.py:83-102`
always starts either `ros2 bag play` or the custom `replay` node. A live
launch would simply omit that action and start the Livox driver instead.

**But see [Appendix A](#appendix-a-blockers-found-in-this-checkout)** — in
*this* working copy the Livox driver and FAST-LIO sources are missing and the
installed Python entry points are broken symlinks, so nothing currently runs
without a rebuild.

### 3.2 Can it take GPS simultaneously?

**Not for real-time georeferencing — no, not today.** See §2.5: `/gnss_pose`
is replayed but unused online. What *does* work:

* `stream_monitor` will watch it and report rates/stamp ordering.
* `record_odom`-style recording of GNSS (`ros2 bag record /gnss_pose`) plus
  post-flight `gnss_warp` → `pose_play` → `dem_node` (pose mode, UTM grid)
  is a **fully supported** path — `dem_utm.yaml` and the `--already-utm`
  branch of `georef_export` exist precisely for it (`georef_export.py:457-496`).

So the honest answer: **the repo supports a record-then-georeference
workflow, not online GNSS fusion.**

### 3.3 Required topics, types, drivers, transforms

**Topics `dem_node` / FAST-LIO consume (all parameterised):**

| topic | type | rate | who needs it |
|---|---|---|---|
| `/mid360/livox/lidar` | `sensor_msgs/msg/PointCloud2` | 10 Hz | FAST-LIO **and** `dem_node` (pose mode) |
| `/mid360/livox/imu` | `sensor_msgs/msg/Imu` | 200 Hz | `imu_relay` only |
| `/mid360/livox/imu_si` | `sensor_msgs/msg/Imu` | 200 Hz | FAST-LIO (`common.imu_topic`) |
| `/gnss_pose` | `geometry_msgs/msg/PoseStamped` | 100 Hz | offline georef only |
| `/cloud_registered` | `sensor_msgs/msg/PointCloud2` | 10 Hz | `dem_node` in `registered` mode |
| `/Odometry` | `nav_msgs/msg/Odometry` | 10 Hz | `dem_node` pose source + `record_odom` |

**Point cloud requirements** (checked in code):

* Must contain `x`, `y`, `z` fields — `pointcloud2_to_xyz`
  raises `KeyError` otherwise (`pointcloud_utils.py:34-46`).
* For deskew, one of `timestamp` / `t` / `time` (`pointcloud_utils.py:60-103`).
  Livox's driver emits `timestamp`; it is detected by magnitude.
* `lidar_type: 4` = "PointCloud2 in Livox format" (`fastlio_mid360.yaml:22`).
  Note: `1 = Livox CustomMsg`, `2 = Velodyne`, `3 = Ouster`, `4 = PointCloud2`.
* FAST-LIO warns `Failed to find match for field 'reflectivity'` for every
  scan in the log — benign, that field is simply not used.

**IMU requirements:**

* **Units must be m/s².** Livox drivers publish *g*. `imu_relay` exists
  solely to multiply acceleration by `9.80665`
  (`pipeline.launch.py:107-115`). If your live driver already publishes SI,
  bypass the relay and point `common.imu_topic` at it.
* FAST-LIO needs a **stationary period at start** for gravity/attitude
  initialisation. The README says the first ~4 s of this recording are static
  and `MAX_INI_COUNT` was raised to 400 (2 s @ 200 Hz). **On a drone this is
  a hard requirement: hold the aircraft still for ~2 s on power-up before
  arming.**
* `extrinsic_est_en: false` (`fastlio_mid360.yaml:47`) — estimating the
  factory Mid-360 IMU↔LiDAR extrinsic online diverged badly on this rig
  (README error #14). Keep it false; supply the factory values
  `extrinsic_T: [-0.011, -0.02329, 0.04412]`, identity R.

**Drivers:**

* LiDAR+IMU: `livox_ros_driver2` (already vendored in `ws/src/`, though
  currently an empty gitlink). It provides `livox_ros_driver2/msg/CustomMsg`
  and republishes `PointCloud2`.
* GPS: **not present.** The repo has no NMEA/UBX driver. You need one
  publishing either `sensor_msgs/msg/NavSatFix` (preferred; `gnss_align`
  already handles `NavSatFix` → UTM via `gnss_align.py:74-81`) or
  `geometry_msgs/msg/PoseStamped` with **pre-converted metric** coordinates.

**Coordinate transformations in play:**

```
sensor frame  ──(extrinsic_T/R)──►  body/IMU frame
body frame    ──(FAST-LIO EKF)──►  camera_init   (local metres, arbitrary yaw)
camera_init   ──(georef fit R,t)──► UTM           (EPSG:32634 here)
UTM           ──(EPSG)───────────► WGS84 lat/lon
```

`georef.py` implements WGS84 ↔ UTM analytically in NumPy (Snyder series,
"accurate to well under a millimetre inside a UTM zone"), so **no pyproj/PROJ
is required** (`georef.py:1-18`). GDAL (`osgeo`) *is* required, but only for
writing/reading GeoTIFF (`georef_export.py:316-345`).

### 3.4 What modifications to go from recorded → live?

Ordered roughly by effort:

1. **Rebuild the workspace.** Required regardless — see Appendix A.
   `colcon build --symlink-install`, then `--packages-select fast_lio`
   (C++).
2. **Fix the hardcoded absolute paths** (`/home/rajesh/proc_dem_creation`):
   `run_pipeline.sh:29-32`, `dem.yaml:51`, `dem_utm.yaml:49`,
   `fastlio_mid360.yaml:12`, `pipeline.launch.py:175`. These are in 5 files.
3. **New launch file** (`live.launch.py`) that starts:
   `livox_ros_driver2` → `imu_relay` → `fastlio_mapping` → `dem_node` →
   (`stream_monitor`, viz). Drop `replay` entirely, and drop `use_sim_time`
   (there is no `/clock`; set it false or the nodes will wait forever).
4. **Topic remapping / naming.** Confirm the live driver's topic names match
   `common.lid_topic`, `common.imu_topic`, and `cloud_topic`. Easiest via
   `ros2 run ... --ros-args -r ...` or parameters, no code change.
5. **QoS check.** This is the classic live-vs-bag trap and is called out as
   README error #5: Livox's driver publishes sensor data as `BEST_EFFORT`.
   `dem_node` already requests `BEST_EFFORT` for clouds, which matches. Verify
   the IMU path too — `imu_relay` and FAST-LIO are C++/default `RELIABLE`,
   and a `RELIABLE` subscriber with a `BEST_EFFORT` publisher **will not
   connect**.
6. **Replace the offline GNSS path.** Either:
   * **(a) Keep it offline** — `ros2 bag record /gnss_pose /Odometry`, then
     `gnss_warp` → `pose_play` → `dem_node` (pose mode, `dem_utm.yaml`) →
     `georef_export --already-utm`. *Least work, and the code for it already
     exists.*
   * **(b) Go online** — write a new node that maintains a sliding-window
     GNSS↔LIO alignment (the mathematics is already in `gnss_warp.py`:
     `estimate_windows`, `fit_smooth_correction`, `slerp_series`) and
     publishes either a corrected `/Odometry` or a live UTM pose stream. This
     is a genuinely new component — a few hundred lines, but new.
   * **(c) True sensor fusion** — put GNSS into the estimator itself.
     FAST-LIO2 has **no GPS fusion**. You would need a different odometry
     stack (LIO-SAM, FAST-LIO2-GPS variants, hdl_localization + NavSatFix,
     or MSF/EKF-SLAM with GPS updates). This is a replacement, not a
     modification.
7. **Teardown/telemetry.** `run_pipeline.sh` is built around "wait for
   replay finished, then call two Trigger services". For live you need a
   mission-end signal (user command / drone landing) to call `/map_save` and
   `/dem_node/save`.

### 3.5 Is it real-time capable at Mid-360 rates?

**The replay run kept up at 1.0× with zero lost messages** (§2.4). That is
the only measurement available in the repo. Break it down:

| stage | rate | budget | assessment |
|---|---|---|---|
| LiDAR in | 10 Hz | 100 ms/scan | — |
| `pointcloud2_to_xyz` | 10 Hz | pure Python/NumPy, ~20k points × 3 fields, several array copies + `float64` casts | **the likely hot spot** — see §6.3 |
| filters | 10 Hz | vectorised NumPy over ≤20k points | sub-ms, cheap |
| `ElevationGrid.update` | 10 Hz | `bincount` + `np.minimum.at` over touched cells | `np.minimum.at` is an unbuffered ufunc.at — notoriously slow; still likely low-ms |
| FAST-LIO2 | 10 Hz | C++, ikd-tree, 3 EKF iterations, `filter_size 0.5`, `point_filter_num: 3` (keeps 1 in 3 points) | the real bottleneck; already tuned |
| publishers | 1–2 Hz | `elevation_image` = 2000×2000 `float32` = **16 MB/message**; `snapshot()` copies `_sum`+`_min`+`_count` = **~80 MB per call**, called **5×/s** (image, cloud, occupancy, stats×2) | **large, but off the sensor path** by design |

Two things to say clearly:

* **The design already isolates the sensor path from the slow path.**
  Publishing is in a separate callback group and snapshots are copied under
  lock then serialised outside it. That is the right structure.
* **The copy volume is worth attacking.** ~80 MB × 5/s ≈ **400 MB/s of
  memcpy** plus 16 MB × 1/s of image serialisation, for data nobody may be
  subscribed to. Cheap wins: gate `image_rate`/`occupancy_rate` to 0 when
  there are no subscribers, and keep a cached float32 elevation rather than
  re-deriving it per publisher.

**Verdict:** at 10 Hz / 20k points it is plausibly real-time on a laptop CPU
and demonstrably kept up in replay. For a drone, **the risks are not raw
throughput but**: (i) unpaced bursts, (ii) no drop accounting, (iii) Python
in the per-scan hot path, (iv) the growing FAST-LIO map. (i)–(iii) are
addressable; see §6.

### 3.6 Continuous incremental updates while flying?

**Yes, this is the core design and it already works that way.**

* Grid merges every scan in place — no batching, no rebuild, no flight-end
  processing (`elevation_grid.py:69-110`).
* Live topics update continuously at 1–2 Hz: `~/elevation_image`,
  `~/elevation_cloud`, `~/occupancy`, `~/trajectory`, `~/stats`.
* Explicit checkpoints on demand: `~/save` (Trigger) and `~/reset`
  (`dem_node.py:210-211`), plus an automatic final save on shutdown
  (`dem_node.py:516-519`).
* The 3D cloud is likewise accumulating live (`map_en: true`), saved on
  demand via `/map_save`.

**Limits on "while flying":**

1. **The grid cannot move or grow.** A fixed 400×400 m window centred at
   `origin_x/y = -200`. Fly outside it and points are silently dropped
   (`updates_out_of_bounds`). For a drone you want either a much larger grid
   (memory grows quadratically: 1 km @ 0.2 m = 25M cells ≈ 500 MB — too
   much) or **a rolling/sliding window** that re-centres on the vehicle and
   shifts cells as it moves. **That is the single most important
   modification for long flights.**
2. **`height_band` is tuned for a ground vehicle.** `band_below: 6.0`,
   `band_above: 4.0` relative to the *pose z*. A drone at 50 m AGL only
   keeps ground within 6 m below its own altitude — i.e. it maps only what
   is essentially at its own height. For aerial mapping this filter is wrong
   and must be widened or replaced (see §4.2).
3. **`max_range: 60.0`** and FAST-LIO `det_range: 100.0` cap useful
   sensing distance. From altitude, 60 m of ground swath is small.

---

## 4. Generating DEMs for Mountains and Other 3D Terrain

### 4.1 Is it limited to ground-surface elevation?

**Yes — structurally. It is a 2.5D heightmap, not a surface model.**

The data structure permits exactly **one z value per (x, y) cell**
(`elevation_grid.py:37-40`, three flat arrays indexed by `iy*nx + ix`). That
is the definition of 2.5D.

And the *filtering* actively removes everything that isn't ground near the
vehicle (`ground_filters.py:57-75`):

```python
class HeightBandFilter(GroundFilter):
    """Keep points in a vertical band around the sensor/vehicle altitude.
    Points much higher than the vehicle (buildings, canopy, wires) and
    points far below it (pits, map drift, returns from voids) are dropped."""
    def mask(self, points, ctx):
        pose_z = ctx.get('pose_z')
        if pose_z is None:
            return np.ones(points.shape[0], dtype=bool)
        z = points[:, 2]
        return (z >= pose_z - self.below) & (z <= pose_z + self.above)
```

With `band_below: 6.0` / `band_above: 4.0`, a mountain face rising 200 m
above the drone is **discarded point-by-point before it ever reaches the
grid**. So the answer to "can it reconstruct a mountain face?" is: *not in
the DEM, and not at all with the current default configuration.*

Available filters (`ground_filters.py:116-123`): `none`, `z_range`,
`height_band`, `radial_range` (always applied first), `max_points` (always
applied last). `none` is explicitly documented as "algorithm testing only -
NOT ground classification".

**There is no ground classifier.** The docstring says as much:
"classification is deliberately pluggable so it can later be replaced by a
proper classifier (plane fitting, cloth simulation, a learned model, ...)
without touching the elevation grid code" (`ground_filters.py:1-6`). The
hooks exist; the implementation does not.

### 4.2 Mountain face + surrounding terrain — can it reconstruct both?

| scenario | in the **DEM** | in the **point cloud** |
|---|---|---|
| Ground slope, gentle | ✅ yes | ✅ yes |
| Mountain face, steep but < ~80°, seen from the side | ⚠️ partial — aliasing, holes | ✅ yes |
| Vertical cliff | ❌ collapses to one z per (x,y) | ✅ yes |
| Overhang / cave / canopy underside | ❌ **impossible** in 2.5D | ✅ yes |
| Terrain above the drone | ❌ filtered out by `height_band` | ✅ yes, if within `det_range` |
| Terrain seen from multiple angles | ⚠️ `min` merges them into the lowest | ✅ yes |

Why the cliff fails mechanically: a vertical wall at x = 10 m has points at
z = 0, 10, 20, 30 m… all mapping to cell `(10, y)`. `min` keeps z = 0 and
discards the rest; `mean` averages them into a value corresponding to
nothing real. `cell_index` (`elevation_grid.py:48-54`) is `floor` on x and y
— z is not part of the address at all.

**The 3D point cloud already handles all of these cases correctly.**
`out/map_fastlio.pcd` is a full unstructured 3D reconstruction in
`camera_init` metres, written by `/map_save`, and it is rotatable/inspectable
in RViz today. The gap is purely that the *grid* representation cannot
express it, and that the *default filter configuration* throws those points
away before they could be used.

### 4.3 Does it assume Z = elevation, X-Y = ground plane?

**Yes, explicitly and everywhere:**

* `cell_index`: `ix = floor((x - origin_x)/res)`, `iy = floor((y - origin_y)/res)`
  — z is not an address dimension (`elevation_grid.py:50-53`).
* Publication: `~/occupancy` is a `nav_msgs/OccupancyGrid` — a **2D** map type
  (`dem_node.py:396-428`).
* Georeferencing: `reproject_grid` transforms cell **centres in XY**
  (`georef_export.py:256+`), preserving z.
* `georef.py:4-5` states the convention outright: "A *local* DEM lives in the
  mapper frame… X/Y are metres".
* The default `--fit yaw` restricts rotation to the gravity axis precisely
  because both frames are assumed **gravity-aligned**, i.e. z is assumed "up"
  in both — see the `yaw_fit` docstring: "Rotation is restricted to the
  gravity axis (both frames are level: the LiDAR map frame comes from an
  IMU-aligned LIO, UTM is a projected CRS)" (`georef_export.py:133-139`);
  choices at `georef_export.py:406`.

### 4.4 What changes would be needed for near-vertical surfaces / overhangs / multi-view?

Ranked from cheapest to most invasive:

**(a) Widen or replace the filter — cheap, necessary, but not sufficient.**
Set `ground_filter: none` (or a much wider `z_range`) so mountain points
survive. This alone does **not** fix the one-z-per-cell problem; it just
stops pre-emptive data loss. *Required for any aerial/mountain work.*

**(b) Multi-layer heightmap — moderate.** Keep k elevation values per cell
(e.g. lowest 3 returns, or a small sorted list). Cheap in memory
(×k), handles a cliff *seen from a consistent side*, still fails on
overhangs (ambiguous which layer a new point belongs to) and on
multi-viewpoint fusion.

**(c) Voxel / occupancy grid — the standard answer.** Address cells as
`(ix, iy, iz)` — the `floor` logic generalises directly. Handles overhangs
and vertical surfaces. Cost: memory scales with the *surface area* of what
you fly around, not with the ground footprint; needs chunking/tiling and
eviction for a long flight. Libraries: Open3D `VoxelGrid`, `octomap`,
`voxblox`/`voxel_hash_mapping`, or a plain `dict`/hash for prototypes.

**(d) Surfel / mesh-based incremental reconstruction.** Maintain oriented
surfaces rather than occupancy. Best visual quality, highest complexity.
Typical choices: ElasticSurfel, incremental Poisson, KinectFusion-style
TSDF + marching cubes.

**(e) Keep the DEM *and* add a 3D map — recommended.** Two representations,
two purposes:

| representation | purpose | status here |
|---|---|---|
| DEM (GeoTIFF, EPSG) | terrain elevation, GIS interchange, slope analysis, orthorectification | ✅ exists |
| 3D point cloud / mesh | complete surfaces, inspection, obstacle geometry, mountains | ⚠️ point cloud ✅, mesh ❌ |

This dual-output pattern is already 90% present — `map_fastlio.pcd` and
`dem_*.npz` are produced side by side. What's missing is (i) the filter
config that preserves 3D structure, (ii) a mesh export path, and (iii) a
shared frame (both must end up in the same UTM/georeferenced coordinates).

**(f) Frame/altitude considerations for flight.** Beyond representation:

* A drone's z comes from the LIO estimate (baro/GPS-altitude aid would help
  a lot — FAST-LIO2 alone drifts vertically; this repo measured ~10 m of z
  drift over 10 min before raising `acc_cov`, README error #16).
* The `height_band` filter's reference is `pose_z`. For a flying platform you
  almost certainly want `ground_filter: none` or `z_range` with an absolute
  window derived from the takeoff altitude.
* Grid `origin_x/y` and `size_x/y` must be derived from the planned survey
  area, not from `dem_utm.yaml`'s hardcoded `origin_x: 572320.0` /
  `origin_y: 6701670.0` (which is specific to this one Turku recording).

### 4.5 Is 2.5D enough, or is full 3D needed?

**It depends on the deliverable — and they are not mutually exclusive.**

* If the goal is **"a DEM" in the GIS sense** (terrain elevation for mapping,
  grading, hydrology, orthophoto generation over a road/field): **2.5D is
  exactly right** and is what every production DEM pipeline uses. Steep-but-not
  vertical terrain is fine.
* If the goal is **"inspect the mountain face / cliff / overhang / structure
  from any angle"**: 2.5D is **structurally incapable**. You need (c), (d) or
  (e) above.
* If the goal is **both** (your stated objective): **keep the DEM and add a
  3D layer.** Do not try to make the DEM do 3D's job — you will get a worse
  DEM *and* a worse 3D model.

Practical recommendation: treat the DEM as a **derived product** of the 3D
map. Maintain the 3D map as the primary structure, and rasterise "lowest
return in each cell" into DEM tiles on demand. That gives you correct
overhangs in 3D *and* a correct ground DEM, from one source of truth. The
current architecture is the reverse (DEM primary, cloud separate), which is
why they can disagree.

---

## 5. DEM Visualization and 3D Interaction

### 5.1 What is the current visualization?

Three things exist:

1. **`dem_viz` / `DemHeatmap`** — matplotlib. `self.im = self.ax.imshow(
   masked, origin='lower', extent=ext)` (`heatmap.py:133`). A **2D raster
   image** with a trajectory overlay, elevation colour ramp, stats panel.
   Interactive only in the sense that matplotlib allows pan/zoom of a 2D
   axes — **no rotation, no tilt, no 3D navigation.** Falls back to PNG
   snapshots when headless (`heatmap_node.py:126-131`).
2. **RViz2** — optional (`--rviz`, `use_rviz:=true`). Shows
   `/cloud_registered`, `/Odometry` path, and `dem_node`'s topics. **This is
   where you get full rotate/pan/zoom — but of the *point cloud*, not of a
   DEM surface.**
3. **Static exports** — `georef_export` writes
   `<prefix>_dem_preview.png` and `<prefix>_dem_hillshade.tif`
   (`georef_export.py:336-384`). Not interactive at all.

### 5.2 What causes the top-down limitation?

**Primarily the data structure and message types — secondarily the viz
library.** In order:

1. **The DEM *is* a 2D raster.** `~/elevation_image` is
   `sensor_msgs/Image` `32FC1` (`dem_node.py:327-343`) and `~/occupancy` is
   `nav_msgs/OccupancyGrid` (`dem_node.py:396-428`) — a literally 2D message
   type. There is no z-extruded geometry anywhere in the DEM publication
   path. You cannot rotate what isn't stored as 3D.
2. **matplotlib is used with `imshow`.** `DemHeatmap` builds one axes and
   draws an image (`heatmap.py:60`, `heatmap.py:133`). matplotlib *can* do
   `plot_surface` / 3D axes, but this code doesn't — and matplotlib's 3D
   interactivity is poor anyway.
3. **The terrain algorithm.** Even with a perfect viewer, one-z-per-cell
   means a "3D surface" built from the DEM is a heightfield: you could rotate
   it, but a cliff would still be a single quad.

**Not a limitation of:** the underlying grid (it has full 3D world
coordinates for cell centres), or the point cloud (fully 3D already).

### 5.3 Can it be a rotatable 3D surface mesh with Z as elevation?

**Yes, and most of the plumbing exists.** Three routes, cheapest first:

**(a) Zero new algorithm — use the existing `~/elevation_cloud`.**
`_publish_cloud` (`dem_node.py:345-394`) already emits every valid cell
centre as a `PointCloud2` with `z` = elevation, RGB-coloured by height. Open
it in RViz → **rotate, pan, zoom, inspect a mountain-shaped surface in 3D
today.** Downsides: it's points not a surface (no interpolation between
cells), it's in the local frame unless you use the UTM path, and
`cloud_max_cells: 150000` subsamples a 4M-cell grid by stride
(`dem_node.py:362-366`).

**(b) Publish a `visualization_msgs/Marker` TRIANGLE_LIST / mesh.**
Build quads from neighbouring valid cells (`np.gradient`-style index
arithmetic on the elevation array) and publish as a mesh marker. RViz and
Foxglove both render it. Moderate code, no new dependencies, no new data
structure. This is the smallest change that yields an actual **surface**.

**(c) Export to a proper 3D viewer.** Write PLY/OBJ/glTF from the elevation
array + valid mask, or stream to **Foxglove / Rerun / Open3D / Polyscope /
rerun.io / a three.js web view**. Best interactivity. Note the repo's
`pcd_io.py` writes **PCD only** (`write_pcd`, `pcd_io.py:108`) and supports
`ascii` and `binary` (not `binary_compressed`, `pcd_io.py:10-21`) — **no
PLY/OBJ/mesh export exists**; that would be new code.

### 5.4 Can a mountain face be inspected from the side?

**Only through the 3D point cloud, not through the DEM.**

* With `dem_utm.yaml` / the warped path, or with a georeferenced
  `map_georef.pcd`, the point cloud in RViz can be orbited to view the face
  head-on. This works with what's in the repo.
* The DEM cannot show a face — by construction (§4.2). Rendering the DEM as
  a 3D surface would show a heightfield *bulge* where the mountain is, viewed
  from the side as a thin sheet.

So: **rotate the cloud for the face, the DEM for the ground.** This is
exactly the dual-representation argument from §4.4(e).

### 5.5 Can it maintain a DEM *and* a 3D cloud/mesh simultaneously?

**DEM + 3D point cloud: yes, already, simultaneously and incrementally.**

They are separate nodes fed by the same stream:

```
/cloud_registered ──┬──► dem_node      ──► DEM grid  (.npz → GeoTIFF)
                    └──► fastlio_mapping ──► map cloud (.pcd, georeferenced)
```

Both update live during the run; both are saved on demand; both can be
georeferenced (`georef_export` takes `--dem` *and* `--map` and transforms
both with the same `R, t`).

**DEM + mesh: no** — no mesh is produced anywhere (§5.3c).

**Caveat — frame consistency.** `dem_utm.yaml` puts the grid in UTM while the
FAST-LIO map stays in `camera_init` unless you run `georef_export --map`
separately with a `--already-utm`-style transform (`georef_export.py:423-431`
handles a warp report for the map). To view both correctly in one frame you
must make sure the same transform is applied to both — `gnss_warp` writes
`base_transform` into the warp report for exactly this purpose
(`gnss_warp.py:323-324`).

---

## 6. Feasibility and Required Changes

### 6.1 What can be reused without modification?

**Directly reusable (no code change):**

| component | why |
|---|---|
| `ElevationGrid` (`elevation_grid.py`) | pure NumPy, no ROS, fixed memory, fast. Reusable as-is for the 2.5D DEM |
| `ground_filters.py` | pluggable registry; adding a filter is additive |
| `georef.py` | complete, dependency-free WGS84↔UTM. Valuable and correct |
| `pcd_io.py` | PCD read/write, transform |
| `pointcloud_utils.py` | robust multi-vendor PointCloud2 parsing + per-point time detection |
| `pose_buffer.py` | time-indexed pose interpolation with per-point deskew — exactly what live needs |
| `gnss_warp.py` maths | sliding-window rigid fits, quaternion slerp, Huber-IRLS correction — reusable as the core of an online alignment node |
| `georef_export` fitting | lat/lon order auto-detection, RMS quality gate, `yaw`/`full` fits |
| `imu_relay` | unit conversion, needed for live too |
| `record_odom`, `stream_monitor` | generic ROS nodes |
| `dem_node` architecture | callback-group separation, snapshot-outside-lock, staged timing stats — the *structure* is right for real-time |
| `bag_convert`, `inspect_bag` | offline tooling; still useful for validating live recordings |

**Reusable with configuration only:** `dem_node`, `fastlio_mapping`,
`pipeline.launch.py`, all YAML (topic names, rates, grid geometry, filters).

### 6.2 What must be modified?

| # | change | effort | why it's mandatory |
|---|---|---|---|
| 1 | Rebuild workspace; restore `FAST_LIO` + `livox_ros_driver2` | **blocker** | Appendix A — nothing runs today |
| 2 | Replace hardcoded `/home/rajesh/proc_dem_creation` (5 files) | trivial | paths don't exist on this machine |
| 3 | `live.launch.py`: driver instead of `replay`, `use_sim_time:=false` | small | `pipeline.launch.py` always starts a player |
| 4 | **Sliding / re-centred grid** | **medium, high value** | fixed 400×400 m window makes long flights impossible (`elevation_grid.py:86-88` silently drops out-of-bounds points) |
| 5 | Filter config for aerial use (`height_band` → `none`/`z_range`) | trivial | default throws away all terrain above/below the drone (§3.6) |
| 6 | **Drop accounting** on the cloud subscription | small | currently invisible loss (§2.4) |
| 7 | Online GNSS path (or accept record-then-`gnss_warp`) | medium / none | GNSS is offline-only today (§3.3) |
| 8 | Live mission-end → call `/map_save`, `/dem_node/save` | small | `run_pipeline.sh` waits for replay "finished" |
| 9 | QoS audit for live drivers | small | README error #5 — `BEST_EFFORT`/`RELIABLE` mismatch silently yields zero data |
| 10 | Stationary IMU init + `MAX_INI_COUNT` policy | small | FAST-LIO needs ~2 s still at start (README error #3) |
| 11 | PLY/mesh export, or a 3D viewer | medium | §5.3 |

**Optional but recommended:** `pcd_save_en: true` + `map_en: true` cost a lot
of memory for a long live flight (README error #13 describes a patch that
makes `/Laser_map` serialisation a no-op with 0 subscribers — worth
re-verifying once FAST_LIO source is restored).

### 6.3 Does the DEM approach need replacing or supplementing?

**Supplement, don't replace.** Reasoning in §4.5. Concretely:

* Keep `ElevationGrid` as the 2.5D terrain product (GIS deliverable).
* **Supplement** with a 3D structure for mountains/overhangs. The cheapest
  useful step: keep the full registered cloud (already happening) and add a
  **voxel layer** for local 3D queries.
* Fix the data flow so the 3D map is the primary structure and the DEM is
  rasterised from it — that resolves the frame-consistency problem in §5.5
  as a side effect.

### 6.4 Main computational bottlenecks

Ranked by likely impact on a real-time drone build:

**1. `pointcloud2_to_xyz` — pure Python/NumPy, per scan, 10 Hz.**
(`pointcloud_utils.py:21-46`) It does `np.frombuffer` → `reshape(width,
point_step)` → per-field `ascontiguousarray` + `view` + `astype(float64)` ×3 →
`np.stack`. Several full-array copies and a `uint8→float64` promotion of
3×20k values. It runs on **every** scan including in `registered` mode where
only x/y/z are needed. *Mitigations:* parse only x/y/z offsets directly,
stay in `float32` until the grid update, or move to C++/Rust/numba.

**2. Snapshot copy volume — ~80 MB × 5/s.**
`copy_state()` copies `_sum` (float64) + `_min` (float64) + `_count` (int32)
= 4M cells × 20 B = **80 MB** (`elevation_grid.py:131-138`), called by image
(1 Hz), cloud (1 Hz), occupancy (1 Hz) and stats (2 Hz) = 5×/s
(`dem_node.py:213-220`) → **~400 MB/s memcpy**, plus a 16 MB image payload.
*Mitigations:* cache a single shared `float32` elevation snapshot and
invalidate on update (one copy instead of five); gate on subscriber count;
publish at lower rates.

**3. `np.minimum.at` for min aggregation.** (`elevation_grid.py:106`)
`ufunc.at` is unbuffered and notoriously slow — typically 5–10× slower than
`bincount`. For `aggregation: min` (the default) this runs every scan.
*Mitigation:* since indices within one scan are effectively unique after
binning, do a `np.minimum.reduceat` over sorted unique indices, or use the
mean/`bincount` path and post-process.

**4. FAST-LIO2 C++ EKF + ikd-tree.** The real CPU consumer. Already tuned
(`point_filter_num: 3`, `filter_size 0.5`, `max_iteration: 3`). The growing
in-memory map is the memory risk over long flights.

**5. Unbounded map growth.** `map_en: true` accumulates every registered
scan forever. Over a long flight this is a leak-equivalent. Needs
submap/local-window management.

**6. Python GIL / multi-threading.** `MultiThreadedExecutor(num_threads=4)`
with a `MutuallyExclusiveCallbackGroup` for sensor callbacks means sensor
processing is **effectively single-threaded**. Correct for safety, but it
means one core does all LiDAR work. *Mitigation:* move the hot path to C++.

**Non-bottlenecks (don't waste time here):** the grid update itself
(vectorised, sub-ms), UTM conversion (analytic NumPy), PCD I/O (offline).

**How to ensure no latency/data loss:**

1. Measure first — `~/stats.processing_ms.{last,mean,p95}` already exists
   (`dem_node.py:447-453`) and `out/logs/mem_*.csv` already traces RAM.
   Compare `input_hz` against the expected 10 Hz continuously.
2. **Add a drop counter** at the subscription (compare received count vs
   published count from `/replay/status`, or use ROS 2 QoS event callbacks).
   Right now loss is silent.
3. Keep queues **bounded** (they are: depth 10) and **`BEST_EFFORT`**
   end-to-end for sensor data — you want fresh data, not stale backlog.
4. Ensure **no blocking work in sensor callbacks**: already done via
   callback groups; preserve that pattern when adding features.
5. **Decimate early** (`point_filter_num`, `max_points_per_scan`) rather
   than late.
6. Consider a **ring/tile map** instead of one giant fixed grid.
7. Isolate nodes in separate processes (already true) so a slow Python
   publisher cannot stall the C++ odometry.

### 6.5 Recommended architecture for a robust real-time system

```
  ┌─ Sensors ────────────────────────────────────────────────────────┐
  │ livox_ros_driver2          GNSS driver (NMEA/UBX → NavSatFix)    │
  │   /mid360/livox/lidar 10Hz    /gnss 1-10 Hz                     │
  │   /mid360/livox/imu 200Hz (g)                                    │
  └──────────┬──────────────────────────────┬────────────────────────┘
             │                              │
             ▼                              ▼
  ┌─ Conditioning ─────────┐    ┌─ Time sync ────────────────────────┐
  │ imu_relay (g→m/s²)     │    │ time-indexed buffer (PoseBuffer-   │
  │ QoS: BEST_EFFORT e2e   │    │  like), header.stamp keyed, bounded│
  │ drop counters          │    │  Δt monitoring                     │
  └──────────┬─────────────┘    └───────────────┬────────────────────┘
             ▼                                  │
  ┌─ Odometry (C++) ───────────────┐            │
  │ FAST-LIO2 (or GPS-aided LIO    │            │
  │  if you need online GNSS)      │            │
  │  • stationary init gate        │            │
  │  • extrinsic_est_en: false     │            │
  │  • LOCAL submap, not global     │            │
  │  → /Odometry, /cloud_registered│            │
  └──────────┬─────────────────────┘            │
             ▼                                  ▼
  ┌─ Mapping core (new / heavily modified) ──────────────────────────┐
  │  shared 3D structure (voxel hash / local surfel map)             │
  │     ├─► rasterise → DEM tiles  (2.5D, incremental, sliding)      │
  │     └─► local 3D cloud/mesh    (mountains, overhangs, obstacles) │
  │  • re-centred / tiled grid (fixed-size windows, evict old)       │
  │  • pluggable ground classifier (not just height_band)            │
  │  • all outputs in a common frame                                 │
  └──────────┬───────────────────────────────────────────────────────┘
             ▼
  ┌─ Georeferencing ─────────────────────────────────────────────────┐
  │ online: sliding-window GNSS↔LIO alignment (math in gnss_warp.py) │
  │   → publishes UTM poses; or RTK/NavSatFix into the estimator     │
  │ offline: gnss_warp → warped_odom.csv → georef_export --already-utm│
  │ quality gate retained (RMS threshold → refuse rather than lie)    │
  └──────────┬───────────────────────────────────────────────────────┘
             ▼
  ┌─ Products & visualization ───────────────────────────────────────┐
  │ GeoTIFF DEM + hillshade      (GIS)                               │
  │ PCD/PLY 3D map               (inspection)                        │
  │ Foxglove / Rerun / web 3D    (interactive, rotatable)            │
  │ live telemetry: input_hz, p95 latency, drop count, RAM           │
  └───────────────────────────────────────────────────────────────────┘
```

**Three decisions that matter most:**

1. **Sliding/tiled map, not one fixed grid.** Without this, flight duration
   is capped by grid extent, and memory is capped by resolution².
2. **3D structure primary, DEM derived.** Solves mountains, overhangs,
   frame-consistency, and obstacle avoidance from one structure.
3. **Decide early whether GNSS is online or offline.** Offline
   (`gnss_warp`-style, already implemented) is far less work and can be
   made near-real-time (60 s window / 30 s step, 10 s knots). Online fusion
   means changing the odometry stack.

---

## Bottom line

**Can this repo be extended from recorded replay to a real-time drone
Livox Mid-360 + GPS mapping system?**

**Yes for the replay→live transition: that part is genuinely close.** The
nodes are topic-parameterised, QoS is already sensor-appropriate, the IMU
unit conversion, per-point deskew, timestamp-keyed pose interpolation and
incremental grid are all exactly what a live system needs, and the pipeline
kept up with every message at 1.0× replay.

**Partially for live GPS:** offline georeferencing is mature (with a
quality gate that refuses bad fits); online GNSS fusion does not exist and
would need new work or a different odometry stack.

**No, not as-is, for mountains and complex 3D terrain.** The DEM is
structurally 2.5D — one z per cell — and the *default filter configuration
actively discards* everything that isn't ground near the vehicle. You must
keep the existing 3D point cloud (which already handles this correctly) and
add a real 3D structure, treating the DEM as a derived product rather than
the primary one.

**And in this specific checkout, nothing runs until it is repaired** — see
Appendix A.

---

## Appendix A: Blockers found in this checkout

Verified in the working tree on 2026-10-20.

| # | finding | evidence | impact |
|---|---|---|---|
| **A1** | **`ws/src/FAST_LIO` is empty** | `ls -la ws/src/FAST_LIO` → only `.`/`..` | FAST-LIO2 source is absent; cannot build or apply the patches the README documents (`publish_map()` early-return, `MAX_INI_COUNT=400`) |
| **A2** | **`FAST_LIO` and `livox_ros_driver2` are dangling gitlinks** | `git ls-files -s` shows mode `160000` for both; **no `.gitmodules` file exists** | They are committed as submodules but with no submodule config → `git submodule update --init` cannot fetch them. The upstream repos must be re-added manually |
| **A3** | **Installed binaries are broken absolute symlinks** | `ws/build/dem_mapping/dem_mapping -> /home/rajesh/proc_dem_creation/ws/src/...`; `ws/install/fast_lio/lib/fast_lio/fastlio_mapping -> /home/rajesh/.../ws/build/fast_lio/fastlio_mapping` | `/home/rajesh` does not exist here → **`ros2 run` will fail for every package**. Rebuild required |
| **A4** | **Hardcoded paths in 5 source files** | `run_pipeline.sh:29-32` (`ROOT`, `BAG`, `WS`, `OUT`), `dem.yaml:51` (`save_dir`), `dem_utm.yaml` (`origin_x/y`, `save_dir`), `fastlio_mid360.yaml:12` (`map_file_path`), `pipeline.launch.py:175` (`default_bag`) | All point at `/home/rajesh/proc_dem_creation` |
| **A5** | **`run_gnss_aided.sh` does not exist** | referenced in `dem_utm.yaml:4`; `find . -name 'run_gnss_aided*'` → nothing | The GNSS-aided workflow described in the config has no runner script |
| **A6** | **`gnss_warp` report writing references undefined names** | `gnss_warp.py:323-324` uses `R0`/`t0`, but they are locals of `main()` (`gnss_warp.py:272`), not globals | Current code should raise `NameError` when writing the report. Confirmed indirectly: `out/warp_report.json` has **no `base_transform` key**, i.e. it was written by an older revision |
| **A7** | **`ws/build`, `ws/install`, `out/*.npz`, `out/logs` are committed** | `git ls-files \| wc -l` = 1555; `.gitignore` only ignores `*.bag`, `*.db3`, `*.pcd` | Build artifacts bloat the repo and *encode the stale `/home/rajesh` symlinks* (A3) |
| **A8** | **README does not document `gnss_warp` / `pose_play` / `dem_utm.yaml`** | README §4/§8 describe only `georef_export`; `setup.py:26` registers `gnss_warp`, `pose_play` is in `dem_replay/setup.py:30` | The newest and most important component (drift-correcting georeferencing) is undiscoverable from the docs |

**Minimum recovery sequence** (for when you do want to build):

1. Restore `FAST_LIO` and `livox_ros_driver2` (the gitlinks reference
   commits `a4743b0…` and `2144554…`; add proper `.gitmodules` entries or
   vendor them).
2. Rewrite the 5 hardcoded paths (A4), ideally from the repo root via
   `$(dirname "$0")`.
3. `rm -rf ws/build ws/install` then `colcon build --symlink-install`
   (README error #1 — stale mixed layouts are the known failure mode).
4. Fix the `R0`/`t0` scope bug in `gnss_warp.py` (A6) before relying on the
   warp report.
5. Rebuild `fast_lio` separately for any C++ change
   (`colcon build --packages-select fast_lio`) and re-verify the README's
   documented patches are present, since the source was lost (A1/A2).

---

## Appendix B: Key file index

| file | role | lines |
|---|---|---|
| `run_pipeline.sh` | end-to-end orchestrator (6 stages) | 239 |
| `ws/src/dem_replay/dem_replay/replay_node.py` | timestamp-preserving paced bag player | 398 |
| `ws/src/dem_replay/dem_replay/imu_relay_node.py` | g → m/s² republisher | 78 |
| `ws/src/dem_replay/dem_replay/record_odom.py` | live trajectory → CSV | 132 |
| `ws/src/dem_replay/dem_replay/pose_play.py` | CSV poses → `/Odometry_warp`, paced by `/clock` | 136 |
| `ws/src/dem_replay/dem_replay/stream_monitor_node.py` | rate/order/sync watchdog | 202 |
| `ws/src/dem_replay/dem_replay/bag_convert.py` | ROS 1 → ROS 2 conversion + validation | 355 |
| `ws/src/dem_mapping/dem_mapping/dem_node.py` | the incremental DEM node | 527 |
| `ws/src/dem_mapping/dem_mapping/elevation_grid.py` | 2.5D grid data structure | 229 |
| `ws/src/dem_mapping/dem_mapping/ground_filters.py` | pluggable ground filters | 139 |
| `ws/src/dem_mapping/dem_mapping/pose_buffer.py` | time-indexed pose interpolation + deskew | 189 |
| `ws/src/dem_mapping/dem_mapping/pointcloud_utils.py` | PointCloud2 → NumPy | 103 |
| `ws/src/dem_mapping/dem_mapping/georef.py` | WGS84 ↔ UTM (no pyproj) | 139 |
| `ws/src/dem_mapping/dem_mapping/georef_export.py` | offline georeference + GeoTIFF export | 750 |
| `ws/src/dem_mapping/dem_mapping/gnss_warp.py` | sliding-window drift correction | 343 |
| `ws/src/dem_mapping/dem_mapping/gnss_align.py` | offline GNSS↔pose feasibility check | 189 |
| `ws/src/dem_mapping/dem_mapping/pcd_io.py` | PCD read/write/transform | 181 |
| `ws/src/dem_bringup/launch/pipeline.launch.py` | full pipeline launch | 209 |
| `ws/src/dem_bringup/config/dem.yaml` | DEM params (local frame) | 51 |
| `ws/src/dem_bringup/config/dem_utm.yaml` | DEM params (UTM, GNSS-aided) | — |
| `ws/src/dem_bringup/config/fastlio_mid360.yaml` | FAST-LIO2 params | 67 |
| `ws/src/dem_viz/dem_viz/heatmap_node.py` | matplotlib live view | 159 |
