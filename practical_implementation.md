# Practical Implementation: Mid-360 + NMEA GPS to Georeferenced DEM

## Scope

This document captures the practical plan and the relevant dataset/sensor facts for
collecting Mid-360 LiDAR data together with a USB/NMEA GPS dongle, then generating a
georeferenced DEM using the existing FAST-LIO2 + DEM + georeferencing pipeline.

X-AnyLabeling is **not** part of this workflow. It is a 2D image/label annotation GUI
and does not replace FAST-LIO2, ground filtering, DEM generation, or georeferencing.
It would only be useful later if a camera is added and images need object labels.

Important distinction:

- The **Mid-360 LiDAR** produces the geometry used for the DEM.
- The **GPS** does not create the DEM. It only anchors the local LiDAR map/DEM into
  WGS84/UTM coordinates.

---

## Dataset GPS Units (`OutdoorRoad_dataset_ros2`)

Topic: `/gnss_pose` (`geometry_msgs/msg/PoseStamped`)

| Field | Meaning | Unit |
|---|---|---|
| `pose.position.x` | WGS84 latitude | degrees |
| `pose.position.y` | WGS84 longitude | degrees |
| `pose.position.z` | altitude | metres |
| `pose.orientation` | quaternion | unitless |
| `header.frame_id` | `imu_link` | - |

Notes:

- Values are approximately `x = 60.44`, `y = 22.31`, `z = 39` — Turku, Finland.
- This is **not** local metric GNSS and **not** the usual `x=East, y=North` order.
- UTM zone is 34N / EPSG:32634.
- Other sensor units for context:
  - LiDAR `x/y/z`: metres
  - IMU linear acceleration: m/s²
  - IMU angular velocity: rad/s

---

## GPS Dongle NMEA Output

The dongle outputs standard NMEA 0183 sentences at roughly 1 Hz.

Observed example state:

- `GLL` / `RMC` status is `V` (void) — no valid fix.
- `GGA` fix quality is `0` — no position.
- `GSA` mode is `1` — no fix.
- `GSV` lists satellites/SNR but no position.

So the sample output is **not yet usable for georeferencing** because there is no
valid position.

When a fix is available:

| Sentence | Useful fields | Units / format |
|---|---|---|
| `GGA` | latitude, longitude, altitude, geoid separation, HDOP, fix quality | `ddmm.mmmm` / `dddmm.mmmm`, metres |
| `RMC` | latitude, longitude, speed, course, UTC time/date, validity | degrees, knots, degrees true |
| `VTG` | speed, course | knots / km/h, degrees |
| `GSA` | fix type, DOP values | 2D/3D, unitless |
| `GSV` | satellite elevation, azimuth, SNR | degrees, dB-Hz |

Key point: NMEA latitude/longitude are in `ddmm.mmmm` format, not decimal degrees.
They must be converted before use.

Current output is enough to diagnose fix status, but **not enough for a georeferenced
DEM until a valid fix appears**.

---

## Accuracy Expectations

- Single-point NMEA GPS is typically about **2–5 m horizontal** and **5–15 m vertical**.
- This is sufficient for a **coarse georeferenced DEM**, not survey-grade work.
- For better accuracy, use RTK/PPK GNSS or ground control points.
- `GGA` altitude is altitude above mean sea level (MSL), not ellipsoidal height.
  Choose and document the vertical datum consistently.

---

## Field Collection Plan

### 1. Before going out

- Confirm the GPS dongle gets a valid fix:
  - `GGA` fix quality `1` or better
  - at least roughly 6 satellites
  - HDOP below about 2
- If the dongle stays at status `V` or quality `0`, do not rely on it for that run.
- Identify the serial port, usually `/dev/ttyUSB0` or `/dev/ttyACM0`.
- Check the baud rate, typically 9600, 4800, or 115200.
- Add the user to the `dialout` group if needed.
- Confirm the Mid-360 driver publishes LiDAR and IMU.
- Record a 30 s test bag before the real run.
- Decide the coordinate convention now: NMEA gives latitude, longitude, and MSL
  altitude; the existing pipeline expects `x=lat`, `y=lon`, `z=alt` in `/gnss_pose`.
- Measure the GPS antenna position relative to the Mid-360/IMU (lever arm). Even rough
  values help.

### 2. Recording setup

Record simultaneously:

- Mid-360 LiDAR + IMU into a rosbag2.
- Raw NMEA to a text file, one line per sentence, with a host timestamp added to each
  line.
- Ideally also run `nmea_navsat_driver` or `gpsd` so the bag contains `/fix` and `/vel`.

Minimum files to keep:

- `rosbag2/` containing `/mid360/livox/lidar` and `/mid360/livox/imu`
- `gps.nmea` raw log
- a short `notes.txt` with date, route, antenna height/position, start/end times, and
  weather/sky-view notes

### 3. Time synchronization

This is the biggest risk.

- Best: GPS and Mid-360 on the same laptop so host time is common.
- Better: GPS with PPS plus `gpsd`/`chrony` to discipline the system clock.
- If the dongle only logs to a pendrive and has no PPS, expect a time offset.
  Mitigate by:
  - logging host time beside every NMEA line,
  - starting and ending with the vehicle stationary for 30–60 s,
  - using NMEA UTC time and host timestamps to estimate offset/drift afterwards.
- Do not assume synchronization; check it numerically later.

### 4. Route and driving

For a usable DEM:

- Start stationary for about 30 s, then move; end stationary for about 30 s.
- Drive slowly, roughly 3–6 m/s, at steady speed.
- Use parallel swaths with overlap, plus one or two loops or figure-eights.
- Include gentle turns and speed changes so FAST-LIO gets IMU excitation.
- Keep good sky view; avoid tunnels, dense tree canopy, and building canyons.
- Stay within Mid-360 ground-return range and keep a sensible sensor height/angle.
- If possible, revisit a known point at the end to check drift and loop closure.

### 5. Post-collection QA

Before processing, check:

- bag duration and message counts
- LiDAR/IMU gaps
- GPS fix percentage, satellite count, and HDOP
- whether NMEA latitude/longitude actually appear in `GGA`/`RMC`
- whether start/end stationary periods match between GPS and LiDAR
- missing or duplicated timestamps

If GPS fix is poor, stop: a local DEM may still be possible, but a trustworthy
georeferenced DEM is not.

---

## Processing Pipeline

1. Run FAST-LIO2 with the Mid-360 config.
   - Keep `extrinsic_est_en: false` as noted in the existing project documentation.
2. Produce the local map and trajectory.
3. Parse NMEA `GGA`/`RMC` into latitude, longitude, altitude, UTC time, and fix quality.
4. Convert WGS84 to UTM using the zone derived from longitude.
5. Interpolate the GNSS track to the LiDAR trajectory timestamps.
6. Estimate a rigid transform between local LIO trajectory and UTM GNSS track:

   `p_utm = R * p_local + t`

   Use robust fitting and inspect the RMS residual.

7. Build the DEM from ground-classified LiDAR points:
   - build locally and transform to UTM, or
   - grid directly in UTM.
8. Export:
   - GeoTIFF DEM
   - DEM PCD
   - georeferenced map PCD
   - provenance JSON with CRS, transform, RMS, and input files
9. Validate:
   - overlay on satellite imagery
   - inspect hillshade
   - check cross-sections
   - check against known points or control points

---

## Failure Modes and Fallbacks

| Problem | Effect | Mitigation |
|---|---|---|
| No GPS fix | No georeferencing | Wait for fix, move to open sky, or use local DEM only |
| Poor GPS accuracy | Shifted/noisy DEM | Use RTK/PPK or ground control points |
| Time offset between GPS and LiDAR | Horizontal/vertical shifts | Log host time, use stationary start/end, estimate offset |
| Wrong antenna lever arm | Biased transform | Measure antenna offset; document it |
| LIO drift | Distorted local map | Slow driving, loops, stable extrinsics, IMU excitation |
| Insufficient ground returns | Poor DEM | Adjust sensor height/angle, slow down, overlap swaths |
| GPS dropouts | Gaps in georeferencing | Interpolate carefully; keep good sky view |

Fallback options:

- No GPS fix: produce a local DEM only.
- Bad GPS: use the existing `OutdoorRoad_dataset_ros2` or add ground control points.
- Weak LIO: slow down, add loops, and keep IMU/LiDAR extrinsics fixed.

---

## Short Version

Collect a Mid-360 rosbag plus timestamped raw NMEA on the same machine, drive slowly
with overlapping loops, QA the GPS fix and time synchronization, then feed the data
through the existing FAST-LIO2 and georeferencing pipeline. GPS only supplies the
absolute reference; the LiDAR supplies the DEM geometry.
