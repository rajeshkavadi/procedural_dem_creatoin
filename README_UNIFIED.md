# Procedural DEM Creation - Unified Tool

Complete pipeline from data collection to georeferenced DEM generation.

## Directory Structure

```
procedural_dem_creatoin/
├── recording/                # Data collection scripts
│   └── record_all_in_one.sh  # Record GPS + LiDAR + IMU in one go
├── tools/
│   └── viewer/              # Point cloud viewer with Cloudflare tunnel
├── ws/                      # ROS workspace (FAST-LIO2, dem nodes)
├── run_pipeline.sh          # Main pipeline: bag -> georeferenced DEM
└── out/                     # Pipeline outputs
```

## 1. Record Data (GPS + LiDAR + IMU)

Connect GPS (u-blox 7) via USB and LiDAR (Mid-360S) via Ethernet. Run as sudo:

```bash
sudo ./recording/record_all_in_one.sh
```

This will:
- Stop and start gpsd, record raw NMEA to `~/gps_raw/ublox7_YYYYMMDD_HHMMSS.txt`
- Wait 60s for GPS warmup
- Configure Ethernet interface (default: `enp0s31f6`) to `192.168.1.50/24`
- Ping LiDAR at `192.168.1.193`
- Launch Livox driver (`rviz_MID360s.launch`)
- Record `/livox/lidar` and `/livox/imu` to `~/livox_bags/mid360s_YYYYMMDD_HHMMSS.bag`
- Press `Ctrl+C` to stop everything

## 2. View Point Cloud in Browser (Cloudflare Tunnel)

One command runs the whole streaming stack — environment, bag replay, GPS
anchor, Cloudflare tunnel and a snapshot/DEM/stream self-test:

```bash
./tools/viewer/scripts/start_cloudflare.sh
```

With no arguments it uses the repository dataset
(`20261030_221123_mid360s.bag` + `ublox7_raw.txt`) and the `/livox/lidar`
topic, prints a public `https://xxx.trycloudflare.com` URL and token
`lidar-snap-2026`. Paste the token in the UI and use **Fetch Point Cloud**,
**Get DEM** and **Start Live View**.

For another recording:

```bash
./tools/viewer/scripts/start_cloudflare.sh \
    --bag ~/livox_bags/rec.bag --gnss ~/gps_raw/ublox7_20261030.txt \
    --topic /livox/lidar
```

Stop viewer:
```bash
./tools/viewer/scripts/stop_all.sh
```

Local only (no tunnel):
```bash
./tools/viewer/scripts/start_cloudflare.sh --no-tunnel
# Open http://127.0.0.1:8085/
```

## 3. Generate Georeferenced DEM from Bag

Make sure workspace is built and sourced (see original README). Run pipeline:

```bash
# Source ROS
source /opt/ros/melodic/setup.bash
source ./ws/devel/setup.bash

# Run pipeline on your bag
./run_pipeline.sh --bag ~/livox_bags/20261030_221123_mid360s.bag --prefix mid360s --out ./out
```

Options:
- `--bag PATH` - ROS bag to process
- `--out DIR` - Output directory (default: ./out)
- `--prefix NAME` - Output prefix (default: outdoorroad)
- `--rate N` - Playback speed (default: 1.0)
- `--rviz` - Open RViz
- `--no-georef` - Stop after DEM/PCD, skip georeferencing

## 4. View Results

Outputs go to `./out/`:
- `*_dem.tif` - Georeferenced DEM (GeoTIFF, WGS84/UTM)
- `*_dem.pcd` - DEM as point cloud
- `*_map_georef.pcd` - Georeferenced FAST-LIO map
- `*_georef.json` - Georeferencing metadata (CRS, transform, RMS)
- `dem_*.npz` - Raw DEM grids
- `odom.csv` - Trajectory

You can view GeoTIFFs with QGIS, or PCD files with CloudCompare/PCL tools.

## Notes

- Your bag must have `/livox/lidar` and `/livox/imu`. For georeferencing with GPS, pipeline expects GNSS data (see README for details).
- GPS raw NMEA is saved to `~/gps_raw/` separately for manual alignment if needed.
- The snapshot viewer works standalone with just the bag (no ROS environment needed).
