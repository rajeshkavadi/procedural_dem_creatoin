# Quick Start Commands

## For your recorded data (from yesterday)

You have:
- Bag: `~/livox_bags/20261030_221123_mid360s.bag` (218s, /livox/lidar + /livox/imu)
- GPS raw: `~/gps_raw/ublox7_raw.txt`

### View in browser now
```bash
cd /home/rajesh/procedural_dem_creatoin
./tools/viewer/scripts/start_cloudflare.sh
```

This starts the service on the repository bag + `ublox7_raw.txt`, opens the
Cloudflare tunnel, and self-tests snapshot, DEM and live stream (local and
through the public URL). Public URL + token will be printed. Stop with
`./tools/viewer/scripts/stop_all.sh`.

Other bag:
```bash
./tools/viewer/scripts/start_cloudflare.sh \
    --bag ~/livox_bags/20261030_221123_mid360s.bag \
    --gnss ~/gps_raw/ublox7_raw.txt --topic /livox/lidar
```

### Run DEM pipeline (if you want georeferenced outputs)
```bash
source /opt/ros/melodic/setup.bash
source /home/rajesh/procedural_dem_creatoin/ws/devel/setup.bash
cd /home/rajesh/procedural_dem_creatoin
./run_pipeline.sh --bag ~/livox_bags/20261030_221123_mid360s.bag --prefix mydata --out ./out
```
Note: Your bag doesn't have `/gnss_pose` topic. If georeferencing fails or you need GPS alignment, you'll need to add GPS data to bag or use the raw NMEA. Check `./out/` for results.

## For future recordings
```bash
sudo ./recording/record_all_in_one.sh
```
