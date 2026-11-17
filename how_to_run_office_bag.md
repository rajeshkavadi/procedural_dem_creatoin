Your office bag has no GPS, so override the script's default GNSS file with a nonexistent path — it then runs with no anchor and the DEM is generated in the local LiDAR frame.
1. Cloudflare viewer + localized DEM (one command)


cd /home/rajesh/procedural_dem_creatoin
./tools/viewer/scripts/start_cloudflare.sh \
  --bag /home/rajesh/procedural_dem_creatoin/datasets/office_out/20261030_221123_mid360s.bag \
  --gnss /nonexistent-office-gps \
  --topic /livox/lidar


It auto-creates the venv, downloads cloudflared if needed, opens the tunnel, and self-tests DEM + live stream. Watch for:
GNSS  : <none>
GNSS: unavailable
PASS DEM (...)
PASS live stream (...)
Public URL: https://xxx.trycloudflare.com
Open the public URL (no token needed), press Start for the live point cloud and switch the View selector for other angles. The DEM tab renders the accumulated map and refreshes every 10 s; its output includes "frame" = local frame.
Stop with:
./tools/viewer/scripts/stop_all.sh
Local only (no tunnel):
./tools/viewer/scripts/start_cloudflare.sh \
  --bag /home/rajesh/procedural_dem_creatoin/datasets/office_out/20261030_221123_mid360s.bag \
  --gnss /nonexistent-office-gps --topic /livox/lidar --no-tunnel
# http://127.0.0.1:8085/
Smoother live stream (fewer points per frame, higher frame-rate cap):
./tools/viewer/scripts/start_cloudflare.sh \
  --bag /home/rajesh/procedural_dem_creatoin/datasets/office_out/20261030_221123_mid360s.bag \
  --gnss /nonexistent-office-gps --topic /livox/lidar \
  --stream-points 9000 --stream-fps 12
2. Offline localized DEM from the bag (no GPS/georeferencing)
source /opt/ros/melodic/setup.bash
source /home/rajesh/procedural_dem_creatoin/ws/devel/setup.bash
cd /home/rajesh/procedural_dem_creatoin
./run_pipeline.sh \
  --bag /home/rajesh/procedural_dem_creatoin/datasets/office_out/20261030_221123_mid360s.bag \
  --prefix office --out ./out/office --no-georef
Outputs land in ./out/office/ — local DEM grids/PCD, no GeoTIFF/UTM. --no-georef is the key flag when there's no GPS.
Build · Big Pickle · 38.8s · 50.6 tok/s
rajesh@rajesh-S410:~/procedural_dem_creatoin$ cd /home/rajesh/procedural_dem_creatoin
rajesh@rajesh-S410:~/procedural_dem_creatoin$ ./tools/viewer/scripts/start_cloudflare.sh \
>   --bag /home/rajesh/procedural_dem_creatoin/datasets/office_out/20261030_221123_mid360s.bag \
>   --gnss /nonexistent-office-gps \
>   --topic /livox/lidar
WARNING: GNSS log not found: /nonexistent-office-gps (continuing without an anchor)
Bag   : /home/rajesh/procedural_dem_creatoin/datasets/office_out/20261030_221123_mid360s.bag
GNSS  : <none>
Topic : /livox/lidar
Port  : 8085
Setting up the Python environment (first run only)…
ERROR: environment setup failed; see /home/rajesh/procedural_dem_creatoin/tools/viewer/.run/bootstrap.log
ERROR: Python 3.8+ is required and none was found.
       Install it (e.g. sudo apt install python3.8-venv) or set PCSNAP_PYTHON.
rajesh@rajesh-S410:~/procedural_dem_creatoin$ what is this issue
+Thought · 329ms
→Explored: 1 read
$ python3 --version; command -v python3 python3.8 python3.9 python3.10 python3.11 python3.12; lsb_release -a 2>/dev/null || cat /etc/os-release | head
Python 3.6.9
/usr/bin/python3
Distributor ID:	Ubuntu
Description:	Ubuntu 18.04.6 LTS
Release:	18.04
Codename:	bionic
+Thought · 1.3s
$ ls /usr/bin/python* /opt/*/bin/python* 2>/dev/null; apt-cache policy python3.8 python3.8-venv 2>/dev/null; command -v add-apt-repository; whoami
(24 earlier lines)
  Candidate: 3.8.0-3ubuntu1~18.04.2
  Version table:
     3.8.0-3ubuntu1~18.04.2 500
        500 http://in.archive.ubuntu.com/ubuntu bionic-updates/universe amd64 Packages
        500 http://security.ubuntu.com/ubuntu bionic-security/universe amd64 Packages
/usr/bin/add-apt-repository
rajesh
+Thought · 2.3s
Issue: Ubuntu 18.04's default python3 is 3.6.9, but the viewer service needs Python 3.8+ (rosbags/FastAPI). No python3.8 is installed, so bootstrap_env.sh aborts.
Good news: python3.8 is available in your configured apt repos (bionic-updates). Install it, then rerun:
sudo apt update
sudo apt install -y python3.8 python3.8-venv python3.8-dev
python3.8 --version
Then run the same command again:
cd /home/rajesh/procedural_dem_creatoin
./tools/viewer/scripts/start_cloudflare.sh \
  --bag /home/rajesh/procedural_dem_creatoin/datasets/office_out/20261030_221123_mid360s.bag \
  --gnss /nonexistent-office-gps \
  --topic /livox/lidar
The bootstrap script will automatically pick python3.8, create /home/rajesh/procedural_dem_creatoin/tools/viewer/.venv, install deps (including the 3.8.0-compatible rosbags pin), and then start the tunnel.
If you already have a 3.8+ interpreter somewhere else, you can skip apt and use:
PCSNAP_PYTHON=/path/to/python3.8 ./tools/viewer/scripts/start_cloudflare.sh ...
After the install, if bootstrap fails again, check:
tail -40 /home/rajesh/procedural_dem_creatoin/tools/viewer/.run/bootstrap.log
