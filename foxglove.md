I verified everything against your machine. Key facts that change the commands from the generic guide:
- Bag plays fine on your install: /home/rajesh/proc_dem_creation/OutdoorRoad_dataset_ros2 (657 s, 44.5 GiB).
- You must pass config_file:=outdoorroad_cut0.yaml. The default is mid360.yaml, which subscribes to /livox/lidar + /livox/imu — those topics don't exist in your bag. outdoorroad_cut0.yaml uses /mid360/livox/lidar + /mid360/livox/imu, which do.
- Its lidar_type: 4 maps to enum MID360 (not "other"), so your custom mid360_handler runs — and the bag's cloud does carry the line + timestamp fields it requires. I checked the raw message.
- ros-humble-foxglove-bridge 3.6.0 is available in apt and supports remote_access + device_token (I unpacked the .deb and read the launch file).
- FAST-LIO publishes /cloud_registered (frame camera_init) and TF camera_init → body.
- Do not use use_sim_time:=true / --clock: FAST-LIO runs on a 100 Hz wall-clock timer and would freeze. All stamps already come from bag message time, so cloud + TF stay consistent.
- Your config has map_en: true, so /Laser_map publishes the whole accumulated map every second — it grows the entire bag. Over Remote Access that message is fragmented and dropped. Whitelist it out of the bridge.
Step 1 — One-time install (any terminal)
sudo apt update
sudo apt install -y ros-humble-foxglove-bridge

# confirm
ros2 pkg prefix foxglove_bridge
ros2 launch foxglove_bridge foxglove_bridge_launch.xml --show-args | grep -E "remote_access|device_token"
Step 2 — One-time: create the device token (browser)
1. Sign up / sign in at https://app.foxglove.dev
2. Go to Devices → Create new device (name it e.g. fastlio-sender)
3. Open the device → Settings tab → Create new token
4. Copy the fox_dt-... string
Store it so you don't paste it every time (don't commit it to git):
mkdir -p ~/.config/foxglove
install -m 600 /dev/null ~/.config/foxglove/device_token.env
echo "export FOXGLOVE_DEVICE_TOKEN='fox_dt-PASTE_YOURS_HERE'" > ~/.config/foxglove/device_token.env
chmod 600 ~/.config/foxglove/device_token.env
Terminal 1 — FAST-LIO (start first)
source ~/workspace-tools/fastlio_ws/install/setup.bash

ros2 launch fast_lio mapping.launch.py \
    config_file:=outdoorroad_cut0.yaml \
    rviz:=false
Expect Node init finished. No data yet — it waits for the bag. Keep this running.
Terminal 2 — Foxglove Bridge + Remote Access
source /opt/ros/humble/setup.bash
source ~/.config/foxglove/device_token.env

ros2 launch foxglove_bridge foxglove_bridge_launch.xml \
    remote_access:=true \
    topic_whitelist:="['/cloud_registered','/Odometry','/path','/tf','/tf_static']"
Watch the log: it should report remote access enabled and register with Foxglove. If the token is bad you'll see an auth error here.
topic_whitelist excludes /Laser_map (the growing mega-cloud) and /gnss_pose etc. — less bandwidth, and only these 5 topics are exposed off the machine.
Terminal 3 — Replay the bag
source /opt/ros/humble/setup.bash

ros2 bag play /home/rajesh/proc_dem_creation/OutdoorRoad_dataset_ros2
Add --loop to keep it streaming for a demo (note: FAST-LIO's state doesn't reset on loop, so the trajectory will jump — restart Terminal 1 for a clean run). Add -r 0.5 if it can't keep up, -r 2.0 to go faster.
Terminal 4 — Verify
source /opt/ros/humble/setup.bash

ros2 topic hz /cloud_registered      # should be ~10 Hz
ros2 topic echo /Odometry --once
ros2 node list                       # /fastlio_mapping, /foxglove_bridge
FAST-LIO needs ~1 s of IMU to initialize before the first cloud appears.
Remote viewer (any machine, no ROS needed)
1. Open https://app.foxglove.dev and sign in to the same account/org
2. Devices → your device → Connect
3. Add a 3D panel
4. Topic: /cloud_registered, Fixed frame: camera_init
5. Optional panels: Path (/path), Pose (/Odometry), Plot, Raw Messages (/tf)
Milestone 0: test on LAN first (no token)
Do this before adding Remote Access, so you can tell "ROS problem" from "network problem":
# Terminal 1 & 3 as above, then Terminal 2 becomes:
source /opt/ros/humble/setup.bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
Then in Foxglove Web: Open connection → Foxglove WebSocket → ws://<THIS_MACHINE_IP>:8765 (get the IP with hostname -I). Works from a browser on the same network. Once that renders, switch Terminal 2 to the remote_access:=true version.
Failure points → your actual symptoms
Symptom	Check
Device not listed in Foxglove	Bridge log in Terminal 2; token validity; sender has internet (curl -s ifconfig.me); Devices page shows the device
Connected, no cloud	ros2 topic hz /cloud_registered. If zero: wrong config_file (default mid360.yaml has wrong topics), bag not running, or FAST-LIO printing "Waiting for IMU data"
Cloud renders but empty/gray	Fixed frame must be camera_init, not map or base_link
Cloud frozen	Is Terminal 3 still playing? Is FAST-LIO still logging?
Huge lag / dropped cloud	/Laser_map in the whitelist (remove it), slow the bag with -r 0.5, reduce point_filter_num
Works, then dies after ~1 min	/Laser_map growth exhausting memory — confirm it's whitelisted out
Security notes
- Use the FOXGLOVE_DEVICE_TOKEN environment variable, not device_token:=... on the command line — a launch arg is readable by any process on the ROS network via parameter services; the env var is not.
- When you only use Remote Access (no LAN ws:// testing), add address:=127.0.0.1 to the bridge command so the unauthenticated WebSocket on :8765 isn't reachable from your LAN. Drop it if the device then fails to appear.
- Keep topic_whitelist minimal — everything whitelisted leaves your network.
- Revoke the token from the device's Settings tab if it leaks; tokens are per-device.
- Traffic is TLS/DTLS encrypted in transit, but not end-to-end (Foxglove's SFU can see the data) — fine for a prototype, don't stream anything sensitive.
- Check Foxglove's current plan limits before a public demo, as you noted.
Milestone order: LAN test → Remote Access with token → replace bag playback with FAST-LIO2 running live on the Orin (same 4 commands, Terminal 3 becomes your sensor drivers).
