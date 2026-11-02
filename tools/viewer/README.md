# Remote LiDAR Point-Cloud Snapshot Service

Replay a recorded **ROS 1 bag** and give a remote browser user the current
point cloud on demand. Clicking **Fetch Point Cloud** captures exactly **one**
`PointCloud2` message and renders **five** PNG views of it; **Get DEM** renders
the elevation grid accumulated since playback started; **Live View** streams
the current cloud as MJPEG.

It is not a streaming application: snapshots and DEMs are produced only when
the user clicks. (Live View is the one continuous mode and is opt-in.)

The service reads exactly **one** `sensor_msgs/PointCloud2` topic — by
default the Livox Mid-360 at `/mid360/livox/lidar`. Change it with `--topic`,
`PCSNAP_TOPIC`, or `topic` in the config file.

---

## 1. Architecture

```
ROS 1 .bag (played/looped by the service, pure-Python rosbags)
        │
        ▼
pc_snapshot/bag_source.py  ── background reader, wall-clock paced
        │
        ▼
PointCloud2 decoder → numpy        (pc_snapshot/decoder.py, ROS-free)
        │
        ▼
SnapshotStore: latest immutable snapshot     (pc_snapshot/store.py)
        │                    │
        │ POST /api/snapshot  │ POST /api/dem
        ▼                    ▼
Open3D / Matplotlib          DemAccumulator → ElevationGrid
five PNG views               (ws/src/dem_mapping) → heatmap PNG
        │                    │
        ▼                    ▼
FastAPI HTTP API            (pc_snapshot/api.py)
        │
        ▼
Browser UI (static/)  ← localhost or Cloudflare quick tunnel
```

Notes:

* **One message per snapshot.** `POST /api/snapshot` captures a reference to
  one decoded message and renders all five views from that same point set and
  one shared bounding box. Incoming messages never mutate or block it.
* **Bounded work.** A non-blocking semaphore caps simultaneous render jobs
  (default 1); a second request returns HTTP 409. Point counts are capped
  (`max_render_points`) with deterministic stride downsampling. Rendered PNGs
  live in a registry with a max-entry cap and a TTL sweeper.
* **Honest staleness.** If the latest message is older than `stale_after_sec`
  the response carries a `warnings[]` entry (or is rejected when
  `stale_policy = "reject"`). A stale message is never labelled "current".
* **Real errors.** Missing topic, no message yet, empty cloud, decode
  failure and render failure each return a specific HTTP status + message.
* **Get DEM.** Every message is also merged **in place** into one fixed
  elevation grid — the same ROS-free `ElevationGrid` / `ground_filters` core
  used by `ws/src/dem_mapping`, imported from the workspace at start-up, so
  there is one implementation of the algorithm. Merging is ~1–2 ms per
  20k-point scan, so the full 10 Hz input is accumulated without dropping
  scans; each click renders whatever has accumulated so far.
* **DEM frame.** The service does no registration, so the grid is filled in
  the raw **sensor frame** (origin at the LiDAR, Z up). That is valid for a
  stationary sensor; point `topic` at a registered topic (for example
  `/cloud_registered` from FAST-LIO2) for a moving one. Points are gated by a
  radial clip (`dem_clip_range_m`), a blind-zone minimum range
  (`dem_min_range_m`, which also drops the Mid-360 driver's `(0,0,0)` filler
  points) and a height band around the sensor (`dem_band_below` /
  `dem_band_above`).
* **GNSS anchor.** `--gnss ublox7_raw.txt` reads the separate gpspipe/NMEA log
  (the recording script stores GPS outside the bag). It reports the valid-fix
  count, satellites, HDOP, first/last fix and the median position with UTM
  coordinates in `/api/health`, annotates the DEM heatmap and includes the
  anchor in the DEM JSON. It does **not** transform the grid: the log has its
  own clock and the service does no registration, so the DEM is still
  sensor-frame and every response says so.

### Renderer threading

This Open3D/Filament build requires a renderer to be used on the thread that
created it, panics if one is destroyed while another exists, and pays a ~5 s
warm-up on the first render. The service therefore keeps **exactly one**
`OffscreenRenderer`, owned by a single dedicated `o3d-render` thread; snapshot
and live-stream jobs are both submitted to it (`renderer.submit_render`), and
it is destroyed once on that thread at shutdown. Matplotlib (Agg) is the
fallback when Open3D is unavailable.

---

## 2. Environment

| Item | Value |
|------|-------|
| OS | Ubuntu 18.04/20.04+ (developed on 22.04, WSL 2) |
| Python | 3.8+ (**required**: FastAPI). 3.8.0 works via `scripts/bootstrap_env.sh`, which installs rosbags despite its 3.8.2 metadata floor |
| ROS | neither `rospy` nor `rosbag` is needed; this runs alongside a ROS Melodic (Python 2) install |
| Open3D | optional (0.17+); Matplotlib is the guaranteed fallback |
| Bag format | ROS 1 `.bag` v2.0 (read with the `rosbags` package) |

### Dataset

Any ROS 1 bag with a `sensor_msgs/PointCloud2` topic. The pipeline recording
used during development (`OutdoorRoad_dataset.bag`) has:

| Topic | Type | Msgs |
|-------|------|------|
| `/mid360/livox/lidar` | `sensor_msgs/PointCloud2` | 6 573 (10 Hz) |
| `/mid360/livox/imu` | `sensor_msgs/Imu` | 131 462 |
| `/gnss_pose` | `geometry_msgs/PoseStamped` | 65 732 |
| ... | | |

The bag holds **raw** LiDAR — there is no `/cloud_registered` in it. The
service is configured for the Livox Mid-360 (`/mid360/livox/lidar`, ~20k
points/message of which ~14k are `(0,0,0)` non-hit fillers, frame
`mid360_frame`); other LiDAR topics are ignored unless you point `topic` at
one. Point fields are read **by name** (`x`, `y`, `z`, `intensity`), so the
Mid-360's extra `tag` / `line` / `timestamp` fields are ignored.

To get a **motion-compensated, registered** cloud, run FAST-LIO2 on the raw
LiDAR + IMU (see the root README), then set `--topic /cloud_registered`. No
code changes are needed.

---

## 3. Setup

```bash
cd snapshot_service

python3 -m venv .venv
. .venv/bin/activate
pip install -r requirements.txt        # fastapi, uvicorn, rosbags, numpy, Pillow, matplotlib, pytest, httpx
# optional, faster/nicer 3-D views:
# pip install open3d
```

The scripts look for `.venv/bin/python` in `snapshot_service/` or the repo
root, then fall back to `python3`. Override with `PCSNAP_PYTHON=/path/to/python`.

---

## 4. Run

### 4.1 One command (service + looping bag)

```bash
./scripts/start_all.sh /path/to/recording.bag
```

Starts the HTTP service on `http://127.0.0.1:8085` and makes the service
itself replay and loop the bag. Health JSON is printed automatically.
Stop everything with `./scripts/stop_all.sh`.

Env overrides: `PCSNAP_TOPIC` (default `/mid360/livox/lidar`),
`PCSNAP_PORT` (8085), `PCSNAP_CLIP_RANGE_M` (60), `PCSNAP_BAG_RATE` (1.0),
`PCSNAP_API_TOKEN`.

### 4.2 Remote access (Cloudflare quick tunnel) — one command

```bash
./scripts/start_cloudflare.sh
```

That is the whole run. With no arguments it uses the recording shipped in the
repository root — `20261030_221123_mid360s.bag` and the GPS log
`ublox7_raw.txt` — on topic `/livox/lidar`, port 8085, and:

1. refuses to run on an unfetched Git LFS pointer (prints the `git lfs pull`
   command instead of starting on a 135-byte file),
2. creates/repairs the Python virtualenv (`scripts/bootstrap_env.sh`),
3. downloads `cloudflared` once if it is missing (Linux x86_64/arm64/arm),
4. starts the service and waits until point clouds really are arriving,
5. opens the quick tunnel and waits until the public URL answers,
6. proves **snapshot**, **DEM** and **live stream** work — first locally, then
   through the Cloudflare URL — and keeps the service running.

```bash
# another dataset / port / playback speed
./scripts/start_cloudflare.sh --bag ~/livox_bags/rec.bag \
    --gnss ~/gps_raw/ublox7_20261030.txt \
    --topic /livox/lidar --port 8085 --rate 1.0

# local only, no tunnel / no self-test
./scripts/start_cloudflare.sh --no-tunnel
./scripts/start_cloudflare.sh --no-test
```

It prints the local URL, the public `https://…trycloudflare.com` URL and the
API token to paste into the UI. `cloudflared` uses only outbound HTTPS, so it
works on networks that block UDP/VPNs. The URL changes on every restart; for
a stable hostname use a named Cloudflare tunnel or Tailscale instead.

A quick-tunnel URL is **publicly reachable by anyone who has the link**, so
the script sets an API token (`lidar-snap-2026` by default — override with
`PCSNAP_API_TOKEN` or `--token`). Do not run a public tunnel without the token.
`start_remote.sh <bag> [topic]` is kept as a backward-compatible wrapper over
this script.

### 4.3 Manual (two terminals)

```bash
# terminal 1 — service with the bag
./scripts/run_service.sh --bag /path/to/recording.bag --topic /mid360/livox/lidar --port 8085

# terminal 2 — live sensor instead of a bag (needs a Python 3 ROS 1 install,
# see §4.5; on Melodic use --bag)
python3 -m pc_snapshot --topic /mid360/livox/lidar
```

Then open `http://127.0.0.1:8085/`.

### 4.4 Test without a recording

```bash
python3 scripts/make_test_bag.py /tmp/test.bag --frames 60 --rate 10
./scripts/run_service.sh --bag /tmp/test.bag
```

This writes a synthetic ROS 1 bag (ground plane + two walls + moving box) with
the same `rosbags` writer the tests use — no ROS installation required.

### 4.5 Useful options

Every option can be set via CLI flag, `PCSNAP_*` environment variable, or a
JSON config file (CLI > env > file > defaults).

```bash
python3 -m pc_snapshot \
  --bag /path/to/recording.bag \      # loop the bag (default); --no-loop plays once
  --bag-rate 1.0 \                    # playback speed
  --bag-start-offset 0 \              # skip seconds into the recording
  --topic /mid360/livox/lidar \
  --gnss /path/to/ublox7_raw.txt \   # raw NMEA: position + DEM annotation
  --port 8085 \
  --renderer auto \                   # auto | open3d | matplotlib
  --up-axis z \                       # z (ROS/robotics) or y (aerospace)
  --color-mode height \               # height | intensity
  --max-render-points 300000 \
  --clip-range 60 \                   # radial clip in meters (0 = off)
  --stale-after 5.0 \
  --api-token mysecret \              # require X-API-Token / ?token=
  --stream-fps 5 \
  --stream-view front \               # top | front | back | left | right
  --dem-resolution 0.25 \             # cell size in metres
  --dem-extent 200 \                  # square grid edge (m), centred on the sensor
  --dem-clip-range 60 \
  --dem-aggregation min \             # min | mean per cell
  --dem-colormap turbo
```

`--no-dem`, `--no-stream` and `--no-ros` disable the DEM, the live stream, or
any point-cloud source respectively. `--inspect` only prints the bag's
topics/duration. If `dem_mapping` cannot be found (`<repo>/ws/src/dem_mapping`
by default, override with `--dem-mapping-path` / `PCSNAP_DEM_MAPPING_PATH`),
the service still starts, reports `dem.enabled = false` with the reason, and
`POST /api/dem` answers 409.

Copy `config.example.json` and pass it with `--config my.json` (or
`PCSNAP_CONFIG=my.json`).

### 4.6 Using a different dataset — the one thing that must match

The service never mixes sensors: it reads **one** topic, the Livox Mid-360 by
default. When another dataset "gives an issue", it is almost always the topic
name or the message type:

| Symptom | Cause | Fix |
|---------|-------|-----|
| `status: "disconnected"`, "no message has arrived on …" | Topic name differs in that dataset | `./scripts/bag_info.sh <bag>` lists the topics; pass the right one: `--topic <topic>` or `PCSNAP_TOPIC` |
| `topic … is not a PointCloud2` / decode failure | It is `livox_ros_driver/CustomMsg` or another type | Convert it to `sensor_msgs/PointCloud2`, or point at a topic that already is one |
| A `.db3` (ROS 2 bag) is rejected | This build reads ROS 1 `.bag` only | record/replay with ROS 1, or convert once with `rosbags-convert` |
| Snapshot renders but looks like a blob at the origin | Topic publishes `(0,0,0)` non-hit fillers | Expected; the renderer clips at `min_range_m` and the DEM drops them at `dem_min_range_m` |
| Image is sparse / low coverage | Far fewer real returns than expected | Accumulate longer, or lower `dem_resolution` to 0.1–0.2 m |
| Nothing happens when clicking Fetch / Get DEM | Bag not replaying, or the service was started without a source | `GET /api/health` shows the source state and message age |

Only `topic` selects the sensor. The default source is the bag; a live ROS 1
subscriber is available when `rospy` is importable from the service's Python 3
interpreter (`ros_master_uri` / `--ros-master`). On a stock Melodic install
`rospy` is Python 2 only, so use `--bag`.

---

## 5. HTTP API

| Method | Path | Description |
|--------|------|-------------|
| GET | `/api/health` | Status, source state, topic, message age, frame id, DEM and GNSS diagnostics |
| POST | `/api/snapshot` | Capture one message, render five views, return metadata + image URLs |
| GET | `/api/snapshot/{id}/{view}.png` | One PNG (`top`, `front`, `back`, `left`, `right`) |
| GET | `/api/snapshot/{id}/download.zip` | ZIP of all five PNGs + `metadata.json` |
| POST | `/api/dem` | Render the DEM accumulated so far |
| GET | `/api/dem/{id}/dem.png` | One top-down elevation heatmap PNG |
| GET | `/api/dem/{id}/download.zip` | ZIP of the heatmap + `metadata.json` |
| GET | `/api/stream?view=…` | Live MJPEG (`multipart/x-mixed-replace`) of the current topic |
| GET | `/` | Browser frontend |

`POST /api/snapshot` returns JSON like:

```json
{
  "snapshot_id": "bfae64ca...",
  "capture_timestamp": 1792999557.6,
  "ros_message_stamp": 1747409937.88,
  "frame_id": "mid360_frame",
  "topic": "/mid360/livox/lidar",
  "original_point_count": 19968,
  "valid_point_count": 19968,
  "rendered_point_count": 19968,
  "render_seconds": 6.7,
  "views": ["top", "front", "back", "left", "right"],
  "images": { "top": ".../top.png", "front": "...", "back": "...", "left": "...", "right": "..." },
  "download_zip": ".../download.zip",
  "warnings": []
}
```

`POST /api/dem` returns the accumulation window, grid shape/resolution/extent,
coverage, elevation range, filters, render time, image URL and ZIP URL.

Status codes: `200` OK · `401` bad/missing token (when `api_token` is set) ·
`404` unknown snapshot/view · `409` no message / stale (reject mode) / render
busy / empty cloud / DEM disabled / nothing accumulated · `500` render
failure.

**Auth:** set `api_token` (or `PCSNAP_API_TOKEN`) to require an `X-API-Token`
header (or `?token=`) on `POST /api/snapshot` and `POST /api/dem`. Leave it
empty only on a private network.

**Live view** — `GET /api/stream?view=front&token=…`:

* Content type `multipart/x-mixed-replace; boundary=frame`; each part is one
  JPEG frame, displayed natively by an `<img>` tag. The token must go in the
  query string (`<img>` cannot set headers).
* `view` selects the camera angle (default `stream_view`, `front`).
  front/back/left/right are first-person: the camera sits at the sensor origin
  and looks outward.
* Frames are rendered only while a viewer is connected, at `stream_fps`
  (default 5) and `stream_width x stream_height` (default 960×540), and stop
  after `stream_idle_stop_sec` without viewers.

---

## 6. Browser frontend

`static/index.html` provides:

* Title **Remote LiDAR Snapshot Viewer** and a connection indicator.
* A configurable **API endpoint** (defaults to the page origin; saved in
  `localStorage`) and an optional API token field.
* A **Fetch Point Cloud** button (single POST; disabled while running).
* A **Get DEM** button and panel: top-down heatmap with colour bar and sensor
  origin marker, plus scans accumulated, points merged, accumulation window,
  grid size/resolution/extent, coverage, elevation range, filters, render
  time, warnings and a **Download PNG** link. Its spinner, status line and
  error box are independent from the snapshot controls.
* A **Live View** panel: Start/Stop, camera-angle selector, the stream
  `<img>`, and a status readout from `/api/health`.
* A five-panel layout labelled Top / Front / Back / Left / Right, capture
  time, ROS message time, frame id, point counts, render time, warnings, error
  area, per-image **Download PNG** links and **Download All Images ZIP**.

Images change only when the corresponding button is clicked; the Live View
stream is continuous by design.

---

## 7. Tests

```bash
cd snapshot_service
python3 -m pytest tests/ -q
```

Tests are ROS-free (duck-typed fake `PointCloud2` messages and a real
synthetic `.bag`). Coverage includes:

- **Decoder**: XYZ + intensity, no-intensity, big-endian, row padding,
  non-finite filtering, empty messages, non-standard field offsets/datatypes,
  missing-field and truncated-buffer errors, downsampling, read-only arrays.
- **Store**: latest-message-wins, capture-before-any-message, empty-message
  rejection, immutable capture, staleness, consistency while new messages
  arrive, thread safety.
- **Renderer**: five views from one snapshot, camera geometry for z-up and
  y-up, bounds/colors, non-blank and distinct PNGs, intensity mode,
  empty/single-point clouds, downsampled counts, forced Open3D backend
  (no silent Matplotlib fallback), shared renderer frame reuse.
- **Bag source**: `.bag` file and split-bag directory resolution, `.db3`
  rejection, `--inspect`, message delivery and header stamps, DEM
  accumulation, single-play `finished` state, auto topic selection, start
  offset, missing-topic and no-PointCloud2 error messages.
- **API**: health with/without data, snapshot rejected when no data, stale
  warn vs reject, full flow returning five valid PNGs, unknown view/id and
  path traversal, ZIP contents, registry eviction, token auth, downsample
  warning, frontend served.
- **Live stream**: multipart framing and JPEG validity, keep-alive
  placeholders, client ref-counting, idle stop, endpoint status codes,
  generator cleanup, health `stream` block.
- **DEM**: grid core import, accumulation/statistics, read-only captures that
  stay stable when new scans arrive, filters, disabled/broken-config paths,
  non-blank heatmaps, empty-grid rejection, health `dem` block, PNG/ZIP flow,
  growth between clicks, token auth, render-busy 409, frontend button.

---

## 8. Project layout

```
snapshot_service/
├── README.md
├── requirements.txt
├── config.example.json
├── pc_snapshot/
│   ├── __init__.py
│   ├── __main__.py          # CLI entrypoint
│   ├── config.py            # env + JSON + CLI configuration
│   ├── decoder.py           # PointCloud2 -> numpy (ROS-free)
│   ├── store.py             # latest-message holder + immutable snapshot
│   ├── bag_source.py        # ROS 1 .bag replay source (rosbags, no ROS)
│   ├── ros_subscriber.py    # optional live rospy subscriber
│   ├── synthetic.py         # synthetic scene + ROS 1 bag writer (tests)
│   ├── renderer.py          # Open3D headless + Matplotlib fallback
│   ├── dem.py               # incremental DEM accumulation + heatmap render│   ├── gnss.py             # gpspipe/NMEA log -> fix summary + anchor
│   ├── stream.py           # live MJPEG frame manager
│   ├── jobs.py              # bounded render jobs + snapshot registry + zip
│   └── api.py               # FastAPI app
├── static/                  # index.html, style.css, app.js
├── scripts/
│   ├── bootstrap_env.sh     # create/repair the Python 3 virtualenv
│   ├── start_cloudflare.sh  # one command: env + bag + tunnel + self-test
│   ├── start_all.sh         # service + looping bag
│   ├── start_remote.sh      # wrapper: start_cloudflare.sh <bag> [topic]
│   ├── run_service.sh       # foreground service
│   ├── stop_all.sh          # stop service + tunnel (+ orphans)
│   ├── bag_info.sh          # topics/duration of a .bag
│   └── make_test_bag.py     # synthetic .bag generator
└── tests/                   # decoder, store, renderer, bag, API, stream, DEM
```

## 9. Notes

* This code is original; the recordings are third-party and are **not**
  redistributed here (only local paths are referenced).
* For a Jetson or another deployment target, copy `snapshot_service/`,
  install `requirements.txt`, and point `--topic` at whatever PointCloud2
  topic the platform publishes (raw sensor or registered). Nothing else
  changes.
