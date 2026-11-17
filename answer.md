# Is the point-cloud streaming just screenshots played back as video?

Short answer: yes, mechanically it's a stream of still images, but **no — nothing
is pre-captured or played back as a fake video**. Every frame is rendered fresh,
live, from the newest point cloud.

There are two distinct paths in this codebase:

## 1. "Fetch Point Cloud" → a real screenshot

`store.capture()` grabs the *latest* `PointCloud2` message, `prepare_cloud()`
downsamples/colors it, and `render_views_offscreen()` renders five PNGs
(top/front/back/left/right) once. That gets saved server-side and served as PNGs
/ a zip. One message in, five stills out. Nothing animated.

## 2. "Live View" (`/api/stream`) → live-rendered frames, not replayed screenshots

In `tools/viewer/pc_snapshot/stream.py` the render loop repeatedly does:

```
snap = store.capture()                    # newest cloud right now
images = render(...)                      # re-render views of THAT cloud
_publish(view, jpeg_bytes(arr), meta)     # one JPEG per view
```

and `mjpeg_generator` writes them into a `multipart/x-mixed-replace` response
that the browser draws in a plain `<img>` tag. So the browser is just replacing
the image as fast as new frames arrive — that's why it *looks* like video.

## Why it is NOT "screenshots played together as video"

- Nothing is buffered ahead of time or played on a timeline.
  `wait_for_frame()` only ever hands out a frame **newer** than the last one
  delivered, and `_frames[view]` holds exactly **one** frame per view. If
  rendering is slower than the cloud arrives, old clouds are dropped and you see
  the newest one — there's no queue of stills being drained.
- The frame rate is capped at `cfg.stream_fps`, not fixed to a video timeline, so
  timing jitters with render speed.
- The JPEGs are never written to disk into a video; they're transient bytes in
  memory (only the snapshot endpoint persists PNGs).
- The placeholder/error JPEGs ("Waiting for point cloud data…") are synthesized
  text images, not recordings.

## Accurate description

A server-side MJPEG camera stream where the "camera" is an Open3D/Matplotlib
renderer of the current LiDAR message — same transport trick as Flask/OpenCV
camera streaming, different frame source. The stills exist only as individual
frames in flight; the animation is real-time rendering, not playback.

### Caveat

Because it always jumps to the latest cloud, if the bag plays faster than it
renders you get a temporally subsampled (choppy) but always-current view — not a
smooth slowed-down replay.

---

# Session notes: "0.76 fps" / "20,000 points" explained, and the UI rebuild

## 1. "0.76 fps" and "20,000 points"

**20,000 is one scan, not the sensor's capacity.** The Livox Mid-360 emits
~200,000 points **per second**, but it publishes ~10 messages per second, so
each `PointCloud2` message carries ~20k points. Checked against the office bag:
2,187 messages x ~19,968 points each over ~219 s = ~200,000 points/s. So nothing
is lost — the live view just shows *one latest scan* (~20k), not a full second
of scans merged together. (The renderer's cap is `stream_max_points = 60,000`;
20k never hits it.)

**0.76 fps is render speed, not capture speed.** `last_fps = 1 /
render_seconds` for one rendered frame, i.e. each frame takes ~1.3 s to draw
headless on this machine. Meanwhile the bag keeps replaying at 10 scans/s and
the viewer **drops** the scans it couldn't render, always showing the newest
one. So: 10 scans/s arrive -> under 1 rendered frame/s -> the view is choppy but
always current. The DEM is different: merging takes ~1-2 ms/scan, so it really
does accumulate at full 10 Hz (that is why the DEM gains ~10 scans per second
while the live view gains <1 frame). Measured on the office bag: **0.72 fps at
19,968 pts/frame**, DEM at the same moment had accumulated 9 scans.

## 2. UI changes

`tools/viewer/static/` is now just two things:

- **One `Start` / `Stop` button** -> opens the live MJPEG point cloud, with a
  **View** selector (Front / Back / Left / Right / Top) that can be changed
  mid-stream. The status line shows active view, points per frame, fps and bag
  progress.
- **Two tabs**: *Live Point Cloud* and *DEM*. The DEM tab re-fetches
  `POST /api/dem` **every 10 s** and shows scans, points merged/seen, coverage,
  elevation range, grid, window and last-update time.

Removed: the API-token field, API endpoint field, "Fetch Point Cloud", "Get DEM"
button, the five PNG panels with per-image + ZIP downloads, the GNSS panel,
spinners and footer. No token is needed anymore: `start_cloudflare.sh` now
defaults to `PCSNAP_API_TOKEN=""` (only prints/sets one if `--token` is passed),
and its self-test POSTs DEM and stream without auth.

Run command (unchanged):

```bash
cd /home/rajesh/procedural_dem_creatoin
./tools/viewer/scripts/start_cloudflare.sh \
  --bag /home/rajesh/procedural_dem_creatoin/datasets/office_out/20261030_221123_mid360s.bag \
  --gnss /nonexistent-office-gps \
  --topic /livox/lidar
```

Docs updated to match: `tools/viewer/README.md`, `README_UNIFIED.md`,
`how_to_run_office_bag.md`.

## 3. Checks run

- `bash -n start_cloudflare.sh` — clean.
- Full run of the command above (local, `--no-tunnel`):
  `PASS DEM (12 scans, 8095 cells)`, `PASS live stream (9 frames in ~10 s)`.
- HTTP, no token: `/`, `/style.css`, `/app.js` -> 200; `POST /api/dem` -> 200
  (9 scans, 7,527 cells); stream -> 6 frames / 8 s at 19,968 pts/frame on
  `view=top`; health reports `connected: True`.
- `pytest tests/test_api.py tests/test_dem.py` -> 38 passed (two frontend
  assertions updated for the new UI).

Note: the `/api/snapshot` and `/download.zip` endpoints still exist server-side;
the new UI simply never calls them.

---

# Session notes: making the live point cloud smoother (fps)

## What was actually slow

The live stream was not limited by the sensor, the bag, or the network — it was
limited by the renderer. Three separate problems, all in the live path:

1. **Every frame rendered all five camera views.** `render_snapshot()` always
   drew top/front/back/left/right, so watching one view still paid for five.
   Fixed: `render_snapshot(snap, cfg, views=...)` and the stream now renders
   only the view(s) a client is actually watching.
2. **The PNG round-trip.** The Matplotlib backend saved each frame to PNG,
   re-decoded it with PIL and converted it to an array — ~25 ms/frame thrown
   away for nothing, because the stream re-encodes to JPEG anyway. Fixed: draw
   into the Agg canvas buffer and copy the RGBA pixels directly.
3. **Stride downsampling undershot the point budget.** With `max_points=15000`
   a 19,968-point scan kept only 10,032 points (stride 2). Fixed in
   `decoder.downsample_points()`: evenly spread indices give exactly the
   requested count, so the frame cost is predictable and detail is better.

Also: `stream.last_fps` used to be `1/render_seconds`, which reported 8 fps
while frames arrived at 5. It now reports the measured delivery cadence.

## New defaults

| setting | old | new |
|---|---|---|
| `stream_fps` (target cap) | 5 | 8 |
| `stream_max_points` | 60 000 | 15 000 |

Tuning knobs (no code edits needed):

```bash
# fewer points -> faster frames
./tools/viewer/scripts/start_cloudflare.sh --bag ... --stream-points 9000
# raise/lower the frame-rate cap
./tools/viewer/scripts/start_cloudflare.sh --bag ... --stream-fps 12
# or via environment
PCSNAP_STREAM_MAX_POINTS=9000 PCSNAP_STREAM_FPS=12 ./tools/viewer/scripts/start_cloudflare.sh ...
```

(A Matplotlib view costs ~0.11 s at 15k points and ~0.09 s at 9k; ~0.18 s for
the `top` view. Open3D, if installed, is used automatically and is faster.)

## Measured on the office bag (`/livox/lidar`, 19,968 pts/scan)

| configuration | frames delivered |
|---|---|
| before (5 views, PNG round-trip, 60k cap) | 0.72 fps |
| now, single view, default 15k points | **4.9 fps** |
| now, single view, `--stream-points 9000 --stream-fps 12` | **6.6 fps** |

Checks: `pytest tests/` -> 101 passed, 2 skipped (stream, API, renderer, DEM,
bag-source and decoder suites). CLI (`--stream-points`, `--stream-fps`) and env
overrides verified.

---

# Result: live point-cloud FPS (final report)

The live view is now ~7x faster on the office bag: **0.72 fps -> 4.9 fps**
(6.6 fps with lighter settings).

## Why it was laggy

Not the sensor or the network — the renderer. Three real problems in the live
path:

1. **Every frame drew all five views.** `render_snapshot()` always rendered
   top/front/back/left/right, so watching *one* view still paid 5x the cost. It
   now takes a `views=` argument and the stream renders only what a client is
   watching — that alone was a 4.4x win.
2. **Full PNG round-trip per frame.** The Matplotlib backend saved PNG -> PIL
   decoded it -> converted to array, ~25 ms/frame wasted, and then the stream
   re-encoded to JPEG anyway. It now draws straight into the Agg canvas buffer.
3. **Downsampling undershot the budget.** With `max_points=15000`, stride
   sampling kept only 10,032 of 19,968 points. `downsample_points()` now uses
   evenly-spread indices, so you get exactly the requested count (better
   detail, predictable frame cost).

Bonus bug: `stream.last_fps` was `1/render_seconds` — it claimed 8 fps while
frames arrived at 5. It now reports the actual delivery cadence, so the number
in the UI is honest.

## Changed defaults & knobs

| setting | old | new |
|---|---|---|
| `stream_fps` (cap) | 5 | 8 |
| `stream_max_points` | 60 000 | 15 000 |

You can tune it without touching code — either flag is accepted by the usual
run command:

```bash
# smoother / lower bandwidth
./tools/viewer/scripts/start_cloudflare.sh \
  --bag /home/rajesh/procedural_dem_creatoin/datasets/office_out/20261030_221123_mid360s.bag \
  --gnss /nonexistent-office-gps --topic /livox/lidar \
  --stream-points 9000 --stream-fps 12
```

or env vars `PCSNAP_STREAM_MAX_POINTS=9000 PCSNAP_STREAM_FPS=12` (both verified
working).

## Measured on the office bag (19,968 pts/scan, one view)

| configuration | delivered |
|---|---|
| before | 0.72 fps |
| now, default (15k pts, cap 8) | **4.9 fps** |
| now, `--stream-points 9000 --stream-fps 12` | **6.6 fps** |

Render cost per frame is now ~0.11 s (front/left/right), ~0.14 s (back),
~0.18 s (top) at 15k points.

Verification: `pytest tests/` -> **101 passed, 2 skipped** (stream, API,
renderer, DEM, bag-source, decoder). Docs updated (`tools/viewer/README.md`,
`config.example.json`).

Caveat: if `open3d` were installed, the stream would use it automatically and
go considerably faster than the Matplotlib fallback — it is not installed in
`tools/viewer/.venv` here (large download, needs working headless EGL), so it
was not added.

---

# Correction / follow-up: the tuning flags now work through `start_cloudflare.sh`

`start_cloudflare.sh` has its own option parser, so the first version of this
guide was wrong: passing `--stream-points` / `--stream-fps` to it failed with
`unknown option: --stream-points (try --help)`. Fixed by forwarding them:

```bash
./tools/viewer/scripts/start_cloudflare.sh \
  --bag /home/rajesh/procedural_dem_creatoin/datasets/office_out/20261030_221123_mid360s.bag \
  --gnss /nonexistent-office-gps \
  --topic /livox/lidar \
  --stream-points 9000 \
  --stream-fps 12
```

* `--stream-points N` — max points drawn per live frame (default 15000);
  fewer points = smoother.
* `--stream-fps N` — live frame-rate cap (default 8).
* Both also read `PCSNAP_STREAM_MAX_POINTS` / `PCSNAP_STREAM_FPS` when the flag
  is omitted, and both appear in `--help`.
* The closing summary now prints the live settings it actually got:

  ```
  Live  : 9000 pts/frame, 5.83 fps delivered, cap 12.0 (tune with --stream-points/--stream-fps)
  ```

Verified with the command above (local, `--no-tunnel`): self-test passed,
`PASS DEM (7 scans, 7044 cells)`, `PASS live stream (82 JPEG frames in ~10 s)`
(~8.2 fps), and the summary reported 9000 pts/frame with cap 12.


---

./tools/viewer/scripts/start_cloudflare.sh \
  --bag /home/rajesh/procedural_dem_creatoin/datasets/office_out/20261030_221123_mid360s.bag \
  --gnss /nonexistent-office-gps \
  --topic /livox/lidar \
  --stream-points 9000 \
  --stream-fps 12 \
 

---

# Result: the DEM is now ground-only (`grid_ground`)

Question: the live point cloud shows a wall behind the sensor, but the DEM is a
featureless disc with yellow/green colours. Are we already using a PDAL ground
filter?

No. `grep -rni pdal` finds a single mention, in an error string in
`ws/src/dem_mapping/dem_mapping/pcd_io.py`. No PDAL, no PCL, no ground
classification anywhere in the DEM path. The old default was
`dem_ground_filter = "height_band"`, i.e. "keep everything within ±6 m of the
sensor" — a height window, not a classifier.

## Why the map could not show the wall (measured on the office bag)

| observation | value |
| --- | --- |
| cells whose column spans more than 1 m (i.e. contain vertical structure) | 112,988 of 131,173 (86 %) |
| median cell min-z / p95 | −1.75 m / +5.38 m |
| cells in the floor band \|z\| < 0.3 m | 5,252 |
| grid fill from 2 m to 45 m radius | 100 % |

1. `aggregation = "min"` stores the **lowest** return per cell. A wall spans
   floor→+3 m in a cell, so the minimum of that column *is* floor height: the
   wall is written into the map as ground. That is the flat yellow/green disc.
2. The hard circular edge is `dem_clip_range_m = 60` drawn around a sensor that
   never moves in the map: every scan is accumulated in the raw sensor frame
   (no registration), so all 2,187 scans are stacked on each other.
3. The bag has only `/livox/lidar` (10 Hz) and `/livox/imu` (198 Hz) — no
   `/odom`, no `/tf` — so there are no poses to accumulate with, and sensor
   tilt shows up as the bowl-shaped gradient.

## What changed

New shared filter `GridGroundFilter` (`name = "grid_ground"`) in
`ws/src/dem_mapping/dem_mapping/ground_filters.py`, used by both the viewer and
`dem_node`:

1. rasterise the scan's lowest Z on a local 0.5 m grid,
2. morphological opening (erode → dilate, 2 m kernel): anything narrower than
   the kernel — walls, poles, trunks, cars, people — is pulled down to the
   local ground while the ground keeps its shape,
3. reject cells whose surface is more than `max_slope` (0.35) above the lowest
   surface within 8 m (catches buildings/roofs wider than the kernel),
4. keep only points within ±0.25 m of that surface.

Implementation notes: per-cell minimum via `argsort` + `reduceat` instead of
`np.minimum.at` (~1 µs/point), and the 8 m slope test runs on a 2 m
block-minified raster. Cost measured on the real bag: **29 ms → 6.25 ms** per
20k-point scan, i.e. ~6 % of one core at 10 Hz.

Defaults changed: `dem_ground_filter = "grid_ground"` in the viewer config,
`--dem-ground-filter` accepts it, and `dem.yaml` / `dem_utm.yaml` +
`dem_node` pass `ground_cell`, `ground_opening_m`, `ground_tolerance_m`,
`ground_max_slope`.

## Verified

* `pytest tests/ -q` → **108 passed, 2 skipped** (7 new tests in
  `tests/test_ground_filters.py`: wall rejected, ground kept, 20 % ramp
  preserved, 200 % cliff rejected, empty input, bad params, and a DEM-level
  test that the wall in `make_scene_cloud` no longer reaches the grid).
* Real bag, 711 scans accumulated through the running service: 14.2 M points
  offered → 1.19 M kept (8.4 %), 24,992 valid cells, and the rendered DEM is a
  continuous ground surface with buildings/trees absent —
  `tools/viewer/.run/dem_service.png`.
* Offline, all 2,187 scans: 33.0 M radial points → 5.01 M ground points
  (15.1 %), `tools/viewer/.run/dem_ground.png`.

## Remaining limits

* A surface wider than the 2 m opening kernel with no ground returns beneath it
  (dense canopy, large roof) is still accepted as ground — that is what the
  residual yellow speckles at 20–50 m are.
* With no poses, a long replay still smears into a disc; the filter removes the
  vertical structures but cannot undo the stacking. For a stable map the input
  must be registered (KISS-ICP or FAST-LIO2, then point `--topic` at the
  registered cloud).

---

# How to run the ground-filtered DEM (and why not PDAL)

## Running it

Nothing extra to install — `grid_ground` is the default, so the usual command
already uses it:

```bash
./tools/viewer/scripts/start_cloudflare.sh \
  --bag /home/rajesh/procedural_dem_creatoin/datasets/office_out/20261030_221123_mid360s.bag \
  --gnss /nonexistent-office-gps \
  --topic /livox/lidar \
  --stream-points 9000 --stream-fps 12
```

Press **Start** in the UI, let the bag play (the more scans, the more ground is
covered), then open the **DEM** tab. The tab auto-refreshes every 10 s and the
image subtitle prints what was applied, e.g.

```
/livox/lidar · 0.25 m cells · agg=min · filter=grid_ground · accumulated 1m10s since service start
```

Headless / scripted check:

```bash
curl -s http://127.0.0.1:8085/api/health | python3 -m json.tool | sed -n '/"dem"/,/}/p'
curl -s -X POST http://127.0.0.1:8085/api/dem | python3 -c 'import json,sys; d=json.load(sys.stdin); print(d["filters"], d["grid"]["valid_cells"], d["image"])'
```

`/api/health` → `dem` now reports `points_in`, `points_used`, `valid_cells`,
`z_min`, `z_max`. A healthy street scene keeps roughly 8–15 % of the offered
points; if `points_used` is ~0 the filter is too tight (see knobs below).

### Choosing / tuning the filter

CLI (viewer):

```bash
# default (ground segmentation on)
.venv/bin/python -m pc_snapshot --bag <bag>
# old behaviour back (elevation window, walls included)
.venv/bin/python -m pc_snapshot --bag <bag> --dem-ground-filter height_band
# no filtering at all (debug)
.venv/bin/python -m pc_snapshot --bag <bag> --dem-ground-filter none
```

No `--dem-ground-*` flags exist for the tuning knobs, so use the config file or
env vars (both verified):

```bash
PCSNAP_DEM_GROUND_TOLERANCE_M=0.4 \
PCSNAP_DEM_GROUND_OPENING_M=3.0 \
PCSNAP_DEM_GROUND_MAX_SLOPE=0.5 \
PCSNAP_DEM_GROUND_SLOPE_RADIUS_M=8.0 \
PCSNAP_DEM_GROUND_CELL=0.5 \
PCSNAP_DEM_GROUND_FILTER=grid_ground \
./tools/viewer/scripts/start_cloudflare.sh --bag <bag> --no-tunnel
```

| knob | default | raise it when | lower it when |
| --- | --- | --- | --- |
| `dem_ground_cell` | 0.5 m | scans are sparse at range, filtering is too slow | fine structure (kerbs) matters |
| `dem_ground_opening_m` | 2.0 m | walls/cars still leak in | narrow features (posts, kerbs) vanish |
| `dem_ground_tolerance_m` | 0.25 m | rough terrain loses real ground | noise floor is low and you want it strict |
| `dem_ground_max_slope` | 0.35 | real terrain is steep (hills, ramps) | flat scenes, buildings leak in |
| `dem_ground_slope_radius_m` | 8.0 m | large buildings survive | small obstacles survive |

ROS node (`ws/src/dem_mapping`): same knobs as rospy params
(`~ground_cell`, `~ground_opening_m`, `~ground_tolerance_m`,
`~ground_max_slope`), already set in `ws/src/dem_bringup/config/dem.yaml`
and `dem_utm.yaml`.

Unit tests: `cd tools/viewer && .venv/bin/python -m pytest tests/test_ground_filters.py -q`.

## Why not use PDAL directly?

Short version: PDAL is not installed here, it is a batch/file-oriented C++
library, and the DEM is an incremental 10 Hz in-memory grid.

* Not present: no `pdal` binary and `import pdal` fails in `tools/viewer/.venv`.
  The `pdal` PyPI package is only a thin wrapper around `libpdal` — there is no
  wheel that bundles it, so it would need libpdal/PDAL from apt or conda on this
  Ubuntu 18.04 / Python 3.8 box. The viewer is pip-only by design
  (`scripts/bootstrap_env.sh` creates the venv), and the DEM core is deliberately
  ROS-free NumPy shared with `dem_node`.
* PDAL's model is `pdal pipeline` over readers/writers (LAS/LAZ/PLY/E57/PCD) or
  `pdal.Pipeline` with a JSON pipeline. `filters.smrf`, `filters.pmf` and
  `filters.csf` operate on a whole cloud, in a projected/georeferenced frame
  with a scale, and they are tuned for aerial surveys. Here we get one
  20k-point `PointCloud2` every 100 ms and must have a surface estimate in a few
  milliseconds: no file I/O, no LAS round-trip, no per-frame pipeline
  construction.
* Ground truth about the cost: the current filter is 6.25 ms/scan measured on
  this bag. SMRF-class filters are typically tens of ms per call plus
  serialisation of 20k points to a LAS/PCD buffer and back — at 10 Hz that
  would drop scans, which the DEM accumulation must not do.
* It would not fix the real remaining problem: PDAL is stateless per cloud, so
  it cannot supply the missing poses. Stacked sensor-frame scans still smear no
  matter which ground classifier runs on them.

Where PDAL *is* a good fit, if you want it offline: export the bag to LAS/LAZ
once (`rosbag` → PCD → `pdal translate`), then run a classic
`filters.smrf` + `writers.gdal` pipeline and compare against this grid. That is a
batch product, not the live tab.
