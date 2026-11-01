# Point-cloud streaming screenshots

One command starts a looping **ROS 1 bag playback** and a small HTTP service
that turns it into browser screenshots and an elevation model:

```
recording.bag ──► pc_snapshot bag reader ──► latest cloud ──► POST /api/snapshot
   (looping)          (pure Python)              │             5 PNG views
                                                 └──────────► POST /api/dem
                                                              top-down DEM heatmap
                                                                     │
                                                          FastAPI + static UI
                                                                     │
                                                     localhost / Cloudflare tunnel
```

* **Fetch Point Cloud** — captures exactly one `PointCloud2` message and
  renders five PNG views (top / front / back / left / right) of that message.
* **Get DEM** — merges every message since playback started into one
  incremental elevation grid and renders a top-down heatmap.
* **Live View** — optional MJPEG stream of the current cloud (`/api/stream`).

The service reads the `.bag` directly with the pure-Python `rosbags` library,
so it runs on Python 3 next to ROS Melodic without needing `rospy` (Melodic's
`rospy`/`rosbag` are Python 2.7 only). No `rosbag play` process is needed.

## Quick start

```bash
cd snapshot_service

# Python 3.8+ with the requirements (FastAPI, rosbags, matplotlib, …)
python3 -m venv .venv && . .venv/bin/activate
pip install -r requirements.txt

# service + looping bag + Cloudflare quick tunnel (prints a public URL)
./scripts/start_remote.sh /path/to/recording.bag
#   or, local only:
./scripts/start_all.sh /path/to/recording.bag
```

Then open the printed URL, paste the API token into the UI, and click
**Fetch Point Cloud** / **Get DEM**. Stop everything with
`./scripts/stop_all.sh`.

Inspect a bag without starting anything:

```bash
./scripts/bag_info.sh /path/to/recording.bag
```

Full documentation — architecture, every option, the HTTP API, the renderer
and DEM internals, tests — is in
[snapshot_service/README.md](snapshot_service/README.md).

## Relation to the DEM pipeline

The service imports `dem_mapping.ElevationGrid` and `ground_filters` from
`../ws/src/dem_mapping`, so the incremental grid here is the same
implementation the offline FAST-LIO2 → DEM → georeference pipeline uses. The
service does **no** scan registration: the grid is filled in the raw sensor
frame (fine for a stationary sensor; point it at a registered topic such as
`/cloud_registered` for a moving one). See the root [README](../README.md) for
the full offline pipeline.
