"""Incremental DEM ("Get DEM") tests — ROS-free, no bag required.

Covers the accumulation core (reusing the dem_mapping grid), the heatmap
renderer, and the HTTP surface (/api/health block, POST /api/dem, image and
ZIP retrieval, auth, error paths).
"""

from __future__ import annotations

import io
import zipfile

import numpy as np
import pytest
from fastapi.testclient import TestClient

from pc_snapshot.api import create_app
from pc_snapshot.config import Config
from pc_snapshot.decoder import decode_pointcloud2
from pc_snapshot.dem import (DemAccumulator, DemRenderError,
                             import_dem_mapping, render_dem)
from pc_snapshot.jobs import (DemCoordinator, RenderCoordinator,
                              SnapshotRegistry)
from pc_snapshot.store import SnapshotStore
from tests.fixtures import make_scene_cloud, make_xyz_cloud


def _dem_cfg(**over) -> Config:
    cfg = Config()
    cfg.api_token = ""
    cfg.image_width = 320
    cfg.image_height = 180
    cfg.dem_extent = 40.0            # 80x80 cells @ 0.5 m -> fast tests
    cfg.dem_resolution = 0.5
    cfg.dem_image_width = 400
    cfg.dem_image_height = 300
    cfg.dem_max_points_per_scan = 5000
    for key, value in over.items():
        setattr(cfg, key, value)
    cfg.validate()
    return cfg


@pytest.fixture()
def acc():
    return DemAccumulator(_dem_cfg())


def _scene_points(n: int = 2500) -> np.ndarray:
    return decode_pointcloud2(make_scene_cloud(n)).points


# ---------------------------------------------------------------------------
# Core: reuse of the dem_mapping grid + accumulation semantics
# ---------------------------------------------------------------------------
def test_dem_mapping_package_importable():
    assert import_dem_mapping() is True


def test_accumulator_starts_empty(acc):
    assert acc.enabled is True
    assert acc.empty() is True
    assert acc.capture() is None or acc.capture().valid_cells == 0
    status = acc.status()
    assert status["enabled"] is True
    assert status["scans_accumulated"] == 0


def test_accumulate_and_capture(acc):
    pts = _scene_points()
    assert acc.add(pts, 100.5) is True
    assert acc.add(pts, 101.5) is True

    assert acc.empty() is False
    cap = acc.capture()
    assert cap.valid_cells > 0
    # Scan window recorded from the ROS stamps we fed.
    assert cap.meta["scans_accumulated"] == 2
    assert cap.meta["first_scan_stamp"] == 100.5
    assert cap.meta["last_scan_stamp"] == 101.5
    assert cap.meta["points_in"] >= cap.meta["points_used"] > 0
    assert cap.meta["topic"] == "/ouster/points"
    assert cap.meta["frame"] == "sensor"
    assert cap.stats["z_min"] <= cap.stats["z_max"]
    # Grid geometry follows the config (square, centred on the sensor).
    assert cap.elevation.shape == (80, 80)
    assert cap.meta["origin"] == [-20.0, -20.0]
    # Read-only: rendering must never mutate live state.
    assert cap.elevation.flags.writeable is False
    assert cap.valid.flags.writeable is False


def test_capture_is_a_consistent_copy(acc):
    """A capture taken earlier must not change when more scans arrive."""
    acc.add(_scene_points(), 100.0)
    before = acc.capture()
    before_cells = before.valid_cells
    acc.add(_scene_points(), 101.0)
    assert before.valid_cells == before_cells
    assert acc.capture().valid_cells >= before_cells


def test_status_reflects_new_scans(acc):
    assert acc.status()["scans_accumulated"] == 0
    acc.add(_scene_points(), 100.0)
    status = acc.status()
    assert status["scans_accumulated"] == 1
    assert status["valid_cells"] > 0
    assert status["z_min"] <= status["z_max"]


def test_radial_clip_and_blind_zone(acc):
    """Points outside the clip annulus (incl. the (0,0,0) filler) are dropped."""
    cfg = _dem_cfg(dem_clip_range_m=10.0, dem_min_range_m=1.0)
    a = DemAccumulator(cfg)
    outside = np.array([[50.0, 0.0, 0.0], [0.0, -45.0, 0.0]])
    filler = np.array([[0.0, 0.0, 0.0]])          # driver no-return filler
    inside = np.array([[5.0, 5.0, -1.0], [-5.0, 5.0, -1.0]])
    assert a.add(np.vstack([outside, filler, inside]), 1.0) is True

    cap = a.capture()
    # Only the two in-range points survive -> a couple of cells, and no
    # observation anywhere near the origin cell.
    assert 0 < cap.valid_cells <= 4
    centres = cap.valid.nonzero()
    ox, oy = cap.stats["origin"]
    res = cap.stats["resolution_m"]
    xs = ox + (centres[1] + 0.5) * res
    ys = oy + (centres[0] + 0.5) * res
    assert np.all(np.hypot(xs, ys) >= 1.0)
    assert np.all(np.hypot(xs, ys) <= 10.0)
    # 5 offered points, 2 accepted (outside + filler dropped).
    assert cap.meta["points_in"] == 5
    assert cap.meta["points_used"] == 2


def test_height_band_filter_drops_high_points(acc):
    a = DemAccumulator(_dem_cfg(dem_band_below=6.0, dem_band_above=6.0))
    low = np.array([[4.0, 4.0, -1.0]])
    high = np.array([[4.0, 4.0, 60.0]])
    a.add(np.vstack([low, high]), 1.0)
    cap = a.capture()
    assert np.isfinite(cap.elevation).sum() >= 1
    assert np.nanmax(cap.elevation) < 10.0     # the 60 m point never landed


def test_max_points_per_scan_is_honoured(acc):
    a = DemAccumulator(_dem_cfg(dem_max_points_per_scan=500))
    rng = np.random.default_rng(0)
    pts = np.column_stack([rng.uniform(-15, 15, size=4000),
                           rng.uniform(-15, 15, size=4000),
                           rng.uniform(-5, 5, size=4000)])
    a.add(pts, 1.0)
    cap = a.capture()
    assert cap.meta["points_in"] == 4000
    assert cap.meta["points_used"] == 500        # CPU guard kicked in
    assert cap.meta["observations"] == cap.meta["points_used"]


def test_mean_and_min_aggregation(acc):
    pts = _scene_points()
    for agg in ("min", "mean"):
        a = DemAccumulator(_dem_cfg(dem_aggregation=agg))
        a.add(pts, 1.0)
        cap = a.capture()
        assert cap.stats["aggregation"] == agg
        assert cap.valid_cells > 0


def test_accumulator_disabled_by_config():
    a = DemAccumulator(_dem_cfg(dem_enabled=False))
    assert a.enabled is False
    assert a.add(_scene_points(), 1.0) is False
    assert a.capture() is None
    status = a.status()
    assert status["enabled"] is False
    assert "disabled" in status["reason"]


def test_unknown_colormap_disables_dem_at_startup():
    a = DemAccumulator(_dem_cfg(dem_colormap="definitely-not-a-cmap"))
    assert a.enabled is False
    assert "colormap" in (a.reason or "")


def test_bad_message_does_not_raise(acc):
    acc.add(np.zeros((0, 3), dtype=np.float32), 1.0)   # empty message
    acc.add(np.zeros((0, 3), dtype=np.float32), 2.0)
    status = acc.status()
    assert status["scans_accumulated"] == 2
    assert status["scans_empty"] == 2
    assert status["valid_cells"] == 0


def test_scan_with_everything_filtered_still_counts(acc):
    """A scan whose points are all rejected is still a received scan."""
    a = DemAccumulator(_dem_cfg(dem_clip_range_m=5.0))
    assert a.add(np.array([[500.0, 0.0, 0.0]]), 1.0) is False
    status = a.status()
    assert status["scans_accumulated"] == 1
    assert status["scans_empty"] == 1
    assert status["points_in"] == 1
    assert status["points_used"] == 0
    assert status["valid_cells"] == 0


# ---------------------------------------------------------------------------
# Rendering
# ---------------------------------------------------------------------------
def test_render_dem_png(acc, tmp_path):
    acc.add(_scene_points(), 100.0)
    cap = acc.capture()
    out = tmp_path / "dem.png"
    seconds = render_dem(cap, acc.cfg, out)
    assert seconds >= 0
    data = out.read_bytes()
    assert data[:8] == b"\x89PNG\r\n\x1a\n"
    assert len(data) > 2000          # not a blank/near-empty image


def test_render_dem_rejects_empty_grid(acc, tmp_path):
    cap = acc.capture()               # allocated but never updated
    with pytest.raises(DemRenderError):
        render_dem(cap, acc.cfg, tmp_path / "never.png")
    assert not (tmp_path / "never.png").exists()


# ---------------------------------------------------------------------------
# HTTP API
# ---------------------------------------------------------------------------
def _make_app(cfg: Config, tmp_path, with_dem: bool = True):
    store = SnapshotStore(topic=cfg.topic, stale_after_sec=cfg.stale_after_sec)
    registry = SnapshotRegistry(tmp_path / "snaps", 5, 300.0)
    coordinator = RenderCoordinator(cfg, registry)
    dem_registry = SnapshotRegistry(tmp_path / "dem", 5, 300.0)
    dem = DemAccumulator(cfg) if with_dem else None
    dem_coord = DemCoordinator(cfg, dem_registry)
    app = create_app(cfg, store, coordinator, listener=None, dem=dem,
                     dem_coordinator=dem_coord)
    return TestClient(app), store, dem, dem_coord


def _feed(store, dem, n: int = 2500, stamp: float = 100.5) -> None:
    dec = decode_pointcloud2(make_scene_cloud(n))
    store.update(dec, frame_id="camera_init", ros_stamp=stamp,
                 topic="/test/points")
    if dem is not None:
        dem.add(dec.points, stamp)


def test_health_reports_dem_block(tmp_path):
    client, store, dem, _ = _make_app(_dem_cfg(), tmp_path)
    h = client.get("/api/health").json()
    assert h["dem"]["enabled"] is True
    assert h["dem"]["scans_accumulated"] == 0
    _feed(store, dem)
    h = client.get("/api/health").json()
    assert h["dem"]["scans_accumulated"] == 1
    assert h["dem"]["valid_cells"] > 0


def test_health_reports_dem_absent(tmp_path):
    cfg = _dem_cfg()
    store = SnapshotStore(topic=cfg.topic, stale_after_sec=5.0)
    registry = SnapshotRegistry(tmp_path / "snaps", 5, 300.0)
    app = create_app(cfg, store, RenderCoordinator(cfg, registry), listener=None)
    h = TestClient(app).get("/api/health").json()
    assert h["dem"]["enabled"] is False
    assert "not configured" in h["dem"]["reason"]


def test_dem_rejected_before_any_message(tmp_path):
    client, store, dem, _ = _make_app(_dem_cfg(), tmp_path)
    r = client.post("/api/dem")
    assert r.status_code == 409
    assert "no point cloud has been accumulated" in r.json()["detail"]


def test_dem_rejected_when_disabled(tmp_path):
    client, store, dem, _ = _make_app(_dem_cfg(dem_enabled=False), tmp_path)
    _feed(store, dem)
    r = client.post("/api/dem")
    assert r.status_code == 409
    assert "disabled" in r.json()["detail"]


def test_dem_full_flow(tmp_path):
    client, store, dem, _ = _make_app(_dem_cfg(), tmp_path)
    _feed(store, dem, stamp=100.5)
    _feed(store, dem, stamp=101.5)
    r = client.post("/api/dem")
    assert r.status_code == 200, r.text
    body = r.json()
    assert body["dem_id"] and len(body["dem_id"]) == 32
    assert body["scans_accumulated"] == 2
    assert body["grid"]["shape"] == [80, 80]
    assert body["grid"]["resolution_m"] == 0.5
    assert body["grid"]["valid_cells"] > 0
    assert body["grid"]["z_min"] <= body["grid"]["z_max"]
    assert body["window"]["first_scan_stamp"] == 100.5
    assert body["window"]["last_scan_stamp"] == 101.5
    assert body["filters"]["ground_filter"] == "height_band"

    img = client.get(body["image"])
    assert img.status_code == 200
    assert img.headers["content-type"] == "image/png"
    assert img.content[:8] == b"\x89PNG\r\n\x1a\n"
    assert len(img.content) > 2000

    z = client.get(body["download_zip"])
    assert z.status_code == 200
    zf = zipfile.ZipFile(io.BytesIO(z.content))
    names = zf.namelist()
    dem_id = body["dem_id"]
    assert f"{dem_id}/dem.png" in names
    assert f"{dem_id}/metadata.json" in names
    meta = zf.read(f"{dem_id}/metadata.json").decode()
    assert '"kind": "dem"' in meta


def test_dem_image_accumulates_over_time(tmp_path):
    """Two clicks give two different captures of the growing grid."""
    client, store, dem, _ = _make_app(_dem_cfg(), tmp_path)
    _feed(store, dem)
    first = client.post("/api/dem").json()
    _feed(store, dem, stamp=200.5)
    second = client.post("/api/dem").json()
    assert first["dem_id"] != second["dem_id"]
    assert second["scans_accumulated"] == 2
    assert first["scans_accumulated"] == 1
    # Both remain retrievable.
    assert client.get(first["image"]).status_code == 200
    assert client.get(second["image"]).status_code == 200


def test_dem_unknown_id_and_traversal(tmp_path):
    client, store, dem, _ = _make_app(_dem_cfg(), tmp_path)
    assert client.post("/api/dem").status_code == 409      # nothing yet
    _feed(store, dem)
    body = client.post("/api/dem").json()
    assert client.get("/api/dem/notanid/dem.png").status_code == 404
    assert client.get("/api/dem/..%2F..%2Fetc%2Fpasswd/dem.png").status_code == 404
    assert client.get(f"/api/dem/{body['dem_id']}/other.png").status_code == 404
    assert client.get(
        "/api/dem/aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa/download.zip").status_code == 404
    assert client.get(f"/api/dem/{body['dem_id']}/download.zip").status_code == 200


def test_dem_token_required(tmp_path):
    cfg = _dem_cfg(api_token="secret123")
    client, store, dem, _ = _make_app(cfg, tmp_path)
    _feed(store, dem)
    assert client.post("/api/dem").status_code == 401
    assert client.post("/api/dem",
                       headers={"X-API-Token": "wrong"}).status_code == 401
    assert client.post("/api/dem",
                       headers={"X-API-Token": "secret123"}).status_code == 200
    assert client.post("/api/dem?token=secret123").status_code == 200


def test_dem_render_busy_returns_409(tmp_path):
    client, store, dem, dem_coord = _make_app(_dem_cfg(), tmp_path)
    _feed(store, dem)
    assert dem_coord._sem.acquire(blocking=False)     # simulate a running job
    try:
        r = client.post("/api/dem")
        assert r.status_code == 409
        assert "already in progress" in r.json()["detail"]
    finally:
        dem_coord._sem.release()
    assert client.post("/api/dem").status_code == 200


def test_dem_frontend_has_get_dem_button(tmp_path):
    client, _, _, _ = _make_app(_dem_cfg(), tmp_path)
    html = client.get("/").text
    assert "Get DEM" in html
    assert 'id="btn-dem"' in html
    assert "Get DEM" in client.get("/app.js").text
