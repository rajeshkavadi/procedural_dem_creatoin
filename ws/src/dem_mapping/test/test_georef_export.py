"""Unit tests for the georeferencing exporter (no ROS, no bag needed).

Run:
    cd ws && python3 -m pytest src/dem_mapping/test -v
"""

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from dem_mapping import pcd_io                                   # noqa: E402
from dem_mapping.georef import (latlon_to_utm, utm_to_latlon,    # noqa: E402
                                utm_zone)
from dem_mapping.georef_export import (fit_hypotheses,           # noqa: E402
                                       reproject_grid, yaw_fit)


# ------------------------------------------------------------------ utm math

def test_utm_zone_turku():
    zone, northern = utm_zone(60.445, 22.315)      # this recording
    assert zone == 34 and northern is True


def test_utm_roundtrip():
    rng = np.random.default_rng(0)
    lat = 55.0 + rng.random(500) * 10.0
    lon = 8.0 + rng.random(500) * 5.0
    e, n, zone, north = latlon_to_utm(lat, lon)
    lat2, lon2 = utm_to_latlon(e, n, zone, north)
    assert np.allclose(lat, lat2, atol=1e-7)
    assert np.allclose(lon, lon2, atol=1e-7)


def test_utm_known_point():
    # equator/central meridian has easting 500000 exactly
    e, n, zone, _ = latlon_to_utm(0.0, 3.0)         # zone 31 -> lon0 = 3
    assert zone == 31
    assert abs(e - 500000.0) < 1e-3
    assert abs(n) < 1e-3


# ------------------------------------------------------------------ fits

def _make_tracks(n=400, yaw=37.0, t=(12345.0, 6789000.0, 41.0), seed=1):
    """Synthetic pair: GNSS track in UTM and the same track in a rotated
    local (mapper) frame."""
    rng = np.random.default_rng(seed)
    s = np.linspace(0.0, 1.0, n)
    utm = np.stack([400000 + 200 * np.cos(6 * s) + 120 * s,
                    6700000 + 180 * np.sin(4 * s) + 60 * s,
                    40 + 3 * np.sin(8 * s)], axis=1)
    th = math.radians(yaw)
    Rz = np.array([[math.cos(th), -math.sin(th), 0],
                   [math.sin(th), math.cos(th), 0],
                   [0, 0, 1.0]])
    local = (utm - np.asarray(t)) @ Rz            # p_utm = Rz p_local + t
    local += rng.normal(0, 0.01, local.shape)     # 1 cm noise
    t_stamps = 1747409700.0 + s * 60.0
    return local, utm, t_stamps


def test_yaw_fit_recovers_transform():
    local, utm, _ = _make_tracks()
    R, t, info = yaw_fit(local, utm)
    assert abs(info['yaw_deg'] - 37.0) < 0.5
    assert info['rms_m'] < 0.05
    np.testing.assert_allclose(np.array([12345.0, 6789000.0, 41.0]), t,
                               atol=0.1)
    back = local @ R.T + t
    assert np.abs(back - utm).max() < 0.2


def test_latlon_order_autodetection():
    """x=lat vs x=lon must be distinguishable by the metric LIO track."""
    local, utm, stamps = _make_tracks()
    # invert UTM -> lat/lon and store them as x/y the way a driver might
    from dem_mapping.georef import utm_to_latlon
    lat, lon = utm_to_latlon(utm[:, 0], utm[:, 1], 34, True)
    cands = fit_hypotheses(local, stamps, lat, lon, utm[:, 2], stamps, 0.2)
    assert cands, 'no candidate'
    assert cands[0]['order'] == 'x=lat'
    assert cands[0]['rms_m'] < 0.05
    assert len(cands) == 2
    # the wrong order must be clearly worse
    assert cands[1]['rms_m'] > 10 * max(cands[0]['rms_m'], 1e-3)


def test_lon_lat_order_autodetection():
    local, utm, stamps = _make_tracks()
    from dem_mapping.georef import utm_to_latlon
    lat, lon = utm_to_latlon(utm[:, 0], utm[:, 1], 34, True)
    cands = fit_hypotheses(local, stamps, lon, lat, utm[:, 2], stamps, 0.2)
    assert cands[0]['order'] == 'x=lon'
    assert cands[0]['rms_m'] < 0.05


def test_matched_samples_guards_short_tracks():
    local, utm, stamps = _make_tracks(n=5)
    from dem_mapping.georef import utm_to_latlon
    lat, lon = utm_to_latlon(utm[:, 0], utm[:, 1], 34, True)
    cands = fit_hypotheses(local, stamps, lat, lon, utm[:, 2], stamps, 0.2)
    assert cands and 'error' in cands[0]


# --------------------------------------------------------------- grid regrid

def test_reproject_grid_roundtrip():
    res, n = 0.2, 150
    ox, oy = -15.0, -15.0
    xs = ox + (np.arange(n) + 0.5) * res
    ys = oy + (np.arange(n) + 0.5) * res
    gx, gy = np.meshgrid(xs, ys)
    z = 0.3 * gx - 0.2 * gy + 5.0                  # tilted plane
    elev = z.astype(np.float32)

    th = math.radians(37.0)
    R = np.array([[math.cos(th), -math.sin(th), 0],
                  [math.sin(th), math.cos(th), 0],
                  [0, 0, 1.0]])
    t = np.array([500000.0, 6700000.0, 12.0])

    out, origin, stats = reproject_grid(elev, res, (ox, oy), R, t)
    # a rotated square fills ~50-60% of its axis-aligned bounding box, so
    # compare against the number of INPUT cells instead of the bbox area
    valid = int(np.isfinite(out).sum())
    assert 0.90 * n * n <= valid <= 1.20 * n * n
    assert stats['fill_ratio'] > 0.4
    # sample a few interior cells: elevation must follow the same plane
    iy, ix = np.indices(out.shape)
    px = origin[0] + (ix + 0.5) * stats['resolution_m']
    py = origin[1] + (iy + 0.5) * stats['resolution_m']
    # invert the transform to local coordinates
    d = np.stack([px - t[0], py - t[1], np.zeros_like(px)], axis=-1)
    loc = d @ R                                       # R^-1 = R^T applied
    # z is also translated into the target frame (t[2])
    expect = 0.3 * loc[..., 0] - 0.2 * loc[..., 1] + 5.0 + t[2]
    got = out
    mask = np.isfinite(got)
    err = np.abs(got[mask] - expect[mask])
    # nearest-neighbour regridding of a smooth plane: <= 1 cell of error
    assert np.percentile(err, 99) < 0.2 * math.hypot(0.3, 0.2) * 1.5


def test_reproject_rejects_empty_grid():
    with pytest.raises(SystemExit):
        reproject_grid(np.full((10, 10), np.nan, dtype=np.float32), 0.2,
                       (0.0, 0.0), np.eye(3), np.zeros(3))


# ------------------------------------------------------------------ pcd i/o

def test_pcd_roundtrip(tmp_path):
    rng = np.random.default_rng(3)
    x, y, z, i = (rng.random(500) * 100 for _ in range(4))
    p = tmp_path / 'a.pcd'
    n = pcd_io.write_pcd(p, [x, y, z, i], ['x', 'y', 'z', 'intensity'])
    assert n == 500
    d = pcd_io.read_pcd(p)
    assert d['count'] == 500
    assert d['fields'] == ['x', 'y', 'z', 'intensity']
    np.testing.assert_allclose(d['points']['z'], z.astype(np.float32),
                               rtol=0, atol=1e-5)


def test_pcd_transform(tmp_path):
    src = tmp_path / 'in.pcd'
    dst = tmp_path / 'out.pcd'
    pcd_io.write_pcd(src, [1.0, 2.0, 3.0], ['x', 'y', 'z'])
    R = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
    pcd_io.transform_pcd(src, dst, R, np.array([10.0, 20.0, 30.0]))
    d = pcd_io.read_pcd(dst)
    np.testing.assert_allclose([d['points']['x'][0], d['points']['y'][0],
                                d['points']['z'][0]],
                               [10.0 - 2.0, 20.0 + 1.0, 30.0 + 3.0],
                               atol=1e-6)
