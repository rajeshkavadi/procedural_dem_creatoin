#!/usr/bin/env python3
"""Georeference the pipeline results: local mapper frame -> WGS84 / UTM.

Why this step exists
--------------------
The DEM produced by ``dem_node`` and the point cloud map produced by
FAST-LIO2 live in the **local mapper frame** (``camera_init``): X/Y are metres
with an arbitrary origin and an arbitrary yaw.  That is a fine engineering
frame but it is not a *geographic* one - you cannot put it on a map, compare
it with satellite imagery or store it as a GeoTIFF.

This tool ties that local frame to the world:

1. reads the GNSS track (``/gnss_pose`` - WGS84 latitude/longitude/altitude
   in degrees/metres) from the bag (ROS 1 ``.bag`` **or** rosbag2),
2. reads the recorded FAST-LIO trajectory (``odom.csv``),
3. matches the two tracks by timestamp and estimates the rigid transform
   ``p_utm = R * p_local + t`` (yaw about the gravity axis by default, so the
   LiDAR's level orientation is preserved),
4. reports how well they agree (RMS in metres) - a *large* RMS means the
   streams are NOT consistent and global georeferencing must not be claimed,
5. re-projects the DEM into a UTM-aligned grid and writes:

   * ``<prefix>_dem.tif``            GeoTIFF (EPSG:32634 by default, NaN nodata)
   * ``<prefix>_dem_hillshade.tif``  hillshade for visual control
   * ``<prefix>_dem_preview.png``    quick-look image with colour bar
   * ``<prefix>_dem.pcd``            DEM cells as a point cloud, UTM metres
   * ``<prefix>_map_georef.pcd``     FAST-LIO map cloud, UTM metres (optional)
   * ``<prefix>_georef.json``        full provenance: CRS, transform, RMS

Lat/lon order is auto-detected: a PoseStamped can hold ``(x=lat, y=lon)`` or
the other way round and both look plausible.  Both hypotheses are fitted and
the one that matches the metric LiDAR trajectory wins (they differ by up to
2x in scale, so this is unambiguous); the losing hypothesis is reported in
the JSON so the decision can be audited.

Usage
-----
    ros2 run dem_mapping georef_export \
        --dem out/dem_20261015.npz \
        --odom out/odom.csv \
        --bag OutdoorRoad_dataset_ros2 \
        --map out/map.pcd \
        --out-dir out --prefix outdoorroad

Exit codes: 0 = written, 1 = bad input, 2 = alignment rejected
(``--allow-poor`` overrides the rejection).
"""

from __future__ import annotations

import argparse
import datetime as _dt
import json
import math
import sys
from pathlib import Path

import numpy as np

from dem_mapping.georef import latlon_to_utm, utm_zone
from dem_mapping.gnss_align import nearest_match, umeyama
from dem_mapping import pcd_io

# ---------------------------------------------------------------- coordinates


def _load_gnss(path: Path, topic: str):
    """Return (t, x, y, z) arrays of the GNSS pose track.

    Accepts a ROS 1 ``.bag`` or a rosbag2 directory; uses ``rosbags`` when
    available (works for both) and falls back to ``rosbag2_py``.
    """
    path = Path(path)
    try:
        return _load_gnss_rosbags(path, topic)
    except ImportError:
        return _load_gnss_rosbag2(path, topic)


def _load_gnss_rosbags(path, topic):
    from rosbags.highlevel import AnyReader
    ts, xs, ys, zs = [], [], [], []
    with AnyReader([path]) as reader:
        conns = [c for c in reader.connections if c.topic == topic]
        if not conns:
            raise SystemExit(
                f'ERROR: topic {topic} not in {path}\n'
                f'       available: {sorted({c.topic for c in reader.connections})}')
        for _c, _ts, data in reader.messages(connections=conns):
            msg = reader.deserialize(data, conns[0].msgtype)
            ts.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
            xs.append(msg.pose.position.x)
            ys.append(msg.pose.position.y)
            zs.append(msg.pose.position.z)
    return (np.asarray(ts), np.asarray(xs), np.asarray(ys), np.asarray(zs))


def _load_gnss_rosbag2(path, topic):
    from rclpy.serialization import deserialize_message
    import rosbag2_py
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topic not in types:
        raise SystemExit(f'ERROR: topic {topic} not in {path} '
                         f'(have: {sorted(types)})')
    cls = get_message(types[topic])
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    ts, xs, ys, zs = [], [], [], []
    while reader.has_next():
        _t, data, _ts = reader.read_next()
        msg = deserialize_message(data, cls)
        ts.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        xs.append(msg.pose.position.x)
        ys.append(msg.pose.position.y)
        zs.append(msg.pose.position.z)
    return (np.asarray(ts), np.asarray(xs), np.asarray(ys), np.asarray(zs))


def load_odom(csv_path):
    """Read the odom CSV written by ``dem_replay record_odom``."""
    data = np.genfromtxt(csv_path, delimiter=',', names=True)
    if data.size == 0:
        raise SystemExit(f'ERROR: {csv_path} holds no rows')
    return (np.asarray(data['t'], dtype=np.float64),
            np.stack([data['x'], data['y'], data['z']], axis=1))


def yaw_fit(P, Q):
    """Rigid fit of local points P onto UTM points Q, yaw rotation only.

    Rotation is restricted to the gravity axis (both frames are level: the
    LiDAR map frame comes from an IMU-aligned LIO, UTM is a projected CRS),
    translation is 3-D.  Returns (R, t, info).
    """
    pc = P.mean(axis=0)
    qc = Q.mean(axis=0)
    p = P[:, :2] - pc[:2]
    q = Q[:, :2] - qc[:2]
    cov = q.T @ p / P.shape[0]                    # 2x2
    U, _, Vt = np.linalg.svd(cov)
    R2 = U @ Vt
    if np.linalg.det(R2) < 0:
        U[:, -1] *= -1
        R2 = U @ Vt
    theta = math.atan2(R2[1, 0], R2[0, 0])
    c, s = math.cos(theta), math.sin(theta)
    R = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    t = np.array([qc[0] - (R2 @ pc[:2])[0],
                  qc[1] - (R2 @ pc[:2])[1],
                  float(np.mean(Q[:, 2] - P[:, 2]))])
    aligned = P @ R.T + t
    resid = np.linalg.norm(aligned - Q, axis=1)
    resid_h = np.linalg.norm(aligned[:, :2] - Q[:, :2], axis=1)
    info = {
        'yaw_deg': math.degrees(theta),
        'rms_m': float(np.sqrt(np.mean(resid ** 2))),
        'rms_horizontal_m': float(np.sqrt(np.mean(resid_h ** 2))),
        'rms_vertical_m': float(np.sqrt(np.mean(
            (aligned[:, 2] - Q[:, 2]) ** 2))),
        'max_error_m': float(resid.max()),
    }
    return R, t, info


def umeyama_scaled(P, Q):
    """Full 3-D rigid fit incl. a scale factor (diagnostic only)."""
    mu_p, mu_q = P.mean(0), Q.mean(0)
    sp, sq = P - mu_p, Q - mu_q
    cov = sq.T @ sp / P.shape[0]
    U, S, Vt = np.linalg.svd(cov)
    D = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        D[2, 2] = -1.0
    R = U @ D @ Vt
    var_p = float(np.sum(sp ** 2) / P.shape[0])
    scale = float(np.sum(S * np.diag(D)) / var_p) if var_p > 0 else 1.0
    t = mu_q - scale * (R @ mu_p)
    aligned = scale * (P @ R.T) + t
    rms = float(np.sqrt(np.mean(np.sum((aligned - Q) ** 2, axis=1))))
    return R, t, scale, rms


def utm_from_track(x, y, z, order):
    """Convert a geographic (x, y, z) track to UTM for a given order.

    ``order`` is ``'x=lat'`` (x holds latitude) or ``'x=lon'``.
    """
    if order == 'x=lat':
        lat, lon = x, y
    else:
        lat, lon = y, x
    zone, northern = utm_zone(float(np.mean(lat)), float(np.mean(lon)))
    e, n, zone, northern = latlon_to_utm(lat, lon, zone=zone,
                                         northern=northern)
    return np.stack([e, n, np.asarray(z, dtype=np.float64)], axis=1), zone, \
        northern, (float(np.min(lat)), float(np.max(lat)),
                   float(np.min(lon)), float(np.max(lon)))


def fit_hypotheses(P_local, t_local, gx, gy, gz, t_gnss, tol):
    """Fit every plausible lat/lon order; return sorted candidate list."""
    lo = abs(float(np.min(gx))) <= 180 and abs(float(np.min(gy))) <= 90 \
        and abs(float(np.max(gx))) <= 180 and abs(float(np.max(gy))) <= 90
    orders = ['x=lat', 'x=lon'] if lo else ['x=meters']
    out = []
    for order in orders:
        if order == 'x=meters':
            Q = np.stack([gx, gy, gz], axis=1)
            zone = northern = None
            bbox = None
        else:
            Q, zone, northern, bbox = utm_from_track(gx, gy, gz, order)
        pick, good = nearest_match(t_local, t_gnss, tol)
        n_match = int(good.sum())
        if n_match < 10:
            out.append({'order': order, 'error':
                        f'only {n_match} time-matched samples (tol {tol}s)'})
            continue
        Pm, Qm = P_local[good], Q[pick[good]]
        R, t, info = yaw_fit(Pm, Qm)
        R3, t3, scale, rms3 = umeyama_scaled(Pm, Qm)
        span_local = float(np.linalg.norm(Pm[-1] - Pm[0]))
        span_gnss = float(np.linalg.norm(Qm[-1] - Qm[0]))
        out.append({
            'order': order,
            'utm_zone': zone,
            'utm_northern': northern,
            'gnss_bbox_lat_lon': bbox,
            'matched_samples': n_match,
            'rms_m': info['rms_m'],
            'rms_horizontal_m': info['rms_horizontal_m'],
            'rms_vertical_m': info['rms_vertical_m'],
            'max_error_m': info['max_error_m'],
            'yaw_deg': info['yaw_deg'],
            'scale_diagnostic': scale,
            'rms_full_fit_m': rms3,
            'chord_local_m': span_local,
            'chord_gnss_m': span_gnss,
            '_R': R, '_t': t,
        })
    # metric hypothesis first when the values are degrees: the correct
    # lat/lon order is the one whose UTM extent matches the metric LIO track
    good = [c for c in out if 'rms_m' in c]
    good.sort(key=lambda c: c['rms_m'])
    return good + [c for c in out if 'rms_m' not in c]


# ------------------------------------------------------------- grid handling


def reproject_grid(elev, res, origin, R, t, out_res=None, max_dist=None):
    """Re-sample a local elevation grid into a UTM-aligned grid.

    Returns (out_elev, out_origin, stats).  The local grid cell centres are
    transformed with ``p -> R p + t`` and injected into a new grid whose
    origin lies on a multiple of ``out_res`` (clean GeoTIFF geotransform).
    Cells are filled by nearest-neighbour lookup, limited to
    ``max_dist`` (default ``0.75 * res``) so the rotated footprint does not
    smear into empty corners.
    """
    from scipy.spatial import cKDTree

    out_res = float(out_res or res)
    max_dist = float(max_dist if max_dist is not None else 0.75 * res)

    ny, nx = elev.shape
    ox, oy = float(origin[0]), float(origin[1])
    iy, ix = np.nonzero(np.isfinite(elev))
    if iy.size == 0:
        raise SystemExit('ERROR: DEM has no valid (non-NaN) cells')
    x = ox + (ix + 0.5) * res
    y = oy + (iy + 0.5) * res
    z = elev[iy, ix].astype(np.float64)

    pts = np.stack([x, y, z], axis=1)
    q = pts @ np.asarray(R, dtype=np.float64).T + \
        np.asarray(t, dtype=np.float64)

    # output grid: aligned to multiples of out_res, covering q with margin
    ox2 = math.floor(q[:, 0].min() / out_res) * out_res
    oy2 = math.floor(q[:, 1].min() / out_res) * out_res
    nx2 = int(math.ceil((q[:, 0].max() - ox2) / out_res)) + 1
    ny2 = int(math.ceil((q[:, 1].max() - oy2) / out_res)) + 1

    cx = ox2 + (np.arange(nx2) + 0.5) * out_res
    cy = oy2 + (np.arange(ny2) + 0.5) * out_res
    gx, gy = np.meshgrid(cx, cy)
    centres = np.stack([gx.ravel(), gy.ravel()], axis=1)

    tree = cKDTree(q[:, :2])
    dist, idx = tree.query(centres, distance_upper_bound=max_dist,
                           workers=-1)
    out = np.full(centres.shape[0], np.nan, dtype=np.float32)
    hit = np.isfinite(dist)
    out[hit] = q[idx[hit], 2].astype(np.float32)
    out = out.reshape(ny2, nx2)

    return out, (ox2, oy2), {
        'input_cells': int(iy.size),
        'output_cells': int(hit.sum()),
        'output_shape': [ny2, nx2],
        'output_origin_utm': [ox2, oy2],
        'resolution_m': out_res,
        'fill_ratio': round(float(hit.sum()) / centres.shape[0], 4),
    }


# ------------------------------------------------------------------ exports


def write_geotiff(path, grid, origin, res, epsg, nodata=np.nan):
    from osgeo import gdal, osr
    ny, nx = grid.shape
    drv = gdal.GetDriverByName('GTiff')
    ds = drv.Create(str(path), nx, ny, 1, gdal.GDT_Float32,
                    options=['COMPRESS=DEFLATE', 'PREDICTOR=3', 'TILED=YES'])
    ds.SetGeoTransform((float(origin[0]), float(res), 0.0,
                        float(origin[1]) + ny * float(res), 0.0,
                        -float(res)))
    srs = osr.SpatialReference()
    srs.ImportFromEPSG(int(epsg))
    ds.SetProjection(srs.ExportToWkt())
    band = ds.GetRasterBand(1)
    band.SetNoDataValue(float(nodata))
    band.WriteArray(grid)
    band.FlushCache()
    ds = None
    return str(path)


def write_hillshade(dem_path, hs_path):
    from osgeo import gdal
    try:
        gdal.DEMProcessing(str(hs_path), str(dem_path), 'hillshade',
                           alg='Horn', azimuth=315.0, altitude=45.0,
                           zFactor=1.0, creationOptions=['COMPRESS=DEFLATE'])
        return str(hs_path)
    except Exception as exc:                                     # pragma: no cover
        return f'skipped ({type(exc).__name__}: {exc})'


def write_preview(tif_path, png_path, epsg, title):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from osgeo import gdal
    ds = gdal.Open(str(tif_path))
    band = ds.GetRasterBand(1)
    arr = band.ReadAsArray().astype(np.float64)
    gt = ds.GetGeoTransform()
    ds = None
    finite = arr[np.isfinite(arr)]
    if finite.size == 0:
        raise SystemExit('ERROR: nothing finite to preview')
    lo, hi = np.percentile(finite, [2, 98])
    if hi <= lo:
        lo, hi = float(finite.min()), float(finite.max()) + 1e-6
    x0, res, _, y0, _, res_y = gt
    extent = (x0, x0 + arr.shape[1] * res, y0 + arr.shape[0] * res_y, y0)

    fig, ax = plt.subplots(figsize=(9, 8), dpi=120)
    im = ax.imshow(arr, extent=extent, cmap='terrain', vmin=lo, vmax=hi,
                   origin='upper')
    cb = fig.colorbar(im, ax=ax, shrink=0.8, pad=0.02)
    cb.set_label('elevation (m)')
    ax.set_xlabel('UTM easting (m)')
    ax.set_ylabel('UTM northing (m)')
    ax.set_title(f'{title}\nEPSG:{epsg}', fontsize=10)
    ax.set_aspect('equal')
    ax.grid(alpha=0.25, linewidth=0.4)
    fig.tight_layout()
    fig.savefig(png_path)
    plt.close(fig)
    return str(png_path)


# ---------------------------------------------------------------------- main


def main(argv=None):
    ap = argparse.ArgumentParser(
        prog='georef_export', description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--dem', required=True, type=Path,
                    help='DEM grid written by dem_node (*.npz)')
    ap.add_argument('--odom', type=Path, default=None,
                    help='CSV written by dem_replay record_odom '
                         '(FAST-LIO trajectory; not needed with '
                         '--already-utm)')
    ap.add_argument('--bag', required=True, type=Path,
                    help='bag holding the GNSS topic (ROS 1 .bag or rosbag2)')
    ap.add_argument('--gnss-topic', default='/gnss_pose')
    ap.add_argument('--map', type=Path, default=None,
                    help='FAST-LIO map PCD (from /map_save) to georeference')
    ap.add_argument('--out-dir', type=Path, default=Path('out'))
    ap.add_argument('--prefix', default='dem')
    ap.add_argument('--resolution', type=float, default=None,
                    help='output grid resolution (default: DEM resolution)')
    ap.add_argument('--tol', type=float, default=0.2,
                    help='max timestamp difference for track matching (s)')
    ap.add_argument('--fit', choices=['yaw', 'full'], default='yaw',
                    help='yaw: rotate about gravity only (default); '
                         'full: free 3-D rotation (Umeyama)')
    ap.add_argument('--epsg', type=int, default=None,
                    help='force an EPSG code instead of auto UTM zone')
    ap.add_argument('--origin-lat', type=float, default=None,
                    help='WGS84 anchor latitude - only for GNSS topics that '
                         'hold a LOCAL metric frame (x=East, y=North)')
    ap.add_argument('--origin-lon', type=float, default=None,
                    help='WGS84 anchor longitude (with --origin-lat)')
    ap.add_argument('--origin-alt', type=float, default=0.0,
                    help='anchor altitude in metres (default 0)')
    ap.add_argument('--rms-threshold', type=float, default=1.0,
                    help='alignment RMS above which the tool refuses to '
                         'write georeferenced products')
    ap.add_argument('--allow-poor', action='store_true',
                    help='write outputs even when the RMS is too large')
    ap.add_argument('--already-utm', action='store_true',
                    help='the DEM grid is already in projected metres (it '
                         'was built by dem_node in pose mode from a '
                         'GNSS-warped trajectory, see gnss_warp): no fit '
                         'and no reprojection, the CRS is only stamped')
    ap.add_argument('--warp-report', type=Path, default=None,
                    help='JSON report of dem_mapping gnss_warp (its '
                         'residuals become the quality gate and provenance '
                         'of an --already-utm export)')
    ap.add_argument('--json', type=Path, default=None,
                    help='report path (default: <out-dir>/<prefix>_georef.json)')
    args = ap.parse_args(argv)

    if not args.dem.exists():
        raise SystemExit(f'ERROR: DEM {args.dem} not found')
    if not args.already_utm:
        if args.odom is None or not args.odom.exists():
            raise SystemExit(f'ERROR: odometry {args.odom} not found - '
                             'record it with: ros2 run dem_replay record_odom'
                             ' -- --out out/odom.csv  (or pass --already-utm '
                             'if the grid is already georeferenced)')
    if not args.bag.exists():
        raise SystemExit(f'ERROR: bag {args.bag} not found')

    args.out_dir.mkdir(parents=True, exist_ok=True)
    prefix = args.prefix

    # ------------------------------------------------------------- load data
    npz = np.load(args.dem, allow_pickle=True)
    elev = np.asarray(npz['elevation'], dtype=np.float32)
    res = float(npz['resolution'])
    origin = tuple(float(v) for v in npz['origin'])
    agg = str(npz['aggregation'])

    if args.already_utm:
        # The grid is already in projected metres (built by dem_node in
        # 'pose' mode from a GNSS-warped trajectory - see gnss_warp).
        # Nothing is fitted or reprojected; the CRS is only stamped and the
        # quality gate uses the warp residuals.
        R, t = np.eye(3), np.zeros(3)
        t_loc = None
        warp = (json.loads(args.warp_report.read_text())
                if args.warp_report and args.warp_report.exists() else {})
        st = warp.get('stats', {})
        rms = float(st.get('residual_warp_rms_m', 0.0))
        rms_h = st.get('residual_warp_h_rms_m')
        rms_v = st.get('residual_warp_v_rms_m')
        order = warp.get('order', 'already-utm')
        epsg = args.epsg or (int(warp['epsg']) if warp.get('epsg') else None)
        if epsg is None:
            raise SystemExit('ERROR: --already-utm needs the UTM zone - '
                             'pass --warp-report (preferred) or --epsg')
        verdict = ('consistent' if rms <= args.rms_threshold else
                   'NOT consistent')
        print(f'\nGNSS-aided build: grid already in EPSG:{epsg}, warp '
              f'residual RMS={rms:.3f} m  (threshold {args.rms_threshold} m)'
              f'  -> {verdict}')
        if warp:
            print(f'  rigid alternative would have been '
                  f'{warp.get("rigid_fit_rms_m", float("nan")):.3f} m RMS; '
                  f'windows {st.get("windows_used")}/'
                  f'{st.get("windows_total")}')
        best = {}
        candidates = []
        if rms > args.rms_threshold and not args.allow_poor:
            print('\nERROR: the GNSS warp residual is too large - the '
                  'trajectory could not be tied to the GNSS track.  Check '
                  'the warp report, re-run gnss_warp, or pass --allow-poor.')
            out_json = args.json or args.out_dir / f'{prefix}_georef.json'
            out_json.write_text(json.dumps({'warp': warp,
                                            'verdict': verdict},
                                           indent=2, default=str))
            print(f'report: {out_json}')
            return 2
    else:
        t_loc, P_loc = load_odom(args.odom)
        t_g, gx, gy, gz = _load_gnss(args.bag, args.gnss_topic)
        print(f'odom : {len(t_loc)} samples, '
              f'chord {np.linalg.norm(P_loc[-1] - P_loc[0]):.1f} m')
        print(f'gnss : {len(t_g)} samples, '
              f'x[{gx.min():.6f},{gx.max():.6f}] '
              f'y[{gy.min():.6f},{gy.max():.6f}] '
              f'z[{gz.min():.2f},{gz.max():.2f}]')

        order = np.argsort(t_loc)
        t_loc, P_loc = t_loc[order], P_loc[order]
        og = np.argsort(t_g)
        t_g, gx, gy, gz = t_g[og], gx[og], gy[og], gz[og]

        # A GNSS topic may hold a local metric frame instead of WGS84
        # degrees.  Then an anchor point is mandatory - without it there is
        # no absolute position on earth, only a local one.
        looks_geographic = (abs(gx).max() <= 180 and abs(gy).max() <= 90
                            and (abs(gx).max() > 1.0 or abs(gy).max() > 1.0))
        if not looks_geographic:
            if args.origin_lat is None or args.origin_lon is None:
                raise SystemExit(
                    'ERROR: the GNSS topic holds a LOCAL METRIC frame '
                    f'(x[{gx.min():.1f},{gx.max():.1f}] '
                    f'y[{gy.min():.1f},{gy.max():.1f}]), not WGS84 '
                    'degrees.\n'
                    '       Absolute georeferencing then needs the '
                    'reference point of that local frame:\n'
                    '         --origin-lat <deg> --origin-lon <deg> '
                    '[--origin-alt <m>]\n'
                    '       Without it only a local (non-geographic) DEM '
                    'can be written - see README.')
            e0, n0, _zone, _north = latlon_to_utm(args.origin_lat,
                                                  args.origin_lon)
            gx = gx + float(e0)
            gy = gy + float(n0)
            gz = gz + float(args.origin_alt)
            print(f'local GNSS frame anchored at {args.origin_lat}, '
                  f'{args.origin_lon} -> UTM ({float(e0):.1f}, '
                  f'{float(n0):.1f})')

        # ----------------------------------------------------------- fit
        candidates = fit_hypotheses(P_loc, t_loc, gx, gy, gz, t_g, args.tol)
        if not candidates or 'rms_m' not in candidates[0]:
            raise SystemExit('ERROR: no fit could be computed '
                             f'({json.dumps(candidates, default=str)[:400]})')
        best = candidates[0]

        if args.fit == 'full':
            # recompute with the free 3-D Umeyama transform
            pick, good = nearest_match(t_loc, t_g, args.tol)
            Q, zone, northern, bbox = utm_from_track(gx, gy, gz,
                                                     best['order'])
            R, t, rms = umeyama(P_loc[good], Q[pick[good]])
            best.update(rms_m=float(rms), yaw_deg=math.degrees(
                math.atan2(R[1, 0], R[0, 0])), rms_horizontal_m=None,
                rms_vertical_m=None, _R=R, _t=t)

        R, t = best['_R'], best['_t']
        span_deg = max(float(np.ptp(gx)), float(np.ptp(gy)))
        if best['order'] != 'x=meters' and span_deg < 1e-4:
            print('WARNING: the GNSS track barely moves horizontally - the '
                  'rotation is not observable.  Treat the yaw as unreliable.')

        epsg = args.epsg
        if epsg is None:
            if best.get('utm_zone'):
                zone = int(best['utm_zone'])
                epsg = (32600 if best['utm_northern'] else 32700) + zone
            else:
                raise SystemExit('ERROR: no UTM zone (local metric GNSS); '
                                 'pass --epsg explicitly')

        rms = float(best['rms_m'])
        rms_h = best.get('rms_horizontal_m')
        rms_v = best.get('rms_vertical_m')
        verdict = ('consistent' if rms <= args.rms_threshold else
                   'NOT consistent')
        print(f'\nfit[{best["order"]}] yaw={best["yaw_deg"]:.2f} deg  '
              f'RMS={rms:.3f} m  (threshold {args.rms_threshold} m)  '
              f'-> {verdict}')
        for c in candidates:
            if 'rms_m' in c:
                print(f'  hypothesis {c["order"]:>7}: '
                      f'RMS {c["rms_m"]:8.3f} m  '
                      f'scale {c["scale_diagnostic"]:.4f}  '
                      f'chord local {c["chord_local_m"]:.1f} m / '
                      f'gnss {c["chord_gnss_m"]:.1f} m')
        if rms > args.rms_threshold and not args.allow_poor:
            print('\nERROR: refusing to georeference with that residual - '
                  'the LiDAR trajectory and the GNSS track do not describe '
                  'the same motion.  Re-run FAST-LIO, check the IMU units '
                  '/ time sync, or pass --allow-poor to write the files '
                  'anyway.')
            report = {'alignment': {k: v for k, v in best.items()
                                    if not k.startswith('_')},
                      'candidates': [{k: v for k, v in c.items()
                                      if not k.startswith('_')}
                                     for c in candidates],
                      'verdict': verdict}
            out_json = args.json or args.out_dir / f'{prefix}_georef.json'
            out_json.write_text(json.dumps(report, indent=2, default=str))
            print(f'report: {out_json}')
            return 2

    from osgeo import osr
    srs = osr.SpatialReference()
    srs.ImportFromEPSG(int(epsg))
    crs_name = srs.GetName()

    # -------------------------------------------------- reproject the grid
    out_res = args.resolution or res
    grid2, origin2, grid_stats = reproject_grid(elev, res, origin, R, t,
                                                out_res=out_res)
    valid = np.isfinite(grid2)
    zmin = float(np.nanmin(grid2))
    zmax = float(np.nanmax(grid2))
    print(f'grid : {grid_stats["output_shape"]} @ {out_res} m, '
          f'{int(valid.sum())} valid cells, z {zmin:.2f} .. {zmax:.2f} m, '
          f'fill {grid_stats["fill_ratio"] * 100:.1f} %')

    dem_tif = args.out_dir / f'{prefix}_dem.tif'
    write_geotiff(dem_tif, grid2, origin2, out_res, epsg)
    hs = write_hillshade(dem_tif, args.out_dir / f'{prefix}_dem_hillshade.tif')
    png = write_preview(dem_tif, args.out_dir / f'{prefix}_dem_preview.png',
                        epsg, f'{prefix} DEM (georeferenced)')

    # DEM as a point cloud in UTM metres
    ny2, nx2 = grid2.shape
    iy, ix = np.nonzero(valid)
    px = origin2[0] + (ix + 0.5) * out_res
    py = origin2[1] + (iy + 0.5) * out_res
    pz = grid2[iy, ix].astype(np.float64)
    intensity = (pz - zmin) / (zmax - zmin + 1e-9) * 255.0
    dem_pcd = args.out_dir / f'{prefix}_dem.pcd'
    n_dem = pcd_io.write_pcd(dem_pcd, [px, py, pz, intensity],
                             ['x', 'y', 'z', 'intensity'],
                             comments=[f'UTM EPSG:{epsg} metres',
                                       f'resolution {out_res} m',
                                       'z = elevation (m), '
                                       'intensity = 0..255 height ramp'])

    # ------------------------------------------------------ the map cloud
    map_out = None
    if args.map:
        if not Path(args.map).exists():
            raise SystemExit(f'ERROR: map {args.map} not found')
        map_R, map_t = R, t
        if args.already_utm and warp.get('base_transform'):
            # the map is in the ORIGINAL local frame (not warped per
            # timestamp) - use the best single rigid tie from the report
            map_R = np.asarray(warp['base_transform']['R'], dtype=np.float64)
            map_t = np.asarray(warp['base_transform']['t'], dtype=np.float64)
            print('map  : applying the base rigid tie from the warp report '
                  '(single transform; the map itself is NOT drift-corrected)')
        map_out = args.out_dir / f'{prefix}_map_georef.pcd'
        info = pcd_io.transform_pcd(
            args.map, map_out, map_R, map_t,
            comments=[f'UTM EPSG:{epsg} metres (FAST-LIO map, georeferenced)'])
        print(f'map  : {info["count"]} points -> {map_out}')

    # ---------------------------------------------------------- the report
    def clean(c):
        return {k: v for k, v in c.items() if not k.startswith('_')}

    report = {
        'generated_utc': _dt.datetime.now(_dt.timezone.utc).isoformat(),
        'tool': 'dem_mapping.georef_export',
        'inputs': {
            'dem': str(args.dem),
            'odometry': str(args.odom),
            'bag': str(args.bag),
            'gnss_topic': args.gnss_topic,
            'map': str(args.map) if args.map else None,
            'dem_aggregation': agg,
            'dem_resolution_m': res,
            'dem_origin_local': list(origin),
        },
        'crs': {'epsg': epsg, 'name': crs_name, 'type': 'WGS84 / UTM'},
        'gnss': ({'order_used': order,
                  'utm_zone': int(epsg) % 100,
                  'note': 'grid built directly in UTM from the GNSS-warped '
                          'trajectory (see gnss_warp report)'}
                 if args.already_utm else
                 {'samples': int(len(t_g)),
                  'time_span_s': [float(t_g[0]), float(t_g[-1])],
                  'order_used': best['order'],
                  'utm_zone': best.get('utm_zone')}),
        'alignment': ({
            'method': 'gnss_aided_time_warp' if args.already_utm else
                      f'rigid_{args.fit}',
            'rotation_matrix': np.round(R, 9).tolist(),
            'translation_m': [round(float(v), 4) for v in t],
            'rms_m': round(rms, 4),
            'rms_horizontal_m': None if rms_h is None
                                else round(float(rms_h), 4),
            'rms_vertical_m': None if rms_v is None
                              else round(float(rms_v), 4),
            'threshold_m': args.rms_threshold,
            'verdict': verdict,
            'warp_report': str(args.warp_report) if args.warp_report
                           else None,
            'warp': warp if args.already_utm else None,
        } if args.already_utm else {
            'fit': args.fit,
            'rotation_matrix': np.round(R, 9).tolist(),
            'translation_m': [round(float(v), 4) for v in t],
            'yaw_deg': round(float(best.get('yaw_deg') or 0.0), 4),
            'rms_m': round(rms, 4),
            'rms_horizontal_m': None if best.get('rms_horizontal_m') is None
                                else round(best['rms_horizontal_m'], 4),
            'rms_vertical_m': None if best.get('rms_vertical_m') is None
                              else round(best['rms_vertical_m'], 4),
            'max_error_m': round(float(best.get('max_error_m') or 0.0), 4),
            'scale_diagnostic': round(float(best['scale_diagnostic']), 5),
            'tolerance_s': args.tol,
            'matched_samples': best['matched_samples'],
            'threshold_m': args.rms_threshold,
            'verdict': verdict,
            'hypotheses': [clean(c) for c in candidates],
        }),
        'grid': {**grid_stats, 'z_min_m': round(zmin, 3),
                 'z_max_m': round(zmax, 3), 'nodata': 'nan'},
        'outputs': {
            'dem_geotiff': str(dem_tif),
            'hillshade': str(hs),
            'preview': str(png),
            'dem_pcd': str(dem_pcd),
            'dem_pcd_points': int(n_dem),
            'map_pcd': str(map_out) if map_out else None,
        },
        'caveats': [
            'Elevations are on the GNSS altitude datum of this receiver '
            '(the map z axis was tied to /gnss_pose altitude).',
            'The transform is rigid: no scale, no shear - if the RMS is '
            'large, do not trust the absolute position.',
            'Re-run this exporter after any change to the LIO parameters; '
            'the transform is dataset specific.',
        ],
    }
    out_json = args.json or args.out_dir / f'{prefix}_georef.json'
    out_json.write_text(json.dumps(report, indent=2))

    print(f'\nEPSG : {epsg} ({crs_name})')
    print(f'transform: p_utm = R * p_local + t   '
          f't = {np.round(t, 3).tolist()}')
    print(f'wrote: {dem_tif}\n       {dem_pcd}\n'
          f'       {map_out if map_out else "-"}\n       {out_json}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
