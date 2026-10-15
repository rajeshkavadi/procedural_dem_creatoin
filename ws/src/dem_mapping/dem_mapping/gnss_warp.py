#!/usr/bin/env python3
"""GNSS-aided trajectory warping: tie a drifting LIO track to GNSS over time.

Why
---
A single rigid transform assumes the LiDAR-inertial trajectory does not
drift.  On vibrating, low-cost-IMU platforms it does: the track agrees with
GNSS locally to a few centimetres but slowly walks away from it (yaw and
height drift over minutes).  A rigid fit then has a multi-metre RMS and is
rightly rejected.

This tool estimates a *time-parameterized* alignment instead:

1. match the LIO trajectory (``odom.csv`` from ``record_odom``) to the GNSS
   track from the bag by timestamp,
2. fit a full 3-D rigid transform in every sliding window (default 60 s,
   step 30 s) - the windows measure the drift, not the local agreement,
3. drop outlier windows (poor local agreement = bad GNSS or bad LIO, not
   drift),
4. interpolate the window rotations (quaternion slerp) and translations
   over time to obtain R(t), t(t),
5. write every LIO pose warped into UTM: ``p' = R(t) p + t(t)`` and
   ``R'_wb(t) = R(t) R_wb(t)``.

The result is a trajectory that is *locally as good as the LIO* and
*globally as accurate as the GNSS*.  Feed it to ``dem_node`` in ``pose``
mode (see ``pose_play``) to build a DEM directly in UTM, or use the JSON
report as provenance.

Usage
-----
    ros2 run dem_mapping gnss_warp -- \
        --odom out/odom.csv --bag OutdoorRoad_dataset_ros2 \
        --out out/warped_odom.csv --report out/warp_report.json
"""

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

from dem_mapping.georef_export import (_load_gnss, load_odom, nearest_match,
                                       utm_from_track, umeyama)


# --------------------------------------------------------------- quaternions

def rot_to_quat(R):
    """(...,3,3) rotation matrices -> (...,4) quaternions (x, y, z, w)."""
    m = np.asarray(R, dtype=np.float64)
    batch = m.shape[:-2]
    m = m.reshape(-1, 3, 3)
    q = np.zeros((m.shape[0], 4), dtype=np.float64)
    for k, r in enumerate(m):
        tr = np.trace(r)
        if tr > 0:
            s = math.sqrt(tr + 1.0) * 2
            q[k] = [(r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s,
                    (r[1, 0] - r[0, 1]) / s, 0.25 * s]
        elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
            s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2
            q[k] = [0.25 * s, (r[0, 1] + r[1, 0]) / s,
                    (r[0, 2] + r[2, 0]) / s, (r[2, 1] - r[1, 2]) / s]
        elif r[1, 1] > r[2, 2]:
            s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2
            q[k] = [(r[0, 1] + r[1, 0]) / s, 0.25 * s,
                    (r[1, 2] + r[2, 1]) / s, (r[0, 2] - r[2, 0]) / s]
        else:
            s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2
            q[k] = [(r[0, 2] + r[2, 0]) / s, (r[1, 2] + r[2, 1]) / s,
                    0.25 * s, (r[1, 0] - r[0, 1]) / s]
    q /= np.linalg.norm(q, axis=1, keepdims=True)
    return q.reshape(batch + (4,))


def quat_mul(a, b):
    """Hamilton product of (...,4) quaternions (x, y, z, w)."""
    ax, ay, az, aw = np.moveaxis(a, -1, 0)
    bx, by, bz, bw = np.moveaxis(b, -1, 0)
    return np.stack([
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz], axis=-1)


def slerp_series(q, t_samples, t_query):
    """Piecewise quaternion slerp of keyframes ``q`` at ``t_samples``."""
    q = np.asarray(q, dtype=np.float64)
    t_samples = np.asarray(t_samples, dtype=np.float64)
    tq = np.asarray(t_query, dtype=np.float64)
    seg = np.clip(np.searchsorted(t_samples, tq) - 1, 0, len(t_samples) - 2)
    t0, t1 = t_samples[seg], t_samples[seg + 1]
    u = np.clip((tq - t0) / np.maximum(t1 - t0, 1e-9), 0.0, 1.0)
    q0, q1 = q[seg], q[seg + 1]
    dot = np.sum(q0 * q1, axis=1, keepdims=True)
    q1 = np.where(dot < 0, -q1, q1)          # shortest arc
    dot = np.abs(dot)
    out = np.empty_like(q0)
    u2 = u[:, None]
    small = dot[:, 0] > 0.9995               # fall back to lerp
    if small.any():
        out[small] = q0[small] + (q1[small] - q0[small]) * u2[small]
    if (~small).any():
        th = np.arccos(np.clip(dot[~small], -1.0, 1.0))
        s = np.sin(th)
        w0 = np.sin((1.0 - u2[~small]) * th) / s
        w1 = np.sin(u2[~small] * th) / s
        out[~small] = q0[~small] * w0 + q1[~small] * w1
    return out / np.linalg.norm(out, axis=1, keepdims=True)


# -------------------------------------------------------------------- warp


def estimate_windows(tm, Pm, Qm, win_s, step_s):
    """Full 3-D rigid fits in sliding windows -> (centers, R, t, rms)."""
    t0 = float(tm[0])
    centers, Rs, ts, rmses = [], [], [], []
    for s in np.arange(0.0, float(tm[-1] - t0) - win_s + 1e-6, step_s):
        m = (tm >= t0 + s) & (tm < t0 + s + win_s)
        if m.sum() < 50:
            continue
        R, t, rms = umeyama(Pm[m], Qm[m])
        centers.append(t0 + s + win_s / 2.0)
        Rs.append(R)
        ts.append(t)
        rmses.append(rms)
    return (np.array(centers), np.array(Rs), np.array(ts),
            np.array(rmses))


def fit_smooth_correction(t, res, knot_s=10.0, iters=4):
    """Robust piecewise-linear fit of a per-timestamp 3-D correction.

    ``t`` (n,) timestamps, ``res`` (n,3) residuals to model (e.g.
    GNSS minus base-aligned LIO position).  Returns a callable evaluating
    the correction at arbitrary timestamps.  Huber-IRLS makes the fit
    insensitive to short LIO glitches while tracking the slow drift (and
    the metre-scale wander during standstills).
    """
    t = np.asarray(t, dtype=np.float64)
    res = np.asarray(res, dtype=np.float64)
    kc = np.arange(t[0], t[-1] + knot_s, knot_s)
    kc = np.maximum.accumulate(np.append(kc, t[-1]))

    def design(query):
        seg = np.clip(np.searchsorted(kc, query) - 1, 0, len(kc) - 2)
        u = (query - kc[seg]) / np.maximum(kc[seg + 1] - kc[seg], 1e-9)
        A = np.zeros((len(query), len(kc)))
        A[np.arange(len(query)), seg] = 1.0 - u
        A[np.arange(len(query)), seg + 1] = u
        return A

    A = design(t)
    x = np.linalg.lstsq(A, res, rcond=None)[0]
    for _ in range(iters):
        r = res - A @ x
        rn = np.linalg.norm(r, axis=1)
        s = 1.4826 * np.median(rn) + 1e-9
        w = np.minimum(1.0, (1.345 * s) / np.maximum(rn, 1e-9))
        x = np.linalg.lstsq(A * w[:, None], res * w[:, None], rcond=None)[0]

    def evaluate(tq):
        return design(np.asarray(tq, dtype=np.float64)) @ x

    return evaluate


def warp_trajectory(t_loc, P, Q0, windows, t_gnss, knot_s=10.0):
    """Apply the interpolated window transform to every LIO pose.

    Returns (P_utm, R_delta (n,3,3), stats dict).
    """
    centers, Rs, ts, rmses = windows
    med = float(np.median(rmses))
    keep = rmses <= max(3.0 * med, 0.5)      # drop locally-inconsistent wins
    if keep.sum() < 2:                        # everything looks the same
        keep = np.ones_like(keep, dtype=bool)
    kc, kR, kt = centers[keep], Rs[keep], ts[keep]

    q_win = rot_to_quat(kR)
    q_s = slerp_series(q_win, kc, t_loc)      # (n,4) rotation per timestamp
    # translation: linear between window translations
    tx = np.interp(t_loc, kc, kt[:, 0])
    ty = np.interp(t_loc, kc, kt[:, 1])
    tz = np.interp(t_loc, kc, kt[:, 2])
    from dem_mapping.pose_buffer import quat_to_rot
    R_s = quat_to_rot(q_s)                    # (n,3,3)
    P_utm = np.einsum('nij,nj->ni', R_s, P) + np.stack([tx, ty, tz], axis=1)

    # residuals on the time-matched GNSS samples: the window stage fixes
    # the smooth drift, but the transform interpolation cannot follow the
    # metre-scale LIO wander during standstills - a robust smooth residual
    # correction finishes the job.
    pick, good = nearest_match(t_loc, t_gnss, 0.2)
    res_win = Q0[pick[good]] - P_utm[good]
    rms_win = float(np.sqrt((res_win ** 2).sum(1).mean()))
    corr = fit_smooth_correction(t_loc[good], res_win, knot_s=knot_s)
    P_utm = P_utm + corr(t_loc)

    res = Q0[pick[good]] - P_utm[good]
    h = np.linalg.norm(res[:, :2], axis=1)
    stats = {
        'windows_total': int(len(rmses)),
        'windows_used': int(keep.sum()),
        'windows_dropped': [float(c) for c in centers[~keep]],
        'window_rms_median_m': med,
        'window_rms_max_m': float(rmses.max()),
        'residual_rigid_rms_m': None,          # filled by caller
        'residual_window_stage_rms_m': rms_win,
        'correction_knot_s': float(knot_s),
        'residual_warp_rms_m': float(np.sqrt((res ** 2).sum(1).mean())),
        'residual_warp_h_rms_m': float(np.sqrt((h ** 2).mean())),
        'residual_warp_v_rms_m': float(np.sqrt((res[:, 2] ** 2).mean())),
        'residual_warp_max_m': float(np.linalg.norm(res, axis=1).max()),
        'matched_samples': int(good.sum()),
    }
    return P_utm, R_s, stats


def main(argv=None):
    ap = argparse.ArgumentParser(
        prog='gnss_warp', description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--odom', required=True, type=Path,
                    help='CSV written by dem_replay record_odom')
    ap.add_argument('--bag', required=True, type=Path)
    ap.add_argument('--gnss-topic', default='/gnss_pose')
    ap.add_argument('--out', required=True, type=Path,
                    help='output CSV: warped poses in UTM metres')
    ap.add_argument('--report', type=Path, default=None)
    ap.add_argument('--window', type=float, default=60.0)
    ap.add_argument('--step', type=float, default=30.0)
    ap.add_argument('--tol', type=float, default=0.2)
    ap.add_argument('--knot', type=float, default=10.0,
                    help='knot spacing (s) of the robust residual '
                         'correction stage (0 disables it)')
    args = ap.parse_args(argv)

    t_loc, P = load_odom(args.odom)
    t_g, gx, gy, gz = _load_gnss(args.bag, args.gnss_topic)
    print(f'odom: {len(P)} samples, {t_loc[-1] - t_loc[0]:.1f} s')
    print(f'gnss: {len(t_g)} samples')

    # lat/lon order: the metric hypothesis with the lower rigid RMS wins
    best = None
    for order in ('x=lat', 'x=lon'):
        Q, zone, north, _bbox = utm_from_track(gx, gy, gz, order)
        pick, good = nearest_match(t_loc, t_g, args.tol)
        if good.sum() < 100:
            continue
        _R, _t, rms = umeyama(P[good], Q[pick[good]])
        print(f'  hypothesis {order}: rigid RMS {rms:.3f} m')
        if best is None or rms < best[0]:
            best = (rms, order, Q, zone, north)
    if best is None:
        raise SystemExit('ERROR: no time-matched GNSS samples - check the '
                         'topic name and the clocks')
    rigid_rms, order, Q, zone, north = best
    epsg = (32600 if north else 32700) + int(zone)
    suffix = 'N' if north else 'S'
    print(f'order: {order} -> UTM zone {int(zone)}{suffix} '
          f'(EPSG:{epsg}), rigid-fit RMS {rigid_rms:.3f} m')

    pick, good = nearest_match(t_loc, t_g, args.tol)
    Pm, Qm, tm = P[good], Q[pick[good]], t_loc[good]

    R0, t0, _rms0 = umeyama(Pm, Qm)      # single rigid base (drift shows up)

    windows = estimate_windows(tm, Pm, Qm, args.window, args.step)
    print(f'windows: {len(windows[0])} fits '
          f'({args.window:.0f} s window, {args.step:.0f} s step), '
          f'median RMS {np.median(windows[3]):.3f} m, '
          f'max {windows[3].max():.3f} m')

    P_utm, R_s, stats = warp_trajectory(t_loc, P, Q, windows, t_g,
                                        knot_s=max(args.knot, 0.1))
    stats['residual_rigid_rms_m'] = float(rigid_rms)
    s = stats
    print(f'post-warp residual: RMS {s["residual_warp_rms_m"]:.3f} m '
          f'(h {s["residual_warp_h_rms_m"]:.3f}, '
          f'v {s["residual_warp_v_rms_m"]:.3f}, '
          f'max {s["residual_warp_max_m"]:.3f}) over '
          f'{s["matched_samples"]} matched samples')

    # warped orientations: R'_wb(t) = R(t) R_wb(t) - handled in _finish
    return _finish(args, t_loc, P, P_utm, R_s, stats, order, zone, north,
                   epsg, rigid_rms, windows)


def _finish(args, t_loc, P, P_utm, R_s, stats, order, zone, north, epsg,
            rigid_rms, windows):
    # re-read the quaternions (load_odom keeps positions only)
    data = np.genfromtxt(args.odom, delimiter=',', names=True)
    q = np.stack([data['qx'], data['qy'], data['qz'], data['qw']], axis=1)
    q_new = quat_mul(rot_to_quat(R_s), q)

    args.out.parent.mkdir(parents=True, exist_ok=True)
    with open(args.out, 'w') as fh:
        fh.write('t,x,y,z,qx,qy,qz,qw\n')
        for i in range(len(t_loc)):
            fh.write(f'{t_loc[i]:.9f},{P_utm[i,0]:.4f},{P_utm[i,1]:.4f},'
                     f'{P_utm[i,2]:.4f},{q_new[i,0]:.9f},{q_new[i,1]:.9f},'
                     f'{q_new[i,2]:.9f},{q_new[i,3]:.9f}\n')
    print(f'wrote {len(t_loc)} warped poses -> {args.out}')

    e, n = P_utm[:, 0], P_utm[:, 1]
    report = {
        'input_odom': str(args.odom),
        'gnss_topic': args.gnss_topic,
        'order': order,
        'utm_zone': int(zone),
        'utm_northern': bool(north),
        'epsg': int(epsg),
        'rigid_fit_rms_m': float(rigid_rms),
        # single rigid transform local -> UTM (the best a non-aided export
        # could do; georef_export --already-utm uses it for the local map
        # cloud, the DEM grid itself is warped per timestamp)
        'base_transform': {'R': np.round(R0, 9).tolist(),
                           't': [round(float(v), 4) for v in t0]},
        'window_s': args.window,
        'step_s': args.step,
        'window_fits': [{'center_s': float(c), 'rms_m': float(r),
                         'used': bool(r <= max(3.0 * np.median(windows[3]),
                                               0.5))}
                        for c, r in zip(windows[0], windows[3])],
        'track_extent_m': [float(e.max() - e.min()), float(n.max() - n.min())],
        'track_bbox_utm': [float(e.min()), float(n.min()),
                           float(e.max()), float(n.max())],
        'stats': stats,
    }
    rep = args.report or args.out.with_name(args.out.stem + '_report.json')
    rep.write_text(json.dumps(report, indent=2, default=str))
    print(f'report: {rep}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
