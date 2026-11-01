#!/usr/bin/env python
"""Offline GNSS <-> pose alignment check (feasibility of georeferencing).

Reads a ROS 1 bag and compares

* the estimated/drone trajectory (odometry or pose topic), and
* the GNSS stream (geometry_msgs/PoseStamped in a local frame, or
  sensor_msgs/NavSatFix in WGS84 degrees which is converted to UTM),

fits the best rigid transform (rotation + translation, no scale - Umeyama)
between the two time-synchronised trajectories and reports the RMS
residual.

Interpretation
--------------
RMS of a few centimetres to ~0.3 m => the two frames are consistent and the
map can be georeferenced with that transform.
RMS of many metres => the streams are NOT consistent (missing extrinsic,
no GNSS fix, different epochs, ...).  Global georeferencing must then NOT
be claimed - use a local DEM instead.

Usage:
    rosrun dem_mapping gnss_align <bag> \
        --pose-topic /Odometry --gnss-topic /gnss_pose
"""

from __future__ import division

import argparse
import json
import sys

import numpy as np


def _sec(stamp):
    return stamp.to_sec()


def read_topic(bag, topic, max_msgs=None):
    """Read every message of ``topic`` from a ROS 1 bag."""
    import rosbag
    out = []
    with rosbag.Bag(bag, 'r') as b:
        info = b.get_type_and_topic_info().topics
        if topic not in info:
            raise SystemExit('topic {} not in bag (have: {})'.format(
                topic, sorted(info)))
        for _topic, msg, _t in b.read_messages(topics=[topic]):
            out.append(msg)
            if max_msgs and len(out) >= max_msgs:
                break
    return out


def pose_xyz(msg):
    name = type(msg).__name__
    if name == 'Odometry':
        p = msg.pose.pose.position
        return _sec(msg.header.stamp), np.array([p.x, p.y, p.z])
    if name == 'PoseStamped':
        p = msg.pose.position
        return _sec(msg.header.stamp), np.array([p.x, p.y, p.z])
    raise SystemExit('unsupported pose message: {}'.format(name))


def gnss_xyz(msg, cache={}):
    name = type(msg).__name__
    if name == 'NavSatFix':
        from dem_mapping.georef import latlon_to_utm
        t = _sec(msg.header.stamp)
        key = cache.setdefault('zone', None)
        e, n, zone, northern = latlon_to_utm(msg.latitude, msg.longitude,
                                             zone=key)
        cache['zone'] = zone
        return t, np.array([float(e), float(n), float(msg.altitude)])
    if name == 'PoseStamped':
        p = msg.pose.position
        return _sec(msg.header.stamp), np.array([p.x, p.y, p.z])
    raise SystemExit('unsupported gnss message: {}'.format(name))


def umeyama(src, dst):
    """Best rigid transform (no scale) mapping src -> dst."""
    mu_s = src.mean(axis=0)
    mu_d = dst.mean(axis=0)
    cs = src - mu_s
    cd = dst - mu_d
    cov = np.dot(cd.T, cs) / src.shape[0]
    U, S, Vt = np.linalg.svd(cov)
    D = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        D[2, 2] = -1.0
    R = np.dot(np.dot(U, D), Vt)
    t = mu_d - np.dot(R, mu_s)
    aligned = np.dot(src, R.T) + t
    rms = float(np.sqrt(np.mean(np.sum((aligned - dst) ** 2, axis=1))))
    return R, t, rms


def nearest_match(t_a, t_b, tol=0.2):
    idx = np.searchsorted(t_b, t_a)
    idx = np.clip(idx, 0, len(t_b) - 1)
    idx0 = np.clip(idx - 1, 0, len(t_b) - 1)
    pick = np.where(np.abs(t_b[idx] - t_a) <= np.abs(t_b[idx0] - t_a),
                    idx, idx0)
    good = np.abs(t_b[pick] - t_a) <= tol
    return pick, good


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('bag')
    ap.add_argument('--pose-topic', default='/Odometry')
    ap.add_argument('--gnss-topic', default='/gnss_pose')
    ap.add_argument('--tol', type=float, default=0.2,
                    help='max time difference for matching samples (s)')
    ap.add_argument('--json', action='store_true')
    args = ap.parse_args(argv)

    poses = read_topic(args.bag, args.pose_topic)
    gnss = read_topic(args.bag, args.gnss_topic)
    if not poses or not gnss:
        raise SystemExit('empty topic(s)')

    tp = np.array([pose_xyz(m)[0] for m in poses])
    pp = np.array([pose_xyz(m)[1] for m in poses])
    tg = np.array([gnss_xyz(m)[0] for m in gnss])
    pg = np.array([gnss_xyz(m)[1] for m in gnss])

    pick, good = nearest_match(tp, tg, args.tol)
    n_match = int(good.sum())
    report = {
        'pose_topic': args.pose_topic,
        'gnss_topic': args.gnss_topic,
        'pose_samples': int(len(tp)),
        'gnss_samples': int(len(tg)),
        'matched_samples': n_match,
        'pose_span_m': float(np.linalg.norm(pp[-1] - pp[0])),
        'gnss_span_m': float(np.linalg.norm(pg[-1] - pg[0])),
    }
    if n_match < 10:
        report['verdict'] = 'insufficient overlapping data'
    else:
        a = pp[good]
        b = pg[pick[good]]
        R, t, rms = umeyama(a, b)
        report.update({
            'rms_raw_m': float(np.sqrt(np.mean(np.sum((a - b) ** 2, axis=1)))),
            'rms_aligned_m': rms,
            'rotation_matrix': np.round(R, 6).tolist(),
            'translation_m': np.round(t, 3).tolist(),
        })
        span = float(np.linalg.norm(a[-1] - a[0]))
        report['pose_path_length_m'] = span
        if report['gnss_span_m'] < 1.0:
            report['verdict'] = (
                'GNSS positions are essentially static - no horizontal GNSS '
                'motion is available. Global georeferencing is NOT possible '
                'with this recording; use the local mapper frame.')
        elif rms < 0.5:
            report['verdict'] = (
                'GNSS and pose trajectory are consistent after a rigid '
                'alignment - georeferencing is plausible. Verify sensor '
                'extrinsics before claiming it.')
        else:
            report['verdict'] = (
                'RMS residual after alignment is {:.2f} m - the frames '
                'are NOT consistent. Do not claim global georeferencing.'.format(rms))

    if args.json:
        print(json.dumps(report, indent=2))
    else:
        for k, v in report.items():
            if isinstance(v, list):
                v = json.dumps(v)
            print('{:>22}: {}'.format(k, v))
    return 0


if __name__ == '__main__':
    sys.exit(main())
