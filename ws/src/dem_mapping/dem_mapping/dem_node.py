#!/usr/bin/env python
"""Incremental CPU elevation-mapping (DEM) node (ROS 1 / Melodic).

Consumes successive point clouds (either already registered into a map frame
by an odometry pipeline such as FAST-LIO2, or raw scans that are transformed
with a live pose stream), filters them, and merges them **in place** into a
2-D elevation grid.  The grid is never rebuilt from historical points.

Published topics
----------------
~/elevation_image   sensor_msgs/Image   32FC1 elevation, NaN = NoData
~/elevation_cloud   sensor_msgs/PointCloud2  cell centres coloured by height
~/occupancy         nav_msgs/OccupancyGrid   -1 = unknown, 0..100 = scaled z
~/trajectory        nav_msgs/Path           vehicle trajectory (if poses seen)
~/stats             std_msgs/String         JSON processing statistics

Services
--------
~/reset  std_srvs/Trigger   clear the grid
~/save   std_srvs/Trigger   dump the grid to <save_dir>/dem_<stamp>.npz
"""

from __future__ import division

import json
import math
import os
import threading
import time
from collections import deque

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse

from dem_mapping.elevation_grid import ElevationGrid
from dem_mapping.ground_filters import build_filter
from dem_mapping.pointcloud_utils import (pointcloud2_to_xyz,
                                          pointcloud2_point_times)
from dem_mapping.pose_buffer import PoseBuffer


def _stamp_to_sec(stamp):
    return stamp.to_sec()


def _elevation_colors(z, z_min, z_max):
    """Simple elevation -> RGB ramp (0=blue .. 1=red), no matplotlib."""
    if not np.isfinite(z_min) or not np.isfinite(z_max) or z_max <= z_min:
        z_min, z_max = 0.0, 1.0
    t = np.clip((np.asarray(z, dtype=np.float64) - z_min) / (z_max - z_min),
                0.0, 1.0)
    # blue -> cyan -> green -> yellow -> red
    stops = np.array([[0.0, 0.0, 0.4],
                      [0.0, 0.5, 1.0],
                      [0.0, 0.8, 0.2],
                      [1.0, 0.9, 0.0],
                      [1.0, 0.1, 0.0]])
    pos = np.linspace(0.0, 1.0, stops.shape[0])
    r = np.interp(t, pos, stops[:, 0])
    g = np.interp(t, pos, stops[:, 1])
    b = np.interp(t, pos, stops[:, 2])
    return (np.stack([r, g, b], axis=-1) * 255.0).astype(np.uint8)


def _message_class(type_str):
    """Resolve a ROS 1 message type ('nav_msgs/Odometry')."""
    from roslib.message import get_message_class
    if not type_str:
        return None
    # tolerate ROS 2 style strings ('nav_msgs/msg/Odometry')
    type_str = type_str.replace('/msg/', '/')
    cls = get_message_class(type_str)
    if cls is None:
        raise ValueError('unknown message type: {}'.format(type_str))
    return cls


class DemNode(object):

    def __init__(self):
        rospy.init_node('dem_node')

        # -------------------------------------------------------- I/O
        self._frame = rospy.get_param('~output_frame', 'map')
        self._mode = rospy.get_param('~mode', 'registered')
        if self._mode not in ('registered', 'pose'):
            raise ValueError("mode must be 'registered' or 'pose'")

        # -------------------------------------------------------- grid
        self._grid = ElevationGrid(
            resolution=float(rospy.get_param('~resolution', 0.2)),
            size_x=float(rospy.get_param('~size_x', 300.0)),
            size_y=float(rospy.get_param('~size_y', 300.0)),
            origin_x=float(rospy.get_param('~origin_x', -150.0)),
            origin_y=float(rospy.get_param('~origin_y', -150.0)),
            aggregation=rospy.get_param('~aggregation', 'min'))

        self._radial = build_filter(
            'radial_range',
            min_range=float(rospy.get_param('~min_range', 1.0)),
            max_range=float(rospy.get_param('~max_range', 60.0)))
        fname = rospy.get_param('~ground_filter', 'height_band')
        self._ground = build_filter(
            fname,
            z_min=float(rospy.get_param('~z_min', -100.0)),
            z_max=float(rospy.get_param('~z_max', 100.0)),
            below=float(rospy.get_param('~band_below', 6.0)),
            above=float(rospy.get_param('~band_above', 4.0)))
        self._subsample = build_filter(
            'max_points',
            max_points=int(rospy.get_param('~max_points_per_scan', 12000)))
        rospy.loginfo('ground filter: %s (%s)', fname,
                      type(self._ground).__name__)

        T = np.asarray(rospy.get_param(
            '~T_body_lidar',
            [1.0, 0.0, 0.0, 0.0,
             0.0, 1.0, 0.0, 0.0,
             0.0, 0.0, 1.0, 0.0,
             0.0, 0.0, 0.0, 1.0]), dtype=np.float64).reshape(4, 4)
        self._R_bl = T[:3, :3]
        self._t_bl = T[:3, 3]

        self._motion_comp = bool(rospy.get_param('~motion_comp', True))
        self._pose_timeout = float(rospy.get_param('~pose_timeout', 0.5))
        self._poses = PoseBuffer(max_span_s=20.0)

        # -------------------------------------------------- statistics
        self._n_clouds = 0
        self._n_dropped_no_pose = 0
        self._n_points_in = 0
        self._n_points_used = 0
        self._hist = deque(maxlen=200)          # (stage, ms) samples
        self._hz = deque(maxlen=50)             # arrival wall times
        self._path = Path()
        self._path.header.frame_id = self._frame

        # rospy dispatches every Subscriber and Timer callback in its own
        # thread; the lock protects the grid and the path from concurrent
        # updates.  Large DEM messages are serialised outside the lock.
        self._lock = threading.Lock()

        cloud_topic = rospy.get_param('~cloud_topic', '/mid360/livox/lidar')
        self._sub_cloud = rospy.Subscriber(
            cloud_topic, PointCloud2, self._cloud_cb, queue_size=10)

        pose_topic = rospy.get_param('~pose_topic', '/Odometry')
        if pose_topic:
            pose_cls = _message_class(
                rospy.get_param('~pose_type', 'nav_msgs/Odometry'))
            self._sub_pose = rospy.Subscriber(
                pose_topic, pose_cls, self._pose_cb, queue_size=200)
            rospy.loginfo('pose source: %s (%s)', pose_topic,
                          pose_cls.__name__)

        self._pub_img = rospy.Publisher(
            '~/elevation_image', Image, queue_size=5)
        self._pub_cloud = rospy.Publisher(
            '~/elevation_cloud', PointCloud2, queue_size=5)
        self._pub_occ = rospy.Publisher(
            '~/occupancy', OccupancyGrid, queue_size=5)
        self._pub_traj = rospy.Publisher(
            '~/trajectory', Path, queue_size=5)
        self._pub_stats = rospy.Publisher(
            '~/stats', String, queue_size=10)

        self._srv_reset = rospy.Service('~/reset', Trigger, self._srv_reset_cb)
        self._srv_save = rospy.Service('~/save', Trigger, self._srv_save_cb)

        self._rate_event('image_rate', 'image', self._publish_image, 1.0)
        self._rate_event('cloud_rate', 'cloud', self._publish_cloud, 1.0)
        self._rate_event('occupancy_rate', 'occ', self._publish_occupancy, 1.0)
        self._rate_event('trajectory_rate', 'traj', self._publish_traj, 2.0)
        self._rate_event('stats_rate', 'stats', self._publish_stats, 2.0)

        rospy.loginfo(
            'dem_node: cloud=%s mode=%s res=%sm grid=%sx%s agg=%s',
            cloud_topic, self._mode, self._grid.resolution,
            self._grid.nx, self._grid.ny, self._grid.aggregation)

        rospy.on_shutdown(self._on_shutdown)

    def _rate_event(self, param, _name, cb, default):
        rate = float(rospy.get_param('~' + param, default))
        if rate > 0:
            rospy.Timer(rospy.Duration(1.0 / rate), cb)

    # ------------------------------------------------------------- pose in
    def _pose_cb(self, msg):
        typ = type(msg).__name__
        if typ in ('Odometry', 'PoseWithCovarianceStamped'):
            t = _stamp_to_sec(msg.header.stamp)
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
        else:                       # PoseStamped (or anything pose-like)
            t = _stamp_to_sec(msg.header.stamp)
            p = msg.pose.position
            q = msg.pose.orientation
        ok = self._poses.add(t, (p.x, p.y, p.z), (q.x, q.y, q.z, q.w))
        if ok:
            ps = PoseStamped()
            ps.header.stamp = msg.header.stamp
            ps.header.frame_id = self._frame
            ps.pose.position = p
            ps.pose.orientation = q
            with self._lock:
                self._path.header.stamp = msg.header.stamp
                self._path.poses.append(ps)
                if len(self._path.poses) > 20000:
                    self._path.poses = self._path.poses[-20000:]

    # ---------------------------------------------------------- cloud in
    def _cloud_cb(self, msg):
        wall_t0 = time.time()
        scan_t = _stamp_to_sec(msg.header.stamp)

        try:
            xyz = pointcloud2_to_xyz(msg)
        except Exception as exc:
            rospy.logwarn('bad point cloud: %s', exc)
            return
        n_in = xyz.shape[0]
        if n_in == 0:
            return
        good = np.isfinite(xyz).all(axis=1)
        xyz = xyz[good]
        t_parse = (time.time() - wall_t0) * 1e3

        # ---- transform into the map frame ----------------------------
        t_xf0 = time.time()
        pose = self._poses.pose_at(scan_t, max_age=self._pose_timeout)
        if pose is None and self._mode == 'pose':
            self._n_dropped_no_pose += 1
            return
        pose_z = pose_xyz = None
        if pose is not None:
            _R_wb, t_wb = pose
            pose_xyz = t_wb
            pose_z = float(t_wb[2])
        else:
            _R_wb, t_wb = None, None

        if self._mode == 'pose':
            pts_body = np.dot(xyz, self._R_bl.T) + self._t_bl
            rel = pointcloud2_point_times(msg) if self._motion_comp else None
            if rel is not None and rel.shape[0] == pts_body.shape[0]:
                t_pts = scan_t + rel
                xyz, _exact, _fb = self._poses.transform_points(
                    t_pts, pts_body, max_age=self._pose_timeout)
            else:
                xyz = np.dot(pts_body, _R_wb.T) + t_wb
        t_xf = (time.time() - t_xf0) * 1e3

        # ---- ground / quality filtering -------------------------------
        t_f0 = time.time()
        ctx = {'pose_z': pose_z, 'pose_xyz': pose_xyz, 'stamp': scan_t}
        xyz = self._radial(xyz, ctx)
        xyz = self._ground(xyz, ctx)
        xyz = self._subsample(xyz, ctx)
        t_filt = (time.time() - t_f0) * 1e3

        # ---- incremental grid update ----------------------------------
        t_u0 = time.time()
        with self._lock:
            res = self._grid.update(xyz[:, :2], xyz[:, 2])
        t_upd = (time.time() - t_u0) * 1e3

        self._n_clouds += 1
        self._n_points_in += n_in
        self._n_points_used += res['accepted']
        self._hist.append({'parse': t_parse, 'transform': t_xf,
                           'filter': t_filt, 'update': t_upd,
                           'total': (time.time() - wall_t0) * 1e3})
        self._hz.append(time.time())

    # --------------------------------------------------------- publishing
    def _publish_image(self, _event=None):
        with self._lock:
            if not self._grid.total_updates:
                return
            elev, _valid, _stats = self._grid.snapshot()
            ny, nx = self._grid.ny, self._grid.nx
        # the snapshot is immutable until the next grid update, so the
        # 9 MB serialisation happens outside the lock
        img = Image()
        img.header.stamp = rospy.Time.now()
        img.header.frame_id = self._frame
        img.height, img.width = ny, nx
        img.encoding = '32FC1'
        img.is_bigendian = 0
        img.step = nx * 4
        img.data = np.ascontiguousarray(elev, dtype=np.float32).tobytes()
        self._pub_img.publish(img)

    def _publish_cloud(self, _event=None):
        with self._lock:
            if not self._grid.total_updates:
                return
            elev, flat_valid, stats = self._grid.snapshot()
            if stats['valid_cells'] == 0:
                return
            res = self._grid.resolution
            ox, oy = self._grid.origin_x, self._grid.origin_y
            nx = self._grid.nx
        # outside the lock: the snapshot is immutable until the next update
        valid_flat = flat_valid.reshape(-1)
        idx = np.flatnonzero(valid_flat)
        z = elev.reshape(-1)[valid_flat]
        xy = np.stack([ox + (idx % nx + 0.5) * res,
                       oy + (idx // nx + 0.5) * res], axis=1)

        max_cells = int(rospy.get_param('~cloud_max_cells', 150000))
        if xy.shape[0] > max_cells:
            stride = int(np.ceil(xy.shape[0] / max_cells))
            xy = xy[::stride]
            z = z[::stride]

        rgb = _elevation_colors(z, stats['z_min'], stats['z_max'])
        n = xy.shape[0]
        msg = PointCloud2()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self._frame
        msg.height = 1
        msg.width = n
        msg.fields = [
            PointField(name='x', offset=0,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32,
                       count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = 16 * n
        msg.is_dense = True
        buf = np.zeros((n, 16), dtype=np.uint8)
        buf[:, 0:4] = xy[:, 0].astype(np.float32).view(np.uint8).reshape(n, 4)
        buf[:, 4:8] = xy[:, 1].astype(np.float32).view(np.uint8).reshape(n, 4)
        buf[:, 8:12] = z.astype(np.float32).view(np.uint8).reshape(n, 4)
        packed = (rgb[:, 0].astype(np.uint32) << 16) | \
                 (rgb[:, 1].astype(np.uint32) << 8) | \
                 rgb[:, 2].astype(np.uint32)
        buf[:, 12:16] = packed.astype(np.uint32).view(np.uint8).reshape(n, 4)
        msg.data = buf.tobytes()
        self._pub_cloud.publish(msg)

    def _publish_occupancy(self, _event=None):
        with self._lock:
            if not self._grid.total_updates:
                return
            elev, _valid, _stats = self._grid.snapshot()
            res = self._grid.resolution
            nx, ny = self._grid.nx, self._grid.ny
            ox, oy = self._grid.origin_x, self._grid.origin_y
        z_min = float(rospy.get_param('~occ_z_min', -6.0))
        z_max = float(rospy.get_param('~occ_z_max', 4.0))
        valid = np.isfinite(elev)
        occ = np.full(elev.shape, -1, dtype=np.int8)
        if valid.any():
            if z_max <= z_min:
                z_min, z_max = float(np.nanmin(elev)), float(np.nanmax(elev))
                if z_max <= z_min:
                    z_max = z_min + 1.0
            scaled = np.clip((elev - z_min) / (z_max - z_min), 0.0, 1.0)
            occ[valid] = (scaled[valid] * 100).astype(np.int8)
        msg = OccupancyGrid()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self._frame
        msg.info.resolution = res
        msg.info.width = nx
        msg.info.height = ny
        msg.info.origin.position.x = ox
        msg.info.origin.position.y = oy
        msg.info.origin.orientation.w = 1.0
        # OccupancyGrid data is int8[] row-major starting at the origin
        # corner; array('b', ...) gives signed ints genpy accepts.
        import array as _array
        msg.data = _array.array('b', occ.tobytes())
        self._pub_occ.publish(msg)

    def _publish_traj(self, _event=None):
        with self._lock:
            if not self._path.poses:
                return
            path = Path()
            path.header = self._path.header
            path.poses = list(self._path.poses)
        self._pub_traj.publish(path)

    def _publish_stats(self, _event=None):
        rate = 0.0
        if len(self._hz) >= 2:
            hz = list(self._hz)
            rate = (len(hz) - 1) / (hz[-1] - hz[0])
        stage_ms = {}
        hist = list(self._hist)
        if hist:
            for key in ('parse', 'transform', 'filter', 'update', 'total'):
                vals = [h[key] for h in hist]
                stage_ms[key] = {
                    'last': round(vals[-1], 3),
                    'mean': round(float(np.mean(vals)), 3),
                    'p95': round(float(np.percentile(vals, 95)), 3),
                }
        with self._lock:
            _elev, _valid, stats = self._grid.snapshot()
        out = {
            'clouds_received': self._n_clouds,
            'clouds_dropped_no_pose': self._n_dropped_no_pose,
            'points_in': self._n_points_in,
            'points_used': self._n_points_used,
            'input_hz': round(rate, 2),
            'processing_ms': stage_ms,
            'grid': stats,
            'pose_buffer_len': len(self._poses),
            'mode': self._mode,
            'frame': self._frame,
        }
        msg = String()
        msg.data = json.dumps(out)
        self._pub_stats.publish(msg)

    # ------------------------------------------------------------ services
    def _srv_reset_cb(self, _req):
        res = TriggerResponse()
        with self._lock:
            self._grid.reset()
            self._path.poses = []
        self._n_clouds = self._n_dropped_no_pose = 0
        self._n_points_in = self._n_points_used = 0
        self._hist.clear()
        res.success = True
        res.message = 'grid cleared'
        rospy.loginfo('DEM grid cleared')
        return res

    def _srv_save_cb(self, _req):
        res = TriggerResponse()
        try:
            path = self._save()
            res.success = True
            res.message = str(path)
        except Exception as exc:
            res.success = False
            res.message = str(exc)
        return res

    def _save(self):
        save_dir = rospy.get_param('~save_dir', '') or '/tmp'
        if not os.path.isdir(save_dir):
            os.makedirs(save_dir)
        name = 'dem_{}.npz'.format(time.strftime('%Y%m%d_%H%M%S'))
        with self._lock:
            return self._grid.save_npz(os.path.join(save_dir, name))

    def _on_shutdown(self):
        try:
            if rospy.get_param('~save_dir', ''):
                rospy.loginfo('final grid saved: %s', self._save())
        except Exception as exc:
            rospy.logwarn('final save failed: %s', exc)


def main():
    DemNode()
    rospy.spin()


if __name__ == '__main__':
    main()
