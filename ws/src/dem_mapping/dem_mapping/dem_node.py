#!/usr/bin/env python3
"""Incremental CPU elevation-mapping (DEM) node.

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

import json
import math
import threading
import time
from collections import deque

import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (HistoryPolicy, QoSProfile, ReliabilityPolicy)
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import String
from std_srvs.srv import Trigger

from dem_mapping.elevation_grid import ElevationGrid
from dem_mapping.ground_filters import build_filter
from dem_mapping.pointcloud_utils import (pointcloud2_to_xyz,
                                          pointcloud2_point_times)
from dem_mapping.pose_buffer import PoseBuffer, quat_to_rot


def _stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


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


class DemNode(Node):

    def __init__(self):
        super().__init__('dem_node')

        # -------------------------------------------------------- I/O
        self.declare_parameter('cloud_topic', '/mid360/livox/lidar')
        self.declare_parameter('mode', 'registered')   # registered | pose
        self.declare_parameter('pose_topic', '/Odometry')
        self.declare_parameter('pose_type', 'nav_msgs/msg/Odometry')
        self.declare_parameter('output_frame', 'map')
        self.declare_parameter('motion_comp', True)
        self.declare_parameter('pose_timeout', 0.5)
        # extrinsic body<-lidar as row-major 4x4 (default: identity)
        self.declare_parameter('T_body_lidar', [1.0, 0.0, 0.0, 0.0,
                                                0.0, 1.0, 0.0, 0.0,
                                                0.0, 0.0, 1.0, 0.0,
                                                0.0, 0.0, 0.0, 1.0])

        # -------------------------------------------------------- grid
        self.declare_parameter('resolution', 0.2)
        self.declare_parameter('size_x', 300.0)
        self.declare_parameter('size_y', 300.0)
        self.declare_parameter('origin_x', -150.0)
        self.declare_parameter('origin_y', -150.0)
        self.declare_parameter('aggregation', 'min')    # min | mean

        # ------------------------------------------------------ filters
        self.declare_parameter('ground_filter', 'height_band')
        self.declare_parameter('z_min', -100.0)
        self.declare_parameter('z_max', 100.0)
        self.declare_parameter('band_below', 6.0)
        self.declare_parameter('band_above', 4.0)
        self.declare_parameter('min_range', 1.0)
        self.declare_parameter('max_range', 60.0)
        self.declare_parameter('max_points_per_scan', 12000)

        # --------------------------------------------------- publishing
        self.declare_parameter('image_rate', 1.0)
        self.declare_parameter('cloud_rate', 1.0)
        self.declare_parameter('cloud_max_cells', 150000)
        self.declare_parameter('occupancy_rate', 1.0)
        self.declare_parameter('occ_z_min', -6.0)
        self.declare_parameter('occ_z_max', 4.0)
        self.declare_parameter('trajectory_rate', 2.0)
        self.declare_parameter('stats_rate', 2.0)
        self.declare_parameter('save_dir', '')

        # --------------------------------------------------------- setup
        self._frame = self.get_parameter('output_frame').value
        self._mode = self.get_parameter('mode').value
        if self._mode not in ('registered', 'pose'):
            raise ValueError("mode must be 'registered' or 'pose'")

        self._grid = ElevationGrid(
            resolution=float(self.get_parameter('resolution').value),
            size_x=float(self.get_parameter('size_x').value),
            size_y=float(self.get_parameter('size_y').value),
            origin_x=float(self.get_parameter('origin_x').value),
            origin_y=float(self.get_parameter('origin_y').value),
            aggregation=self.get_parameter('aggregation').value)

        self._radial = build_filter(
            'radial_range',
            min_range=float(self.get_parameter('min_range').value),
            max_range=float(self.get_parameter('max_range').value))
        fname = self.get_parameter('ground_filter').value
        self._ground = build_filter(
            fname,
            z_min=float(self.get_parameter('z_min').value),
            z_max=float(self.get_parameter('z_max').value),
            below=float(self.get_parameter('band_below').value),
            above=float(self.get_parameter('band_above').value))
        self._subsample = build_filter(
            'max_points',
            max_points=int(self.get_parameter('max_points_per_scan').value))
        self.get_logger().info(
            f'ground filter: {fname} ({type(self._ground).__name__})')

        T = np.asarray(self.get_parameter('T_body_lidar').value,
                       dtype=np.float64).reshape(4, 4)
        self._R_bl = T[:3, :3]
        self._t_bl = T[:3, 3]

        self._motion_comp = bool(self.get_parameter('motion_comp').value)
        self._pose_timeout = float(self.get_parameter('pose_timeout').value)
        self._poses = PoseBuffer(max_span_s=20.0)

        # -------------------------------------------------- statistics
        self._n_clouds = 0
        self._n_dropped_no_pose = 0
        self._n_points_in = 0
        self._n_points_used = 0
        self._last_ms = {}
        self._hist = deque(maxlen=200)          # (stage, ms) samples
        self._hz = deque(maxlen=50)             # arrival wall times
        self._grid_publish_t = 0.0
        self._path = Path()
        self._path.header.frame_id = self._frame

        # Sensor callbacks and publishing timers run in separate mutually
        # exclusive groups on a multi-threaded executor: the large DEM
        # messages (9 MB image, 2 MB occupancy grid, ...) must never delay
        # incoming scans, otherwise clouds are dropped by QoS.
        # Heavy publishers (image/cloud/occupancy) are split from the light
        # ones (trajectory/stats) so telemetry is never starved.
        self._lock = threading.Lock()
        self._cb_subs = MutuallyExclusiveCallbackGroup()
        self._cb_pubs = MutuallyExclusiveCallbackGroup()
        self._cb_light = MutuallyExclusiveCallbackGroup()

        sensor_qos = QoSProfile(depth=10,
                                reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST)
        reliable_qos = QoSProfile(depth=10,
                                  reliability=ReliabilityPolicy.RELIABLE,
                                  history=HistoryPolicy.KEEP_LAST)

        cloud_topic = self.get_parameter('cloud_topic').value
        self.create_subscription(PointCloud2, cloud_topic, self._cloud_cb,
                                 sensor_qos, callback_group=self._cb_subs)

        pose_topic = self.get_parameter('pose_topic').value
        if pose_topic:
            from rosidl_runtime_py.utilities import get_message
            pose_cls = get_message(self.get_parameter('pose_type').value)
            self.create_subscription(pose_cls, pose_topic, self._pose_cb,
                                     reliable_qos,
                                     callback_group=self._cb_subs)
            self.get_logger().info(f'pose source: {pose_topic} '
                                   f'({pose_cls.__name__})')

        self._pub_img = self.create_publisher(Image, '~/elevation_image', 5)
        self._pub_cloud = self.create_publisher(PointCloud2,
                                                '~/elevation_cloud', 5)
        self._pub_occ = self.create_publisher(OccupancyGrid, '~/occupancy', 5)
        self._pub_traj = self.create_publisher(Path, '~/trajectory', 5)
        self._pub_stats = self.create_publisher(String, '~/stats', 10)

        self.create_service(Trigger, '~/reset', self._srv_reset)
        self.create_service(Trigger, '~/save', self._srv_save)

        self._rate_event('image', 'image_rate', self._publish_image,
                         heavy=True)
        self._rate_event('cloud', 'cloud_rate', self._publish_cloud,
                         heavy=True)
        self._rate_event('occ', 'occupancy_rate', self._publish_occupancy,
                         heavy=True)
        self._rate_event('traj', 'trajectory_rate', self._publish_traj)
        self._rate_event('stats', 'stats_rate', self._publish_stats)

        self.get_logger().info(
            f'dem_node: cloud={cloud_topic} mode={self._mode} '
            f'res={self._grid.resolution}m '
            f'grid={self._grid.nx}x{self._grid.ny} '
            f'agg={self._grid.aggregation}')

    def _rate_event(self, _name, param, cb, heavy=False):
        rate = float(self.get_parameter(param).value)
        if rate > 0:
            group = self._cb_pubs if heavy else self._cb_light
            self.create_timer(1.0 / rate, cb, callback_group=group)

    # ------------------------------------------------------------- pose in
    def _pose_cb(self, msg):
        typ = type(msg).__name__
        if typ == 'Odometry':
            t = _stamp_to_sec(msg.header.stamp)
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
        elif typ == 'PoseWithCovarianceStamped':
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
            ps.pose.position = p if hasattr(p, 'x') else msg.pose.position
            ps.pose.orientation = q
            with self._lock:
                self._path.header.stamp = msg.header.stamp
                self._path.poses.append(ps)
                if len(self._path.poses) > 20000:
                    self._path.poses = self._path.poses[-20000:]

    # ---------------------------------------------------------- cloud in
    def _cloud_cb(self, msg: PointCloud2):
        wall_t0 = time.perf_counter()
        scan_t = _stamp_to_sec(msg.header.stamp)

        try:
            xyz = pointcloud2_to_xyz(msg)
        except Exception as exc:
            self.get_logger().warn(f'bad point cloud: {exc}')
            return
        n_in = xyz.shape[0]
        if n_in == 0:
            return
        good = np.isfinite(xyz).all(axis=1)
        xyz = xyz[good]
        t_parse = (time.perf_counter() - wall_t0) * 1e3

        # ---- transform into the map frame ----------------------------
        t_xf0 = time.perf_counter()
        pose = self._poses.pose_at(scan_t, max_age=self._pose_timeout)
        if pose is None and self._mode == 'pose':
            self._n_dropped_no_pose += 1
            return
        pose_z = pose_xyz = None
        if pose is not None:
            R_wb, t_wb = pose
            pose_xyz = t_wb
            pose_z = float(t_wb[2])
        else:
            R_wb, t_wb = None, None

        if self._mode == 'pose':
            pts_body = xyz @ self._R_bl.T + self._t_bl
            rel = pointcloud2_point_times(msg) if self._motion_comp else None
            if rel is not None and rel.shape[0] == pts_body.shape[0]:
                t_pts = scan_t + rel
                xyz, _exact, _fb = self._poses.transform_points(
                    t_pts, pts_body, max_age=self._pose_timeout)
            else:
                xyz = pts_body @ R_wb.T + t_wb
        t_xf = (time.perf_counter() - t_xf0) * 1e3

        # ---- ground / quality filtering -------------------------------
        t_f0 = time.perf_counter()
        ctx = {'pose_z': pose_z, 'pose_xyz': pose_xyz, 'stamp': scan_t}
        xyz = self._radial(xyz, ctx)
        xyz = self._ground(xyz, ctx)
        xyz = self._subsample(xyz, ctx)
        t_filt = (time.perf_counter() - t_f0) * 1e3

        # ---- incremental grid update ----------------------------------
        t_u0 = time.perf_counter()
        with self._lock:
            res = self._grid.update(xyz[:, :2], xyz[:, 2])
        t_upd = (time.perf_counter() - t_u0) * 1e3

        self._n_clouds += 1
        self._n_points_in += n_in
        self._n_points_used += res['accepted']
        self._hist.append({'parse': t_parse, 'transform': t_xf,
                           'filter': t_filt, 'update': t_upd,
                           'total': (time.perf_counter() - wall_t0) * 1e3})
        self._hz.append(time.monotonic())

    # --------------------------------------------------------- publishing
    def _publish_image(self):
        with self._lock:
            if not self._grid.total_updates:
                return
            elev, _valid, _stats = self._grid.snapshot()
            ny, nx = self._grid.ny, self._grid.nx
        # the snapshot is immutable until the next grid update, so the
        # 9 MB serialisation happens outside the lock
        img = Image()
        img.header.stamp = self.get_clock().now().to_msg()
        img.header.frame_id = self._frame
        img.height, img.width = ny, nx
        img.encoding = '32FC1'
        img.is_bigendian = 0
        img.step = nx * 4
        img.data = np.ascontiguousarray(elev, dtype=np.float32).tobytes()
        self._pub_img.publish(img)

    def _publish_cloud(self):
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

        max_cells = int(self.get_parameter('cloud_max_cells').value)
        if xy.shape[0] > max_cells:
            stride = int(np.ceil(xy.shape[0] / max_cells))
            xy = xy[::stride]
            z = z[::stride]

        rgb = _elevation_colors(z, stats['z_min'], stats['z_max'])
        n = xy.shape[0]
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame
        msg.height = 1
        msg.width = n
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
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
                 (rgb[:, 1].astype(np.uint32) << 8) | rgb[:, 2].astype(np.uint32)
        buf[:, 12:16] = packed.astype(np.uint32).view(np.uint8).reshape(n, 4)
        msg.data = buf.tobytes()
        self._pub_cloud.publish(msg)

    def _publish_occupancy(self):
        with self._lock:
            if not self._grid.total_updates:
                return
            elev, _valid, _stats = self._grid.snapshot()
            res = self._grid.resolution
            nx, ny = self._grid.nx, self._grid.ny
            ox, oy = self._grid.origin_x, self._grid.origin_y
        z_min = float(self.get_parameter('occ_z_min').value)
        z_max = float(self.get_parameter('occ_z_max').value)
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
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame
        msg.info.resolution = res
        msg.info.width = nx
        msg.info.height = ny
        msg.info.origin.position.x = ox
        msg.info.origin.position.y = oy
        msg.info.origin.orientation.w = 1.0
        # OccupancyGrid data is int8[] row-major starting at the origin
        # corner; array('b', ...) gives signed ints rclpy accepts.
        import array as _array
        msg.data = _array.array('b', occ.tobytes())
        self._pub_occ.publish(msg)

    def _publish_traj(self):
        with self._lock:
            if not self._path.poses:
                return
            path = Path()
            path.header = self._path.header
            path.poses = list(self._path.poses)
        self._pub_traj.publish(path)

    def _publish_stats(self):
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
    def _srv_reset(self, _req, res):
        with self._lock:
            self._grid.reset()
            self._path.poses = []
        self._n_clouds = self._n_dropped_no_pose = 0
        self._n_points_in = self._n_points_used = 0
        self._hist.clear()
        res.success = True
        res.message = 'grid cleared'
        self.get_logger().info('DEM grid cleared')
        return res

    def _srv_save(self, _req, res):
        try:
            path = self._save()
            res.success = True
            res.message = str(path)
        except Exception as exc:
            res.success = False
            res.message = str(exc)
        return res

    def _save(self):
        import os
        save_dir = self.get_parameter('save_dir').value
        save_dir = save_dir or '/tmp'
        os.makedirs(save_dir, exist_ok=True)
        name = f'dem_{time.strftime("%Y%m%d_%H%M%S")}.npz'
        with self._lock:
            return self._grid.save_npz(os.path.join(save_dir, name))


def main(args=None):
    rclpy.init(args=args)
    node = DemNode()
    # sensor callbacks and heavy publish timers run concurrently
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if node.get_parameter('save_dir').value:
                node.get_logger().info(f'final grid saved: {node._save()}')
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
