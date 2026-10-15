#!/usr/bin/env python3
"""Live DEM heatmap node.

Subscribes to the DEM node outputs (and the replay status), renders an
interactive matplotlib window while playback runs and - when no display is
available - falls back to saving PNG snapshots.

Parameters
----------
update_rate   : redraw frequency (Hz)
elevation_topic, stats_topic, trajectory_topic, replay_topic
save_dir      : if non-empty, PNG snapshots are written there
"""

import json
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from sensor_msgs.msg import Image
from std_msgs.msg import String

from dem_viz.heatmap import DemHeatmap, pick_backend


class HeatmapNode(Node):

    def __init__(self, backend):
        super().__init__('dem_heatmap')
        self.declare_parameter('update_rate', 5.0)
        self.declare_parameter('elevation_topic', '/dem_node/elevation_image')
        self.declare_parameter('stats_topic', '/dem_node/stats')
        self.declare_parameter('trajectory_topic', '/dem_node/trajectory')
        self.declare_parameter('replay_topic', '/replay/status')
        self.declare_parameter('save_dir', '/tmp/dem_snapshots')
        self.declare_parameter('title', 'Incremental DEM - simulated flight')

        self._backend = backend
        self._heat = DemHeatmap(
            self, title=self.get_parameter('title').value)

        self.create_subscription(
            Image, self.get_parameter('elevation_topic').value,
            self._on_image, 5)
        self.create_subscription(
            String, self.get_parameter('stats_topic').value,
            self._on_stats, 10)
        self.create_subscription(
            String, self.get_parameter('replay_topic').value,
            self._on_replay, 10)
        self.create_subscription(
            Path, self.get_parameter('trajectory_topic').value,
            self._on_traj, 5)

        self.get_logger().info(f'heatmap backend: {backend}')

    # ----------------------------------------------------------- callbacks
    def _on_image(self, msg: Image):
        if msg.encoding not in ('32FC1', '32FC'):
            return
        arr = np.frombuffer(bytes(msg.data), dtype=np.float32)
        if arr.size != msg.height * msg.width:
            return
        elev = arr.reshape(msg.height, msg.width)
        # origin/resolution travel with the stats topic (see _on_stats)
        origin = getattr(self, '_origin_cache', (0.0, 0.0))
        res = getattr(self, '_res_cache', 0.2)
        self._heat.set_elevation(elev, origin, res)

    def _on_stats(self, msg: String):
        try:
            d = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self._heat.set_stats(d)
        grid = d.get('grid', {})
        if 'origin' in grid and 'resolution_m' in grid:
            self._origin_cache = (grid['origin'][0], grid['origin'][1])
            self._res_cache = float(grid['resolution_m'])
            self._heat._origin = self._origin_cache
            self._heat._res = self._res_cache

    def _on_replay(self, msg: String):
        try:
            d = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self._heat.set_replay(d)

    def _on_traj(self, msg: Path):
        if not msg.poses:
            return
        xy = np.array([[p.pose.position.x, p.pose.position.y]
                       for p in msg.poses[-10000:]], dtype=np.float64)
        cur = (xy[-1, 0], xy[-1, 1])
        self._heat.set_trajectory(xy, cur)


def main(args=None):
    rclpy.init(args=args)
    backend = pick_backend()
    # construct the ROS node only after the backend is fixed
    node = HeatmapNode(backend)

    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()

    import matplotlib.pyplot as plt
    interactive = backend != 'Agg'
    if interactive:
        plt.ion()
        plt.show(block=False)

    rate = float(node.get_parameter('update_rate').value)
    interval = 1.0 / max(rate, 0.1)
    last_png = 0.0
    try:
        while rclpy.ok():
            node._heat.draw(min_interval=interval)
            if interactive:
                plt.pause(0.05)
            else:
                # headless: refresh a PNG snapshot at <= 1 Hz
                now = time.monotonic()
                if now - last_png > 1.0:
                    last_png = now
                    _save_headless(node)
                time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            _save_headless(node)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _save_headless(node):
    save_dir = node.get_parameter('save_dir').value
    if not save_dir:
        return
    import os
    os.makedirs(save_dir, exist_ok=True)
    path = os.path.join(save_dir, 'dem_headless_latest.png')
    node._heat.fig.savefig(path, dpi=110)


def main_headless(args=None):       # pragma: no cover
    main(args)


if __name__ == '__main__':
    main()
