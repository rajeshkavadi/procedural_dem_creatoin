#!/usr/bin/env python
"""Live DEM heatmap node (ROS 1 / Melodic).

Subscribes to the DEM node outputs (and the replay status), renders an
interactive matplotlib window while playback runs and - when no display is
available - falls back to saving PNG snapshots.

Parameters
----------
~update_rate       : redraw frequency (Hz)
~elevation_topic, ~stats_topic, ~trajectory_topic, ~replay_topic
~save_dir          : if non-empty, PNG snapshots are written there
~title             : window title
"""

from __future__ import division

import json
import os
import time

import numpy as np
import rospy
from nav_msgs.msg import Path
from sensor_msgs.msg import Image
from std_msgs.msg import String

from dem_viz.heatmap import DemHeatmap, pick_backend


class HeatmapNode(object):

    def __init__(self, backend):
        rospy.init_node('dem_heatmap')
        self._backend = backend
        self._save_dir = rospy.get_param('~save_dir', '/tmp/dem_snapshots')
        self._heat = DemHeatmap(
            self, title=rospy.get_param(
                '~title', 'Incremental DEM - simulated flight'),
            save_dir=self._save_dir)

        self._origin_cache = (0.0, 0.0)
        self._res_cache = 0.2

        self._sub_img = rospy.Subscriber(
            rospy.get_param('~elevation_topic', '/dem_node/elevation_image'),
            Image, self._on_image, queue_size=5)
        self._sub_stats = rospy.Subscriber(
            rospy.get_param('~stats_topic', '/dem_node/stats'),
            String, self._on_stats, queue_size=10)
        self._sub_replay = rospy.Subscriber(
            rospy.get_param('~replay_topic', '/replay/status'),
            String, self._on_replay, queue_size=10)
        self._sub_traj = rospy.Subscriber(
            rospy.get_param('~trajectory_topic', '/dem_node/trajectory'),
            Path, self._on_traj, queue_size=5)

        rospy.loginfo('heatmap backend: %s', backend)

    # ----------------------------------------------------------- callbacks
    def _on_image(self, msg):
        if msg.encoding not in ('32FC1', '32FC'):
            return
        arr = np.frombuffer(bytes(msg.data), dtype=np.float32)
        if arr.size != msg.height * msg.width:
            return
        elev = arr.reshape(msg.height, msg.width)
        # origin/resolution travel with the stats topic (see _on_stats)
        self._heat.set_elevation(elev, self._origin_cache, self._res_cache)

    def _on_stats(self, msg):
        try:
            d = json.loads(msg.data)
        except ValueError:
            return
        self._heat.set_stats(d)
        grid = d.get('grid', {})
        if 'origin' in grid and 'resolution_m' in grid:
            self._origin_cache = (grid['origin'][0], grid['origin'][1])
            self._res_cache = float(grid['resolution_m'])
            self._heat._origin = self._origin_cache
            self._heat._res = self._res_cache

    def _on_replay(self, msg):
        try:
            d = json.loads(msg.data)
        except ValueError:
            return
        self._heat.set_replay(d)

    def _on_traj(self, msg):
        if not msg.poses:
            return
        xy = np.array([[p.pose.position.x, p.pose.position.y]
                       for p in msg.poses[-10000:]], dtype=np.float64)
        cur = (xy[-1, 0], xy[-1, 1])
        self._heat.set_trajectory(xy, cur)


def _ensure_dir(path):
    if path and not os.path.isdir(path):
        os.makedirs(path)


def _save_headless(node):
    save_dir = node._save_dir
    if not save_dir:
        return
    _ensure_dir(save_dir)
    path = os.path.join(save_dir, 'dem_headless_latest.png')
    node._heat.fig.savefig(path, dpi=110)


def main():
    backend = pick_backend()
    # construct the ROS node only after the backend is fixed
    node = HeatmapNode(backend)

    import matplotlib.pyplot as plt
    interactive = backend != 'Agg'
    if interactive:
        plt.ion()
        plt.show(block=False)

    rate = float(rospy.get_param('~update_rate', 5.0))
    interval = 1.0 / max(rate, 0.1)
    last_png = 0.0
    try:
        while not rospy.is_shutdown():
            node._heat.draw(min_interval=interval)
            if interactive:
                plt.pause(0.05)
            else:
                # headless: refresh a PNG snapshot at <= 1 Hz
                now = time.time()
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


if __name__ == '__main__':
    main()
