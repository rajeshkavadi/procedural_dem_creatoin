"""Live ROS 1 subscriber for the point-cloud topic (optional path).

Prefer :class:`pc_snapshot.bag_source.BagCloudSource` when replaying a
recording: it reads the ``.bag`` directly and needs no ROS environment.

This module is the *live sensor* path. It subscribes with ROS 1 ``rospy`` and
feeds the same ``SnapshotStore`` / ``DemAccumulator`` pipeline. It imports
rospy lazily so the rest of the package (decoder, renderer, API tests) works
without ROS installed.

.. note::
   ROS Melodic ships ``rospy`` for **Python 2.7 only**. The HTTP service
   requires Python 3 (FastAPI), so on a stock Melodic install this listener
   cannot be used from the service process: /api/health reports the import
   error and ``--bag`` should be used instead. On a Python 3 ROS 1 install
   (source build / ROS Noetic) the same code works unchanged.
"""

from __future__ import annotations

import logging
import os
import threading
import time
from typing import Optional

from .decoder import PointCloudDecodeError, decode_pointcloud2
from .dem import DemAccumulator
from .store import SnapshotStore

log = logging.getLogger(__name__)


class Ros1CloudListener:
    """Background ROS 1 ``rospy`` subscriber feeding a SnapshotStore."""

    kind = "ros"

    def __init__(self, topic: str, store: SnapshotStore,
                 master_uri: Optional[str] = None,
                 dem: Optional[DemAccumulator] = None):
        self.topic = topic
        self.store = store
        self.master_uri = master_uri
        self.dem = dem
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._started = False
        self._start_error: Optional[str] = None
        self._messages = 0

    # ------------------------------------------------------------------
    def start(self) -> None:
        if self.master_uri:
            os.environ["ROS_MASTER_URI"] = str(self.master_uri)
        try:
            import rospy  # noqa: F401
        except Exception as exc:
            self._start_error = (
                f"rospy unavailable ({exc}). Melodic's rospy is Python 2 "
                f"only; use --bag to replay a recording without ROS.")
            log.error(self._start_error)
            raise RuntimeError(self._start_error)
        self._thread = threading.Thread(target=self._run, name="ros-cloud-listener",
                                        daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=5.0)

    @property
    def alive(self) -> bool:
        return self._started and self._thread is not None and self._thread.is_alive()

    @property
    def start_error(self) -> Optional[str]:
        return self._start_error

    def status(self) -> dict:
        return {
            "kind": "ros",
            "master_uri": self.master_uri or os.environ.get("ROS_MASTER_URI", ""),
            "messages_delivered": self._messages,
            "last_error": self._start_error,
        }

    # ------------------------------------------------------------------
    def _run(self) -> None:
        import rospy
        from sensor_msgs.msg import PointCloud2

        node_name = "pc_snapshot_subscriber"
        try:
            rospy.init_node(node_name, anonymous=True, disable_signals=True)
        except Exception as exc:
            # A node may already be initialised in-process; that is fine.
            self._start_error = f"rospy.init_node failed: {exc}"
            log.error(self._start_error)
            self.store.record_error(self._start_error)
            return

        def _on_msg(msg: "PointCloud2") -> None:
            try:
                decoded = decode_pointcloud2(msg)
            except PointCloudDecodeError as exc:
                self.store.record_error(f"decode failed on {self.topic}: {exc}")
                log.warning("decode failed: %s", exc)
                return
            if decoded.points.shape[0] == 0:
                self.store.record_error(f"empty cloud on {self.topic}")
                return
            stamp = 0.0
            try:
                stamp = float(msg.header.stamp.to_sec())
            except Exception:
                pass
            frame_id = ""
            try:
                frame_id = msg.header.frame_id
            except Exception:
                pass
            self.store.update(decoded, frame_id=frame_id, ros_stamp=stamp,
                              topic=self.topic)
            # Accumulate into the incremental DEM *after* the store update: a
            # snapshot must never wait on the grid, and the DEM keeps every
            # message since the service started ("Get DEM").
            if self.dem is not None:
                self.dem.add(decoded.points, stamp)
            self._messages += 1

        try:
            sub = rospy.Subscriber(self.topic, PointCloud2, _on_msg,
                                   queue_size=10)
            log.info("subscribed to %s via rospy", self.topic)
            self._started = True
            while not self._stop.is_set() and not rospy.is_shutdown():
                time.sleep(0.1)
            sub.unregister()
        except Exception as exc:
            self._start_error = str(exc)
            log.error("ROS listener stopped with error: %s", exc)
            self.store.record_error(f"ROS listener error: {exc}")
        finally:
            self._started = False
            log.info("ROS listener exited")


# Backwards-compatible alias (older docs/scripts referenced this name).
RosCloudListener = Ros1CloudListener
