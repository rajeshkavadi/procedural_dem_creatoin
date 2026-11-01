"""Direct ROS 1 ``.bag`` replay source (no ROS installation required).

Why this exists
---------------
On Ubuntu 18.04 / ROS Melodic the toolchain is Python 2: ``rospy`` and
``rosbag`` ship only for Python 2.7, while the HTTP service in this package
requires Python 3 (FastAPI, dataclasses, type hints).  Running ``rosbag play``
and subscribing from Python 3 is therefore impossible on a stock Melodic
install.

This module removes the problem entirely: it reads the ROS 1 bag file
directly with the pure-Python `rosbags <https://pypi.org/project/rosbags/>`_
library, decodes ``sensor_msgs/PointCloud2`` messages with the same decoder
the live subscriber uses, and feeds the exact same ``SnapshotStore`` /
``DemAccumulator`` pipeline.  The service consequently needs **no** ROS
environment, no ``source setup.bash`` and no ``rosbag play`` process -- it
only needs the ``.bag`` file.

Behaviour
---------
* messages are replayed in recorded temporal order (the bag is read once,
  sequentially),
* wall-clock pacing is scaled by ``rate`` (0.5x, 1x, 2x, ...),
* ``loop`` repeats playback from the start (default: on, so the live stream
  never runs dry),
* ``start_offset_sec`` skips into the recording,
* when the configured topic is missing but the bag contains exactly one
  PointCloud2 topic, that topic is used automatically (``auto_topic``),
* the reader is closed on :meth:`stop` so the file handle never leaks.

The ``rosbags`` import is lazy: the rest of the package (decoder, renderer,
API, tests) keeps working without the dependency installed.
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional

from .decoder import PointCloudDecodeError, decode_pointcloud2
from .dem import DemAccumulator
from .store import SnapshotStore

log = logging.getLogger(__name__)

# Prefixes used by the common LiDAR drivers; used only to rank candidates when
# auto-detecting a topic (never to silently pick a wrong type).
_LIDAR_TOPIC_HINTS = ("livox", "lidar", "points", "cloud", "velodyne", "rslidar",
                      "ouster", "hesai")


class BagSourceError(Exception):
    """The bag could not be opened, or the requested topic is unavailable."""


@dataclass(frozen=True)
class BagInfo:
    """Lightweight description of a ROS 1 bag (for CLI/diagnostics)."""

    paths: List[Path]
    topics: Dict[str, str]          # topic -> message type
    message_count: int
    duration_sec: float
    start_time_ns: int
    end_time_ns: int

    def pointcloud_topics(self) -> List[str]:
        return [t for t, typ in self.topics.items() if typ.endswith("PointCloud2")]

    def describe(self) -> str:
        lines = [f"bag: {', '.join(str(p) for p in self.paths)}",
                 f"duration: {self.duration_sec:.1f}s, messages: {self.message_count}",
                 "topics:"]
        for topic, typ in sorted(self.topics.items()):
            lines.append(f"  {topic}  [{typ}]")
        return "\n".join(lines)


def resolve_bag_paths(path: str) -> List[Path]:
    """Expand a ``.bag`` file or a directory of split ``.bag`` files.

    Accepts:
      * a single ``.bag`` file,
      * a directory containing one or more ``*.bag`` files (ROS 1 split bags
        are numbered, so lexical order is chronological order),
      * an explicit list separated by ``os.pathsep``.
    """
    import os

    parts = [p for p in str(path).split(os.pathsep) if p] or [str(path)]
    resolved: List[Path] = []
    for raw in parts:
        p = Path(raw).expanduser()
        if p.is_dir():
            bags = sorted(p.glob("*.bag"))
            if not bags:
                raise BagSourceError(
                    f"no .bag files found in directory: {p} "
                    f"(ROS 2 .db3 recordings cannot be read here; see README)")
            resolved.extend(bags)
        elif p.is_file():
            if p.suffix.lower() == ".db3":
                raise BagSourceError(
                    f"{p} is a ROS 2 bag. This build reads ROS 1 .bag files "
                    f"directly. Convert it once with `rosbags-convert` "
                    f"(see README) or record with `rosbag record`.")
            resolved.append(p)
        else:
            raise BagSourceError(f"bag path not found: {p}")
    return resolved


def inspect_bag(path: str) -> BagInfo:
    """Read only the bag index and describe it (fast, no message decoding)."""
    paths = resolve_bag_paths(path)
    try:
        from rosbags.highlevel import AnyReader
    except Exception as exc:  # pragma: no cover - dependency guard
        raise BagSourceError(
            f"the 'rosbags' package is required to read .bag files: {exc}")

    try:
        with AnyReader(paths) as reader:
            topics = {c.topic: c.msgtype for c in reader.connections}
            count = sum(c.msgcount for c in reader.connections)
            return BagInfo(
                paths=paths,
                topics=topics,
                message_count=int(count),
                duration_sec=float(reader.duration) / 1e9,
                start_time_ns=int(reader.start_time),
                end_time_ns=int(reader.end_time),
            )
    except BagSourceError:
        raise
    except Exception as exc:
        raise BagSourceError(f"cannot open bag {paths}: {exc}")


def _pick_topic(connections, requested: str, auto: bool) -> str:
    """Return the topic to replay, or raise a helpful BagSourceError."""
    topics = [c.topic for c in connections]
    if requested in topics:
        return requested

    clouds = [c.topic for c in connections if c.msgtype.endswith("PointCloud2")]
    if auto and len(clouds) == 1:
        log.warning("topic %r not in bag; auto-selecting the only "
                    "PointCloud2 topic %r", requested, clouds[0])
        return clouds[0]

    if not clouds:
        raise BagSourceError(
            f"the bag contains no sensor_msgs/PointCloud2 topic "
            f"(available: {sorted(topics) or 'none'})")

    # Rank candidates so the error message can suggest the most likely one.
    def score(topic: str) -> tuple:
        low = topic.lower()
        return (0 if any(h in low for h in _LIDAR_TOPIC_HINTS) else 1,
                len(topic), topic)

    suggestion = sorted(clouds, key=score)[0]
    raise BagSourceError(
        f"topic {requested!r} is not in the bag. PointCloud2 topics: "
        f"{sorted(clouds)}. Use --topic {suggestion}")


class BagCloudSource:
    """Background ROS 1 bag reader feeding a SnapshotStore (+ DEM).

    Mirrors :class:`pc_snapshot.ros_subscriber.RosCloudListener` so the API
    and /api/health do not care which source is active.
    """

    kind = "bag"

    def __init__(self, bag_path: str, topic: str, store: SnapshotStore,
                 dem: Optional[DemAccumulator] = None,
                 rate: float = 1.0, loop: bool = True,
                 start_offset_sec: float = 0.0,
                 auto_topic: bool = True):
        if rate <= 0:
            raise ValueError("bag rate must be > 0")
        if start_offset_sec < 0:
            raise ValueError("bag start offset must be >= 0")
        self.bag_path = bag_path
        self.topic = topic
        self.store = store
        self.dem = dem
        self.rate = float(rate)
        self.loop = bool(loop)
        self.start_offset_sec = float(start_offset_sec)
        self.auto_topic = bool(auto_topic)

        self._paths: List[Path] = []
        self._active_topic = topic
        self._info: Optional[BagInfo] = None
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._started = False
        self._start_error: Optional[str] = None

        # diagnostics (written by the reader thread, read by /api/health)
        self._lock = threading.Lock()
        self._state = "starting"            # starting|playing|finished|stopped|error
        self._messages = 0
        self._loops = 0
        self._bag_time_s = 0.0
        self._progress = 0.0
        self._last_error: Optional[str] = None

    # ------------------------------------------------------------------ API
    def start(self) -> None:
        try:
            import rosbags  # noqa: F401
        except Exception as exc:
            self._start_error = f"rosbags unavailable: {exc}"
            self._state = "error"
            log.error(self._start_error)
            raise RuntimeError(self._start_error)

        # Validate the bag/topic up front: fail fast with a useful message
        # instead of dying silently in the worker thread.
        try:
            self._paths = resolve_bag_paths(self.bag_path)
            self._info = inspect_bag(self.bag_path)
            self._active_topic = self._probe_topic()
        except BagSourceError as exc:
            self._start_error = str(exc)
            self._state = "error"
            log.error("bag source disabled: %s", exc)
            raise RuntimeError(str(exc))

        self._thread = threading.Thread(target=self._run, name="bag-cloud-source",
                                        daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=5.0)
        with self._lock:
            if self._state not in ("error", "finished"):
                self._state = "stopped"

    @property
    def alive(self) -> bool:
        return self._started and self._thread is not None and self._thread.is_alive()

    @property
    def start_error(self) -> Optional[str]:
        return self._start_error

    @property
    def active_topic(self) -> str:
        return self._active_topic

    def status(self) -> dict:
        with self._lock:
            info = self._info
            return {
                "kind": "bag",
                "bag_path": str(self._paths[0]) if self._paths else self.bag_path,
                "topic": self._active_topic,
                "configured_topic": self.topic,
                "state": self._state,
                "rate": self.rate,
                "loop": self.loop,
                "start_offset_sec": self.start_offset_sec,
                "messages_delivered": self._messages,
                "loops_completed": self._loops,
                "bag_time_sec": round(self._bag_time_s, 3),
                "bag_duration_sec": (round(info.duration_sec, 3)
                                     if info is not None else None),
                "progress": round(self._progress, 4),
                "last_error": self._last_error,
            }

    # ------------------------------------------------------------- internals
    def _probe_topic(self) -> str:
        """Open the bag once and resolve the concrete topic to replay."""
        from rosbags.highlevel import AnyReader
        try:
            with AnyReader(self._paths) as reader:
                return _pick_topic(reader.connections, self.topic,
                                   self.auto_topic)
        except BagSourceError:
            raise
        except Exception as exc:
            raise BagSourceError(f"cannot open bag: {exc}")

    def _run(self) -> None:
        self._started = True
        try:
            while not self._stop.is_set():
                finished = self._play_once()
                if self._stop.is_set():
                    break
                if finished and not self.loop:
                    with self._lock:
                        self._state = "finished"
                        self._progress = 1.0
                    log.info("bag playback finished (%d messages delivered); "
                             "waiting for stop", self._messages)
                    while not self._stop.wait(0.25):
                        pass
                    break
                with self._lock:
                    self._loops += 1
                if finished:
                    log.info("bag loop %d complete; restarting", self._loops)
        except Exception as exc:
            self._start_error = str(exc)
            with self._lock:
                self._state = "error"
                self._last_error = str(exc)
            log.error("bag source stopped with error: %s", exc)
            self.store.record_error(f"bag source error: {exc}")
        finally:
            self._started = False
            with self._lock:
                if self._state != "error":
                    self._state = "stopped"
            log.info("bag source exited")

    def _play_once(self) -> bool:
        """Play the bag once. Returns True if it reached the end."""
        from rosbags.highlevel import AnyReader

        with AnyReader(self._paths) as reader:
            conns = [c for c in reader.connections
                     if c.topic == self._active_topic]
            if not conns:
                # Topic vanished between probe and play (unlikely); re-probe.
                self._active_topic = _pick_topic(reader.connections, self.topic,
                                                 self.auto_topic)
                conns = [c for c in reader.connections
                         if c.topic == self._active_topic]

            with self._lock:
                self._state = "playing"

            offset_ns = int(self.start_offset_sec * 1e9)
            t_first: Optional[int] = None
            base_wall = 0.0
            duration_ns = max(1, int(reader.duration))

            for conn, timestamp, raw in reader.messages(connections=conns):
                if self._stop.is_set():
                    return False

                if t_first is None:
                    t_first = timestamp
                    # Pace from the first *emitted* message so a start offset
                    # does not cause a giant initial sleep.
                    base_wall = time.monotonic()
                    if offset_ns:
                        log.info("skipping %.1fs into the bag",
                                 self.start_offset_sec)

                rel_ns = timestamp - t_first
                if rel_ns < offset_ns:
                    continue

                target = base_wall + (rel_ns - offset_ns) / 1e9 / self.rate
                if not self._wait_until(target):
                    return False

                self._deliver(reader, conn, raw, timestamp)

                with self._lock:
                    self._bag_time_s = rel_ns / 1e9
                    self._progress = min(1.0, max(0.0, rel_ns / duration_ns))
            return True

    def _wait_until(self, target: float) -> bool:
        """Sleep until the monotonic clock reaches *target*.

        Returns False when the source is stopping so the caller can exit
        promptly (bag playback at 1x can otherwise sleep for seconds).
        """
        while True:
            remaining = target - time.monotonic()
            if remaining <= 0:
                return True
            if self._stop.wait(min(0.05, remaining)):
                return False

    def _deliver(self, reader, conn, raw: bytes, timestamp: int) -> None:
        msg = reader.deserialize(raw, conn.msgtype)
        try:
            decoded = decode_pointcloud2(msg)
        except PointCloudDecodeError as exc:
            self.store.record_error(f"decode failed on {self._active_topic}: {exc}")
            log.warning("decode failed: %s", exc)
            return
        if decoded.points.shape[0] == 0:
            self.store.record_error(f"empty cloud on {self._active_topic}")
            return

        # The bag timestamp is authoritative for pacing; the header stamp is
        # what the sensor recorded and is what the UI displays.
        stamp = 0.0
        try:
            stamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        except Exception:
            stamp = timestamp / 1e9
        frame_id = ""
        try:
            frame_id = msg.header.frame_id
        except Exception:
            pass

        self.store.update(decoded, frame_id=frame_id, ros_stamp=stamp,
                          topic=self._active_topic)
        if self.dem is not None:
            self.dem.add(decoded.points, stamp)
        with self._lock:
            self._messages += 1
