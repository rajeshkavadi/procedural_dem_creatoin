"""Tests for the direct ROS 1 bag source (no ROS installation required)."""

import time
from pathlib import Path

import numpy as np
import pytest

from pc_snapshot.bag_source import (BagCloudSource, BagSourceError,
                                    inspect_bag, resolve_bag_paths)
from pc_snapshot.config import Config
from pc_snapshot.dem import DemAccumulator
from pc_snapshot.store import SnapshotStore
from pc_snapshot.synthetic import write_test_bag

pytest.importorskip("rosbags")


@pytest.fixture()
def bag(tmp_path):
    return write_test_bag(tmp_path / "scene.bag", frames=12, rate=20.0)


def wait_for(predicate, timeout=10.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    return False


def make_source(bag_path, topic="/mid360/livox/lidar", **kwargs):
    store = SnapshotStore(topic=topic, stale_after_sec=5.0)
    cfg = Config()
    cfg.dem_enabled = False
    dem = DemAccumulator(cfg)
    source = BagCloudSource(str(bag_path), topic, store, dem=dem, **kwargs)
    return source, store, dem


def test_resolve_bag_file(bag):
    assert resolve_bag_paths(str(bag)) == [bag]


def test_resolve_bag_directory(bag):
    assert resolve_bag_paths(str(bag.parent)) == [bag]


def test_resolve_missing_path(tmp_path):
    with pytest.raises(BagSourceError, match="not found"):
        resolve_bag_paths(str(tmp_path / "nope.bag"))


def test_resolve_rejects_ros2_db3(tmp_path):
    db3 = tmp_path / "recording.db3"
    db3.write_bytes(b"")
    with pytest.raises(BagSourceError, match="ROS 2 bag"):
        resolve_bag_paths(str(db3))


def test_inspect_bag(bag):
    info = inspect_bag(str(bag))
    assert info.message_count == 12
    assert info.topics == {"/mid360/livox/lidar":
                           "sensor_msgs/msg/PointCloud2"}
    assert info.pointcloud_topics() == ["/mid360/livox/lidar"]
    assert 0.4 < info.duration_sec < 0.7
    assert "PointCloud2" in info.describe()


def test_source_delivers_clouds(bag):
    source, store, dem = make_source(bag, rate=50.0, loop=False)
    try:
        source.start()
        assert wait_for(lambda: store.health()["messages_received"] >= 3)
        assert source.alive
        latest = store.capture()
        assert latest is not None
        assert latest.point_count > 500
        assert latest.frame_id == "camera_init"
        # Header stamps from the bag are preserved as ROS seconds.
        assert latest.ros_stamp > 1_700_000_000
        assert source.status()["topic"] == "/mid360/livox/lidar"
    finally:
        source.stop()


def test_source_runs_dem_accumulation(bag):
    cfg = Config()
    store = SnapshotStore(topic="/mid360/livox/lidar")
    dem = DemAccumulator(cfg)
    source = BagCloudSource(str(bag), "/mid360/livox/lidar", store, dem=dem,
                            rate=50.0, loop=False)
    try:
        source.start()
        assert wait_for(lambda: dem.status().get("points_used", 0) > 0)
        capture = dem.capture()
        assert capture is not None
        assert capture.valid_cells > 0
    finally:
        source.stop()


def test_source_finishes_when_not_looping(bag):
    source, store, _ = make_source(bag, rate=100.0, loop=False)
    try:
        source.start()
        assert wait_for(lambda: source.status()["state"] == "finished",
                        timeout=15.0)
        status = source.status()
        assert status["progress"] == 1.0
        assert status["loops_completed"] == 0
        # The last cloud stays available for snapshots after the bag ends.
        assert store.capture() is not None
    finally:
        source.stop()


def test_source_missing_topic_suggests_candidates(bag):
    store = SnapshotStore(topic="/wrong")
    source = BagCloudSource(str(bag), "/wrong", store, auto_topic=False)
    with pytest.raises(RuntimeError, match="--topic /mid360/livox/lidar"):
        source.start()
    assert "not in the bag" in source.start_error


def test_source_auto_selects_only_pointcloud_topic(bag):
    source, store, _ = make_source(bag, topic="/wrong", rate=100.0, loop=False)
    try:
        source.start()
        assert source.active_topic == "/mid360/livox/lidar"
        assert wait_for(lambda: store.health()["messages_received"] >= 1)
    finally:
        source.stop()


def test_source_rejects_bad_arguments(bag):
    store = SnapshotStore()
    with pytest.raises(ValueError):
        BagCloudSource(str(bag), "/t", store, rate=0)
    with pytest.raises(ValueError):
        BagCloudSource(str(bag), "/t", store, start_offset_sec=-1)


def test_source_start_offset_skips_into_bag(bag):
    source, store, _ = make_source(bag, rate=100.0, loop=False,
                                   start_offset_sec=0.3)
    try:
        source.start()
        assert wait_for(lambda: store.health()["messages_received"] >= 1)
        # First delivered message must be past the 0.3s offset.
        assert store.health()["latest_ros_stamp"] >= 1_700_000_000.3
    finally:
        source.stop()


def test_source_bad_topic_type_is_reported(tmp_path):
    """A bag with no PointCloud2 topic must fail loudly, not silently."""
    from rosbags.rosbag1 import Writer
    from rosbags.typesys import Stores, get_typestore

    ts = get_typestore(Stores.ROS1_NOETIC)
    path = tmp_path / "odom.bag"
    msgtype = "nav_msgs/msg/Odometry"
    Odometry = ts.types[msgtype]
    Header = ts.types["std_msgs/msg/Header"]
    Time = ts.types["builtin_interfaces/msg/Time"]
    with Writer(path) as writer:
        conn = writer.add_connection("/odom", msgtype, typestore=ts)
        msg = Odometry(header=Header(seq=0, stamp=Time(sec=1, nanosec=0),
                                     frame_id="map"),
                       child_frame_id="base",
                       pose=ts.types["geometry_msgs/msg/PoseWithCovariance"](
                           pose=ts.types["geometry_msgs/msg/Pose"](
                               position=ts.types["geometry_msgs/msg/Point"](
                                   x=0.0, y=0.0, z=0.0),
                               orientation=ts.types[
                                   "geometry_msgs/msg/Quaternion"](
                                   x=0.0, y=0.0, z=0.0, w=1.0)),
                           covariance=np.zeros(36)),
                       twist=ts.types["geometry_msgs/msg/TwistWithCovariance"](
                           twist=ts.types["geometry_msgs/msg/Twist"](
                               linear=ts.types["geometry_msgs/msg/Vector3"](
                                   x=0.0, y=0.0, z=0.0),
                               angular=ts.types["geometry_msgs/msg/Vector3"](
                                   x=0.0, y=0.0, z=0.0)),
                           covariance=np.zeros(36)))
        writer.write(conn, 1, ts.serialize_ros1(msg, msgtype))

    store = SnapshotStore(topic="/mid360/livox/lidar")
    source = BagCloudSource(str(path), "/mid360/livox/lidar", store)
    with pytest.raises(RuntimeError, match="no sensor_msgs/PointCloud2"):
        source.start()
