#!/usr/bin/env python3
"""PointPillars inference on a ROS bag or a live PointCloud2 topic.

Examples
--------
Play a bag through the node (publishes boxes to RViz):

    python3 tools/bag_inference.py --bag /path/to/recording.bag

Live mode (subscribe while `rosbag play` runs elsewhere):

    python3 tools/bag_inference.py --topic /livox/lidar

Headless, no roscore needed:

    python3 tools/bag_inference.py --bag rec.bag --no-publish --save-json out.json

The first frame can be slow on a Jetson (CUDA context + cuDNN autotune); the
printed ``infer`` time is the steady-state per-frame latency.
"""

import argparse
import json
import sys
import time
from pathlib import Path

PKG_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG_ROOT))

from pointpillars.config import ConfigError, load_config  # noqa: E402
from pointpillars.pipeline import PointPillarDetector  # noqa: E402
from pointpillars.ros_utils import (  # noqa: E402
    build_marker_array, make_pointcloud2, pointcloud2_to_xyz_intensity,
)


def parse_args():
    p = argparse.ArgumentParser(description="PointPillars inference on Livox ROS data")
    p.add_argument("--bag", default=None, help="read frames from this .bag instead of a live topic")
    p.add_argument("--topic", default=None, help="PointCloud2 topic (default: config or /livox/lidar)")
    p.add_argument("--config", default=str(PKG_ROOT / "config" / "pointpillar_car.yaml"))
    p.add_argument("--checkpoint", default=None, help="override checkpoint path")
    p.add_argument("--device", default=None, help="auto | cuda | cpu")
    p.add_argument("--rate", type=float, default=1.0,
                   help="bag playback speed (1.0 = real time, 0 = as fast as possible)")
    p.add_argument("--max-frames", type=int, default=0, help="stop after N frames (0 = all)")
    p.add_argument("--score-thresh", type=float, default=None)
    p.add_argument("--nms-thresh", type=float, default=None)
    p.add_argument("--no-publish", action="store_true", help="do not start a ROS node")
    p.add_argument("--publish-points", dest="publish_points", action="store_true", default=True,
                   help="republish the received cloud on /pointpillars/points (default)")
    p.add_argument("--no-publish-points", dest="publish_points", action="store_false")
    p.add_argument("--boxes-topic", default="/pointpillars/boxes")
    p.add_argument("--points-topic", default="/pointpillars/points")
    p.add_argument("--save-json", default=None, help="write per-frame detections to this JSON file")
    p.add_argument("--print-every", type=int, default=1, help="log every N frames")
    return p.parse_args()


def ros_master_alive():
    try:
        import rosgraph
        rosgraph.Master("/pointpillars_check").getPid()
        return True
    except Exception:
        return False


class Publishers(object):
    def __init__(self, boxes_topic, points_topic, node_name):
        import rospy
        from sensor_msgs.msg import PointCloud2
        from visualization_msgs.msg import MarkerArray

        rospy.init_node(node_name, anonymous=True)
        self.rospy = rospy
        self.boxes_pub = rospy.Publisher(boxes_topic, MarkerArray, queue_size=2)
        self.points_pub = rospy.Publisher(points_topic, PointCloud2, queue_size=2)

    def publish(self, det, points, class_names, header, publish_points):
        boxes_pub = self.boxes_pub
        arr = build_marker_array(det["boxes"], det["scores"], det["labels"],
                                 class_names, header)
        boxes_pub.publish(arr)
        if publish_points:
            self.points_pub.publish(make_pointcloud2(points, header))

    def spin_once(self):
        # let rospy flush its publisher queues
        time.sleep(0.001)


def _json_record(frame_idx, stamp, det, class_names):
    recs = []
    for box, score, label in zip(det["boxes"], det["scores"], det["labels"]):
        recs.append({
            "box": [round(float(v), 4) for v in box],
            "score": round(float(score), 4),
            "label": int(label),
            "class": class_names[int(label) - 1] if 0 < int(label) <= len(class_names) else "?",
        })
    return {"frame": frame_idx, "stamp": stamp, "num_points": det["num_points"],
            "num_voxels": det["num_voxels"], "detections": recs}


def _log(frame_idx, det, args):
    if args.print_every <= 0 or frame_idx % args.print_every != 0:
        return
    t = det["timings"]
    print("[frame %5d] pts=%6d vox=%6d det=%3d | pre=%5.1f infer=%7.1f post=%6.1f total=%7.1f ms"
          % (frame_idx, det["num_points"], det["num_voxels"], det["boxes"].shape[0],
             t["preprocess"], t["inference"], t["postprocess"], t["total"]))
    sys.stdout.flush()


def run_bag(args, detector, cfg):
    try:
        import rosbag
    except ImportError:
        print("ERROR: python 'rosbag' is not importable. Install it with:\n"
              "  sudo apt-get install python3-rosbag   (or source /opt/ros/<distro>/setup.bash)")
        return 1

    bag_path = Path(args.bag).expanduser()
    if not bag_path.is_file():
        print("ERROR: bag not found: %s" % bag_path)
        return 1

    bag = rosbag.Bag(str(bag_path), "r")
    try:
        info = bag.get_type_and_topic_info().topics
        cloud_topics = [name for name, meta in info.items()
                        if meta.msg_type == "sensor_msgs/PointCloud2"]
        if not cloud_topics:
            print("ERROR: no sensor_msgs/PointCloud2 topics in %s\nTopics:\n%s"
                  % (bag_path, "\n".join("  %s (%s)" % (n, m.msg_type) for n, m in info.items())))
            return 1

        topic = args.topic
        if topic is None:
            topic = cfg.get("topic", "/livox/lidar")
        if topic not in info or info[topic].msg_type != "sensor_msgs/PointCloud2":
            print("Topic %r not a PointCloud2 in this bag; using %r" % (topic, cloud_topics[0]))
            topic = cloud_topics[0]

        print("bag: %s" % bag_path)
        print("topic: %s | messages: %d | duration: %.1f s"
              % (topic, info[topic].message_count, bag.get_end_time() - bag.get_start_time()))

        publish = not args.no_publish
        if publish and not ros_master_alive():
            print("No ROS master reachable -> running headless (pass --no-publish to silence this).")
            publish = False
        pubs = Publishers(args.boxes_topic, args.points_topic, "pointpillars_bag") if publish else None

        records = []
        frame_idx = 0
        t_start = None
        wall_start = None
        for _, msg, t in bag.read_messages(topics=[topic]):
            stamp = t.to_sec()
            if t_start is None:
                t_start = stamp
                wall_start = time.time()

            if pubs is not None and args.rate > 0:
                target = (stamp - t_start) / args.rate
                delay = target - (time.time() - wall_start)
                if delay > 0:
                    time.sleep(delay)

            points = pointcloud2_to_xyz_intensity(msg)
            det = detector.detect(points)
            _log(frame_idx, det, args)

            if pubs is not None:
                pubs.publish(det, points, detector.class_names, msg.header, args.publish_points)
            if args.save_json:
                records.append(_json_record(frame_idx, stamp, det, detector.class_names))

            frame_idx += 1
            if args.max_frames and frame_idx >= args.max_frames:
                break
    finally:
        bag.close()

    if args.save_json:
        out = Path(args.save_json)
        out.parent.mkdir(parents=True, exist_ok=True)
        with open(str(out), "w") as f:
            json.dump(records, f)
        print("wrote %d frames to %s" % (len(records), out))
    print("done: %d frames" % frame_idx)
    return 0


def run_live(args, detector, cfg):
    try:
        import rospy
        from sensor_msgs.msg import PointCloud2
    except ImportError:
        print("ERROR: rospy/sensor_msgs not importable. Source your ROS setup.bash first.")
        return 1

    if not ros_master_alive():
        print("ERROR: no ROS master. Start `roscore` (or use --bag for offline inference).")
        return 1

    pubs = Publishers(args.boxes_topic, args.points_topic, "pointpillars_live")
    topic = args.topic or cfg.get("topic", "/livox/lidar")
    state = {"frame": 0, "records": []}

    def callback(msg):
        try:
            points = pointcloud2_to_xyz_intensity(msg)
        except Exception as e:  # malformed frame must not kill the node
            rospy.logwarn("PointCloud2 conversion failed: %s" % e)
            return
        det = detector.detect(points)
        _log(state["frame"], det, args)
        pubs.publish(det, points, detector.class_names, msg.header, args.publish_points)
        if args.save_json:
            state["records"].append(_json_record(state["frame"], msg.header.stamp.to_sec(),
                                                 det, detector.class_names))
        state["frame"] += 1

        if args.max_frames and state["frame"] >= args.max_frames:
            rospy.signal_shutdown("max frames reached")

    sub = rospy.Subscriber(topic, PointCloud2, callback, queue_size=1, buff_size=2 ** 24)
    print("subscribed to %s - waiting for PointCloud2 messages (Ctrl-C to stop)" % topic)
    rospy.spin()
    del sub

    if args.save_json and state["records"]:
        out = Path(args.save_json)
        out.parent.mkdir(parents=True, exist_ok=True)
        with open(str(out), "w") as f:
            json.dump(state["records"], f)
        print("wrote %d frames to %s" % (len(state["records"]), out))
    print("done: %d frames" % state["frame"])
    return 0


def main():
    args = parse_args()
    try:
        cfg = load_config(args.config)
        if args.score_thresh is not None:
            cfg["score_thresh"] = float(args.score_thresh)
        if args.nms_thresh is not None:
            cfg["nms_thresh"] = float(args.nms_thresh)
        if args.device:
            cfg["device"] = args.device
        detector = PointPillarDetector(cfg, checkpoint=args.checkpoint, device=args.device)
    except (ConfigError, RuntimeError) as e:
        print("ERROR: %s" % e)
        return 2

    print("config: %s" % cfg["config_path"])
    print("range : %s | voxel: %.3g x %.3g x %.3g"
          % (cfg["point_cloud_range"], cfg["voxel_size"][0], cfg["voxel_size"][1],
             cfg["voxel_size"][2]))

    if args.bag:
        return run_bag(args, detector, cfg)
    return run_live(args, detector, cfg)


if __name__ == "__main__":
    sys.exit(main())
