#!/usr/bin/env python3
"""
Oracle weed detector.

Reads the same ground-truth file that built the Gazebo world and publishes
weed positions as if a perception stack had found them. No neural network
is involved, and that is the point: it decouples the targeting chain from
perception quality so you can test one without the other.

Crucially it is NOT a cheat that hides the hard cases. The noise model
below injects position error, false positives and missed detections, all
configurable at runtime. That means you can *command* the failure modes a
real detector produces only by luck:

    ros2 param set /oracle_detector position_noise_std 0.004
    ros2 param set /oracle_detector miss_rate 0.3
    ros2 param set /oracle_detector false_positive_rate 0.2

A real YOLO node would publish the same message on the same topic, so
everything downstream is unchanged when you swap it in.

Publishes:
    /weeds/detections   vision_msgs/Detection3DArray   in work_surface frame
    /weeds/markers      visualization_msgs/MarkerArray for RViz
"""

import random

import rclpy
import yaml
from rclpy.node import Node
from vision_msgs.msg import (
    Detection3D,
    Detection3DArray,
    ObjectHypothesisWithPose,
)
from visualization_msgs.msg import Marker, MarkerArray


class OracleDetector(Node):

    def __init__(self):
        super().__init__("oracle_detector")

        self.declare_parameter("ground_truth_file", "")
        self.declare_parameter("frame_id", "work_surface")
        self.declare_parameter("rate_hz", 2.0)

        # --- noise model -------------------------------------------------
        # Defaults are deliberately mild. Turn them up to stress the
        # supervisor and the workspace-bounds checking.
        self.declare_parameter("position_noise_std", 0.0015)   # metres
        self.declare_parameter("miss_rate", 0.0)               # 0..1
        self.declare_parameter("false_positive_rate", 0.0)     # per cycle
        self.declare_parameter("confidence_mean", 0.88)
        self.declare_parameter("seed", 0)

        path = self.get_parameter("ground_truth_file").value
        if not path:
            raise RuntimeError("ground_truth_file parameter is required")

        with open(path) as f:
            data = yaml.safe_load(f)

        self.weeds = data.get("weeds", [])
        self.half = data["work_surface"]["half_extent"]
        self.frame = self.get_parameter("frame_id").value

        seed = self.get_parameter("seed").value
        self.rng = random.Random(seed if seed else None)

        self.pub = self.create_publisher(
            Detection3DArray, "/weeds/detections", 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, "/weeds/markers", 10)

        rate = self.get_parameter("rate_hz").value
        self.timer = self.create_timer(1.0 / rate, self.tick)

        self.get_logger().info(
            f"oracle up: {len(self.weeds)} weeds in ground truth, "
            f"publishing on /weeds/detections in frame '{self.frame}'")

    # ---------------------------------------------------------------- #

    def tick(self):
        p = self.get_parameter
        noise = p("position_noise_std").value
        miss = p("miss_rate").value
        fp_rate = p("false_positive_rate").value
        conf_mean = p("confidence_mean").value

        msg = Detection3DArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame

        for w in self.weeds:
            if self.rng.random() < miss:
                continue          # detector missed this one this cycle

            u = w["u"] + self.rng.gauss(0.0, noise)
            v = w["v"] + self.rng.gauss(0.0, noise)
            conf = max(0.0, min(1.0, self.rng.gauss(conf_mean, 0.05)))
            msg.detections.append(
                self._detection(w["id"], u, v, conf, w.get("size", 1.0)))

        # Hallucinate a detection on bare soil now and then. This is the
        # case that matters: firing a laser at nothing wastes a cycle,
        # firing it at a crop destroys the crop.
        if self.rng.random() < fp_rate:
            u = self.rng.uniform(-self.half, self.half)
            v = self.rng.uniform(-self.half, self.half)
            msg.detections.append(
                self._detection("false_positive", u, v,
                                self.rng.uniform(0.35, 0.6), 1.0))

        self.pub.publish(msg)
        self.marker_pub.publish(self._markers(msg))

    def _detection(self, det_id, u, v, conf, size):
        d = Detection3D()
        d.id = det_id
        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = "weed"
        hyp.hypothesis.score = conf
        hyp.pose.pose.position.x = float(u)
        hyp.pose.pose.position.y = float(v)
        hyp.pose.pose.position.z = 0.0
        hyp.pose.pose.orientation.w = 1.0
        d.results.append(hyp)
        d.bbox.center.position.x = float(u)
        d.bbox.center.position.y = float(v)
        d.bbox.center.orientation.w = 1.0
        d.bbox.size.x = 0.03 * size
        d.bbox.size.y = 0.03 * size
        d.bbox.size.z = 0.04 * size
        return d

    def _markers(self, det_msg):
        arr = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)

        for i, d in enumerate(det_msg.detections):
            m = Marker()
            m.header = det_msg.header
            m.ns = "weed_detections"
            m.id = i
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose = d.bbox.center
            m.pose.position.z = 0.001
            m.scale.x = m.scale.y = 0.035
            m.scale.z = 0.002
            score = d.results[0].hypothesis.score
            # low confidence renders orange so false positives are obvious
            m.color.r = 1.0 if score < 0.7 else 0.9
            m.color.g = 0.55 if score < 0.7 else 0.1
            m.color.b = 0.0
            m.color.a = 0.85
            arr.markers.append(m)
        return arr


def main():
    rclpy.init()
    node = OracleDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
