#!/usr/bin/env python3
"""
Targeting node.

Takes weed detections, works out where the laser currently points, and
commands the joints to close the gap.

WHY THERE IS NO IK LIBRARY HERE
-------------------------------
For a 2-DOF Cartesian gantry, inverse kinematics is the identity function
plus a constant offset. Moving joint_x by 10 mm moves the beam 10 mm along
world X. That is the entire relationship. Bringing in MoveIt to solve it
would mean an SRDF, planning groups and OMPL sampling a configuration
space with a closed-form answer.

Instead this node asks TF where the beam actually lands, compares that to
the target, and applies the difference to the joints. One shot converges
exactly because the mapping is linear. It is also self-correcting: if the
URDF offsets change, this code does not.

Subscribes:  /weeds/detections   vision_msgs/Detection3DArray
             /joint_states       sensor_msgs/JointState
Publishes:   /gantry_controller/joint_trajectory
             /targeting/markers  beam + aim point for RViz
Service:     ~/treat_next        std_srvs/Trigger - target the next weed
"""

import math

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs  # noqa: F401  registers PointStamped transforms
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from vision_msgs.msg import Detection3DArray
from visualization_msgs.msg import Marker, MarkerArray


class TargetingNode(Node):

    def __init__(self):
        super().__init__("targeting_node")

        self.declare_parameter("travel_x", 0.250)
        self.declare_parameter("travel_y", 0.250)
        self.declare_parameter("margin", 0.002)      # keep off the hard stops
        self.declare_parameter("max_velocity", 0.035)
        self.declare_parameter("min_confidence", 0.6)
        self.declare_parameter("work_frame", "work_surface")
        self.declare_parameter("aim_frame", "laser_aim_link")
        self.declare_parameter("auto_cycle", True)
        self.declare_parameter("dwell_s", 2.0)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.joints = {}
        self.detections = []
        self.queue = []
        self.treated = []
        self.busy_until = None

        self.create_subscription(
            JointState, "/joint_states", self.on_joints, 10)
        self.create_subscription(
            Detection3DArray, "/weeds/detections", self.on_detections, 10)

        self.traj_pub = self.create_publisher(
            JointTrajectory, "/gantry_controller/joint_trajectory", 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, "/targeting/markers", 10)

        self.create_service(Trigger, "~/treat_next", self.srv_treat_next)

        self.create_timer(0.2, self.tick)
        self.get_logger().info("targeting node up")

    # ---------------------------------------------------------------- #
    # callbacks
    # ---------------------------------------------------------------- #

    def on_joints(self, msg):
        for n, p in zip(msg.name, msg.position):
            self.joints[n] = p

    def on_detections(self, msg):
        self.detections = msg.detections
        self.det_frame = msg.header.frame_id

    # ---------------------------------------------------------------- #
    # the actual targeting
    # ---------------------------------------------------------------- #

    def beam_ground_point(self):
        """
        Where does the beam hit the soil right now?

        Asks TF for laser_aim_link expressed in work_surface. Because
        laser_aim_link's Z axis is the beam direction and work_surface is
        the soil plane, the X and Y of that transform ARE the strike point.
        No projection maths, no homography - the URDF already encodes it.
        """
        try:
            tf = self.tf_buffer.lookup_transform(
                self.get_parameter("work_frame").value,
                self.get_parameter("aim_frame").value,
                rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f"TF not ready: {e}", throttle_duration_sec=3.0)
            return None
        t = tf.transform.translation
        return (t.x, t.y)

    def solve(self, target_u, target_v):
        """
        Joint values that put the beam on (target_u, target_v).

        Cartesian gantry, so the Jacobian is a constant sign matrix.
        Measure the current error and add it to the current joints.
        joint_y is negated because the CAD carries a 180 degree roll,
        which makes machine +Y run along world -Y.
        """
        here = self.beam_ground_point()
        if here is None:
            return None
        if "joint_x" not in self.joints or "joint_y" not in self.joints:
            return None

        err_u = target_u - here[0]
        err_v = target_v - here[1]

        jx = self.joints["joint_x"] + err_u
        jy = self.joints["joint_y"] - err_v
        return jx, jy

    def in_workspace(self, jx, jy):
        m = self.get_parameter("margin").value
        tx = self.get_parameter("travel_x").value
        ty = self.get_parameter("travel_y").value
        return (m <= jx <= tx - m) and (m <= jy <= ty - m)

    def send(self, jx, jy):
        """
        Duration from distance and the velocity cap. The cap comes from
        lead screw critical speed, not motor torque - see the design notes.
        Padded 40% so the controller is never chasing an impossible ramp,
        which is what trips the trajectory tolerance.
        """
        vmax = self.get_parameter("max_velocity").value
        dx = abs(jx - self.joints.get("joint_x", 0.0))
        dy = abs(jy - self.joints.get("joint_y", 0.0))
        dist = max(dx, dy)
        secs = max(1.5, (dist / vmax) * 1.4)

        msg = JointTrajectory()
        msg.joint_names = ["joint_x", "joint_y"]
        pt = JointTrajectoryPoint()
        pt.positions = [float(jx), float(jy)]
        pt.time_from_start = Duration(
            sec=int(secs), nanosec=int((secs % 1) * 1e9))
        msg.points.append(pt)
        self.traj_pub.publish(msg)
        return secs

    # ---------------------------------------------------------------- #

    def next_target(self):
        """Nearest untreated weed above the confidence threshold."""
        min_conf = self.get_parameter("min_confidence").value
        here = self.beam_ground_point()
        if here is None:
            return None

        best, best_d = None, 1e9
        for d in self.detections:
            if not d.results:
                continue
            if d.results[0].hypothesis.score < min_conf:
                continue
            if d.id in self.treated:
                continue
            u = d.bbox.center.position.x
            v = d.bbox.center.position.y
            dist = math.hypot(u - here[0], v - here[1])
            if dist < best_d:
                best, best_d = d, dist
        return best

    def tick(self):
        self.publish_markers()

        if not self.get_parameter("auto_cycle").value:
            return

        now = self.get_clock().now()
        if self.busy_until is not None and now < self.busy_until:
            return

        target = self.next_target()
        if target is None:
            return

        u = target.bbox.center.position.x
        v = target.bbox.center.position.y

        sol = self.solve(u, v)
        if sol is None:
            return
        jx, jy = sol

        if not self.in_workspace(jx, jy):
            self.get_logger().warn(
                f"{target.id} at ({u:+.3f},{v:+.3f}) needs "
                f"({jx:.3f},{jy:.3f}) - OUTSIDE WORKSPACE, rejecting")
            self.treated.append(target.id)
            return

        secs = self.send(jx, jy)
        dwell = self.get_parameter("dwell_s").value
        self.busy_until = now + rclpy.duration.Duration(
            seconds=secs + dwell)
        self.treated.append(target.id)
        self.get_logger().info(
            f"targeting {target.id} at ({u:+.3f},{v:+.3f}) -> "
            f"joints ({jx:.3f},{jy:.3f}) in {secs:.1f}s")

    def srv_treat_next(self, req, resp):
        t = self.next_target()
        if t is None:
            resp.success = False
            resp.message = "no untreated weeds above confidence threshold"
            return resp
        sol = self.solve(t.bbox.center.position.x, t.bbox.center.position.y)
        if sol is None:
            resp.success = False
            resp.message = "TF or joint states unavailable"
            return resp
        jx, jy = sol
        if not self.in_workspace(jx, jy):
            resp.success = False
            resp.message = f"{t.id} outside workspace"
            self.treated.append(t.id)
            return resp
        self.send(jx, jy)
        self.treated.append(t.id)
        resp.success = True
        resp.message = f"targeting {t.id} -> ({jx:.4f}, {jy:.4f})"
        return resp

    # ---------------------------------------------------------------- #

    def publish_markers(self):
        here = self.beam_ground_point()
        if here is None:
            return
        arr = MarkerArray()
        frame = self.get_parameter("work_frame").value
        stamp = self.get_clock().now().to_msg()

        aim = Marker()
        aim.header.frame_id = frame
        aim.header.stamp = stamp
        aim.ns = "aim"
        aim.id = 0
        aim.type = Marker.CYLINDER
        aim.action = Marker.ADD
        aim.pose.position.x = here[0]
        aim.pose.position.y = here[1]
        aim.pose.position.z = 0.002
        aim.pose.orientation.w = 1.0
        aim.scale.x = aim.scale.y = 0.012
        aim.scale.z = 0.001
        aim.color.r, aim.color.g, aim.color.b, aim.color.a = 0.1, 1.0, 0.2, 0.95
        arr.markers.append(aim)

        beam = Marker()
        beam.header.frame_id = frame
        beam.header.stamp = stamp
        beam.ns = "beam"
        beam.id = 1
        beam.type = Marker.CYLINDER
        beam.action = Marker.ADD
        beam.pose.position.x = here[0]
        beam.pose.position.y = here[1]
        beam.pose.position.z = 0.15
        beam.pose.orientation.w = 1.0
        beam.scale.x = beam.scale.y = 0.003
        beam.scale.z = 0.30
        beam.color.r, beam.color.g, beam.color.b, beam.color.a = 1.0, 0.1, 0.1, 0.35
        arr.markers.append(beam)

        self.marker_pub.publish(arr)


def main():
    rclpy.init()
    node = TargetingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
