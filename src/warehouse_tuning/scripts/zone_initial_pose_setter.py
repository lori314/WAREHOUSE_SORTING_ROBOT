#!/usr/bin/env python3

import math
import os

import rospy
import tf
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped


class ZoneInitialPoseSetter:
    def __init__(self):
        self.zone_file = os.path.abspath(os.path.expanduser(rospy.get_param("~zone_file", "$HOME/maps/abc_zones.yaml")))
        self.zone_name = str(rospy.get_param("~zone_name", "C"))
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.amcl_topic = rospy.get_param("~amcl_topic", "/amcl_pose")
        self.repeat = int(rospy.get_param("~repeat", 12))
        self.rate_hz = float(rospy.get_param("~rate", 4.0))
        self.verify_timeout = float(rospy.get_param("~verify_timeout", 10.0))
        self.xy_tolerance = float(rospy.get_param("~xy_tolerance", 0.25))
        self.yaw_tolerance = float(rospy.get_param("~yaw_tolerance", 0.35))
        self.pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1, latch=True)

    def run(self):
        pose = self._load_zone_pose()
        msg = self._pose_msg(pose)
        rospy.sleep(1.0)
        rate = rospy.Rate(max(0.1, self.rate_hz))
        for _ in range(max(1, self.repeat)):
            msg.header.stamp = rospy.Time.now()
            self.pub.publish(msg)
            rate.sleep()
        if self._verify(pose):
            rospy.loginfo(
                "[zone_initial_pose] accepted zone=%s pose=(%.3f, %.3f, %.3f)"
                % (self.zone_name, pose["x"], pose["y"], pose["yaw"])
            )
            return 0
        rospy.logwarn("[zone_initial_pose] zone=%s pose not verified within %.1fs" % (self.zone_name, self.verify_timeout))
        return 2

    def _load_zone_pose(self):
        with open(self.zone_file, "r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream) or {}
        zones = data.get("warehouse_tuning", {}).get("abc_zones", {})
        zone = zones.get(self.zone_name)
        if not isinstance(zone, dict):
            raise RuntimeError("zone %s not found in %s" % (self.zone_name, self.zone_file))
        return {
            "x": float(zone.get("x", 0.0)),
            "y": float(zone.get("y", 0.0)),
            "yaw": float(zone.get("yaw", 0.0)),
        }

    def _pose_msg(self, pose):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.map_frame
        msg.pose.pose.position.x = pose["x"]
        msg.pose.pose.position.y = pose["y"]
        quat = tf.transformations.quaternion_from_euler(0.0, 0.0, pose["yaw"])
        msg.pose.pose.orientation.x = quat[0]
        msg.pose.pose.orientation.y = quat[1]
        msg.pose.pose.orientation.z = quat[2]
        msg.pose.pose.orientation.w = quat[3]
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.0685
        return msg

    def _verify(self, pose):
        deadline = rospy.Time.now().to_sec() + self.verify_timeout
        while not rospy.is_shutdown() and rospy.Time.now().to_sec() < deadline:
            try:
                msg = rospy.wait_for_message(self.amcl_topic, PoseWithCovarianceStamped, timeout=0.5)
            except Exception:
                continue
            q = msg.pose.pose.orientation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            )
            xy_error = math.hypot(msg.pose.pose.position.x - pose["x"], msg.pose.pose.position.y - pose["y"])
            yaw_error = abs(self._normalize(yaw - pose["yaw"]))
            if xy_error <= self.xy_tolerance and yaw_error <= self.yaw_tolerance:
                return True
        return False

    def _normalize(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


if __name__ == "__main__":
    rospy.init_node("zone_initial_pose_setter")
    raise SystemExit(ZoneInitialPoseSetter().run())
