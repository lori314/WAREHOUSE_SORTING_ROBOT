#!/usr/bin/env python3

import math

import rospy
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped


class InitialPoseSetter:
    def __init__(self):
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.amcl_topic = rospy.get_param("~amcl_topic", "/amcl_pose")
        self.x = float(rospy.get_param("~x", 0.0))
        self.y = float(rospy.get_param("~y", 0.0))
        self.yaw = float(rospy.get_param("~yaw", 0.0))
        self.repeat = int(rospy.get_param("~repeat", 8))
        self.rate_hz = float(rospy.get_param("~rate", 5.0))
        self.verify_timeout = float(rospy.get_param("~verify_timeout", 8.0))
        self.xy_tolerance = float(rospy.get_param("~xy_tolerance", 0.15))
        self.yaw_tolerance = float(rospy.get_param("~yaw_tolerance", 0.25))
        self.pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1, latch=True)

    def run(self):
        rate = rospy.Rate(max(0.1, self.rate_hz))
        msg = self._pose_msg()
        rospy.sleep(0.5)
        for _ in range(max(1, self.repeat)):
            self.pub.publish(msg)
            rate.sleep()
        if self._verify():
            rospy.loginfo("[initial_pose] accepted pose=(%.3f, %.3f, %.3f)" % (self.x, self.y, self.yaw))
            return 0
        rospy.logwarn("[initial_pose] pose not verified within %.1fs" % self.verify_timeout)
        return 2

    def _pose_msg(self):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.map_frame
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        quat = tf.transformations.quaternion_from_euler(0.0, 0.0, self.yaw)
        msg.pose.pose.orientation.x = quat[0]
        msg.pose.pose.orientation.y = quat[1]
        msg.pose.pose.orientation.z = quat[2]
        msg.pose.pose.orientation.w = quat[3]
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.0685
        return msg

    def _verify(self):
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
            xy_error = math.hypot(msg.pose.pose.position.x - self.x, msg.pose.pose.position.y - self.y)
            yaw_error = abs(self._normalize(yaw - self.yaw))
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
    rospy.init_node("initial_pose_setter")
    raise SystemExit(InitialPoseSetter().run())
