#!/usr/bin/env python3

import math

import rospy
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry


class SimLocalizationNode:
    def __init__(self):
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        self.base_frame = rospy.get_param("~base_frame", "base_footprint")
        self.odom_topic = rospy.get_param("~odom_topic", "/odom")
        self.amcl_topic = rospy.get_param("~amcl_topic", "/amcl_pose")
        self.publish_rate = float(rospy.get_param("~publish_rate", 20.0))
        self.default_initial_x = float(rospy.get_param("~initial_x", 0.0))
        self.default_initial_y = float(rospy.get_param("~initial_y", 0.0))
        self.default_initial_yaw = float(rospy.get_param("~initial_yaw", 0.0))

        self.odom_pose = None
        self.map_to_odom = (0.0, 0.0, 0.0)
        self.initialized = False
        self.br = tf.TransformBroadcaster()
        self.amcl_pub = rospy.Publisher(self.amcl_topic, PoseWithCovarianceStamped, queue_size=5, latch=True)
        rospy.Subscriber(self.odom_topic, Odometry, self._on_odom, queue_size=1)
        rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self._on_initialpose, queue_size=1)
        rospy.loginfo("[sim_localization] waiting odom=%s initialpose=/initialpose" % self.odom_topic)

    def _on_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self.odom_pose = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
            float(yaw),
        )
        if not self.initialized:
            self._set_initial_pose(self.default_initial_x, self.default_initial_y, self.default_initial_yaw)

    def _on_initialpose(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self._set_initial_pose(float(msg.pose.pose.position.x), float(msg.pose.pose.position.y), yaw)
        rospy.loginfo(
            "[sim_localization] initialized pose=(%.3f, %.3f, %.3f)"
            % (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
        )

    def _set_initial_pose(self, map_x, map_y, map_yaw):
        if self.odom_pose is None:
            return
        odom_x, odom_y, odom_yaw = self.odom_pose
        yaw = self._normalize(map_yaw - odom_yaw)
        tx = map_x - (math.cos(yaw) * odom_x - math.sin(yaw) * odom_y)
        ty = map_y - (math.sin(yaw) * odom_x + math.cos(yaw) * odom_y)
        self.map_to_odom = (tx, ty, yaw)
        self.initialized = True

    def _map_pose(self):
        if self.odom_pose is None:
            return None
        tx, ty, yaw_tf = self.map_to_odom
        odom_x, odom_y, odom_yaw = self.odom_pose
        x = tx + math.cos(yaw_tf) * odom_x - math.sin(yaw_tf) * odom_y
        y = ty + math.sin(yaw_tf) * odom_x + math.cos(yaw_tf) * odom_y
        yaw = self._normalize(yaw_tf + odom_yaw)
        return x, y, yaw

    def spin(self):
        rate = rospy.Rate(max(0.1, self.publish_rate))
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            tx, ty, yaw = self.map_to_odom
            quat = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)
            self.br.sendTransform((tx, ty, 0.0), quat, now, self.odom_frame, self.map_frame)
            pose = self._map_pose()
            if pose is not None:
                self.amcl_pub.publish(self._amcl_msg(now, pose))
            rate.sleep()

    def _amcl_msg(self, stamp, pose):
        x, y, yaw = pose
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.map_frame
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        quat = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)
        msg.pose.pose.orientation.x = quat[0]
        msg.pose.pose.orientation.y = quat[1]
        msg.pose.pose.orientation.z = quat[2]
        msg.pose.pose.orientation.w = quat[3]
        msg.pose.covariance[0] = 0.02
        msg.pose.covariance[7] = 0.02
        msg.pose.covariance[35] = 0.01
        return msg

    def _normalize(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


if __name__ == "__main__":
    rospy.init_node("sim_localization")
    try:
        SimLocalizationNode().spin()
    except rospy.ROSInterruptException:
        pass
