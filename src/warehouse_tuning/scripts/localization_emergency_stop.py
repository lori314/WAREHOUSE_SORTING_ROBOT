#!/usr/bin/env python3

import json
from typing import List

import rosnode
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


def _list_param(name, default):
    value = rospy.get_param(name, default)
    if isinstance(value, str):
        return [item.strip() for item in value.split(",") if item.strip()]
    if isinstance(value, (list, tuple)):
        return [str(item).strip() for item in value if str(item).strip()]
    return list(default)


class LocalizationEmergencyStop:
    def __init__(self):
        self.cmd_vel_topic = str(rospy.get_param("~cmd_vel_topic", "/cmd_vel"))
        self.behaviors_topic = str(rospy.get_param("~behaviors_topic", "/wpb_home/behaviors"))
        self.emergency_topic = str(rospy.get_param("~emergency_topic", "/warehouse_tuning/emergency_stop"))
        self.amcl_topic = str(rospy.get_param("~amcl_topic", "/amcl_pose"))
        self.scan_topic = str(rospy.get_param("~scan_topic", "/scan"))
        self.monitored_nodes: List[str] = _list_param("~monitored_nodes", ["/amcl"])

        self.startup_grace_seconds = float(rospy.get_param("~startup_grace_seconds", 8.0))
        self.node_check_period = float(rospy.get_param("~node_check_period", 0.25))
        self.amcl_timeout = float(rospy.get_param("~amcl_timeout", 0.0))
        self.scan_timeout = float(rospy.get_param("~scan_timeout", 0.0))
        self.stop_hz = float(rospy.get_param("~stop_hz", 30.0))
        self.behavior_stop_hz = float(rospy.get_param("~behavior_stop_hz", 2.0))
        self.check_amcl_topic = bool(rospy.get_param("~check_amcl_topic", False))
        self.check_scan_topic = bool(rospy.get_param("~check_scan_topic", False))
        self.check_nodes = bool(rospy.get_param("~check_nodes", True))

        self.cmd_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=10)
        self.behaviors_pub = rospy.Publisher(self.behaviors_topic, String, queue_size=10)
        self.emergency_pub = rospy.Publisher(self.emergency_topic, String, queue_size=1, latch=True)

        self.last_amcl_at = 0.0
        self.last_scan_at = 0.0
        self.started_at = rospy.Time.now().to_sec()
        self.last_node_check_at = 0.0
        self.last_behavior_stop_at = 0.0
        self.emergency_active = False
        self.emergency_reason = ""

        rospy.Subscriber(self.amcl_topic, PoseWithCovarianceStamped, self._on_amcl, queue_size=1)
        rospy.Subscriber(self.scan_topic, LaserScan, self._on_scan, queue_size=1)
        rospy.logwarn(
            "[LOCALIZATION-E-STOP] armed nodes=%s amcl=%s scan=%s cmd_vel=%s emergency=%s grace=%.1fs topic_timeout_checks=(amcl:%s scan:%s)",
            ",".join(self.monitored_nodes),
            self.amcl_topic,
            self.scan_topic,
            self.cmd_vel_topic,
            self.emergency_topic,
            self.startup_grace_seconds,
            str(self.check_amcl_topic),
            str(self.check_scan_topic),
        )

    def _on_amcl(self, _msg):
        self.last_amcl_at = rospy.Time.now().to_sec()

    def _on_scan(self, _msg):
        self.last_scan_at = rospy.Time.now().to_sec()

    def _trigger(self, reason):
        if self.emergency_active:
            return
        self.emergency_active = True
        self.emergency_reason = reason
        payload = {
            "reason": reason,
            "stamp": rospy.Time.now().to_sec(),
            "monitored_nodes": self.monitored_nodes,
            "amcl_topic": self.amcl_topic,
            "scan_topic": self.scan_topic,
        }
        rospy.logerr("[LOCALIZATION-E-STOP] %s", json.dumps(payload, sort_keys=True))
        self.emergency_pub.publish(String(data=json.dumps(payload, sort_keys=True)))

    def _check_nodes_alive(self, now):
        if not self.check_nodes:
            return
        if now - self.last_node_check_at < self.node_check_period:
            return
        self.last_node_check_at = now
        try:
            alive = set(rosnode.get_node_names())
        except Exception as exc:
            self._trigger("rosnode_check_failed:%s" % exc)
            return
        missing = [name for name in self.monitored_nodes if name not in alive]
        if missing:
            self._trigger("monitored_node_missing:%s" % ",".join(missing))

    def _check_topic_timeouts(self, now):
        if self.check_amcl_topic:
            if self.last_amcl_at <= 0.0:
                rospy.logwarn_throttle(
                    5.0,
                    "[LOCALIZATION-E-STOP] waiting for first %s before arming AMCL topic timeout",
                    self.amcl_topic,
                )
            elif self.amcl_timeout > 0.0 and now - self.last_amcl_at > self.amcl_timeout:
                self._trigger("amcl_timeout %.2fs > %.2fs" % (now - self.last_amcl_at, self.amcl_timeout))
        if self.check_scan_topic:
            if self.last_scan_at <= 0.0:
                rospy.logwarn_throttle(
                    5.0,
                    "[LOCALIZATION-E-STOP] waiting for first %s before arming scan topic timeout",
                    self.scan_topic,
                )
            elif self.scan_timeout > 0.0 and now - self.last_scan_at > self.scan_timeout:
                self._trigger("scan_timeout %.2fs > %.2fs" % (now - self.last_scan_at, self.scan_timeout))

    def _publish_stop(self, now):
        self.cmd_pub.publish(Twist())
        if now - self.last_behavior_stop_at >= 1.0 / max(0.1, self.behavior_stop_hz):
            self.behaviors_pub.publish(String(data="grab stop"))
            self.behaviors_pub.publish(String(data="object_detect stop"))
            self.last_behavior_stop_at = now

    def run(self):
        rate = rospy.Rate(max(1.0, self.stop_hz))
        while not rospy.is_shutdown():
            now = rospy.Time.now().to_sec()
            if now - self.started_at >= self.startup_grace_seconds and not self.emergency_active:
                self._check_nodes_alive(now)
                self._check_topic_timeouts(now)

            if self.emergency_active:
                self._publish_stop(now)

            rate.sleep()


def main():
    rospy.init_node("localization_emergency_stop")
    LocalizationEmergencyStop().run()


if __name__ == "__main__":
    main()
