#!/usr/bin/env python3

import math
from typing import Dict, List, Optional

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String


def normalize(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_odom(msg: Odometry) -> float:
    q = msg.pose.pose.orientation
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class SimMappingDrive:
    ROUTES = {
        "field_loop": [
            {"x": -0.55, "y": 0.00, "yaw": 0.0},
            {"x": -0.55, "y": 0.75, "yaw": math.pi / 2.0},
            {"x": 0.25, "y": 0.75, "yaw": 0.0},
            {"x": 0.25, "y": 0.25, "yaw": -math.pi / 2.0},
            {"x": -0.55, "y": -0.75, "yaw": -math.pi / 2.0},
            {"x": 0.25, "y": -0.75, "yaw": 0.0},
            {"x": 0.15, "y": -0.20, "yaw": math.pi / 2.0},
            {"x": 0.00, "y": 0.00, "yaw": 0.0},
        ],
        "short_loop": [
            {"x": -0.35, "y": 0.00, "yaw": 0.0},
            {"x": -0.35, "y": 0.45, "yaw": math.pi / 2.0},
            {"x": 0.15, "y": 0.45, "yaw": 0.0},
            {"x": 0.00, "y": 0.00, "yaw": 0.0},
        ],
    }

    def __init__(self):
        self.route_name = str(rospy.get_param("~route", "field_loop"))
        self.odom_topic = str(rospy.get_param("~odom_topic", "/odom"))
        self.cmd_topic = str(rospy.get_param("~cmd_topic", "/cmd_vel"))
        self.status_topic = str(rospy.get_param("~status_topic", "/warehouse_tuning/sim_mapping_drive_status"))
        self.rate_hz = float(rospy.get_param("~rate", 20.0))
        self.timeout = float(rospy.get_param("~timeout", 90.0))
        self.xy_tolerance = float(rospy.get_param("~xy_tolerance", 0.06))
        self.yaw_tolerance = float(rospy.get_param("~yaw_tolerance", 0.10))
        self.k_xy = float(rospy.get_param("~k_xy", 0.85))
        self.k_yaw = float(rospy.get_param("~k_yaw", 1.15))
        self.max_xy_speed = float(rospy.get_param("~max_xy_speed", 0.18))
        self.max_yaw_speed = float(rospy.get_param("~max_yaw_speed", 0.55))

        self.pose = None
        self.cmd_pub = rospy.Publisher(self.cmd_topic, Twist, queue_size=5)
        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=5, latch=True)
        rospy.Subscriber(self.odom_topic, Odometry, self._on_odom, queue_size=1)

    def _on_odom(self, msg: Odometry):
        self.pose = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
            yaw_from_odom(msg),
        )

    def _route(self) -> List[Dict[str, float]]:
        if self.route_name not in self.ROUTES:
            raise RuntimeError("unknown mapping route: %s" % self.route_name)
        return self.ROUTES[self.route_name]

    def _publish_status(self, state: str, detail: Dict):
        payload = {"state": state, "route": self.route_name}
        payload.update(detail)
        self.status_pub.publish(String(data=str(payload)))

    def _stop(self):
        self.cmd_pub.publish(Twist())

    def _wait_for_odom(self):
        deadline = rospy.Time.now().to_sec() + min(10.0, self.timeout)
        while not rospy.is_shutdown() and self.pose is None:
            if rospy.Time.now().to_sec() >= deadline:
                raise RuntimeError("no odom received on %s" % self.odom_topic)
            rospy.sleep(0.1)

    def run(self):
        self._wait_for_odom()
        route = self._route()
        rospy.loginfo("[SIM-MAPPING] drive route=%s waypoints=%d", self.route_name, len(route))
        self._publish_status("RUNNING", {"waypoints": len(route)})

        rate = rospy.Rate(max(1.0, self.rate_hz))
        deadline = rospy.Time.now().to_sec() + self.timeout
        for idx, target in enumerate(route, start=1):
            if rospy.is_shutdown():
                break
            rospy.loginfo(
                "[SIM-MAPPING] waypoint %d/%d target=(%.2f, %.2f, %.2f)",
                idx,
                len(route),
                target["x"],
                target["y"],
                target["yaw"],
            )
            while not rospy.is_shutdown():
                if rospy.Time.now().to_sec() >= deadline:
                    self._stop()
                    raise RuntimeError("mapping drive timeout at waypoint %d" % idx)
                if self.pose is None:
                    rate.sleep()
                    continue
                cmd = self._command_to(target)
                self.cmd_pub.publish(cmd)
                if self._reached(target):
                    self._stop()
                    rospy.sleep(0.4)
                    rospy.loginfo("[SIM-MAPPING] waypoint %d/%d reached", idx, len(route))
                    break
                rate.sleep()
        self._stop()
        self._publish_status("DONE", {"waypoints": len(route)})
        rospy.loginfo("[SIM-MAPPING] drive complete")

    def _command_to(self, target: Dict[str, float]) -> Twist:
        x, y, yaw = self.pose
        dx = target["x"] - x
        dy = target["y"] - y
        yaw_err = normalize(target["yaw"] - yaw)

        cmd = Twist()
        local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
        local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
        cmd.linear.x = self._clamp(local_x * self.k_xy, self.max_xy_speed)
        cmd.linear.y = self._clamp(local_y * self.k_xy, self.max_xy_speed)
        cmd.angular.z = self._clamp(yaw_err * self.k_yaw, self.max_yaw_speed)
        return cmd

    def _reached(self, target: Dict[str, float]) -> bool:
        x, y, yaw = self.pose
        dist = math.hypot(target["x"] - x, target["y"] - y)
        yaw_err = abs(normalize(target["yaw"] - yaw))
        return dist <= self.xy_tolerance and yaw_err <= self.yaw_tolerance

    @staticmethod
    def _clamp(value: float, limit: float) -> float:
        return max(-limit, min(limit, value))


if __name__ == "__main__":
    rospy.init_node("sim_mapping_drive")
    try:
        SimMappingDrive().run()
    except Exception as exc:
        rospy.logerr("[SIM-MAPPING] drive failed: %s", str(exc))
        raise
