#!/usr/bin/env python3

import math
import os
from datetime import datetime

import rospy
import tf
import yaml
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class AbcZoneCaptureNode:
    def __init__(self):
        defaults = rospy.get_param("/warehouse_tuning/abc_zone_capture", {})
        self.zone_name = rospy.get_param("~zone_name", defaults.get("zone_name", "A"))
        self.zone_role = rospy.get_param("~zone_role", defaults.get("zone_role", "source"))
        self.color = rospy.get_param("~color", defaults.get("color", "green"))
        self.output_file = rospy.get_param(
            "~output_file",
            defaults.get("output_file", "/tmp/warehouse_abc_zones.yaml"),
        )
        self.odom_topic = rospy.get_param("~odom_topic", defaults.get("odom_topic", "/odom"))
        self.pose_source = rospy.get_param("~pose_source", defaults.get("pose_source", "tf"))
        self.map_frame = rospy.get_param("~map_frame", defaults.get("map_frame", "map"))
        self.base_frame = rospy.get_param("~base_frame", defaults.get("base_frame", "base_footprint"))
        self.gazebo_model_name = rospy.get_param("~gazebo_model_name", defaults.get("gazebo_model_name", "wpb_home"))
        self.gazebo_reference_frame = rospy.get_param("~gazebo_reference_frame", defaults.get("gazebo_reference_frame", "world"))
        self.wait_timeout = float(rospy.get_param("~wait_timeout", defaults.get("wait_timeout", 5.0)))
        self.service_name = rospy.get_param(
            "~service_name",
            defaults.get("service_name", "/warehouse_tuning/capture_abc_zone"),
        )
        self.status_topic = rospy.get_param(
            "~status_topic",
            defaults.get("status_topic", "/warehouse_tuning/abc_zone_status"),
        )
        self.stack_anchor_forward_offset = float(
            rospy.get_param("~stack_anchor_forward_offset", defaults.get("stack_anchor_forward_offset", 0.56))
        )
        self.stack_anchor_side_offset = float(
            rospy.get_param("~stack_anchor_side_offset", defaults.get("stack_anchor_side_offset", 0.0))
        )
        self.table_height = float(rospy.get_param("~table_height", defaults.get("table_height", 0.0)))
        self.pose = None
        self.tf_listener = tf.TransformListener()
        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=5, latch=True)
        rospy.Subscriber(self.odom_topic, Odometry, self._on_odom, queue_size=1)
        rospy.Service(self.service_name, Trigger, self.handle_capture)
        self._status(
            "ready zone=%s role=%s color=%s pose_source=%s output=%s service=%s"
            % (self.zone_name, self.zone_role, self.color, self.pose_source, self.output_file, self.service_name)
        )

    def _on_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self.pose = {
            "x": float(msg.pose.pose.position.x),
            "y": float(msg.pose.pose.position.y),
            "yaw": float(yaw),
        }

    def handle_capture(self, _request):
        try:
            pose = self._latest_pose()
            summary = self._write_pose(pose)
        except Exception as exc:
            message = "capture failed: %s" % exc
            self._status(message)
            return TriggerResponse(False, message)
        message = "captured zone=%s role=%s %s to %s" % (
            self.zone_name,
            self.zone_role,
            summary,
            self.output_file,
        )
        self._status(message)
        return TriggerResponse(True, message)

    def _latest_pose(self):
        if str(self.pose_source).lower() in ("gazebo", "world"):
            return self._latest_gazebo_pose()

        if str(self.pose_source).lower() in ("tf", "map"):
            pose = self._latest_tf_pose()
            if pose is not None:
                return pose
            if str(self.pose_source).lower() == "tf":
                self._status("tf pose unavailable, falling back to odom")

        if self.pose is None:
            msg = rospy.wait_for_message(self.odom_topic, Odometry, timeout=self.wait_timeout)
            self._on_odom(msg)
        if self.pose is None:
            raise RuntimeError("no odom pose on %s" % self.odom_topic)
        return dict(self.pose)

    def _latest_gazebo_pose(self):
        try:
            from gazebo_msgs.srv import GetModelState
        except ImportError as exc:
            raise RuntimeError("pose_source=gazebo requires gazebo_msgs; use pose_source=tf on the real robot") from exc
        rospy.wait_for_service("/gazebo/get_model_state", timeout=self.wait_timeout)
        result = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)(
            self.gazebo_model_name,
            self.gazebo_reference_frame,
        )
        if not result.success:
            raise RuntimeError("gazebo model pose unavailable: %s" % self.gazebo_model_name)
        yaw = yaw_from_quaternion(result.pose.orientation)
        return {
            "x": float(result.pose.position.x),
            "y": float(result.pose.position.y),
            "yaw": float(yaw),
        }

    def _latest_tf_pose(self):
        try:
            self.tf_listener.waitForTransform(
                self.map_frame,
                self.base_frame,
                rospy.Time(0),
                rospy.Duration(self.wait_timeout),
            )
            trans, rot = self.tf_listener.lookupTransform(self.map_frame, self.base_frame, rospy.Time(0))
        except Exception as exc:
            rospy.logwarn("[abc_zone] tf lookup %s -> %s failed: %s", self.map_frame, self.base_frame, exc)
            return None
        yaw = tf.transformations.euler_from_quaternion(rot)[2]
        return {
            "x": float(trans[0]),
            "y": float(trans[1]),
            "yaw": float(yaw),
        }

    def _write_pose(self, pose):
        path = os.path.abspath(os.path.expanduser(self.output_file))
        directory = os.path.dirname(path)
        if directory and not os.path.isdir(directory):
            os.makedirs(directory)

        data = {}
        if os.path.exists(path):
            with open(path, "r", encoding="utf-8") as stream:
                data = yaml.safe_load(stream) or {}

        tuning = data.setdefault("warehouse_tuning", {})
        zones = tuning.setdefault("abc_zones", {})
        zones[str(self.zone_name)] = {
            "role": str(self.zone_role),
            "color": str(self.color) if self._is_drop_role() else "",
            "frame": self._capture_frame(),
            "x": round(pose["x"], 4),
            "y": round(pose["y"], 4),
            "yaw": round(pose["yaw"], 4),
            "captured_at": datetime.now().isoformat(),
        }

        stack = data.setdefault("stack_sort_pipeline", {})
        if self.table_height > 0.0:
            stack.setdefault("field_dimensions", {})["table_height"] = round(self.table_height, 3)
        anchor = None
        if self._is_source_role():
            stack["tabletop_return_base_target"] = self._round_pose(pose)
        elif self._is_drop_role():
            color = str(self.color)
            stack.setdefault("tabletop_drop_base_targets", {})[color] = self._round_pose(pose)
            anchor = self._stack_anchor_from_base(pose)
            stack.setdefault("tabletop_stack_anchors", {})[color] = self._round_pose(anchor)
        else:
            raise RuntimeError("zone_role must be source or drop")

        with open(path, "w", encoding="utf-8") as stream:
            yaml.safe_dump(data, stream, default_flow_style=False, sort_keys=False)
        pose_summary = "pose=(%.3f,%.3f,%.3f)" % (pose["x"], pose["y"], pose["yaw"])
        if anchor is not None:
            pose_summary += " stack_anchor=(%.3f,%.3f,%.3f)" % (
                anchor["x"],
                anchor["y"],
                anchor["yaw"],
            )
        return pose_summary

    def _stack_anchor_from_base(self, pose):
        yaw = pose["yaw"]
        forward = self.stack_anchor_forward_offset
        side = self.stack_anchor_side_offset
        return {
            "x": pose["x"] + math.cos(yaw) * forward - math.sin(yaw) * side,
            "y": pose["y"] + math.sin(yaw) * forward + math.cos(yaw) * side,
            "yaw": yaw,
        }

    def _round_pose(self, pose):
        return {
            "x": round(float(pose["x"]), 4),
            "y": round(float(pose["y"]), 4),
            "yaw": round(float(pose["yaw"]), 4),
        }

    def _capture_frame(self):
        source = str(self.pose_source).lower()
        if source in ("tf", "map"):
            return self.map_frame
        if source in ("gazebo", "world"):
            return self.gazebo_reference_frame
        return "odom"

    def _is_source_role(self):
        return str(self.zone_role).lower() in ("source", "pickup", "pick", "a")

    def _is_drop_role(self):
        return str(self.zone_role).lower() in ("drop", "destination", "stack", "b", "c")

    def _status(self, message):
        rospy.loginfo("[abc_zone] %s", message)
        self.status_pub.publish(String(data=message))


if __name__ == "__main__":
    rospy.init_node("abc_zone_capture")
    AbcZoneCaptureNode()
    rospy.spin()
