#!/usr/bin/env python3

import math
import os
import select
import sys
from typing import Dict, List, Optional

import rospy
import tf
import yaml
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty, Trigger, TriggerResponse
from visualization_msgs.msg import Marker, MarkerArray


class FieldNavSmoke:
    def __init__(self):
        self.zone_file = os.path.abspath(os.path.expanduser(rospy.get_param("~zone_file", "$HOME/maps/abc_zones.yaml")))
        self.sequence = self._parse_sequence(rospy.get_param("~sequence", "A,B,C,A"))
        self.map_frame = str(rospy.get_param("~map_frame", "map"))
        self.base_frame = str(rospy.get_param("~base_frame", "base_footprint"))
        self.map_topic = str(rospy.get_param("~map_topic", "/map"))
        self.scan_topic = str(rospy.get_param("~scan_topic", "/scan"))
        self.odom_topic = str(rospy.get_param("~odom_topic", "/odom"))
        self.amcl_topic = str(rospy.get_param("~amcl_topic", "/amcl_pose"))
        self.cmd_topic = str(rospy.get_param("~cmd_topic", "/cmd_vel"))
        self.marker_topic = str(rospy.get_param("~marker_topic", "/field_nav_smoke/markers"))
        self.planned_path_topic = str(rospy.get_param("~planned_path_topic", "/field_nav_smoke/planned_path"))
        self.actual_path_topic = str(rospy.get_param("~actual_path_topic", "/field_nav_smoke/actual_path"))
        self.dry_run = bool(rospy.get_param("~dry_run", False))
        self.localization_mode = str(rospy.get_param("~localization_mode", "manual")).lower()
        self.set_initial_pose = bool(rospy.get_param("~set_initial_pose", False))
        self.initial_pose_zone = str(rospy.get_param("~initial_pose_zone", self.sequence[0] if self.sequence else "A"))
        self.initial_pose_repeat = int(rospy.get_param("~initial_pose_repeat", 12))
        self.initial_pose_rate = float(rospy.get_param("~initial_pose_rate", 4.0))
        self.startup_timeout = float(rospy.get_param("~startup_timeout", 15.0))
        self.localization_timeout = float(rospy.get_param("~localization_timeout", 120.0))
        self.manual_assist_after = float(rospy.get_param("~manual_assist_after", 15.0))
        self.confirm_before_cruise = bool(rospy.get_param("~confirm_before_cruise", True))
        self.amcl_xy_cov_threshold = float(rospy.get_param("~amcl_xy_cov_threshold", 0.25))
        self.amcl_yaw_cov_threshold = float(rospy.get_param("~amcl_yaw_cov_threshold", 0.25))
        self.localization_stable_samples = int(rospy.get_param("~localization_stable_samples", 3))
        self.goal_timeout = float(rospy.get_param("~goal_timeout", 120.0))
        self.dist_tolerance = float(rospy.get_param("~dist_tolerance", 0.12))
        self.yaw_tolerance = float(rospy.get_param("~yaw_tolerance", 0.20))
        self.final_yaw_distance = float(rospy.get_param("~final_yaw_distance", 0.30))
        self.drive_mode = str(rospy.get_param("~drive_mode", "forward")).lower()
        self.path_yaw_tolerance = float(rospy.get_param("~path_yaw_tolerance", 0.22))
        self.placeholder_epsilon = float(rospy.get_param("~placeholder_epsilon", 0.05))
        self.allow_placeholder_pose = bool(rospy.get_param("~allow_placeholder_pose", False))
        self.max_linear = float(rospy.get_param("~max_linear", 0.25))
        self.max_angular = float(rospy.get_param("~max_angular", 0.60))
        self.min_linear = float(rospy.get_param("~min_linear", 0.08))
        self.min_angular = float(rospy.get_param("~min_angular", 0.12))
        self.linear_kp = float(rospy.get_param("~linear_kp", 0.7))
        self.angular_kp = float(rospy.get_param("~angular_kp", 1.2))
        self.control_hz = float(rospy.get_param("~control_hz", 20.0))
        self.log_interval = float(rospy.get_param("~log_interval", 1.0))
        self.path_min_distance = float(rospy.get_param("~path_min_distance", 0.03))

        self.tf_listener = tf.TransformListener()
        self.cmd_pub = rospy.Publisher(self.cmd_topic, Twist, queue_size=5)
        self.initial_pose_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1, latch=True)
        self.marker_pub = rospy.Publisher(self.marker_topic, MarkerArray, queue_size=1, latch=True)
        self.planned_path_pub = rospy.Publisher(self.planned_path_topic, Path, queue_size=1, latch=True)
        self.actual_path_pub = rospy.Publisher(self.actual_path_topic, Path, queue_size=1, latch=True)
        self.amcl_pose: Optional[PoseWithCovarianceStamped] = None
        self.actual_path = Path()
        self.actual_path.header.frame_id = self.map_frame
        self.last_actual_path_pose = None
        self.confirmed = False
        self.manual_initial_pose_received = False
        self.manual_initial_pose: Optional[PoseWithCovarianceStamped] = None
        rospy.Subscriber(self.amcl_topic, PoseWithCovarianceStamped, self._on_amcl_pose, queue_size=1)
        rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self._on_initial_pose, queue_size=1)
        rospy.Service("~confirm_localized", Trigger, self._handle_confirm_localized)
        rospy.loginfo(
            "[field_nav_smoke] drive_mode=%s max_linear=%.3f min_linear=%.3f max_angular=%.3f min_angular=%.3f",
            self.drive_mode,
            self.max_linear,
            self.min_linear,
            self.max_angular,
            self.min_angular,
        )

    def run(self) -> int:
        try:
            zones = self._load_zones()
            self._validate_sequence(zones)
            self._publish_route_visuals(zones)
            self._wait_for_inputs()
            if not self._localize(zones):
                return 2
            self._reset_actual_path()
            self._append_current_pose_to_path(force=True)
            if self.dry_run:
                self._dry_run_report(zones)
                return 0
            if self.confirm_before_cruise and not self._wait_for_user_confirmation():
                return 2
            for zone_name in self.sequence:
                if not self._drive_to_zone(zone_name, zones[zone_name]):
                    self._stop()
                    rospy.logerr("[field_nav_smoke] failed at zone=%s", zone_name)
                    return 2
            self._stop()
            rospy.loginfo("[field_nav_smoke] sequence complete: %s", ",".join(self.sequence))
            return 0
        except Exception as exc:
            self._stop()
            rospy.logerr("[field_nav_smoke] %s", exc)
            return 1

    def _parse_sequence(self, raw) -> List[str]:
        if isinstance(raw, list):
            seq = [str(item).strip() for item in raw]
        else:
            seq = [part.strip() for part in str(raw).split(",")]
        seq = [item for item in seq if item]
        if not seq:
            raise RuntimeError("sequence is empty")
        return seq

    def _load_zones(self) -> Dict[str, Dict[str, float]]:
        with open(self.zone_file, "r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream) or {}
        raw_zones = data.get("warehouse_tuning", {}).get("abc_zones", {})
        if not isinstance(raw_zones, dict):
            raise RuntimeError("warehouse_tuning/abc_zones missing in %s" % self.zone_file)
        zones = {}
        for name, raw in raw_zones.items():
            if not isinstance(raw, dict):
                continue
            try:
                zones[str(name)] = {
                    "x": float(raw["x"]),
                    "y": float(raw["y"]),
                    "yaw": float(raw["yaw"]),
                }
            except (KeyError, TypeError, ValueError):
                rospy.logwarn("[field_nav_smoke] ignoring invalid zone=%s", name)
        if not zones:
            raise RuntimeError("no valid zones in %s" % self.zone_file)
        rospy.loginfo("[field_nav_smoke] loaded zones from %s: %s", self.zone_file, ",".join(sorted(zones.keys())))
        return zones

    def _validate_sequence(self, zones: Dict[str, Dict[str, float]]):
        missing = [name for name in self.sequence if name not in zones]
        if self.initial_pose_zone not in zones:
            missing.append(self.initial_pose_zone)
        if missing:
            raise RuntimeError("missing zone(s) in %s: %s" % (self.zone_file, ",".join(sorted(set(missing)))))
        if self.allow_placeholder_pose:
            return
        for name in sorted(set(self.sequence + [self.initial_pose_zone])):
            pose = zones[name]
            if (
                abs(pose["x"]) < self.placeholder_epsilon
                and abs(pose["y"]) < self.placeholder_epsilon
                and abs(pose["yaw"]) < self.placeholder_epsilon
            ):
                raise RuntimeError(
                    "zone %s looks like placeholder pose (%.3f, %.3f, %.3f); recapture A/B/C or set allow_placeholder_pose:=true"
                    % (name, pose["x"], pose["y"], pose["yaw"])
                )

    def _wait_for_inputs(self):
        rospy.loginfo("[field_nav_smoke] waiting for map/scan/odom")
        rospy.wait_for_message(self.map_topic, OccupancyGrid, timeout=self.startup_timeout)
        rospy.wait_for_message(self.scan_topic, LaserScan, timeout=self.startup_timeout)
        rospy.wait_for_message(self.odom_topic, Odometry, timeout=self.startup_timeout)
        rospy.loginfo("[field_nav_smoke] map/scan/odom ready")

    def _localize(self, zones: Dict[str, Dict[str, float]]) -> bool:
        mode = "zone" if self.set_initial_pose else self.localization_mode
        if mode not in ("global", "zone", "manual", "none"):
            raise RuntimeError("unsupported localization_mode=%s" % mode)
        if mode == "zone":
            self._publish_initial_pose(zones[self.initial_pose_zone])
        elif mode == "manual":
            if not self._wait_for_manual_initial_pose():
                return False
        elif mode == "global":
            self._request_global_localization()
        self._wait_for_localization_tf()
        return self._wait_for_localization_convergence(mode)

    def _wait_for_manual_initial_pose(self) -> bool:
        rospy.logwarn(
            "[field_nav_smoke] localization_mode=manual. Set the real robot pose in RViz with 2D Pose Estimate; "
            "the cruise will not continue until /initialpose is received."
        )
        deadline = rospy.Time.now().to_sec() + self.localization_timeout
        next_log = 0.0
        while not rospy.is_shutdown() and rospy.Time.now().to_sec() < deadline:
            if self.manual_initial_pose_received and self.manual_initial_pose is not None:
                pose = self.manual_initial_pose.pose.pose
                yaw = tf.transformations.euler_from_quaternion(
                    [
                        pose.orientation.x,
                        pose.orientation.y,
                        pose.orientation.z,
                        pose.orientation.w,
                    ]
                )[2]
                rospy.loginfo(
                    "[field_nav_smoke] received manual initial pose=(%.3f, %.3f, %.3f)",
                    pose.position.x,
                    pose.position.y,
                    yaw,
                )
                rospy.sleep(0.5)
                self._request_nomotion_update()
                return True
            now = rospy.Time.now().to_sec()
            if now >= next_log:
                rospy.logwarn("[field_nav_smoke] waiting for RViz 2D Pose Estimate on /initialpose")
                next_log = now + max(0.5, self.log_interval)
            rospy.sleep(0.2)
        rospy.logerr("[field_nav_smoke] no manual initial pose received within %.1fs", self.localization_timeout)
        return False

    def _wait_for_localization_tf(self):
        rospy.loginfo("[field_nav_smoke] waiting for amcl/tf")
        rospy.wait_for_message(self.amcl_topic, PoseWithCovarianceStamped, timeout=self.startup_timeout)
        self.tf_listener.waitForTransform(
            self.map_frame,
            self.base_frame,
            rospy.Time(0),
            rospy.Duration(self.startup_timeout),
        )
        rospy.loginfo("[field_nav_smoke] localization stack ready")

    def _request_global_localization(self):
        service_name = "/global_localization"
        try:
            rospy.wait_for_service(service_name, timeout=self.startup_timeout)
            rospy.ServiceProxy(service_name, Empty)()
            rospy.loginfo("[field_nav_smoke] requested AMCL global localization")
        except Exception as exc:
            rospy.logwarn("[field_nav_smoke] global localization service unavailable: %s", exc)
        self._request_nomotion_update()

    def _request_nomotion_update(self):
        service_name = "/request_nomotion_update"
        try:
            rospy.wait_for_service(service_name, timeout=2.0)
            rospy.ServiceProxy(service_name, Empty)()
        except Exception:
            pass

    def _wait_for_localization_convergence(self, mode: str) -> bool:
        if mode == "none":
            pose = self._current_pose()
            rospy.loginfo(
                "[field_nav_smoke] using existing localization current=(%.3f, %.3f, %.3f)",
                pose["x"],
                pose["y"],
                pose["yaw"],
            )
            return True

        deadline = rospy.Time.now().to_sec() + self.localization_timeout
        assist_at = rospy.Time.now().to_sec() + self.manual_assist_after
        next_log = 0.0
        stable_count = 0
        while not rospy.is_shutdown() and rospy.Time.now().to_sec() < deadline:
            cov = self._amcl_covariance()
            pose = self._safe_current_pose()
            if cov is not None:
                cov_ok = cov["x"] <= self.amcl_xy_cov_threshold and cov["y"] <= self.amcl_xy_cov_threshold and cov["yaw"] <= self.amcl_yaw_cov_threshold
                stable_count = stable_count + 1 if cov_ok and pose is not None else 0
                if stable_count >= max(1, self.localization_stable_samples):
                    rospy.loginfo(
                        "[field_nav_smoke] localized pose=(%.3f, %.3f, %.3f) cov=(%.4f, %.4f, %.4f)",
                        pose["x"],
                        pose["y"],
                        pose["yaw"],
                        cov["x"],
                        cov["y"],
                        cov["yaw"],
                    )
                    return True
            now = rospy.Time.now().to_sec()
            if now >= next_log:
                if cov is None:
                    rospy.loginfo("[field_nav_smoke] waiting for AMCL covariance")
                else:
                    rospy.loginfo(
                        "[field_nav_smoke] localization cov=(%.4f, %.4f, %.4f) stable=%d/%d threshold=(%.3f, %.3f)",
                        cov["x"],
                        cov["y"],
                        cov["yaw"],
                        stable_count,
                        max(1, self.localization_stable_samples),
                        self.amcl_xy_cov_threshold,
                        self.amcl_yaw_cov_threshold,
                    )
                if mode == "global" and now >= assist_at:
                    rospy.logwarn(
                        "[field_nav_smoke] localization not converged yet. Use joystick to move/rotate the robot slowly, then wait for convergence."
                    )
                next_log = now + max(0.5, self.log_interval)
                self._request_nomotion_update()
            rospy.sleep(0.2)
        rospy.logerr("[field_nav_smoke] localization did not converge within %.1fs", self.localization_timeout)
        return False

    def _publish_initial_pose(self, pose: Dict[str, float]):
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
        rate = rospy.Rate(max(0.1, self.initial_pose_rate))
        rospy.loginfo(
            "[field_nav_smoke] publishing initial pose zone=%s pose=(%.3f, %.3f, %.3f)",
            self.initial_pose_zone,
            pose["x"],
            pose["y"],
            pose["yaw"],
        )
        for _ in range(max(1, self.initial_pose_repeat)):
            msg.header.stamp = rospy.Time.now()
            self.initial_pose_pub.publish(msg)
            rate.sleep()
        rospy.sleep(0.5)

    def _on_amcl_pose(self, msg):
        self.amcl_pose = msg

    def _on_initial_pose(self, msg):
        self.manual_initial_pose_received = True
        self.manual_initial_pose = msg

    def _handle_confirm_localized(self, _request):
        self.confirmed = True
        return TriggerResponse(True, "confirmed")

    def _amcl_covariance(self):
        if self.amcl_pose is None:
            return None
        cov = self.amcl_pose.pose.covariance
        return {"x": float(cov[0]), "y": float(cov[7]), "yaw": float(cov[35])}

    def _safe_current_pose(self):
        try:
            return self._current_pose()
        except Exception:
            return None

    def _wait_for_user_confirmation(self) -> bool:
        pose = self._current_pose()
        rospy.loginfo(
            "[field_nav_smoke] localization ready current=(%.3f, %.3f, %.3f)",
            pose["x"],
            pose["y"],
            pose["yaw"],
        )
        rospy.logwarn(
            "[field_nav_smoke] Check RViz/localization. Press Enter in this terminal to start ABC cruise, or call: rosservice call /field_nav_smoke/confirm_localized"
        )
        if sys.stdin is not None and sys.stdin.isatty():
            while not rospy.is_shutdown() and not self.confirmed:
                readable, _, _ = select.select([sys.stdin], [], [], 0.2)
                if readable:
                    sys.stdin.readline()
                    self.confirmed = True
                    break
            return self.confirmed
        while not rospy.is_shutdown() and not self.confirmed:
            rospy.sleep(0.2)
        return self.confirmed

    def _dry_run_report(self, zones: Dict[str, Dict[str, float]]):
        pose = self._current_pose()
        rospy.loginfo(
            "[field_nav_smoke] dry_run current=(%.3f, %.3f, %.3f)",
            pose["x"],
            pose["y"],
            pose["yaw"],
        )
        for name in self.sequence:
            target = zones[name]
            dist, yaw_err = self._pose_error(pose, target)
            rospy.loginfo(
                "[field_nav_smoke] dry_run target=%s pose=(%.3f, %.3f, %.3f) dist=%.3f yaw_err=%.3f",
                name,
                target["x"],
                target["y"],
                target["yaw"],
                dist,
                yaw_err,
            )

    def _drive_to_zone(self, zone_name: str, target: Dict[str, float]) -> bool:
        rospy.loginfo(
            "[field_nav_smoke] drive target=%s pose=(%.3f, %.3f, %.3f)",
            zone_name,
            target["x"],
            target["y"],
            target["yaw"],
        )
        rate = rospy.Rate(max(1.0, self.control_hz))
        deadline = rospy.Time.now().to_sec() + self.goal_timeout
        next_log = 0.0
        final_dist = 999.0
        final_yaw_err = 999.0
        while not rospy.is_shutdown():
            pose = self._current_pose()
            self._append_pose_to_path(pose)
            final_dist, final_yaw_err = self._pose_error(pose, target)
            if final_dist <= self.dist_tolerance and abs(final_yaw_err) <= self.yaw_tolerance:
                self._stop()
                rospy.loginfo(
                    "[field_nav_smoke] reached target=%s dist=%.3f yaw_err=%.3f",
                    zone_name,
                    final_dist,
                    final_yaw_err,
                )
                return True
            if rospy.Time.now().to_sec() > deadline:
                self._stop()
                rospy.logwarn(
                    "[field_nav_smoke] timeout target=%s dist=%.3f yaw_err=%.3f",
                    zone_name,
                    final_dist,
                    final_yaw_err,
                )
                return False
            cmd = self._twist_to_target(pose, target, final_dist, final_yaw_err)
            self.cmd_pub.publish(cmd)
            now = rospy.Time.now().to_sec()
            if now >= next_log:
                rospy.loginfo(
                    "[field_nav_smoke] target=%s current=(%.3f, %.3f, %.3f) dist=%.3f yaw_err=%.3f cmd=(%.3f, %.3f, %.3f)",
                    zone_name,
                    pose["x"],
                    pose["y"],
                    pose["yaw"],
                    final_dist,
                    final_yaw_err,
                    cmd.linear.x,
                    cmd.linear.y,
                    cmd.angular.z,
                )
                next_log = now + max(0.2, self.log_interval)
            rate.sleep()
        self._stop()
        return False

    def _current_pose(self) -> Dict[str, float]:
        trans, rot = self.tf_listener.lookupTransform(self.map_frame, self.base_frame, rospy.Time(0))
        yaw = tf.transformations.euler_from_quaternion(rot)[2]
        return {"x": float(trans[0]), "y": float(trans[1]), "yaw": float(yaw)}

    def _publish_route_visuals(self, zones: Dict[str, Dict[str, float]]):
        now = rospy.Time.now()
        markers = MarkerArray()
        markers.markers.append(self._delete_all_marker(now))

        marker_id = 1
        for name in sorted(zones.keys()):
            pose = zones[name]
            color = self._zone_color(name)
            markers.markers.append(self._zone_sphere_marker(marker_id, name, pose, color, now))
            marker_id += 1
            markers.markers.append(self._zone_text_marker(marker_id, name, pose, color, now))
            marker_id += 1
            markers.markers.append(self._zone_heading_marker(marker_id, name, pose, color, now))
            marker_id += 1

        markers.markers.append(self._planned_route_marker(marker_id, zones, now))
        self.marker_pub.publish(markers)
        self._publish_planned_path(zones, now)

    def _delete_all_marker(self, stamp):
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = stamp
        marker.action = Marker.DELETEALL
        return marker

    def _base_marker(self, marker_id: int, namespace: str, stamp):
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = stamp
        marker.ns = namespace
        marker.id = marker_id
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    def _zone_sphere_marker(self, marker_id: int, name: str, pose: Dict[str, float], color, stamp):
        marker = self._base_marker(marker_id, "abc_zones", stamp)
        marker.type = Marker.SPHERE
        marker.pose.position.x = pose["x"]
        marker.pose.position.y = pose["y"]
        marker.pose.position.z = 0.07
        marker.scale.x = 0.22
        marker.scale.y = 0.22
        marker.scale.z = 0.12
        self._set_color(marker, color, 0.95)
        return marker

    def _zone_text_marker(self, marker_id: int, name: str, pose: Dict[str, float], color, stamp):
        marker = self._base_marker(marker_id, "abc_labels", stamp)
        marker.type = Marker.TEXT_VIEW_FACING
        marker.text = name
        marker.pose.position.x = pose["x"]
        marker.pose.position.y = pose["y"]
        marker.pose.position.z = 0.42
        marker.scale.z = 0.32
        self._set_color(marker, color, 1.0)
        return marker

    def _zone_heading_marker(self, marker_id: int, name: str, pose: Dict[str, float], color, stamp):
        marker = self._base_marker(marker_id, "abc_heading", stamp)
        marker.type = Marker.ARROW
        marker.pose.position.x = pose["x"]
        marker.pose.position.y = pose["y"]
        marker.pose.position.z = 0.12
        quat = tf.transformations.quaternion_from_euler(0.0, 0.0, pose["yaw"])
        marker.pose.orientation.x = quat[0]
        marker.pose.orientation.y = quat[1]
        marker.pose.orientation.z = quat[2]
        marker.pose.orientation.w = quat[3]
        marker.scale.x = 0.45
        marker.scale.y = 0.07
        marker.scale.z = 0.07
        self._set_color(marker, color, 0.85)
        return marker

    def _planned_route_marker(self, marker_id: int, zones: Dict[str, Dict[str, float]], stamp):
        marker = self._base_marker(marker_id, "planned_route", stamp)
        marker.type = Marker.LINE_STRIP
        marker.scale.x = 0.05
        self._set_color(marker, (1.0, 0.70, 0.05), 0.95)
        for name in self.sequence:
            pose = zones[name]
            point = Point()
            point.x = pose["x"]
            point.y = pose["y"]
            point.z = 0.09
            marker.points.append(point)
        return marker

    def _publish_planned_path(self, zones: Dict[str, Dict[str, float]], stamp):
        path = Path()
        path.header.frame_id = self.map_frame
        path.header.stamp = stamp
        for name in self.sequence:
            path.poses.append(self._pose_stamped_from_pose(zones[name], stamp))
        self.planned_path_pub.publish(path)

    def _reset_actual_path(self):
        self.actual_path = Path()
        self.actual_path.header.frame_id = self.map_frame
        self.actual_path.header.stamp = rospy.Time.now()
        self.last_actual_path_pose = None
        self.actual_path_pub.publish(self.actual_path)

    def _append_current_pose_to_path(self, force=False):
        try:
            self._append_pose_to_path(self._current_pose(), force)
        except Exception:
            pass

    def _append_pose_to_path(self, pose: Dict[str, float], force=False):
        if self.last_actual_path_pose is not None and not force:
            dist = math.hypot(pose["x"] - self.last_actual_path_pose["x"], pose["y"] - self.last_actual_path_pose["y"])
            if dist < self.path_min_distance:
                return
        stamp = rospy.Time.now()
        self.actual_path.header.stamp = stamp
        self.actual_path.poses.append(self._pose_stamped_from_pose(pose, stamp))
        self.last_actual_path_pose = dict(pose)
        self.actual_path_pub.publish(self.actual_path)

    def _pose_stamped_from_pose(self, pose: Dict[str, float], stamp):
        msg = PoseStamped()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = stamp
        msg.pose.position.x = pose["x"]
        msg.pose.position.y = pose["y"]
        quat = tf.transformations.quaternion_from_euler(0.0, 0.0, pose["yaw"])
        msg.pose.orientation.x = quat[0]
        msg.pose.orientation.y = quat[1]
        msg.pose.orientation.z = quat[2]
        msg.pose.orientation.w = quat[3]
        return msg

    def _zone_color(self, name: str):
        colors = {
            "A": (0.10, 0.55, 1.0),
            "B": (0.10, 0.80, 0.35),
            "C": (1.0, 0.25, 0.20),
        }
        return colors.get(name, (0.85, 0.85, 0.85))

    def _set_color(self, marker, color, alpha):
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = alpha

    def _pose_error(self, pose: Dict[str, float], target: Dict[str, float]):
        dist = math.hypot(target["x"] - pose["x"], target["y"] - pose["y"])
        yaw_err = self._normalize(target["yaw"] - pose["yaw"])
        return dist, yaw_err

    def _twist_to_target(self, pose: Dict[str, float], target: Dict[str, float], dist: float, yaw_err: float):
        if self.drive_mode == "holonomic":
            return self._holonomic_twist_to_target(pose, target, dist, yaw_err)
        return self._forward_twist_to_target(pose, target, dist, yaw_err)

    def _forward_twist_to_target(self, pose: Dict[str, float], target: Dict[str, float], dist: float, yaw_err: float):
        cmd = Twist()
        if dist > self.dist_tolerance:
            dx = target["x"] - pose["x"]
            dy = target["y"] - pose["y"]
            path_yaw = math.atan2(dy, dx)
            path_yaw_err = self._normalize(path_yaw - pose["yaw"])
            if abs(path_yaw_err) > self.path_yaw_tolerance:
                yaw_speed = min(self.max_angular, max(self.min_angular, self.angular_kp * abs(path_yaw_err)))
                cmd.angular.z = yaw_speed if path_yaw_err >= 0.0 else -yaw_speed
            if abs(path_yaw_err) < 1.10:
                speed = min(self.max_linear, max(self.min_linear, self.linear_kp * dist))
                cmd.linear.x = speed * max(0.35, math.cos(abs(path_yaw_err)))
            return cmd
        if abs(yaw_err) > self.yaw_tolerance:
            yaw_speed = min(self.max_angular, max(self.min_angular, self.angular_kp * abs(yaw_err)))
            cmd.angular.z = yaw_speed if yaw_err >= 0.0 else -yaw_speed
        return cmd

    def _holonomic_twist_to_target(self, pose: Dict[str, float], target: Dict[str, float], dist: float, yaw_err: float):
        dx = target["x"] - pose["x"]
        dy = target["y"] - pose["y"]
        body_x = math.cos(pose["yaw"]) * dx + math.sin(pose["yaw"]) * dy
        body_y = -math.sin(pose["yaw"]) * dx + math.cos(pose["yaw"]) * dy
        cmd = Twist()
        if dist > self.dist_tolerance:
            speed = min(self.max_linear, max(self.min_linear, self.linear_kp * dist))
            cmd.linear.x = body_x / max(dist, 1e-6) * speed
            cmd.linear.y = body_y / max(dist, 1e-6) * speed
        if dist <= self.final_yaw_distance and abs(yaw_err) > self.yaw_tolerance:
            yaw_speed = min(self.max_angular, max(self.min_angular, self.angular_kp * abs(yaw_err)))
            cmd.angular.z = yaw_speed if yaw_err >= 0.0 else -yaw_speed
        return cmd

    def _stop(self):
        if rospy.core.is_initialized():
            self.cmd_pub.publish(Twist())

    def _normalize(self, angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


if __name__ == "__main__":
    rospy.init_node("field_nav_smoke")
    raise SystemExit(FieldNavSmoke().run())
