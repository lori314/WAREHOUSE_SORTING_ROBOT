#!/usr/bin/env python3
import threading
import math
import json
import os
import csv
import select
import sys
from datetime import datetime
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import cv2
import numpy as np
import rospy
import tf
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import Image, JointState, LaserScan
from std_msgs.msg import String
from std_srvs.srv import Empty, Trigger, TriggerResponse
from visualization_msgs.msg import Marker, MarkerArray
from wpb_home_behaviors.msg import Coord


@dataclass
class Detection:
    color: str
    cx: int
    cy: int
    depth: float
    area: float


@dataclass
class PickSignature:
    cx: int
    cy: int
    depth: float
    area: float


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float


class FieldGeometry:
    def __init__(self):
        raw = rospy.get_param("~field_dimensions", {})
        if not isinstance(raw, dict):
            raw = {}
        box = raw.get("box_size", {})
        if not isinstance(box, dict):
            box = {}

        self.table_height = self._float(raw.get("table_height"), 0.75)
        self.box_x = self._float(box.get("x"), 0.10)
        self.box_y = self._float(box.get("y"), 0.10)
        self.box_z = self._float(box.get("z"), 0.10)
        self.pick_lift_clearance = self._float(raw.get("pick_lift_clearance"), -0.04)
        self.carry_lift_clearance = self._float(raw.get("carry_lift_clearance"), 0.18)
        self.drop_lift_clearance = self._float(raw.get("drop_lift_clearance"), 0.04)
        self.stack_lift_margin = self._float(raw.get("stack_lift_margin"), 0.025)
        self.gripper_open_clearance = self._float(raw.get("gripper_open_clearance"), 0.10)
        self.gripper_closed_clearance = self._float(raw.get("gripper_closed_clearance"), -0.005)
        self.source_pick_forward_offset = self._float(raw.get("source_pick_forward_offset"), 0.70)
        self.carry_forward_offset = self._float(raw.get("carry_forward_offset"), 0.64)
        self.model_tabletop_z_offset = self._float(raw.get("model_tabletop_z_offset"), 0.028)

    def _float(self, value, default: float) -> float:
        try:
            return float(value)
        except (TypeError, ValueError):
            return float(default)

    @property
    def pick_lift_height(self) -> float:
        return self.table_height + self.pick_lift_clearance

    @property
    def carry_lift_height(self) -> float:
        return self.table_height + self.carry_lift_clearance

    @property
    def base_drop_lift(self) -> float:
        return self.table_height + self.drop_lift_clearance

    @property
    def stack_lift_step(self) -> float:
        return self.box_z + self.stack_lift_margin

    @property
    def open_gripper(self) -> float:
        return max(0.0, max(self.box_x, self.box_y) + self.gripper_open_clearance)

    @property
    def closed_gripper(self) -> float:
        return max(0.0, max(self.box_x, self.box_y) + self.gripper_closed_clearance)

    @property
    def tabletop_model_z(self) -> float:
        return self.table_height + self.model_tabletop_z_offset

    def to_dict(self):
        return {
            "table_height": self.table_height,
            "box_size": {"x": self.box_x, "y": self.box_y, "z": self.box_z},
            "pick_lift_height_default": self.pick_lift_height,
            "carry_lift_height_default": self.carry_lift_height,
            "base_drop_lift_default": self.base_drop_lift,
            "stack_lift_step_default": self.stack_lift_step,
            "open_gripper_default": self.open_gripper,
            "closed_gripper_default": self.closed_gripper,
            "source_pick_forward_offset_default": self.source_pick_forward_offset,
            "carry_forward_offset_default": self.carry_forward_offset,
            "tabletop_model_z_default": self.tabletop_model_z,
        }


class RunMetrics:
    def __init__(self, colors=None):
        colors = [str(c) for c in (colors or ["green", "red"])]
        self.pick_attempts = 0
        self.pick_retries = 0
        self.success_count = 0
        self.fail_count = 0
        self.motion_segments = 0
        self.motion_failures = 0
        self.per_color_success = {color: 0 for color in colors}
        self.cycle_start = None
        self.current_color = None
        self.cycle_times = []
        self.cycle_records = []
        self.drop_poses = {color: [] for color in colors}
        self.anchor_errors = {color: [] for color in colors}

    def _ensure_color(self, color: str):
        self.per_color_success.setdefault(color, 0)
        self.drop_poses.setdefault(color, [])
        self.anchor_errors.setdefault(color, [])

    def start_cycle(self, color: str):
        self.current_color = color
        self.cycle_start = rospy.Time.now()

    def mark_pick_attempt(self):
        self.pick_attempts += 1

    def mark_retry(self):
        self.pick_retries += 1

    def add_motion_result(self, name: str, success: bool, final_error: float):
        self.motion_segments += 1
        if not success:
            self.motion_failures += 1
        rospy.loginfo(
            "[MOTION] name=%s success=%s error=%.3f"
            % (name, str(success), final_error)
        )

    def add_drop_pose(self, color: str, pose: Optional[Pose2D]):
        if pose is None:
            return
        self._ensure_color(color)
        self.drop_poses[color].append((pose.x, pose.y))

    def add_anchor_error(self, color: str, error: float):
        self._ensure_color(color)
        self.anchor_errors[color].append(error)

    def _stack_consistency(self) -> str:
        chunks = []
        for color, pts in self.drop_poses.items():
            if len(pts) < 2:
                chunks.append("%s:n/a" % color)
                continue
            xs = [p[0] for p in pts]
            ys = [p[1] for p in pts]
            cx = sum(xs) / float(len(xs))
            cy = sum(ys) / float(len(ys))
            radii = [math.hypot(x - cx, y - cy) for (x, y) in pts]
            mean_r = sum(radii) / float(len(radii))
            chunks.append("%s:%.03fm" % (color, mean_r))
        return ",".join(chunks)

    def finish_cycle(self, color: str, success: bool, reason: str):
        dt = 0.0
        if self.cycle_start is not None:
            dt = (rospy.Time.now() - self.cycle_start).to_sec()
            self.cycle_times.append(dt)
        if success:
            self.success_count += 1
            self._ensure_color(color)
            self.per_color_success[color] += 1
        else:
            self.fail_count += 1
        rospy.loginfo(
            "[METRICS] cycle color=%s success=%s reason=%s dt=%.2fs"
            % (color, str(success), reason, dt)
        )
        self.cycle_records.append(
            {
                "color": color,
                "success": bool(success),
                "reason": reason,
                "cycle_time_sec": dt,
                "retries_total": self.pick_retries,
                "timestamp": datetime.now().isoformat(),
            }
        )
        self.current_color = None
        self.cycle_start = None

    def _anchor_error_summary(self) -> str:
        chunks = []
        for color, vals in self.anchor_errors.items():
            if not vals:
                chunks.append("%s:n/a" % color)
                continue
            mean_v = sum(vals) / float(len(vals))
            chunks.append("%s:%.03fm" % (color, mean_v))
        return ",".join(chunks)

    def summary(self) -> str:
        avg_cycle = 0.0
        if self.cycle_times:
            avg_cycle = sum(self.cycle_times) / float(len(self.cycle_times))
        return (
            "attempts=%d retries=%d success=%d fail=%d motion_fail=%d/%d avg_cycle=%.2fs consistency=[%s] anchor_error=[%s] per_color=%s"
            % (
                self.pick_attempts,
                self.pick_retries,
                self.success_count,
                self.fail_count,
                self.motion_failures,
                self.motion_segments,
                avg_cycle,
                self._stack_consistency(),
                self._anchor_error_summary(),
                str(self.per_color_success),
            )
        )

    def to_dict(self):
        avg_cycle = 0.0
        if self.cycle_times:
            avg_cycle = sum(self.cycle_times) / float(len(self.cycle_times))
        return {
            "pick_attempts": self.pick_attempts,
            "pick_retries": self.pick_retries,
            "success_count": self.success_count,
            "fail_count": self.fail_count,
            "motion_segments": self.motion_segments,
            "motion_failures": self.motion_failures,
            "avg_cycle_sec": avg_cycle,
            "per_color_success": dict(self.per_color_success),
            "drop_poses": dict(self.drop_poses),
            "anchor_errors": dict(self.anchor_errors),
            "cycle_records": list(self.cycle_records),
            "summary": self.summary(),
        }


class ReportExporter:
    def __init__(self):
        self.enabled = rospy.get_param("~enable_report_export", True)
        self.output_dir = rospy.get_param("~report_output_dir", "/tmp/arm_grab_task_reports")
        self.file_prefix = rospy.get_param("~report_prefix", "stack_sort")

    def export(self, metrics: RunMetrics, stack_count: Dict[str, int], params: Dict[str, float]):
        if not self.enabled:
            return None

        os.makedirs(self.output_dir, exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        file_path = os.path.join(self.output_dir, "%s_%s.json" % (self.file_prefix, ts))

        payload = {
            "timestamp": datetime.now().isoformat(),
            "stack_count": dict(stack_count),
            "params": dict(params),
            "metrics": metrics.to_dict(),
        }

        with open(file_path, "w", encoding="utf-8") as f:
            json.dump(payload, f, ensure_ascii=False, indent=2)

        cycles_csv = file_path.replace(".json", "_cycles.csv")
        with open(cycles_csv, "w", encoding="utf-8", newline="") as f:
            writer = csv.DictWriter(
                f,
                fieldnames=["color", "success", "reason", "cycle_time_sec", "retries_total", "timestamp"],
            )
            writer.writeheader()
            for row in metrics.cycle_records:
                writer.writerow(row)

        drop_csv = file_path.replace(".json", "_drop_points.csv")
        with open(drop_csv, "w", encoding="utf-8", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=["color", "idx", "x", "y"])
            writer.writeheader()
            for color, pts in metrics.drop_poses.items():
                for idx, (x, y) in enumerate(pts):
                    writer.writerow({"color": color, "idx": idx, "x": x, "y": y})

        summary_txt = file_path.replace(".json", "_summary.txt")
        with open(summary_txt, "w", encoding="utf-8") as f:
            f.write(metrics.summary() + "\n")
            f.write("stack_count=" + str(stack_count) + "\n")
            f.write("params=" + str(params) + "\n")

        rospy.loginfo("[REPORT] exported %s" % file_path)
        rospy.loginfo("[REPORT] exported %s" % cycles_csv)
        rospy.loginfo("[REPORT] exported %s" % drop_csv)
        rospy.loginfo("[REPORT] exported %s" % summary_txt)
        return file_path


class ColorBoxPerception:
    def __init__(self):
        self.bridge = CvBridge()
        self.depth_image = None
        self.lock = threading.Lock()
        self.detections: Dict[str, Detection] = {}
        self.enabled = True
        self.enabled_since = rospy.Time.now().to_sec()
        self.last_rgb_received_at = 0.0
        self.last_depth_received_at = 0.0
        self.show_debug = rospy.get_param("~show_debug", True)

        self.min_area = rospy.get_param("~min_box_area", 120.0)
        self.max_area = rospy.get_param("~max_box_area", 500000.0)
        self.square_filter_enabled = bool(rospy.get_param("~square_filter_enabled", True))
        self.square_min_side_px = float(rospy.get_param("~square_min_side_px", 10.0))
        self.square_max_aspect_ratio = float(rospy.get_param("~square_max_aspect_ratio", 1.65))
        self.square_min_fill_ratio = float(rospy.get_param("~square_min_fill_ratio", 0.45))
        self.color_width = float(rospy.get_param("~color_image_width", 960.0))
        self.color_height = float(rospy.get_param("~color_image_height", 540.0))

        self.color_ranges = self._load_color_ranges()
        rospy.loginfo("[VISION] color_ranges=%s" % ",".join(sorted(self.color_ranges.keys())))
        self.rgb_topic = rospy.get_param("~rgb_topic", "/kinect2/sd/image_color_rect")
        self.depth_topic = rospy.get_param("~depth_topic", "/kinect2/sd/image_depth_rect")
        self.depth_unit_auto_scale = bool(rospy.get_param("~depth_unit_auto_scale", True))
        self.depth_mm_threshold = float(rospy.get_param("~depth_mm_threshold", 20.0))
        self.depth_scale = float(rospy.get_param("~depth_scale", 1.0))
        rospy.loginfo("[VISION] rgb_topic=%s depth_topic=%s" % (self.rgb_topic, self.depth_topic))
        rospy.loginfo(
            "[VISION] depth_unit_auto_scale=%s depth_mm_threshold=%.2f depth_scale=%.4f"
            % (self.depth_unit_auto_scale, self.depth_mm_threshold, self.depth_scale)
        )
        rospy.loginfo(
            "[VISION] square_filter enabled=%s min_side=%.1f max_aspect=%.2f min_fill=%.2f"
            % (
                str(self.square_filter_enabled),
                self.square_min_side_px,
                self.square_max_aspect_ratio,
                self.square_min_fill_ratio,
            )
        )

        self.image_sub = rospy.Subscriber(
            self.rgb_topic, Image, self.image_cb, queue_size=1
        )
        self.depth_sub = rospy.Subscriber(
            self.depth_topic, Image, self.depth_cb, queue_size=1
        )

    def _default_color_ranges(self):
        return {
            "red": [
                (np.array([0, 80, 80]), np.array([10, 255, 255])),
                (np.array([160, 80, 80]), np.array([180, 255, 255])),
            ],
            "green": [
                (np.array([35, 70, 70]), np.array([90, 255, 255])),
            ],
            "blue": [
                (np.array([95, 70, 70]), np.array([140, 255, 255])),
            ],
        }

    def _load_color_ranges(self):
        ranges = self._default_color_ranges()
        configured = self._parse_color_ranges(rospy.get_param("~color_ranges", {}))
        ranges.update(configured)
        return ranges

    def _parse_color_ranges(self, raw_ranges):
        parsed = {}
        if not isinstance(raw_ranges, dict):
            return parsed
        for color, raw in raw_ranges.items():
            entries = raw if isinstance(raw, list) else [raw]
            color_entries = []
            for entry in entries:
                if isinstance(entry, dict):
                    lower = entry.get("lower", entry.get("hsv_lower"))
                    upper = entry.get("upper", entry.get("hsv_upper"))
                elif isinstance(entry, (list, tuple)) and len(entry) == 2:
                    lower, upper = entry
                else:
                    continue
                try:
                    lower_arr = np.array([int(v) for v in lower], dtype=np.uint8)
                    upper_arr = np.array([int(v) for v in upper], dtype=np.uint8)
                except (TypeError, ValueError):
                    continue
                if lower_arr.shape != (3,) or upper_arr.shape != (3,):
                    continue
                color_entries.append((lower_arr, upper_arr))
            if color_entries:
                parsed[str(color)] = color_entries
        return parsed

    def set_enabled(self, enabled: bool):
        enabled = bool(enabled)
        with self.lock:
            if self.enabled == enabled:
                return
            self.enabled = enabled
            if enabled:
                self.enabled_since = rospy.Time.now().to_sec()
            if not enabled:
                self.detections = {}
                self.depth_image = None
        rospy.loginfo("[VISION] %s" % ("enabled" if enabled else "disabled"))

    def depth_cb(self, msg):
        if not self.enabled:
            return
        with self.lock:
            self.last_depth_received_at = rospy.Time.now().to_sec()
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")
        except Exception:
            return

    def _depth_at(self, qhd_x, qhd_y):
        if self.depth_image is None:
            return 0.0
        depth_h, depth_w = self.depth_image.shape[:2]
        color_w = max(1.0, self.color_width)
        color_h = max(1.0, self.color_height)
        dx = int(qhd_x * float(depth_w) / color_w)
        dy = int(qhd_y * float(depth_h) / color_h)
        dx = max(0, min(depth_w - 1, dx))
        dy = max(0, min(depth_h - 1, dy))
        d = float(self.depth_image[dy, dx])
        if (not np.isfinite(d)) or d <= 0.0:
            return 0.0
        if self.depth_unit_auto_scale and d > self.depth_mm_threshold:
            d *= 0.001
        else:
            d *= self.depth_scale
        return d

    def _looks_like_square_block(self, contour, area: float) -> bool:
        if not self.square_filter_enabled:
            return True
        x, y, w, h = cv2.boundingRect(contour)
        if w < self.square_min_side_px or h < self.square_min_side_px:
            return False
        short_side = max(1.0, float(min(w, h)))
        aspect = float(max(w, h)) / short_side
        fill = float(area) / max(1.0, float(w * h))
        return aspect <= self.square_max_aspect_ratio and fill >= self.square_min_fill_ratio

    def image_cb(self, msg):
        if not self.enabled:
            return
        with self.lock:
            self.last_rgb_received_at = rospy.Time.now().to_sec()
        try:
            image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception:
            return

        self.color_height, self.color_width = image.shape[:2]
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        kernel = np.ones((5, 5), np.uint8)

        dets: Dict[str, Detection] = {}
        for color, ranges in self.color_ranges.items():
            mask = None
            for lower, upper in ranges:
                part = cv2.inRange(hsv, lower, upper)
                mask = part if mask is None else cv2.bitwise_or(mask, part)

            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            best = None
            best_area = 0.0
            for contour in contours:
                area = cv2.contourArea(contour)
                if not (self.min_area < area < self.max_area):
                    continue
                if not self._looks_like_square_block(contour, area):
                    continue
                if area > best_area:
                    best = contour
                    best_area = area

            if best is None:
                continue

            m = cv2.moments(best)
            if m["m00"] == 0:
                continue

            cx = int(m["m10"] / m["m00"])
            cy = int(m["m01"] / m["m00"])
            depth = self._depth_at(cx, cy)
            if depth <= 0.0:
                continue

            dets[color] = Detection(color=color, cx=cx, cy=cy, depth=depth, area=best_area)

            if self.show_debug:
                cv2.drawContours(image, [best], -1, (0, 255, 0), 2)
                cv2.circle(image, (cx, cy), 4, (0, 0, 255), -1)
                cv2.putText(
                    image,
                    "%s d=%.2f a=%.0f" % (color, depth, best_area),
                    (cx + 8, cy - 8),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.52,
                    (255, 255, 255),
                    1,
                )

        if self.show_debug:
            cv2.imshow("stack_sort_debug", image)
            cv2.waitKey(1)

        with self.lock:
            self.detections = dets

    def get_detections(self) -> Dict[str, Detection]:
        with self.lock:
            return dict(self.detections)

    def health(self, timeout: float):
        timeout = max(0.5, float(timeout))
        now = rospy.Time.now().to_sec()
        with self.lock:
            enabled = bool(self.enabled)
            enabled_since = float(self.enabled_since)
            rgb_at = float(self.last_rgb_received_at)
            depth_at = float(self.last_depth_received_at)
        enabled_age = max(0.0, now - enabled_since) if enabled else None
        rgb_age = None if rgb_at <= 0.0 else max(0.0, now - rgb_at)
        depth_age = None if depth_at <= 0.0 else max(0.0, now - depth_at)
        in_startup_grace = enabled and enabled_age is not None and enabled_age <= timeout
        rgb_ok = (not enabled) or in_startup_grace or (rgb_age is not None and rgb_age <= timeout)
        depth_ok = (not enabled) or in_startup_grace or (depth_age is not None and depth_age <= timeout)
        return {
            "enabled": enabled,
            "enabled_age_sec": enabled_age,
            "rgb_topic": self.rgb_topic,
            "depth_topic": self.depth_topic,
            "rgb_age_sec": rgb_age,
            "depth_age_sec": depth_age,
            "rgb_ok": rgb_ok,
            "depth_ok": depth_ok,
            "timeout_sec": timeout,
            "device_ok": (not enabled) or (rgb_ok and depth_ok),
        }


class TaskPlanner:
    def __init__(self, field: FieldGeometry):
        self.field = field
        default_colors = ["green", "red"]
        active_colors = rospy.get_param("~active_colors", default_colors)
        if isinstance(active_colors, str):
            active_colors = [c.strip() for c in active_colors.split(",") if c.strip()]
        self.active_colors = [str(c) for c in active_colors if str(c).strip()]
        if not self.active_colors:
            self.active_colors = default_colors

        self.stack_count = {color: 0 for color in self.active_colors}
        self.base_drop_lift = rospy.get_param("~base_drop_lift", field.base_drop_lift)
        self.stack_lift_step = rospy.get_param("~stack_lift_step", field.stack_lift_step)
        self.max_per_color = int(rospy.get_param("~max_per_color", 2))

        self.drop_zones = {
            "red": {"yaw": 0.55, "forward": 0.70},
            "green": {"yaw": 0.00, "forward": 0.80},
            "blue": {"yaw": -0.55, "forward": 0.70},
        }
        zone_overrides = rospy.get_param("~drop_zones", {})
        if isinstance(zone_overrides, dict):
            for color, zone in zone_overrides.items():
                if not isinstance(zone, dict):
                    continue
                self.drop_zones.setdefault(str(color), {"yaw": 0.0, "forward": 0.80})
                if "yaw" in zone:
                    self.drop_zones[str(color)]["yaw"] = float(zone["yaw"])
                if "forward" in zone:
                    self.drop_zones[str(color)]["forward"] = float(zone["forward"])
        for color in self.active_colors:
            if color not in self.drop_zones:
                self.drop_zones[color] = {"yaw": 0.0, "forward": 0.80}
                rospy.logwarn("[CONFIG] no drop_zones entry for %s, using yaw=0 forward=0.80" % color)

        self.drop_anchor: Dict[str, Optional[Pose2D]] = {color: None for color in self.active_colors}

    def total_done(self):
        return sum(self.stack_count.values())

    def total_goal(self):
        return self.max_per_color * len(self.active_colors)

    def select_target(self, detections: Dict[str, Detection]) -> Optional[str]:
        candidates = []
        for color, det in detections.items():
            if color not in self.stack_count:
                continue
            if self.stack_count[color] >= self.max_per_color:
                continue
            candidates.append((self.stack_count[color], det.depth, det))
        if not candidates:
            return None
        candidates.sort(key=lambda item: (item[0], item[1]))
        return candidates[0][2].color

    def get_drop_plan(self, color: str):
        zone = self.drop_zones[color]
        lift = self.base_drop_lift + self.stack_count[color] * self.stack_lift_step
        return zone["yaw"], zone["forward"], lift

    def mark_placed(self, color: str):
        self.stack_count[color] += 1

    def get_drop_anchor(self, color: str) -> Optional[Pose2D]:
        anchor = self.drop_anchor[color]
        if anchor is None:
            return None
        return Pose2D(anchor.x, anchor.y, anchor.yaw)

    def set_drop_anchor_if_empty(self, color: str, pose: Optional[Pose2D]):
        if pose is None:
            return
        if self.drop_anchor[color] is None:
            self.drop_anchor[color] = Pose2D(pose.x, pose.y, pose.yaw)


class MotionArmController:
    def __init__(self):
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.mani_pub = rospy.Publisher("/wpb_home/mani_ctrl", JointState, queue_size=10)

        self.pose_source = str(rospy.get_param("~pose_source", "odom")).lower()
        self.pose_frame = rospy.get_param("~pose_frame", rospy.get_param("~visualization_frame", "map"))
        self.base_frame = rospy.get_param("~base_frame", "base_footprint")
        self.tf_listener = tf.TransformListener() if self.pose_source == "tf" else None
        self.use_odom_control = rospy.get_param("~use_odom_control", True)
        self.odom_topic = rospy.get_param("~odom_topic", "/odom")
        self.dist_tolerance = float(rospy.get_param("~odom_dist_tolerance", 0.03))
        self.yaw_tolerance = float(rospy.get_param("~odom_yaw_tolerance", 0.05))
        self.lin_kp = float(rospy.get_param("~odom_ctrl_kp_lin", 0.90))
        self.yaw_kp = float(rospy.get_param("~odom_ctrl_kp_yaw", 1.20))
        self.min_lin_speed = float(rospy.get_param("~odom_min_lin_speed", 0.06))
        self.max_lin_speed = float(rospy.get_param("~odom_max_lin_speed", 0.18))
        self.min_yaw_speed = float(rospy.get_param("~odom_min_yaw_speed", 0.16))
        self.max_yaw_speed = float(rospy.get_param("~odom_max_yaw_speed", 0.60))
        self.move_timeout = float(rospy.get_param("~odom_move_timeout", 12.0))
        self.rotate_timeout = float(rospy.get_param("~odom_rotate_timeout", 8.0))
        self.pose_recovery_timeout = float(rospy.get_param("~motion_pose_recovery_timeout", 3.0))
        self.pose_recovery_poll_hz = float(rospy.get_param("~motion_pose_recovery_poll_hz", 10.0))

        self.odom_lock = threading.Lock()
        self.odom_pose: Optional[Pose2D] = None
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb, queue_size=1)

        self.joint = JointState()
        self.joint.name = ["lift", "gripper"]
        self.joint.position = [0.0, 0.16]
        self.joint.velocity = [0.12, 5.0]

    def odom_cb(self, msg: Odometry):
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        with self.odom_lock:
            self.odom_pose = Pose2D(
                x=msg.pose.pose.position.x,
                y=msg.pose.pose.position.y,
                yaw=yaw,
            )

    def get_pose(self) -> Optional[Pose2D]:
        if self.tf_listener is not None:
            try:
                trans, rot = self.tf_listener.lookupTransform(self.pose_frame, self.base_frame, rospy.Time(0))
                yaw = math.atan2(
                    2.0 * (rot[3] * rot[2] + rot[0] * rot[1]),
                    1.0 - 2.0 * (rot[1] * rot[1] + rot[2] * rot[2]),
                )
                return Pose2D(trans[0], trans[1], yaw)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as exc:
                rospy.logwarn_throttle(
                    2.0,
                    "[POSE] waiting for tf %s -> %s: %s" % (self.pose_frame, self.base_frame, exc),
                )
                return None
        with self.odom_lock:
            if self.odom_pose is None:
                return None
            return Pose2D(self.odom_pose.x, self.odom_pose.y, self.odom_pose.yaw)

    def tf_age(self) -> Optional[float]:
        if self.tf_listener is None:
            return None
        try:
            stamp = self.tf_listener.getLatestCommonTime(self.pose_frame, self.base_frame)
            age = rospy.Time.now().to_sec() - stamp.to_sec()
            return max(0.0, age)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return None

    def get_lift(self) -> float:
        return float(self.joint.position[0])

    def odom_ready(self) -> bool:
        return self.get_pose() is not None

    def wait_for_pose(self, reason: str, timeout: Optional[float] = None) -> Optional[Pose2D]:
        timeout = self.pose_recovery_timeout if timeout is None else float(timeout)
        timeout = max(0.0, timeout)
        deadline = rospy.Time.now().to_sec() + timeout
        rate = rospy.Rate(max(1.0, self.pose_recovery_poll_hz))
        while not rospy.is_shutdown():
            pose = self.get_pose()
            if pose is not None:
                return pose
            self.stop_base()
            if rospy.Time.now().to_sec() >= deadline:
                break
            rospy.logwarn_throttle(
                1.0,
                "[MOTION] waiting for pose recovery reason=%s timeout=%.1fs",
                reason,
                timeout,
            )
            rate.sleep()
        self.stop_base()
        return None

    def _normalize_angle(self, angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def stop_base(self, repeat_seconds: float = 0.0, hz: float = 20.0):
        stop_msg = Twist()
        self.cmd_pub.publish(stop_msg)
        if repeat_seconds <= 0.0:
            return
        rate = rospy.Rate(hz)
        end_t = rospy.Time.now() + rospy.Duration(repeat_seconds)
        while rospy.Time.now() < end_t and not rospy.is_shutdown():
            self.cmd_pub.publish(stop_msg)
            rate.sleep()

    def publish_vel(self, linear_x=0.0, linear_y=0.0, angular_z=0.0):
        msg = Twist()
        msg.linear.x = linear_x
        msg.linear.y = linear_y
        msg.angular.z = angular_z
        self.cmd_pub.publish(msg)

    def drive_for(self, seconds, linear_x=0.0, angular_z=0.0, hz=20.0):
        rate = rospy.Rate(hz)
        end_t = rospy.Time.now() + rospy.Duration(seconds)
        while rospy.Time.now() < end_t and not rospy.is_shutdown():
            self.publish_vel(linear_x=linear_x, angular_z=angular_z)
            rate.sleep()
        self.stop_base()

    def move_distance(self, distance: float, dist_tolerance: Optional[float] = None, abort_check=None):
        if abs(distance) < 1e-3:
            return True, 0.0

        if not self.use_odom_control:
            speed = 0.15 if distance >= 0 else -0.15
            seconds = abs(distance) / 0.15
            self.drive_for(seconds=seconds, linear_x=speed)
            return True, 0.0

        start = self.wait_for_pose("forward_start")
        if start is None:
            self.stop_base()
            rospy.logerr("[MOTION] forward failed: pose unavailable before move after recovery wait")
            return False, 999.0
        goal = distance
        sign = 1.0 if goal >= 0.0 else -1.0
        tolerance = self.dist_tolerance if dist_tolerance is None else max(0.0, dist_tolerance)
        t0 = rospy.Time.now().to_sec()
        rate = rospy.Rate(30)

        while not rospy.is_shutdown():
            if abort_check is not None:
                abort_reason = abort_check()
                if abort_reason:
                    self.stop_base()
                    rospy.logerr("[MOTION] forward aborted: %s", abort_reason)
                    return False, 999.0
            now = self.get_pose()
            if now is None:
                self.stop_base()
                rospy.logwarn("[MOTION] forward pose lost, pausing until pose recovers")
                now = self.wait_for_pose("forward_recovery")
                if now is None:
                    rospy.logerr("[MOTION] forward failed: pose lost during move after recovery wait")
                    return False, 999.0
            dx = now.x - start.x
            dy = now.y - start.y
            traveled = math.cos(start.yaw) * dx + math.sin(start.yaw) * dy
            error = goal - traveled

            if abs(error) <= tolerance:
                self.stop_base()
                return True, error

            if rospy.Time.now().to_sec() - t0 > self.move_timeout:
                self.stop_base()
                return False, error

            speed = self.lin_kp * abs(error)
            speed = max(self.min_lin_speed, min(self.max_lin_speed, speed))
            self.publish_vel(linear_x=sign * speed)
            rate.sleep()

        self.stop_base()
        return False, 999.0

    def rotate_angle(self, angle: float, abort_check=None):
        if abs(angle) < 1e-3:
            return True, 0.0

        if not self.use_odom_control:
            speed = 0.4 if angle >= 0 else -0.4
            seconds = abs(angle) / 0.4
            self.drive_for(seconds=seconds, angular_z=speed)
            return True, 0.0

        start = self.wait_for_pose("rotate_start")
        if start is None:
            self.stop_base()
            rospy.logerr("[MOTION] rotate failed: pose unavailable before rotate after recovery wait")
            return False, 999.0
        target_yaw = self._normalize_angle(start.yaw + angle)
        t0 = rospy.Time.now().to_sec()
        rate = rospy.Rate(30)

        while not rospy.is_shutdown():
            if abort_check is not None:
                abort_reason = abort_check()
                if abort_reason:
                    self.stop_base()
                    rospy.logerr("[MOTION] rotate aborted: %s", abort_reason)
                    return False, 999.0
            now = self.get_pose()
            if now is None:
                self.stop_base()
                rospy.logwarn("[MOTION] rotate pose lost, pausing until pose recovers")
                now = self.wait_for_pose("rotate_recovery")
                if now is None:
                    rospy.logerr("[MOTION] rotate failed: pose lost during rotate after recovery wait")
                    return False, 999.0
            error = self._normalize_angle(target_yaw - now.yaw)

            if abs(error) <= self.yaw_tolerance:
                self.stop_base()
                return True, error

            if rospy.Time.now().to_sec() - t0 > self.rotate_timeout:
                self.stop_base()
                return False, error

            speed = self.yaw_kp * abs(error)
            speed = max(self.min_yaw_speed, min(self.max_yaw_speed, speed))
            self.publish_vel(angular_z=speed if error >= 0 else -speed)
            rate.sleep()

        self.stop_base()
        return False, 999.0

    def set_lift_and_gripper(self, lift, gripper):
        self.joint.position[0] = lift
        self.joint.position[1] = gripper
        self.mani_pub.publish(self.joint)

    def move_lift_and_gripper(self, lift, gripper, duration=0.0, hz=15.0):
        if duration <= 0.0:
            self.set_lift_and_gripper(lift, gripper)
            return

        start_lift = float(self.joint.position[0])
        start_gripper = float(self.joint.position[1])
        steps = max(1, int(duration * hz))
        rate = rospy.Rate(hz)
        for i in range(1, steps + 1):
            if rospy.is_shutdown():
                return
            t = float(i) / float(steps)
            self.set_lift_and_gripper(
                start_lift + (lift - start_lift) * t,
                start_gripper + (gripper - start_gripper) * t,
            )
            rate.sleep()


class WPBGrabActionClient:
    def __init__(self):
        self.enabled = bool(rospy.get_param("~use_wpb_grab_action", False))
        self.behaviors_topic = str(rospy.get_param("~wpb_behaviors_topic", "/wpb_home/behaviors"))
        self.objects_topic = str(rospy.get_param("~wpb_objects_topic", "/wpb_home/objects_3d"))
        self.grab_action_topic = str(rospy.get_param("~wpb_grab_action_topic", "/wpb_home/grab_action"))
        self.grab_target_color_topic = str(
            rospy.get_param("~wpb_grab_target_color_topic", "/wpb_home/grab_target_color")
        )
        self.grab_result_topic = str(rospy.get_param("~wpb_grab_result_topic", "/wpb_home/grab_result"))
        self.object_wait_timeout = float(rospy.get_param("~wpb_grab_object_wait_timeout", 8.0))
        self.result_timeout = float(rospy.get_param("~wpb_grab_result_timeout", 75.0))
        self.result_done_token = str(rospy.get_param("~wpb_grab_done_token", "done")).strip().lower()
        self.min_probability = float(rospy.get_param("~wpb_grab_min_probability", 0.0))
        self.stop_object_detect_after_grab_pose = bool(
            rospy.get_param("~wpb_stop_object_detect_after_grab_pose", False)
        )

        self.behaviors_pub = rospy.Publisher(self.behaviors_topic, String, queue_size=10)
        self.grab_pub = rospy.Publisher(self.grab_action_topic, Pose, queue_size=1)
        self.target_color_pub = rospy.Publisher(self.grab_target_color_topic, String, queue_size=1, latch=True)
        self.objects_lock = threading.Lock()
        self.objects = []
        self.objects_stamp = 0.0
        self.result_lock = threading.Lock()
        self.last_result = ""
        self.last_result_stamp = 0.0

        if self.enabled:
            rospy.Subscriber(self.objects_topic, Coord, self._objects_cb, queue_size=1)
            rospy.Subscriber(self.grab_result_topic, String, self._result_cb, queue_size=10)

    def _objects_cb(self, msg: Coord):
        objects = []
        count = min(len(msg.name), len(msg.x), len(msg.y), len(msg.z), len(msg.probability))
        for i in range(count):
            probability = float(msg.probability[i])
            if probability < self.min_probability:
                continue
            objects.append(
                {
                    "name": str(msg.name[i]),
                    "x": float(msg.x[i]),
                    "y": float(msg.y[i]),
                    "z": float(msg.z[i]),
                    "probability": probability,
                }
            )
        with self.objects_lock:
            self.objects = objects
            self.objects_stamp = rospy.Time.now().to_sec()

    def _result_cb(self, msg: String):
        value = msg.data.strip().lower()
        with self.result_lock:
            if value and value != self.last_result:
                rospy.loginfo("[PICK-WPB] result=%s" % value)
            self.last_result = value
            self.last_result_stamp = rospy.Time.now().to_sec()

    def _publish_behavior(self, command: str):
        self.behaviors_pub.publish(String(data=command))

    def _clear_runtime_state(self):
        with self.objects_lock:
            self.objects = []
            self.objects_stamp = 0.0
        with self.result_lock:
            self.last_result = ""
            self.last_result_stamp = 0.0

    def _latest_objects_since(self, started_at: float):
        with self.objects_lock:
            if self.objects_stamp < started_at:
                return []
            return list(self.objects)

    def _last_result_since(self, started_at: float) -> str:
        with self.result_lock:
            if self.last_result_stamp < started_at:
                return ""
            return self.last_result

    def stop(self, repeat_seconds: float = 0.0, hz: float = 10.0):
        if not self.enabled:
            return
        end_t = rospy.Time.now() + rospy.Duration(max(0.0, repeat_seconds))
        while not rospy.is_shutdown():
            self._publish_behavior("grab stop")
            self._publish_behavior("object_detect stop")
            if repeat_seconds <= 0.0 or rospy.Time.now() >= end_t:
                break
            rospy.Rate(hz).sleep()

    def execute(self, color: Optional[str], abort_check=None) -> bool:
        if not self.enabled:
            return False

        self._clear_runtime_state()
        started_at = rospy.Time.now().to_sec()
        target_color = str(color or "").strip().lower()
        self.target_color_pub.publish(String(data=target_color))
        rospy.loginfo(
            "[PICK-WPB] start color=%s object_topic=%s action_topic=%s color_topic=%s result_topic=%s",
            target_color or "unknown",
            self.objects_topic,
            self.grab_action_topic,
            self.grab_target_color_topic,
            self.grab_result_topic,
        )
        self._publish_behavior("object_detect start")

        rate = rospy.Rate(10)
        object_deadline = rospy.Time.now().to_sec() + self.object_wait_timeout
        selected = None
        while not rospy.is_shutdown() and rospy.Time.now().to_sec() < object_deadline:
            if abort_check is not None:
                abort_reason = abort_check()
                if abort_reason:
                    rospy.logerr("[PICK-WPB] abort before grab: %s", abort_reason)
                    self.stop()
                    return False
            objects = self._latest_objects_since(started_at)
            if objects:
                selected = objects[0]
                break
            rate.sleep()

        if selected is None:
            rospy.logerr(
                "[PICK-WPB] no 3D object from %s within %.1fs; check /kinect2/qhd/points and wpb_home_objects_3d",
                self.objects_topic,
                self.object_wait_timeout,
            )
            self.stop()
            return False

        pose = Pose()
        pose.position.x = selected["x"]
        pose.position.y = selected["y"]
        pose.position.z = selected["z"]
        rospy.loginfo(
            "[PICK-WPB] grab object=%s xyz=(%.3f, %.3f, %.3f) for color=%s",
            selected["name"],
            pose.position.x,
            pose.position.y,
            pose.position.z,
            color,
        )
        self.grab_pub.publish(pose)
        if self.stop_object_detect_after_grab_pose:
            self._publish_behavior("object_detect stop")

        result_started_at = rospy.Time.now().to_sec()
        result_deadline = result_started_at + self.result_timeout
        while not rospy.is_shutdown() and rospy.Time.now().to_sec() < result_deadline:
            if abort_check is not None:
                abort_reason = abort_check()
                if abort_reason:
                    rospy.logerr("[PICK-WPB] abort while waiting result: %s", abort_reason)
                    self.stop()
                    return False
            result = self._last_result_since(result_started_at)
            if self.result_done_token and self.result_done_token in result:
                rospy.loginfo("[PICK-WPB] done color=%s", color)
                self._publish_behavior("object_detect stop")
                return True
            if "fail" in result or "abort" in result or "error" in result:
                rospy.logerr("[PICK-WPB] failed result=%s color=%s", result, color)
                self.stop()
                return False
            rate.sleep()

        rospy.logerr(
            "[PICK-WPB] timeout waiting for %s on %s after %.1fs",
            self.result_done_token,
            self.grab_result_topic,
            self.result_timeout,
        )
        self.stop()
        return False


class GazeboCarryHelper:
    def __init__(self, ctrl: MotionArmController, field: FieldGeometry):
        self.attach_enabled = bool(rospy.get_param("~gazebo_attach_on_pick", False))
        self.ctrl = ctrl
        self.model_by_color = rospy.get_param("~gazebo_model_by_color", {})
        self.model_queues = self._normalize_model_queues(self.model_by_color)
        self.attach_counts = {color: 0 for color in self.model_queues}
        self.forward_offset = float(rospy.get_param("~gazebo_carry_forward_offset", field.carry_forward_offset))
        self.side_offset = float(rospy.get_param("~gazebo_carry_side_offset", 0.0))
        self.carry_z = float(rospy.get_param("~gazebo_carry_z", field.carry_lift_height))
        self.drop_z = float(rospy.get_param("~gazebo_drop_z", field.tabletop_model_z))
        self.drop_poses = rospy.get_param("~gazebo_drop_poses", {})
        self.use_absolute_drop_poses = bool(rospy.get_param("~gazebo_use_absolute_drop_poses", False))
        self.robot_model_name = str(rospy.get_param("~gazebo_robot_model_name", "wpb_home"))
        self.use_robot_model_pose = bool(rospy.get_param("~gazebo_use_robot_model_pose", True))
        self.follow_lift_z = bool(rospy.get_param("~gazebo_follow_lift_z", True))
        self.lift_z_offset = float(rospy.get_param("~gazebo_lift_z_offset", -0.012))
        self.carry_max_step = float(rospy.get_param("~gazebo_carry_max_step", 0.035))
        self.validate_grasp_window = bool(rospy.get_param("~gazebo_validate_grasp_window", True))
        self.grasp_max_xy_error = float(rospy.get_param("~gazebo_grasp_max_xy_error", 0.18))
        self.grasp_max_z_error = float(rospy.get_param("~gazebo_grasp_max_z_error", 0.08))
        self.use_source_pick_targets = bool(rospy.get_param("~gazebo_use_source_pick_targets", False))
        self.source_pick_yaw = float(rospy.get_param("~gazebo_source_pick_yaw", 0.0))
        self.source_pick_forward_offset = float(
            rospy.get_param("~gazebo_source_pick_forward_offset", field.source_pick_forward_offset)
        )
        self.source_pick_side_offset = float(rospy.get_param("~gazebo_source_pick_side_offset", self.side_offset))
        self.source_pick_use_current_yaw = bool(rospy.get_param("~gazebo_source_pick_use_current_yaw", False))
        self.stabilize_stack_on_release = bool(rospy.get_param("~gazebo_stabilize_stack_on_release", False))
        self.stable_stack_base_z = float(rospy.get_param("~gazebo_stable_stack_base_z", field.table_height))
        self.stable_stack_poses = rospy.get_param(
            "~gazebo_stable_stack_poses",
            rospy.get_param("~tabletop_stack_anchors", {}),
        )
        self.verify_physical_pick = bool(rospy.get_param("~gazebo_verify_physical_pick", False))
        self.physical_pick_min_lift = float(rospy.get_param("~gazebo_physical_pick_min_lift", 0.045))
        self.physical_pick_settle_seconds = float(rospy.get_param("~gazebo_physical_pick_settle_seconds", 0.35))
        self.drop_counts = {}
        self.stack_step = float(rospy.get_param("~gazebo_stack_step", field.stack_lift_step))
        self.initial_reset_delay = float(rospy.get_param("~gazebo_initial_reset_delay", 0.0))
        self.initial_model_wait_timeout = float(rospy.get_param("~gazebo_initial_model_wait_timeout", 5.0))
        self.initial_model_poses = rospy.get_param("~gazebo_initial_model_poses", {})
        self.helper_requested = bool(rospy.get_param("~gazebo_enable_helper", True))
        self.current_model = None
        self.current_color = None
        self.current_local_offset: Optional[Tuple[float, float, float]] = None
        self.stabilized_models = {}
        self.pending_model = None
        self.pending_color = None
        self.pending_start_z = 0.0
        self.source_pick_models = {}
        self.used_models = set()
        self.last_grasp_ok = True
        self.last_grasp_reason = ""
        self.lock = threading.Lock()
        self.set_model_state = None
        self.get_model_state = None
        self.model_state_cls = None
        self.thread = None
        if not self.helper_requested:
            self.attach_enabled = False
            self.use_source_pick_targets = False
            self.stabilize_stack_on_release = False
            self.verify_physical_pick = False
            self.use_absolute_drop_poses = False
            self.use_robot_model_pose = False
            self.initial_model_poses = {}

        self.enabled = bool(
            self.helper_requested
            and (
            self.attach_enabled
            or self.use_source_pick_targets
            or self.stabilize_stack_on_release
            or self.verify_physical_pick
            or self.use_absolute_drop_poses
            or self.initial_model_poses
            )
        )

        if self.enabled:
            self._load_gazebo_api()
            rospy.wait_for_service("/gazebo/set_model_state", timeout=10.0)
            self.set_model_state = rospy.ServiceProxy("/gazebo/set_model_state", self.set_model_state_srv)
            rospy.wait_for_service("/gazebo/get_model_state", timeout=10.0)
            self.get_model_state = rospy.ServiceProxy("/gazebo/get_model_state", self.get_model_state_srv)
            self._reset_initial_models()
            if self.attach_enabled or self.stabilize_stack_on_release:
                self.thread = threading.Thread(target=self._run, daemon=True)
                self.thread.start()

    def _load_gazebo_api(self):
        try:
            from gazebo_msgs.msg import ModelState
            from gazebo_msgs.srv import GetModelState, SetModelState
        except ImportError as exc:
            raise RuntimeError(
                "Gazebo helper is enabled but gazebo_msgs is not installed. "
                "On the real robot, set all gazebo_* helper parameters to false."
            ) from exc
        self.model_state_cls = ModelState
        self.get_model_state_srv = GetModelState
        self.set_model_state_srv = SetModelState

    def _normalize_model_queues(self, model_by_color):
        queues = {}
        if not isinstance(model_by_color, dict):
            return queues
        for color, models in model_by_color.items():
            if isinstance(models, list):
                queues[color] = [str(m) for m in models if str(m)]
            elif isinstance(models, str) and models:
                queues[color] = [models]
        return queues

    def attach(self, color: Optional[str]) -> bool:
        self.last_grasp_ok = True
        self.last_grasp_reason = ""
        self.pending_model = None
        self.pending_color = None
        self.pending_start_z = 0.0
        if not self.enabled or color is None:
            return True

        selected = self._select_grasp_candidate(color)
        if selected is None:
            self.last_grasp_ok = False
            if not self.last_grasp_reason:
                self.last_grasp_reason = "no_model_in_grasp_window"
            rospy.logwarn(
                "[%s] rejected color=%s reason=%s"
                % ("SIM-GRASP" if self.attach_enabled else "PHYS-GRASP", color, self.last_grasp_reason)
            )
            return False

        model_name, local_offset, xy_error, z_error = selected
        if not self.attach_enabled:
            pose = self._get_current_pose(model_name)
            self.pending_model = model_name
            self.pending_color = color
            self.pending_start_z = float(pose.position.z) if pose is not None else 0.0
            rospy.loginfo(
                "[PHYS-GRASP] candidate model=%s color=%s xy_err=%.3f z_err=%.3f start_z=%.3f"
                % (model_name, color, xy_error, z_error, self.pending_start_z)
            )
            return True

        rospy.loginfo(
            "[SIM-GRASP] locked model=%s color=%s xy_err=%.3f z_err=%.3f local=(%.3f,%.3f,%.3f)"
            % (
                model_name,
                color,
                xy_error,
                z_error,
                local_offset[0],
                local_offset[1],
                local_offset[2],
            )
        )
        with self.lock:
            self.current_model = model_name
            self.current_color = color
            self.current_local_offset = local_offset
            self.used_models.add(model_name)
        return True

    def verify_after_lift(self, color: Optional[str]) -> bool:
        if self.attach_enabled or not self.verify_physical_pick:
            return self.last_grasp_ok
        if color is None or self.pending_model is None or self.pending_color != color:
            return True

        if self.physical_pick_settle_seconds > 0.0:
            rospy.sleep(self.physical_pick_settle_seconds)
        pose = self._get_current_pose(self.pending_model)
        if pose is None:
            self.last_grasp_ok = False
            self.last_grasp_reason = "no_model_state_after_lift"
            rospy.logwarn("[PHYS-GRASP] rejected color=%s reason=%s" % (color, self.last_grasp_reason))
            return False

        dz = float(pose.position.z) - self.pending_start_z
        if dz < self.physical_pick_min_lift:
            self.last_grasp_ok = False
            self.last_grasp_reason = "lift_%.3f_lt_%.3f" % (dz, self.physical_pick_min_lift)
            rospy.logwarn(
                "[PHYS-GRASP] rejected model=%s color=%s reason=%s z=%.3f start_z=%.3f"
                % (self.pending_model, color, self.last_grasp_reason, pose.position.z, self.pending_start_z)
            )
            return False

        self.used_models.add(self.pending_model)
        if self.source_pick_models.get(color) == self.pending_model:
            self.source_pick_models.pop(color, None)
        rospy.loginfo(
            "[PHYS-GRASP] lifted model=%s color=%s dz=%.3f z=%.3f"
            % (self.pending_model, color, dz, pose.position.z)
        )
        return True

    def place_and_detach(self):
        if not self.attach_enabled:
            return
        with self.lock:
            model_name = self.current_model
            color = self.current_color
            local_offset = self.current_local_offset
            self.current_model = None
            self.current_color = None
            self.current_local_offset = None
        if model_name:
            if self.stabilize_stack_on_release and self._stabilize_model_at_stack(model_name, color):
                pass
            elif self.use_absolute_drop_poses and color in self.drop_poses:
                self._place_at_drop_pose(model_name, color)
            else:
                self._set_model_pose(model_name, self._target_z(), smooth=False, local_offset=local_offset)
            rospy.loginfo("[SIM-GRASP] released model=%s color=%s" % (model_name, str(color)))

    def _run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            with self.lock:
                model_name = self.current_model
                local_offset = self.current_local_offset
                stabilized_models = dict(self.stabilized_models)
            if model_name:
                self._set_model_pose(model_name, self._target_z(), smooth=True, local_offset=local_offset)
            for stable_model, stable_pose in stabilized_models.items():
                self._set_absolute_model_pose(stable_model, *stable_pose)
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                break

    def _reset_initial_models(self):
        if not isinstance(self.initial_model_poses, dict) or not self.initial_model_poses:
            return
        if self.initial_reset_delay > 0.0:
            rospy.sleep(self.initial_reset_delay)
        for model_name, pose in self.initial_model_poses.items():
            if not isinstance(pose, dict):
                continue
            if not self._wait_for_model(model_name, self.initial_model_wait_timeout):
                rospy.logwarn("[SIM-INIT] model=%s not available for initial pose reset" % model_name)
                continue
            self._set_absolute_model_pose(
                model_name=model_name,
                x=float(pose.get("x", 0.0)),
                y=float(pose.get("y", 0.0)),
                z=float(pose.get("z", self.drop_z)),
                yaw=float(pose.get("yaw", 0.0)),
            )
            rospy.loginfo("[SIM-INIT] reset model=%s to tabletop pose" % model_name)

    def _wait_for_model(self, model_name: str, timeout: float) -> bool:
        if self.get_model_state is None:
            return False
        deadline = rospy.Time.now().to_sec() + max(0.0, timeout)
        while not rospy.is_shutdown():
            pose = self._get_current_pose(model_name)
            if pose is not None:
                return True
            if rospy.Time.now().to_sec() >= deadline:
                return False
            rospy.sleep(0.1)
        return False

    def _place_at_drop_pose(self, model_name: str, color: str):
        pose = self.drop_poses.get(color, {})
        if not isinstance(pose, dict):
            self._set_model_pose(model_name, self.drop_z, smooth=False)
            return
        idx = self.drop_counts.get(color, 0)
        self.drop_counts[color] = idx + 1
        self._set_absolute_model_pose(
            model_name=model_name,
            x=float(pose.get("x", 0.0)),
            y=float(pose.get("y", 0.0)),
            z=float(pose.get("z", self.drop_z)) + idx * self.stack_step,
            yaw=float(pose.get("yaw", 0.0)),
        )

    def _stabilize_model_at_stack(self, model_name: str, color: Optional[str]) -> bool:
        if color is None or not isinstance(self.stable_stack_poses, dict):
            return False
        pose = self.stable_stack_poses.get(color)
        if not isinstance(pose, dict):
            return False
        idx = self.drop_counts.get(color, 0)
        self.drop_counts[color] = idx + 1
        x = float(pose.get("x", 0.0))
        y = float(pose.get("y", 0.0))
        z = float(pose.get("z", self.stable_stack_base_z)) + idx * self.stack_step
        yaw = float(pose.get("yaw", 0.0))
        stable_pose = (x, y, z, yaw)
        self._set_absolute_model_pose(model_name, *stable_pose)
        with self.lock:
            self.stabilized_models[model_name] = stable_pose
        rospy.loginfo(
            "[SIM-STACK] stabilized model=%s color=%s layer=%d pose=(%.2f,%.2f,%.3f,%.2f)"
            % (model_name, color, idx + 1, x, y, z, yaw)
        )
        return True

    def _target_z(self):
        if not self.follow_lift_z:
            return self.carry_z
        return max(self.drop_z, self.ctrl.get_lift() + self.lift_z_offset)

    def robot_pose(self) -> Optional[Pose2D]:
        if self.use_robot_model_pose and self.get_model_state is not None:
            pose = self._get_current_pose(self.robot_model_name)
            if pose is not None:
                q = pose.orientation
                yaw = math.atan2(
                    2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z),
                )
                return Pose2D(pose.position.x, pose.position.y, yaw)
        return self.ctrl.get_pose()

    def _target_model_state(self, z: float, local_offset: Optional[Tuple[float, float, float]] = None):
        pose = self.robot_pose()
        if pose is None:
            return None
        state = self.model_state_cls()
        state.reference_frame = "world"
        if local_offset is None:
            local_x = self.forward_offset
            local_y = self.side_offset
            local_z = 0.0
        else:
            local_x, local_y, local_z = local_offset
        state.pose.position.x = pose.x + math.cos(pose.yaw) * local_x - math.sin(pose.yaw) * local_y
        state.pose.position.y = pose.y + math.sin(pose.yaw) * local_x + math.cos(pose.yaw) * local_y
        state.pose.position.z = z + local_z
        state.pose.orientation.z = math.sin(pose.yaw * 0.5)
        state.pose.orientation.w = math.cos(pose.yaw * 0.5)
        return state

    def _world_to_local_offset(self, pose, target_z: float) -> Optional[Tuple[float, float, float]]:
        base_pose = self.ctrl.get_pose()
        if base_pose is None:
            return None
        dx = pose.position.x - base_pose.x
        dy = pose.position.y - base_pose.y
        local_x = math.cos(base_pose.yaw) * dx + math.sin(base_pose.yaw) * dy
        local_y = -math.sin(base_pose.yaw) * dx + math.cos(base_pose.yaw) * dy
        local_z = 0.0
        return (local_x, local_y, local_z)

    def get_current_local_offset(self) -> Optional[Tuple[float, float, float]]:
        with self.lock:
            if self.current_local_offset is None:
                return None
            return tuple(self.current_local_offset)

    def _fallback_model_for_color(self, color: str) -> Optional[str]:
        models = self.model_queues.get(color, [])
        idx = self.attach_counts.get(color, 0)
        while idx < len(models) and models[idx] in self.used_models:
            idx += 1
        self.attach_counts[color] = idx + 1
        if idx >= len(models):
            return None
        return models[idx]

    def _next_unpicked_model(self, color: str) -> Optional[str]:
        for model_name in self.model_queues.get(color, []):
            if model_name not in self.used_models:
                return model_name
        return None

    def source_pick_base_target(self, color: str, yaw_override: Optional[float] = None) -> Optional[Pose2D]:
        if not self.enabled or not self.use_source_pick_targets:
            return None
        model_name = self._next_unpicked_model(color)
        if model_name is None:
            return None
        pose = self._get_current_pose(model_name)
        if pose is None:
            return None

        self.source_pick_models[color] = model_name
        yaw = self.source_pick_yaw if yaw_override is None else yaw_override
        local_x = self.source_pick_forward_offset
        local_y = self.source_pick_side_offset
        return Pose2D(
            x=pose.position.x - math.cos(yaw) * local_x + math.sin(yaw) * local_y,
            y=pose.position.y - math.sin(yaw) * local_x - math.cos(yaw) * local_y,
            yaw=yaw,
        )

    def _select_grasp_candidate(self, color: str):
        models = self.model_queues.get(color, [])
        if not models:
            self.last_grasp_reason = "no_models_for_color"
            return None

        target_z = self._target_z()
        target = self._target_model_state(target_z)
        if target is None or self.get_model_state is None:
            model_name = self._fallback_model_for_color(color)
            if model_name is None:
                self.last_grasp_reason = "no_remaining_model"
                return None
            return (model_name, (self.forward_offset, self.side_offset, 0.0), 0.0, 0.0)

        intended_model = self.source_pick_models.get(color)
        candidate_models = models
        if intended_model in models and intended_model not in self.used_models:
            candidate_models = [intended_model]

        best = None
        for model_name in candidate_models:
            if model_name in self.used_models:
                continue
            pose = self._get_current_pose(model_name)
            if pose is None:
                continue
            dx = pose.position.x - target.pose.position.x
            dy = pose.position.y - target.pose.position.y
            dz = pose.position.z - target.pose.position.z
            xy_error = math.hypot(dx, dy)
            z_error = abs(dz)
            local_offset = self._world_to_local_offset(pose, target_z)
            if local_offset is None:
                continue
            score = xy_error + z_error
            if best is None or score < best[0]:
                best = (score, model_name, local_offset, xy_error, z_error)

        if best is None:
            self.last_grasp_reason = "no_unpicked_model_state"
            return None

        _, model_name, local_offset, xy_error, z_error = best
        if self.validate_grasp_window:
            if xy_error > self.grasp_max_xy_error:
                self.last_grasp_reason = "xy_error_%.3f_gt_%.3f" % (xy_error, self.grasp_max_xy_error)
                return None
            if z_error > self.grasp_max_z_error:
                self.last_grasp_reason = "z_error_%.3f_gt_%.3f" % (z_error, self.grasp_max_z_error)
                return None
        return (model_name, local_offset, xy_error, z_error)

    def _get_current_pose(self, model_name: str):
        if self.get_model_state is None:
            return None
        try:
            result = self.get_model_state(model_name, "world")
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "[SIM-GRASP] get_model_state failed: %s" % str(exc))
            return None
        if not result.success:
            return None
        return result.pose

    def _set_model_pose(
        self,
        model_name: str,
        z: float,
        smooth: bool,
        local_offset: Optional[Tuple[float, float, float]] = None,
    ):
        if self.set_model_state is None:
            return
        state = self._target_model_state(z, local_offset=local_offset)
        if state is None:
            return
        state.model_name = model_name

        if smooth and self.carry_max_step > 0.0:
            current = self._get_current_pose(model_name)
            if current is not None:
                dx = state.pose.position.x - current.position.x
                dy = state.pose.position.y - current.position.y
                dz = state.pose.position.z - current.position.z
                dist = math.sqrt(dx * dx + dy * dy + dz * dz)
                if dist > self.carry_max_step:
                    ratio = self.carry_max_step / dist
                    state.pose.position.x = current.position.x + dx * ratio
                    state.pose.position.y = current.position.y + dy * ratio
                    state.pose.position.z = current.position.z + dz * ratio

        try:
            self.set_model_state(state)
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "[SIM-GRASP] set_model_state failed: %s" % str(exc))

    def _set_absolute_model_pose(self, model_name: str, x: float, y: float, z: float, yaw: float):
        if self.set_model_state is None:
            return
        state = self.model_state_cls()
        state.model_name = model_name
        state.reference_frame = "world"
        state.pose.position.x = x
        state.pose.position.y = y
        state.pose.position.z = z
        state.pose.orientation.z = math.sin(yaw * 0.5)
        state.pose.orientation.w = math.cos(yaw * 0.5)
        try:
            self.set_model_state(state)
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "[SIM-INIT] reset model failed: %s" % str(exc))


class TaskStatusPublisher:
    def __init__(
        self,
        planner: TaskPlanner,
        ctrl: MotionArmController,
        gazebo_carry: GazeboCarryHelper,
        field: FieldGeometry,
    ):
        self.planner = planner
        self.ctrl = ctrl
        self.gazebo_carry = gazebo_carry
        self.field = field
        self.enabled = bool(rospy.get_param("~enable_status_publish", True))
        self.status_topic = str(rospy.get_param("~status_topic", "/stack_sort/status"))
        self.marker_topic = str(rospy.get_param("~marker_topic", "/stack_sort/markers"))
        self.frame_id = str(rospy.get_param("~visualization_frame", "map"))
        self.publish_rate = float(rospy.get_param("~status_publish_rate", 2.0))
        self.last_publish = 0.0
        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=10, latch=True)
        self.marker_pub = rospy.Publisher(self.marker_topic, MarkerArray, queue_size=2, latch=True)

    def publish(
        self,
        state: str,
        state_reason: str,
        target_color: Optional[str],
        detections: Dict[str, Detection],
        metrics: RunMetrics,
        last_depth: float,
        last_align_error: float,
        pick_retry_count: int,
        stack_anchors: Dict[str, Pose2D],
        drop_base_targets: Dict[str, Pose2D],
        vision_health: Dict,
    ):
        if not self.enabled:
            return
        now = rospy.Time.now().to_sec()
        interval = 1.0 / max(0.1, self.publish_rate)
        if now - self.last_publish < interval:
            return
        self.last_publish = now

        robot_pose = self.gazebo_carry.robot_pose()
        payload = {
            "timestamp": datetime.now().isoformat(),
            "state": state,
            "state_reason": state_reason,
            "last_error": state_reason if state == "ERROR" else "",
            "target_color": target_color,
            "active_colors": list(self.planner.active_colors),
            "stack_count": dict(self.planner.stack_count),
            "total_done": self.planner.total_done(),
            "total_goal": self.planner.total_goal(),
            "last_depth": last_depth,
            "last_align_error_px": last_align_error,
            "pick_retry_count": pick_retry_count,
            "pick_attempts": metrics.pick_attempts,
            "pick_retries_total": metrics.pick_retries,
            "vision_health": dict(vision_health or {}),
            "detections": {
                color: {
                    "cx": det.cx,
                    "cy": det.cy,
                    "depth": det.depth,
                    "area": det.area,
                }
                for color, det in detections.items()
            },
            "robot_pose": self._pose_to_dict(robot_pose),
        }
        self.status_pub.publish(String(data=json.dumps(payload, sort_keys=True)))
        self.marker_pub.publish(self._marker_array(state, target_color, robot_pose, stack_anchors, drop_base_targets))

    def _pose_to_dict(self, pose: Optional[Pose2D]):
        if pose is None:
            return None
        return {"x": pose.x, "y": pose.y, "yaw": pose.yaw}

    def _marker_array(
        self,
        state: str,
        target_color: Optional[str],
        robot_pose: Optional[Pose2D],
        stack_anchors: Dict[str, Pose2D],
        drop_base_targets: Dict[str, Pose2D],
    ):
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        marker_id = 1
        stamp = rospy.Time.now()
        for color, pose in sorted(stack_anchors.items()):
            if pose is None:
                continue
            marker = self._base_marker(stamp, marker_id, "stack_anchor_%s" % color, color)
            marker.type = Marker.CUBE
            marker.pose.position.x = pose.x
            marker.pose.position.y = pose.y
            marker.pose.position.z = self.field.table_height + 0.025
            marker.pose.orientation.z = math.sin(pose.yaw * 0.5)
            marker.pose.orientation.w = math.cos(pose.yaw * 0.5)
            marker.scale.x = max(0.04, self.field.box_x)
            marker.scale.y = max(0.04, self.field.box_y)
            marker.scale.z = 0.05
            markers.markers.append(marker)
            marker_id += 1

        for color, pose in sorted(drop_base_targets.items()):
            if pose is None:
                continue
            marker = self._base_marker(stamp, marker_id, "drop_base_%s" % color, color)
            marker.type = Marker.ARROW
            marker.pose.position.x = pose.x
            marker.pose.position.y = pose.y
            marker.pose.position.z = 0.08
            marker.pose.orientation.z = math.sin(pose.yaw * 0.5)
            marker.pose.orientation.w = math.cos(pose.yaw * 0.5)
            marker.scale.x = 0.32
            marker.scale.y = 0.045
            marker.scale.z = 0.045
            markers.markers.append(marker)
            marker_id += 1

        text_pose = robot_pose or Pose2D(0.0, 0.0, 0.0)
        marker = self._base_marker(stamp, marker_id, "stack_sort_status", "white")
        marker.type = Marker.TEXT_VIEW_FACING
        marker.pose.position.x = text_pose.x
        marker.pose.position.y = text_pose.y
        marker.pose.position.z = max(1.0, self.field.table_height + 0.45)
        marker.scale.z = 0.18
        marker.text = "state=%s target=%s progress=%d/%d" % (
            state,
            target_color or "none",
            self.planner.total_done(),
            self.planner.total_goal(),
        )
        markers.markers.append(marker)
        return markers

    def _base_marker(self, stamp, marker_id: int, namespace: str, color: str):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = stamp
        marker.ns = namespace
        marker.id = marker_id
        marker.action = Marker.ADD
        marker.lifetime = rospy.Duration(2.0 / max(0.1, self.publish_rate))
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = self._rgba(color)
        return marker

    def _rgba(self, color: str):
        table = {
            "red": (1.0, 0.05, 0.05, 0.85),
            "green": (0.05, 0.9, 0.20, 0.85),
            "blue": (0.05, 0.35, 1.0, 0.85),
            "yellow": (1.0, 0.9, 0.1, 0.85),
            "white": (1.0, 1.0, 1.0, 0.9),
        }
        return table.get(color, (1.0, 0.6, 0.1, 0.85))


class StackSortOrchestrator:
    def __init__(self):
        self.field = FieldGeometry()
        self.perception = ColorBoxPerception()
        self.planner = TaskPlanner(self.field)
        self.ctrl = MotionArmController()
        self.wpb_grab = WPBGrabActionClient()
        self.gazebo_carry = GazeboCarryHelper(self.ctrl, self.field)
        self.metrics = RunMetrics(self.planner.active_colors)
        self.reporter = ReportExporter()
        self.status = TaskStatusPublisher(self.planner, self.ctrl, self.gazebo_carry, self.field)

        self.state = "SEARCH"
        self.state_reason = "init"
        self.target_color: Optional[str] = None
        self.last_depth = 0.0
        self.last_align_error = 0.0
        self.lost_cycles = 0
        self.pick_retry_count = 0
        self.pick_signature: Optional[PickSignature] = None
        self.near_pick_align_cycles = 0
        self.align_watchdog_cycles = 0
        self.source_pose_ready = False
        self.demo_travel_lift_ready = False
        self.wait_for_localization_inputs = bool(rospy.get_param("~wait_for_localization_inputs", True))
        self.startup_timeout = float(rospy.get_param("~startup_timeout", 15.0))
        self.map_topic = str(rospy.get_param("~map_topic", "/map"))
        self.scan_topic = str(rospy.get_param("~scan_topic", "/scan"))
        self.odom_topic = str(rospy.get_param("~odom_topic", self.ctrl.odom_topic))
        self.amcl_topic = str(rospy.get_param("~amcl_topic", "/amcl_pose"))
        self.localization_watchdog_enabled = bool(rospy.get_param("~localization_watchdog_enabled", False))
        self.scan_watchdog_timeout = float(rospy.get_param("~scan_watchdog_timeout", 5.0))
        self.amcl_watchdog_timeout = float(rospy.get_param("~amcl_watchdog_timeout", 8.0))
        self.localization_fault_active = False
        self.emergency_stop_active = False
        self.emergency_stop_reason = ""
        self.emergency_stop_topic = str(rospy.get_param("~emergency_stop_topic", "/warehouse_tuning/emergency_stop"))
        self.last_scan_received_at = 0.0
        self.last_amcl_received_at = 0.0
        rospy.Subscriber(self.scan_topic, LaserScan, self._on_scan_watchdog, queue_size=1)
        rospy.Subscriber(self.amcl_topic, PoseWithCovarianceStamped, self._on_amcl_watchdog, queue_size=1)
        rospy.Subscriber(self.emergency_stop_topic, String, self._on_emergency_stop, queue_size=1)
        self.require_initial_pose_before_start = bool(rospy.get_param("~require_initial_pose_before_start", False))
        self.initial_pose_received = not self.require_initial_pose_before_start
        self.initial_pose_received_at = 0.0
        self.initial_pose_nomotion_update_sent = False
        self.initial_pose_localization_ready = not self.require_initial_pose_before_start
        self.initial_pose_topic = str(rospy.get_param("~initial_pose_topic", "/initialpose"))
        self.initial_pose_settle_time = float(rospy.get_param("~initial_pose_settle_time", 1.0))
        self.initial_pose_request_nomotion_update = bool(rospy.get_param("~initial_pose_request_nomotion_update", True))
        self.confirm_before_start = bool(rospy.get_param("~confirm_before_start", True))
        self.start_confirmed = False
        self.confirm_start_service_name = rospy.resolve_name("~confirm_start")
        rospy.Service("~confirm_start", Trigger, self._handle_confirm_start)
        if self.require_initial_pose_before_start:
            rospy.Subscriber(self.initial_pose_topic, PoseWithCovarianceStamped, self._on_initial_pose, queue_size=1)

        default_center_x = float(rospy.get_param("~color_image_width", 960.0)) / 2.0
        self.center_x = float(rospy.get_param("~camera_center_x", default_center_x))
        self.deadband_px = float(rospy.get_param("~align_deadband_px", 28.0))
        self.near_pick_align_deadband_px = float(rospy.get_param("~near_pick_align_deadband_px", 140.0))
        self.max_near_pick_align_cycles = int(rospy.get_param("~max_near_pick_align_cycles", 45))
        self.max_align_watchdog_cycles = int(rospy.get_param("~max_align_watchdog_cycles", 180))
        self.align_force_pick_depth = float(rospy.get_param("~align_force_pick_depth", 1.05))
        self.align_progress_log_cycles = int(rospy.get_param("~align_progress_log_cycles", 45))
        self.align_kp = float(rospy.get_param("~align_kp", 0.003))
        self.search_yaw_rate = float(rospy.get_param("~search_yaw_rate", 0.35))
        self.search_spin_when_no_target = bool(rospy.get_param("~search_spin_when_no_target", True))
        self.approach_realign_error_px = float(
            rospy.get_param(
                "~approach_realign_error_px",
                max(self.near_pick_align_deadband_px * 2.0, 220.0),
            )
        )
        self.approach_speed = float(rospy.get_param("~approach_speed", 0.12))
        self.pick_stop_depth = float(rospy.get_param("~pick_stop_depth", 1.08))
        self.lost_depth_fallback = float(rospy.get_param("~lost_depth_fallback", 1.35))
        self.max_target_lost_cycles = int(rospy.get_param("~max_target_lost_cycles", 8))
        self.vision_topic_timeout = float(rospy.get_param("~vision_topic_timeout", 5.0))
        self.pick_blind_push_seconds = float(rospy.get_param("~pick_blind_push_seconds", 2.6))
        self.pick_blind_push_speed = float(rospy.get_param("~pick_blind_push_speed", 0.12))
        self.pick_insert_distance = float(rospy.get_param("~pick_insert_distance", 0.0))
        self.pick_insert_tolerance = float(rospy.get_param("~pick_insert_tolerance", 0.006))
        self.wpb_direct_pick_after_color_lock = bool(rospy.get_param("~wpb_direct_pick_after_color_lock", True))
        self.wpb_pregrab_distance_guard_enabled = bool(rospy.get_param("~wpb_pregrab_distance_guard_enabled", True))
        self.wpb_pregrab_min_depth = float(rospy.get_param("~wpb_pregrab_min_depth", 0.85))
        self.wpb_pregrab_target_depth = float(rospy.get_param("~wpb_pregrab_target_depth", 1.00))
        self.wpb_pregrab_max_backup = float(rospy.get_param("~wpb_pregrab_max_backup", 0.40))
        self.pick_lift_min_clearance = float(rospy.get_param("~pick_lift_min_clearance", 0.02))
        self.pick_lift_height = float(rospy.get_param("~pick_lift_height", self.field.pick_lift_height))
        self.carry_lift_height = float(rospy.get_param("~carry_lift_height", self.field.carry_lift_height))
        min_pick_lift = self.field.table_height + self.pick_lift_min_clearance
        if self.pick_lift_height < min_pick_lift:
            rospy.logwarn(
                "[PICK] pick_lift_height %.3f below table safety %.3f, clamping to avoid tabletop collision",
                self.pick_lift_height,
                min_pick_lift,
            )
            self.pick_lift_height = min_pick_lift
        self.demo_mode = str(rospy.get_param("~demo_mode", "none")).strip().lower()
        self.demo_target_color = str(rospy.get_param("~demo_target_color", "auto")).strip().lower()
        self.demo_travel_lift_height = float(rospy.get_param("~demo_travel_lift_height", self.carry_lift_height))
        self.demo_grasp_lift_height = float(rospy.get_param("~demo_grasp_lift_height", self.pick_lift_height))
        self.demo_pick_insert_distance = float(rospy.get_param("~demo_pick_insert_distance", self.pick_insert_distance))
        self.demo_stop_after_pick = bool(rospy.get_param("~demo_stop_after_pick", self.demo_mode == "pick_only"))
        if self.demo_grasp_lift_height < min_pick_lift:
            rospy.logwarn(
                "[PICK-DEMO] demo_grasp_lift_height %.3f below table safety %.3f, clamping",
                self.demo_grasp_lift_height,
                min_pick_lift,
            )
            self.demo_grasp_lift_height = min_pick_lift
        self.open_gripper = float(rospy.get_param("~open_gripper", self.field.open_gripper))
        self.closed_gripper = float(rospy.get_param("~closed_gripper", self.field.closed_gripper))
        self.arm_open_seconds = float(rospy.get_param("~arm_open_seconds", 1.8))
        self.arm_close_seconds = float(rospy.get_param("~arm_close_seconds", 1.2))
        self.arm_lift_seconds = float(rospy.get_param("~arm_lift_seconds", 1.2))
        self.arm_place_seconds = float(rospy.get_param("~arm_place_seconds", 1.0))
        self.enable_pick_verification = bool(rospy.get_param("~enable_pick_verification", True))
        self.max_pick_retries = int(rospy.get_param("~max_pick_retries", 2))
        self.recover_back_distance = float(rospy.get_param("~recover_back_distance", 0.20))
        self.rotate_speed = float(rospy.get_param("~rotate_speed", 0.4))
        default_drop_hold_gripper = 0.025 if self.wpb_grab.enabled else self.closed_gripper
        self.drop_hold_gripper = float(rospy.get_param("~drop_hold_gripper", default_drop_hold_gripper))
        self.drop_open_gripper = float(rospy.get_param("~drop_open_gripper", self.open_gripper))
        self.drop_release_clearance = float(rospy.get_param("~drop_release_clearance", 0.05))
        self.drop_travel_lift_margin = float(rospy.get_param("~drop_travel_lift_margin", 0.10))
        self.drop_safe_lift_height = float(rospy.get_param("~drop_safe_lift_height", self.carry_lift_height))
        self.drop_release_confirm_seconds = float(rospy.get_param("~drop_release_confirm_seconds", 1.2))
        self.drop_open_seconds = float(rospy.get_param("~drop_open_seconds", max(self.arm_open_seconds, 2.0)))
        self.reset_arm_on_start = bool(rospy.get_param("~reset_arm_on_start", True))
        self.startup_arm_lift_height = float(rospy.get_param("~startup_arm_lift_height", self.carry_lift_height))
        self.startup_arm_gripper = float(rospy.get_param("~startup_arm_gripper", self.open_gripper))
        self.startup_arm_reset_seconds = float(rospy.get_param("~startup_arm_reset_seconds", 2.0))
        self.vision_arm_stow_enabled = bool(rospy.get_param("~vision_arm_stow_enabled", True))
        self.vision_arm_lift_height = float(rospy.get_param("~vision_arm_lift_height", 0.35))
        self.vision_arm_gripper = float(rospy.get_param("~vision_arm_gripper", 0.08))
        self.vision_arm_settle_seconds = float(rospy.get_param("~vision_arm_settle_seconds", 0.8))
        self.source_travel_arm_enabled = bool(rospy.get_param("~source_travel_arm_enabled", True))
        self.source_travel_lift_height = float(rospy.get_param("~source_travel_lift_height", self.field.table_height + 0.10))
        self.source_travel_gripper = float(rospy.get_param("~source_travel_gripper", self.vision_arm_gripper))
        self.approach_arm_guard_enabled = bool(rospy.get_param("~approach_arm_guard_enabled", True))
        self.vision_arm_ready = False
        self.arm_travel_ready = False
        self.tabletop_drop_mode = bool(rospy.get_param("~tabletop_drop_mode", False))
        self.tabletop_drop_yaw_scale = float(rospy.get_param("~tabletop_drop_yaw_scale", 0.45))
        self.tabletop_drop_settle_seconds = float(rospy.get_param("~tabletop_drop_settle_seconds", 0.8))
        self.tabletop_retreat_after_place = float(rospy.get_param("~tabletop_retreat_after_place", 0.0))
        self.tabletop_use_base_targets = bool(rospy.get_param("~tabletop_use_base_targets", False))
        self.tabletop_return_to_pick_pose = bool(rospy.get_param("~tabletop_return_to_pick_pose", True))
        self.tabletop_drop_base_targets = self._load_pose_targets(
            rospy.get_param("~tabletop_drop_base_targets", {})
        )
        self.tabletop_return_base_target = self._load_pose_target(
            rospy.get_param("~tabletop_return_base_target", {}),
            "tabletop_return_base_target",
        )
        self.drive_to_source_before_search = bool(rospy.get_param("~drive_to_source_before_search", False))
        self.tabletop_use_dynamic_stack_targets = bool(rospy.get_param("~tabletop_use_dynamic_stack_targets", False))
        self.tabletop_stack_anchors = self._load_pose_targets(
            rospy.get_param("~tabletop_stack_anchors", {})
        )
        self.pose_drive_refine_attempts = int(rospy.get_param("~pose_drive_refine_attempts", 0))
        self.pose_drive_pick_dist_tolerance = float(rospy.get_param("~pose_drive_pick_dist_tolerance", 0.035))
        self.pose_drive_pick_yaw_tolerance = float(rospy.get_param("~pose_drive_pick_yaw_tolerance", 0.080))
        self.pose_drive_pick_attempts = int(rospy.get_param("~pose_drive_pick_attempts", 3))
        self.pose_drive_drop_dist_tolerance = float(rospy.get_param("~pose_drive_drop_dist_tolerance", 0.08))
        self.pose_drive_drop_yaw_tolerance = float(rospy.get_param("~pose_drive_drop_yaw_tolerance", 0.14))
        self.enable_drop_anchor_correction = bool(rospy.get_param("~enable_drop_anchor_correction", True))
        self.drop_anchor_tol = float(rospy.get_param("~drop_anchor_tol", 0.04))
        self.drop_anchor_max_step = float(rospy.get_param("~drop_anchor_max_step", 0.20))
        self.report_exported = False

        self.verify_cx_tol = float(rospy.get_param("~verify_cx_tol", 120.0))
        self.verify_depth_tol = float(rospy.get_param("~verify_depth_tol", 0.20))
        self.verify_area_ratio = float(rospy.get_param("~verify_area_ratio", 0.45))
        vision_states = rospy.get_param(
            "~vision_enabled_states",
            ["SEARCH", "ALIGN", "APPROACH", "PICK", "RECOVER_RETRY"],
        )
        if isinstance(vision_states, str):
            vision_states = [s.strip() for s in vision_states.split(",") if s.strip()]
        self.vision_enabled_states = set(str(s) for s in vision_states)

        self.scene_profile = str(rospy.get_param("~scene_profile", "default"))
        self.enable_visual_drop_refine = bool(rospy.get_param("~enable_visual_drop_refine", True))
        self.visual_refine_cycles = int(rospy.get_param("~visual_refine_cycles", 10))
        self.visual_refine_deadband_px = float(rospy.get_param("~visual_refine_deadband_px", 24.0))
        self.visual_refine_depth_target = float(rospy.get_param("~visual_refine_depth_target", 0.95))
        self.visual_refine_depth_tol = float(rospy.get_param("~visual_refine_depth_tol", 0.10))
        self.visual_refine_yaw_gain = float(rospy.get_param("~visual_refine_yaw_gain", 0.0028))
        self.visual_refine_lin_gain = float(rospy.get_param("~visual_refine_lin_gain", 0.22))

        self._apply_scene_profile()
        self._validate_field_targets()
        self._log_runtime_config()
        self._reset_arm_for_startup()

    def _apply_scene_profile(self):
        profile = self.scene_profile.lower()
        if profile == "safe_demo":
            self.approach_speed = min(self.approach_speed, 0.10)
            self.max_target_lost_cycles = max(self.max_target_lost_cycles, 10)
            self.pick_blind_push_seconds = max(self.pick_blind_push_seconds, 2.8)
            self.drop_anchor_tol = min(self.drop_anchor_tol, 0.035)
        elif profile == "fast_demo":
            self.approach_speed = max(self.approach_speed, 0.14)
            self.max_target_lost_cycles = min(self.max_target_lost_cycles, 7)
            self.pick_blind_push_seconds = min(self.pick_blind_push_seconds, 2.4)
        elif profile == "tight_stack":
            self.drop_anchor_tol = min(self.drop_anchor_tol, 0.025)
            self.drop_anchor_max_step = min(self.drop_anchor_max_step, 0.14)
            self.enable_visual_drop_refine = True
            self.visual_refine_cycles = max(self.visual_refine_cycles, 12)
        rospy.loginfo("[PROFILE] scene_profile=%s" % self.scene_profile)

    def _log_runtime_config(self):
        rospy.loginfo("[CONFIG] field_dimensions=%s" % json.dumps(self.field.to_dict(), sort_keys=True))
        rospy.loginfo(
            "[CONFIG] active_colors=%s max_per_color=%d base_drop_lift=%.3f stack_lift_step=%.3f"
            % (
                ",".join(self.planner.active_colors),
                self.planner.max_per_color,
                self.planner.base_drop_lift,
                self.planner.stack_lift_step,
            )
        )
        rospy.loginfo(
            "[CONFIG] pick_lift=%.3f carry_lift=%.3f gripper_open=%.3f gripper_closed=%.3f center_x=%.1f"
            % (
                self.pick_lift_height,
                self.carry_lift_height,
                self.open_gripper,
                self.closed_gripper,
                self.center_x,
            )
        )
        rospy.loginfo(
            "[CONFIG] demo_mode=%s demo_target_color=%s demo_travel_lift=%.3f demo_grasp_lift=%.3f demo_insert=%.3f stop_after_pick=%s"
            % (
                self.demo_mode,
                self.demo_target_color,
                self.demo_travel_lift_height,
                self.demo_grasp_lift_height,
                self.demo_pick_insert_distance,
                str(self.demo_stop_after_pick),
            )
        )
        rospy.loginfo(
            "[CONFIG] wpb_grab_action enabled=%s objects=%s action=%s result=%s object_timeout=%.1f result_timeout=%.1f stop_detect_after_pose=%s"
            % (
                str(self.wpb_grab.enabled),
                self.wpb_grab.objects_topic,
                self.wpb_grab.grab_action_topic,
                self.wpb_grab.grab_result_topic,
                self.wpb_grab.object_wait_timeout,
                self.wpb_grab.result_timeout,
                str(self.wpb_grab.stop_object_detect_after_grab_pose),
            )
        )
        rospy.loginfo(
            "[CONFIG] drop hold_gripper=%.3f open_gripper=%.3f release_clearance=%.3f safe_lift=%.3f travel_margin=%.3f open_seconds=%.2f confirm=%.2f"
            % (
                self.drop_hold_gripper,
                self.drop_open_gripper,
                self.drop_release_clearance,
                self.drop_safe_lift_height,
                self.drop_travel_lift_margin,
                self.drop_open_seconds,
                self.drop_release_confirm_seconds,
            )
        )
        rospy.loginfo(
            "[CONFIG] wpb_direct_pick=%s pregrab_guard=%s min_depth=%.2f target_depth=%.2f max_backup=%.2f"
            % (
                str(self.wpb_direct_pick_after_color_lock),
                str(self.wpb_pregrab_distance_guard_enabled),
                self.wpb_pregrab_min_depth,
                self.wpb_pregrab_target_depth,
                self.wpb_pregrab_max_backup,
            )
        )
        rospy.loginfo(
            "[CONFIG] reset_arm_on_start=%s startup_lift=%.3f startup_gripper=%.3f reset_seconds=%.2f"
            % (
                str(self.reset_arm_on_start),
                self.startup_arm_lift_height,
                self.startup_arm_gripper,
                self.startup_arm_reset_seconds,
            )
        )
        rospy.loginfo(
            "[CONFIG] vision_arm_stow enabled=%s lift=%.3f gripper=%.3f settle=%.2f source_travel_lift=%.3f source_travel_gripper=%.3f approach_guard=%s"
            % (
                str(self.vision_arm_stow_enabled),
                self.vision_arm_lift_height,
                self.vision_arm_gripper,
                self.vision_arm_settle_seconds,
                self.source_travel_lift_height,
                self.source_travel_gripper,
                str(self.approach_arm_guard_enabled),
            )
        )
        rospy.loginfo(
            "[CONFIG] localization_watchdog enabled=%s scan_timeout=%.1f amcl_timeout=%.1f"
            % (
                str(self.localization_watchdog_enabled),
                self.scan_watchdog_timeout,
                self.amcl_watchdog_timeout,
            )
        )
        rospy.loginfo("[CONFIG] emergency_stop_topic=%s" % self.emergency_stop_topic)
        rospy.loginfo(
            "[CONFIG] status_topic=%s marker_topic=%s frame=%s vision_states=%s"
            % (
                self.status.status_topic,
                self.status.marker_topic,
                self.status.frame_id,
                ",".join(sorted(self.vision_enabled_states)),
            )
        )

    def _validate_field_targets(self):
        if not self.tabletop_use_base_targets:
            return
        bad_targets = []
        if self._is_placeholder_pose(self.tabletop_return_base_target):
            bad_targets.append("tabletop_return_base_target")
        for color in self.planner.active_colors:
            if self._is_placeholder_pose(self.tabletop_drop_base_targets.get(color)):
                bad_targets.append("tabletop_drop_base_targets/%s" % color)
        if bad_targets:
            raise RuntimeError(
                "field target looks uncalibrated: %s; recapture A/B/C or fix abc_zones.yaml"
                % ", ".join(bad_targets)
            )

    def _is_placeholder_pose(self, pose: Optional[Pose2D]) -> bool:
        if pose is None:
            return True
        return abs(pose.x) < 0.05 and abs(pose.y) < 0.05 and abs(pose.yaw) < 0.05

    def _reset_arm_for_startup(self):
        if not self.reset_arm_on_start:
            return
        if self.vision_arm_stow_enabled:
            self._set_arm_for_vision("startup", force=True)
            return
        safe_lift = max(self.startup_arm_lift_height, self.field.table_height + self.drop_release_clearance)
        rospy.loginfo(
            "[STARTUP] reset arm lift=%.3f gripper=%.3f table=%.3f",
            safe_lift,
            self.startup_arm_gripper,
            self.field.table_height,
        )
        self._hold_lift_and_gripper(
            lift=safe_lift,
            gripper=self.startup_arm_gripper,
            duration=self.startup_arm_reset_seconds,
        )
        self.vision_arm_ready = False
        self.arm_travel_ready = False

    def _set_arm_for_vision(self, reason: str, force: bool = False):
        if not self.vision_arm_stow_enabled:
            return
        if self.vision_arm_ready and not force:
            return
        rospy.loginfo(
            "[ARM] vision stow reason=%s lift=%.3f gripper=%.3f",
            reason,
            self.vision_arm_lift_height,
            self.vision_arm_gripper,
        )
        self._hold_lift_and_gripper(
            lift=self.vision_arm_lift_height,
            gripper=self.vision_arm_gripper,
            duration=self.vision_arm_settle_seconds,
        )
        self.vision_arm_ready = True
        self.arm_travel_ready = False

    def _set_arm_for_source_travel(self, reason: str, force: bool = False):
        if not self.source_travel_arm_enabled:
            return
        if self.arm_travel_ready and not force:
            return
        travel_lift = max(self.source_travel_lift_height, self.field.table_height + self.drop_release_clearance)
        rospy.loginfo(
            "[ARM] source travel reason=%s lift=%.3f gripper=%.3f table=%.3f",
            reason,
            travel_lift,
            self.source_travel_gripper,
            self.field.table_height,
        )
        self._hold_lift_and_gripper(
            lift=travel_lift,
            gripper=self.source_travel_gripper,
            duration=self.vision_arm_settle_seconds,
        )
        self.vision_arm_ready = False
        self.arm_travel_ready = True

    def _set_state(self, new_state: str, reason: str):
        if self.state == new_state and self.state_reason == reason:
            return
        rospy.loginfo("[STATE] %s -> %s reason=%s" % (self.state, new_state, reason))
        self.state = new_state
        self.state_reason = reason

    def _load_pose_targets(self, raw_targets):
        targets = {}
        if not isinstance(raw_targets, dict):
            return targets
        for color, raw in raw_targets.items():
            target = self._load_pose_target(raw, "tabletop base target color=%s" % str(color))
            if target is not None:
                targets[str(color)] = target
        return targets

    def _load_pose_target(self, raw, label: str) -> Optional[Pose2D]:
        if not isinstance(raw, dict) or not raw:
            return None
        try:
            return Pose2D(
                x=float(raw["x"]),
                y=float(raw["y"]),
                yaw=float(raw["yaw"]),
            )
        except (KeyError, TypeError, ValueError):
            rospy.logwarn("[DROP] invalid %s" % label)
            return None

    def _target_detection(self) -> Optional[Detection]:
        if not self.target_color:
            return None
        return self.perception.get_detections().get(self.target_color)

    def _clamp(self, value, low, high):
        return max(low, min(high, value))

    def _normalize_angle(self, angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def _rotate_by(self, yaw):
        ok, err = self.ctrl.rotate_angle(yaw, abort_check=self._localization_fault_reason)
        self.metrics.add_motion_result("rotate", ok, abs(err))
        if not ok:
            fault = self._localization_fault_reason()
            if fault:
                self._enter_localization_fault(fault)
        return ok

    def _forward_by(self, distance, dist_tolerance: Optional[float] = None):
        ok, err = self.ctrl.move_distance(
            distance,
            dist_tolerance=dist_tolerance,
            abort_check=self._localization_fault_reason,
        )
        self.metrics.add_motion_result("forward", ok, abs(err))
        if not ok:
            fault = self._localization_fault_reason()
            if fault:
                self._enter_localization_fault(fault)
        return ok

    def _drive_pose(self) -> Optional[Pose2D]:
        pose = self.gazebo_carry.robot_pose()
        if pose is not None:
            return pose
        return self.ctrl.get_pose()

    def _wait_for_drive_pose(self, reason: str) -> Optional[Pose2D]:
        pose = self._drive_pose()
        if pose is not None:
            return pose
        self.ctrl.stop_base()
        timeout = max(0.0, self.ctrl.pose_recovery_timeout)
        deadline = rospy.Time.now().to_sec() + timeout
        rate = rospy.Rate(max(1.0, self.ctrl.pose_recovery_poll_hz))
        while not rospy.is_shutdown():
            pose = self._drive_pose()
            if pose is not None:
                return pose
            self.ctrl.stop_base()
            if rospy.Time.now().to_sec() >= deadline:
                break
            rospy.logwarn_throttle(
                1.0,
                "[POSE] waiting for drive pose recovery reason=%s timeout=%.1fs",
                reason,
                timeout,
            )
            rate.sleep()
        self.ctrl.stop_base()
        return None

    def _use_pose_feedback_drive(self) -> bool:
        return self.gazebo_carry.enabled and self.gazebo_carry.use_robot_model_pose

    def _rotate_to_yaw_feedback(self, target_yaw: float, yaw_limit: float) -> bool:
        rate = rospy.Rate(30)
        deadline = rospy.Time.now().to_sec() + self.ctrl.rotate_timeout
        final_error = 999.0

        while not rospy.is_shutdown():
            fault = self._localization_fault_reason()
            if fault:
                self.ctrl.stop_base()
                self.metrics.add_motion_result("rotate", False, 999.0)
                self._enter_localization_fault(fault)
                return False
            current = self._wait_for_drive_pose("feedback_rotate")
            if current is None:
                return False

            final_error = self._normalize_angle(target_yaw - current.yaw)
            if abs(final_error) <= yaw_limit:
                self.ctrl.stop_base()
                self.metrics.add_motion_result("rotate", True, abs(final_error))
                return True

            if rospy.Time.now().to_sec() > deadline:
                self.ctrl.stop_base()
                self.metrics.add_motion_result("rotate", False, abs(final_error))
                return False

            speed = self.ctrl.yaw_kp * abs(final_error)
            speed = max(self.ctrl.min_yaw_speed, min(self.ctrl.max_yaw_speed, speed))
            self.ctrl.publish_vel(angular_z=speed if final_error >= 0.0 else -speed)
            rate.sleep()

        self.ctrl.stop_base()
        self.metrics.add_motion_result("rotate", False, abs(final_error))
        return False

    def _drive_to_xy_feedback(self, target: Pose2D, dist_limit: float) -> bool:
        rate = rospy.Rate(30)
        deadline = rospy.Time.now().to_sec() + self.ctrl.move_timeout
        final_dist = 999.0

        while not rospy.is_shutdown():
            fault = self._localization_fault_reason()
            if fault:
                self.ctrl.stop_base()
                self.metrics.add_motion_result("forward", False, 999.0)
                self._enter_localization_fault(fault)
                return False
            current = self._wait_for_drive_pose("feedback_xy")
            if current is None:
                return False

            dx = target.x - current.x
            dy = target.y - current.y
            final_dist = math.hypot(dx, dy)
            if final_dist <= dist_limit:
                self.ctrl.stop_base()
                self.metrics.add_motion_result("forward", True, final_dist)
                return True

            if rospy.Time.now().to_sec() > deadline:
                self.ctrl.stop_base()
                self.metrics.add_motion_result("forward", False, final_dist)
                return False

            body_x = math.cos(current.yaw) * dx + math.sin(current.yaw) * dy
            body_y = -math.sin(current.yaw) * dx + math.cos(current.yaw) * dy
            yaw_error = self._normalize_angle(target.yaw - current.yaw)
            lin_speed = self.ctrl.lin_kp * final_dist
            lin_speed = max(self.ctrl.min_lin_speed, min(self.ctrl.max_lin_speed, lin_speed))
            linear_x = body_x / final_dist * lin_speed
            linear_y = body_y / final_dist * lin_speed
            yaw_speed = self.ctrl.yaw_kp * abs(yaw_error)
            yaw_speed = min(self.ctrl.max_yaw_speed, yaw_speed)
            if abs(yaw_error) > 0.02:
                yaw_speed = max(self.ctrl.min_yaw_speed, yaw_speed)
            self.ctrl.publish_vel(
                linear_x=linear_x,
                linear_y=linear_y,
                angular_z=yaw_speed if yaw_error >= 0.0 else -yaw_speed,
            )
            rate.sleep()

        self.ctrl.stop_base()
        self.metrics.add_motion_result("forward", False, final_dist)
        return False

    def _drive_to_pose_feedback(
        self,
        target: Pose2D,
        label: str,
        dist_limit: float,
        yaw_limit: float,
        attempts: int,
    ) -> bool:
        ok = True
        final_dist = 999.0
        final_yaw_err = 999.0

        for _ in range(attempts + 1):
            current = self._wait_for_drive_pose("feedback_pose_start_%s" % label)
            if current is None:
                return False

            dx = target.x - current.x
            dy = target.y - current.y
            dist = math.hypot(dx, dy)
            if dist > dist_limit:
                ok = self._drive_to_xy_feedback(target, dist_limit) and ok

            current = self._wait_for_drive_pose("feedback_pose_yaw_%s" % label)
            if current is None:
                return False
            yaw_error = self._normalize_angle(target.yaw - current.yaw)
            if abs(yaw_error) > yaw_limit:
                ok = self._rotate_to_yaw_feedback(target.yaw, yaw_limit) and ok

            current = self._wait_for_drive_pose("feedback_pose_final_%s" % label)
            if current is None:
                return False
            final_dist = math.hypot(target.x - current.x, target.y - current.y)
            final_yaw_err = abs(self._normalize_angle(target.yaw - current.yaw))
            if final_dist <= dist_limit and final_yaw_err <= yaw_limit:
                break

        reached = final_dist <= dist_limit and final_yaw_err <= yaw_limit
        rospy.loginfo(
            "[POSE] %s target=(%.2f, %.2f, %.2f) reached=%s dist=%.3f yaw_err=%.3f"
            % (label, target.x, target.y, target.yaw, str(reached), final_dist, final_yaw_err)
        )
        return reached

    def _drive_to_pose(self, target: Optional[Pose2D], label: str):
        if target is None:
            return False
        is_drop = label.startswith("drop_base")
        is_pick = label.startswith("pick_base")
        if is_pick:
            dist_limit = self.pose_drive_pick_dist_tolerance
            yaw_limit = self.pose_drive_pick_yaw_tolerance
            attempts = max(0, self.pose_drive_pick_attempts)
        elif is_drop:
            dist_limit = self.pose_drive_drop_dist_tolerance
            yaw_limit = self.pose_drive_drop_yaw_tolerance
            attempts = max(0, self.pose_drive_refine_attempts)
        else:
            dist_limit = max(self.ctrl.dist_tolerance * 2.0, 0.14)
            yaw_limit = max(self.ctrl.yaw_tolerance * 2.0, 0.16)
            attempts = 1

        if self._use_pose_feedback_drive():
            return self._drive_to_pose_feedback(target, label, dist_limit, yaw_limit, attempts)

        ok = True
        final_dist = 999.0
        final_yaw_err = 999.0

        for _ in range(attempts + 1):
            current = self._wait_for_drive_pose("pose_start_%s" % label)
            if current is None:
                return False

            dx = target.x - current.x
            dy = target.y - current.y
            dist = math.hypot(dx, dy)
            if dist > dist_limit:
                yaw_to_target = math.atan2(dy, dx)
                ok = self._rotate_by(self._normalize_angle(yaw_to_target - current.yaw)) and ok
                ok = self._forward_by(dist) and ok

            current = self._wait_for_drive_pose("pose_yaw_%s" % label)
            if current is None:
                return False
            yaw_error = self._normalize_angle(target.yaw - current.yaw)
            if abs(yaw_error) > yaw_limit:
                ok = self._rotate_by(yaw_error) and ok

            current = self._wait_for_drive_pose("pose_final_%s" % label)
            if current is None:
                return False
            final_dist = math.hypot(target.x - current.x, target.y - current.y)
            final_yaw_err = abs(self._normalize_angle(target.yaw - current.yaw))
            if final_dist <= dist_limit and final_yaw_err <= yaw_limit:
                break

        reached = final_dist <= dist_limit and final_yaw_err <= yaw_limit
        rospy.loginfo(
            "[POSE] %s target=(%.2f, %.2f, %.2f) reached=%s dist=%.3f yaw_err=%.3f"
            % (label, target.x, target.y, target.yaw, str(reached), final_dist, final_yaw_err)
        )
        return reached

    def _base_target_for_stack_anchor(
        self,
        anchor: Optional[Pose2D],
        local_offset: Optional[Tuple[float, float, float]],
    ) -> Optional[Pose2D]:
        if anchor is None or local_offset is None:
            return None
        local_x, local_y, _ = local_offset
        return Pose2D(
            x=anchor.x - math.cos(anchor.yaw) * local_x + math.sin(anchor.yaw) * local_y,
            y=anchor.y - math.sin(anchor.yaw) * local_x - math.cos(anchor.yaw) * local_y,
            yaw=anchor.yaw,
        )

    def _lock_target(self, color: str, det: Detection):
        self.target_color = color
        self.last_depth = det.depth
        self.last_align_error = 0.0
        self.lost_cycles = 0
        self.pick_retry_count = 0
        self.pick_signature = None
        self.near_pick_align_cycles = 0
        self.align_watchdog_cycles = 0
        self.metrics.start_cycle(color)
        rospy.loginfo("Target locked: %s depth=%.2f" % (color, det.depth))

    def _clear_target(self):
        self.target_color = None
        self.last_depth = 0.0
        self.last_align_error = 0.0
        self.lost_cycles = 0
        self.pick_retry_count = 0
        self.pick_signature = None
        self.near_pick_align_cycles = 0
        self.align_watchdog_cycles = 0

    def _capture_pick_signature(self, det: Optional[Detection]):
        if det is None:
            self.pick_signature = None
            return
        self.pick_signature = PickSignature(cx=det.cx, cy=det.cy, depth=det.depth, area=det.area)

    def _drive_to_source_pick_target(self, color: str) -> bool:
        yaw_override = None
        if self.gazebo_carry.source_pick_use_current_yaw:
            current = self._drive_pose()
            if current is not None:
                yaw_override = current.yaw
        target = self.gazebo_carry.source_pick_base_target(color, yaw_override=yaw_override)
        if target is None and self.tabletop_use_base_targets:
            target = self.tabletop_return_base_target
        if target is None:
            return False
        rospy.loginfo(
            "[PICK] source base target color=%s target=(%.2f, %.2f, %.2f)"
            % (color, target.x, target.y, target.yaw)
        )
        reached = self._drive_to_pose(target, "pick_base_%s" % color)
        if reached:
            self._capture_pick_signature(self._target_detection())
        return reached

    def _active_pick_lift_height(self) -> float:
        if self.demo_mode == "pick_only":
            return self.demo_grasp_lift_height
        return self.pick_lift_height

    def _active_pick_insert_distance(self) -> float:
        if self.demo_mode == "pick_only":
            return self.demo_pick_insert_distance
        return self.pick_insert_distance

    def _prepare_pick_demo_travel_pose(self):
        if self.demo_mode != "pick_only" or self.demo_travel_lift_ready:
            return
        rospy.loginfo(
            "[PICK-DEMO] travel lift=%.3f open=%.3f",
            self.demo_travel_lift_height,
            self.open_gripper,
        )
        self.ctrl.move_lift_and_gripper(
            lift=self.demo_travel_lift_height,
            gripper=self.open_gripper,
            duration=self.arm_lift_seconds,
        )
        self.demo_travel_lift_ready = True

    def _prepare_wpb_pregrab_distance(self) -> bool:
        if not (self.wpb_grab.enabled and self.wpb_pregrab_distance_guard_enabled):
            return True
        det = self._target_detection()
        if det is None or det.depth <= 0.0:
            return True
        if det.depth >= self.wpb_pregrab_min_depth:
            return True
        backup = self.wpb_pregrab_target_depth - det.depth
        backup = max(0.0, min(self.wpb_pregrab_max_backup, backup))
        if backup <= 0.0:
            return True
        rospy.logwarn(
            "[PICK-WPB] target too close before grab_demo depth=%.2f < %.2f; backing up %.3fm",
            det.depth,
            self.wpb_pregrab_min_depth,
            backup,
        )
        ok = self._forward_by(-backup, dist_tolerance=self.pick_insert_tolerance)
        if ok:
            rospy.sleep(0.4)
        return ok

    def _pick_sequence(self) -> bool:
        self.metrics.mark_pick_attempt()
        self.vision_arm_ready = False
        self.arm_travel_ready = False
        if self.wpb_grab.enabled:
            if not self._prepare_wpb_pregrab_distance():
                return False
            rospy.loginfo("[PICK-WPB] delegate pick to wpb_home_grab_action color=%s" % self.target_color)
            return self.wpb_grab.execute(self.target_color, abort_check=self._localization_fault_reason)

        pick_lift_height = self._active_pick_lift_height()
        pick_insert_distance = self._active_pick_insert_distance()
        rospy.loginfo(
            "[PICK] prepare color=%s lift=%.3f carry=%.3f gripper_open=%.3f gripper_closed=%.3f box=(%.3f,%.3f,%.3f)"
            % (
                self.target_color,
                pick_lift_height,
                self.carry_lift_height,
                self.open_gripper,
                self.closed_gripper,
                self.field.box_x,
                self.field.box_y,
                self.field.box_z,
            )
        )
        self.ctrl.move_lift_and_gripper(
            lift=pick_lift_height,
            gripper=self.open_gripper,
            duration=self.arm_open_seconds,
        )

        if pick_insert_distance > 0.0:
            rospy.loginfo(
                "[PICK] insert forward %.3f tolerance=%.3f"
                % (pick_insert_distance, self.pick_insert_tolerance)
            )
            if not self._forward_by(pick_insert_distance, dist_tolerance=self.pick_insert_tolerance):
                rospy.logerr("[PICK] insert forward failed, aborting pick before closing gripper")
                return False
        elif self.pick_blind_push_seconds > 0.0:
            rospy.loginfo("[PICK] blind push")
            self.ctrl.drive_for(seconds=self.pick_blind_push_seconds, linear_x=self.pick_blind_push_speed)

        rospy.loginfo("[PICK] close gripper")
        self.ctrl.move_lift_and_gripper(
            lift=pick_lift_height,
            gripper=self.closed_gripper,
            duration=self.arm_close_seconds,
        )
        grasp_ok = self.gazebo_carry.attach(self.target_color)
        if not grasp_ok:
            return False

        rospy.loginfo("[PICK] lift")
        self.ctrl.move_lift_and_gripper(
            lift=self.carry_lift_height,
            gripper=self.closed_gripper,
            duration=self.arm_lift_seconds,
        )
        return self.gazebo_carry.verify_after_lift(self.target_color)

    def _pick_looks_failed(self) -> bool:
        if not self.enable_pick_verification:
            return False
        if self.pick_signature is None:
            return False
        det = self._target_detection()
        if det is None:
            return False

        same_pos = abs(det.cx - self.pick_signature.cx) <= self.verify_cx_tol
        same_depth = abs(det.depth - self.pick_signature.depth) <= self.verify_depth_tol
        same_area = det.area >= self.pick_signature.area * self.verify_area_ratio
        failed = same_pos and same_depth and same_area
        if failed:
            rospy.logwarn(
                "[VERIFY] pick_looks_failed color=%s cx_delta=%.1f depth_delta=%.3f area=%.1f signature_area=%.1f"
                % (
                    self.target_color,
                    abs(det.cx - self.pick_signature.cx),
                    abs(det.depth - self.pick_signature.depth),
                    det.area,
                    self.pick_signature.area,
                )
            )
        return failed

    def _recover_for_retry(self):
        rospy.logwarn("[RETRY] recover and reacquire")
        self.ctrl.move_lift_and_gripper(
            lift=self.carry_lift_height,
            gripper=self.open_gripper,
            duration=self.arm_open_seconds,
        )
        self._forward_by(-self.recover_back_distance)
        yaw = 0.16 if self.last_align_error >= 0 else -0.16
        self._rotate_by(yaw)
        self._set_arm_for_vision("recover_retry", force=True)

    def _hold_lift_and_gripper(self, lift: float, gripper: float, duration: float, hz: float = 15.0):
        end_t = rospy.Time.now().to_sec() + max(0.0, duration)
        rate = rospy.Rate(hz)
        self.ctrl.set_lift_and_gripper(lift=lift, gripper=gripper)
        while not rospy.is_shutdown() and rospy.Time.now().to_sec() < end_t:
            self.ctrl.set_lift_and_gripper(lift=lift, gripper=gripper)
            rate.sleep()

    def _safe_drop_release_lift(self, planned_lift: float) -> float:
        min_lift = self.field.table_height + self.drop_release_clearance
        if planned_lift < min_lift:
            rospy.logwarn(
                "[DROP] planned release lift %.3f below table safety %.3f, clamping",
                planned_lift,
                min_lift,
            )
        return max(planned_lift, min_lift)

    def _safe_drop_travel_lift(self, release_lift: float) -> float:
        return max(
            self.drop_safe_lift_height,
            release_lift + self.drop_travel_lift_margin,
            self.field.table_height + self.drop_release_clearance,
        )

    def _prepare_payload_for_drop_travel(self, release_lift: float):
        travel_lift = self._safe_drop_travel_lift(release_lift)
        rospy.loginfo(
            "[DROP] travel safe pose lift=%.3f hold_gripper=%.3f table=%.3f",
            travel_lift,
            self.drop_hold_gripper,
            self.field.table_height,
        )
        self._hold_lift_and_gripper(
            lift=travel_lift,
            gripper=self.drop_hold_gripper,
            duration=self.arm_lift_seconds,
        )
        self.vision_arm_ready = False
        self.arm_travel_ready = False

    def _release_payload_on_table(self, color: str, release_lift: float):
        rospy.loginfo(
            "[DROP] release color=%s lift=%.3f open=%.3f hold=%.3f clearance=%.3f",
            color,
            release_lift,
            self.drop_open_gripper,
            self.drop_hold_gripper,
            release_lift - self.field.table_height,
        )
        if release_lift < self.field.table_height + self.drop_release_clearance:
            rospy.logerr(
                "[DROP] release lift %.3f is too close to table %.3f, abort release",
                release_lift,
                self.field.table_height,
            )
            return False

        self.ctrl.move_lift_and_gripper(
            lift=release_lift,
            gripper=self.drop_hold_gripper,
            duration=self.arm_place_seconds,
        )
        self.vision_arm_ready = False
        self.arm_travel_ready = False
        rospy.sleep(self.tabletop_drop_settle_seconds)
        self.ctrl.stop_base(repeat_seconds=0.2)
        rospy.loginfo(
            "[DROP] open gripper to max before retreat open=%.3f duration=%.2fs",
            self.drop_open_gripper,
            self.drop_open_seconds,
        )
        self.ctrl.move_lift_and_gripper(
            lift=release_lift,
            gripper=self.drop_open_gripper,
            duration=self.drop_open_seconds,
        )
        rospy.loginfo("[DROP] release confirm wait %.2fs", self.drop_release_confirm_seconds)
        self._hold_lift_and_gripper(
            lift=release_lift,
            gripper=self.drop_open_gripper,
            duration=self.drop_release_confirm_seconds,
        )
        self.ctrl.stop_base(repeat_seconds=0.2)
        self.gazebo_carry.place_and_detach()
        return True

    def _drop_sequence(self, color: str):
        if self.tabletop_drop_mode:
            return self._tabletop_drop_sequence(color)

        yaw, forward, planned_drop_lift = self.planner.get_drop_plan(color)
        drop_lift = self._safe_drop_release_lift(planned_drop_lift)
        rospy.loginfo("[DROP] move to zone color=%s" % color)
        self._prepare_payload_for_drop_travel(drop_lift)
        if not self._rotate_by(yaw):
            rospy.logerr("[DROP] rotate to zone failed")
            return False
        if not self._forward_by(forward):
            rospy.logerr("[DROP] forward to zone failed")
            return False

        self._visual_refine_drop_alignment(color)
        self._correct_drop_pose_to_anchor(color)

        if not self._release_payload_on_table(color, drop_lift):
            return False

        self.metrics.add_drop_pose(color, self.ctrl.get_pose())

        rospy.loginfo("[DROP] retreat and restore heading")
        if not self._forward_by(-forward):
            rospy.logerr("[DROP] retreat from zone failed")
            return False
        self._hold_lift_and_gripper(
            lift=self._safe_drop_travel_lift(drop_lift),
            gripper=self.drop_open_gripper,
            duration=self.arm_lift_seconds,
        )
        if not self._rotate_by(-yaw):
            rospy.logerr("[DROP] restore heading failed")
            return False
        return True

    def _tabletop_drop_sequence(self, color: str):
        yaw, forward, planned_drop_lift = self.planner.get_drop_plan(color)
        drop_lift = self._safe_drop_release_lift(planned_drop_lift)
        yaw = yaw * self.tabletop_drop_yaw_scale
        rospy.loginfo(
            "[DROP] tabletop place color=%s layer=%d yaw=%.3f forward=%.3f lift=%.3f planned_lift=%.3f settle=%.2f"
            % (
                color,
                self.planner.stack_count.get(color, 0) + 1,
                yaw,
                forward,
                drop_lift,
                planned_drop_lift,
                self.tabletop_drop_settle_seconds,
            )
        )
        pick_pose = self.ctrl.get_pose()
        base_target = self.tabletop_drop_base_targets.get(color)
        if self.tabletop_use_dynamic_stack_targets:
            dynamic_target = self._base_target_for_stack_anchor(
                self.tabletop_stack_anchors.get(color),
                self.gazebo_carry.get_current_local_offset(),
            )
            if dynamic_target is not None:
                base_target = dynamic_target
        self._prepare_payload_for_drop_travel(drop_lift)
        if self.tabletop_use_base_targets and base_target is not None:
            rospy.loginfo(
                "[DROP] tabletop base target color=%s target=(%.3f, %.3f, %.3f)",
                color,
                base_target.x,
                base_target.y,
                base_target.yaw,
            )
            if not self._drive_to_pose(base_target, "drop_base_%s" % color):
                rospy.logerr("[DROP] failed to reach tabletop drop base target for %s", color)
                return False
        else:
            if not self._rotate_by(yaw):
                rospy.logerr("[DROP] fallback rotate failed for %s", color)
                return False
            if not self._forward_by(forward):
                rospy.logerr("[DROP] fallback forward failed for %s", color)
                return False
        if not self._release_payload_on_table(color, drop_lift):
            return False
        self.metrics.add_drop_pose(color, self.ctrl.get_pose())

        retreated = False
        if self.tabletop_retreat_after_place > 0.0:
            rospy.loginfo("[DROP] retreat from tabletop %.3f with gripper open", self.tabletop_retreat_after_place)
            if not self._forward_by(-self.tabletop_retreat_after_place):
                rospy.logerr("[DROP] retreat from tabletop failed for %s", color)
                return False
            retreated = True

        self._hold_lift_and_gripper(
            lift=self._safe_drop_travel_lift(drop_lift),
            gripper=self.drop_open_gripper,
            duration=self.arm_lift_seconds,
        )
        if self.tabletop_use_base_targets and self.tabletop_return_to_pick_pose and pick_pose is not None:
            return_target = self.tabletop_return_base_target or pick_pose
            if not self._drive_to_pose(return_target, "return_pick_%s" % color):
                rospy.logerr("[DROP] failed to return to pick pose for %s", color)
                return False
        else:
            if retreated:
                remaining = max(0.0, forward - self.tabletop_retreat_after_place)
                if remaining > 0.0:
                    if not self._forward_by(-remaining):
                        rospy.logerr("[DROP] fallback return forward failed for %s", color)
                        return False
            else:
                if not self._forward_by(-forward):
                    rospy.logerr("[DROP] fallback return backward failed for %s", color)
                    return False
            if not self._rotate_by(-yaw):
                rospy.logerr("[DROP] fallback return rotate failed for %s", color)
                return False
        return True

    def _visual_refine_drop_alignment(self, color: str):
        if not self.enable_visual_drop_refine:
            return

        rate = rospy.Rate(12)
        converged = False
        for _ in range(self.visual_refine_cycles):
            if rospy.is_shutdown():
                break

            det = self.perception.get_detections().get(color)
            if det is None:
                self.ctrl.publish_vel(angular_z=0.08)
                rate.sleep()
                continue

            # Ignore too-close artifacts that may come from self-view or gripper reflections.
            if det.depth < 0.55:
                self.ctrl.publish_vel(angular_z=0.06)
                rate.sleep()
                continue

            err_x = self.center_x - det.cx
            err_d = det.depth - self.visual_refine_depth_target
            if abs(err_x) <= self.visual_refine_deadband_px and abs(err_d) <= self.visual_refine_depth_tol:
                converged = True
                break

            yaw = self._clamp(err_x * self.visual_refine_yaw_gain, -0.22, 0.22)
            lin = self._clamp(err_d * self.visual_refine_lin_gain, -0.08, 0.08)
            self.ctrl.publish_vel(linear_x=lin, angular_z=yaw)
            rate.sleep()

        self.ctrl.stop_base()
        self.metrics.add_motion_result("drop_visual_refine", converged, 0.0 if converged else 1.0)

    def _correct_drop_pose_to_anchor(self, color: str):
        if not self.enable_drop_anchor_correction:
            return

        current = self.ctrl.get_pose()
        if current is None:
            return

        anchor = self.planner.get_drop_anchor(color)
        if anchor is None:
            self.planner.set_drop_anchor_if_empty(color, current)
            rospy.loginfo("[DROP-ANCHOR] set anchor color=%s" % color)
            return

        dx = anchor.x - current.x
        dy = anchor.y - current.y
        dist = math.hypot(dx, dy)
        self.metrics.add_anchor_error(color, dist)
        rospy.loginfo("[DROP-ANCHOR] color=%s error=%.3f" % (color, dist))

        if dist <= self.drop_anchor_tol:
            return

        step = min(dist, self.drop_anchor_max_step)
        desired_yaw = math.atan2(dy, dx)
        yaw_turn = self._normalize_angle(desired_yaw - current.yaw)

        self._rotate_by(yaw_turn)
        self._forward_by(step)
        self._rotate_by(-yaw_turn)

    def _pose_to_dict(self, pose: Optional[Pose2D]):
        if pose is None:
            return None
        return {"x": pose.x, "y": pose.y, "yaw": pose.yaw}

    def _params_snapshot(self):
        return {
            "scene_profile": self.scene_profile,
            "field_dimensions": self.field.to_dict(),
            "active_colors": list(self.planner.active_colors),
            "camera_center_x": self.center_x,
            "align_deadband_px": self.deadband_px,
            "near_pick_align_deadband_px": self.near_pick_align_deadband_px,
            "max_near_pick_align_cycles": self.max_near_pick_align_cycles,
            "max_align_watchdog_cycles": self.max_align_watchdog_cycles,
            "align_force_pick_depth": self.align_force_pick_depth,
            "align_progress_log_cycles": self.align_progress_log_cycles,
            "align_kp": self.align_kp,
            "search_yaw_rate": self.search_yaw_rate,
            "search_spin_when_no_target": self.search_spin_when_no_target,
            "approach_realign_error_px": self.approach_realign_error_px,
            "depth_unit_auto_scale": self.perception.depth_unit_auto_scale,
            "depth_mm_threshold": self.perception.depth_mm_threshold,
            "depth_scale": self.perception.depth_scale,
            "square_filter_enabled": self.perception.square_filter_enabled,
            "square_min_side_px": self.perception.square_min_side_px,
            "square_max_aspect_ratio": self.perception.square_max_aspect_ratio,
            "square_min_fill_ratio": self.perception.square_min_fill_ratio,
            "approach_speed": self.approach_speed,
            "pick_stop_depth": self.pick_stop_depth,
            "lost_depth_fallback": self.lost_depth_fallback,
            "max_target_lost_cycles": self.max_target_lost_cycles,
            "vision_topic_timeout": self.vision_topic_timeout,
            "pick_blind_push_seconds": self.pick_blind_push_seconds,
            "pick_blind_push_speed": self.pick_blind_push_speed,
            "pick_insert_distance": self.pick_insert_distance,
            "pick_insert_tolerance": self.pick_insert_tolerance,
            "wpb_direct_pick_after_color_lock": self.wpb_direct_pick_after_color_lock,
            "wpb_pregrab_distance_guard_enabled": self.wpb_pregrab_distance_guard_enabled,
            "wpb_pregrab_min_depth": self.wpb_pregrab_min_depth,
            "wpb_pregrab_target_depth": self.wpb_pregrab_target_depth,
            "wpb_pregrab_max_backup": self.wpb_pregrab_max_backup,
            "pick_lift_min_clearance": self.pick_lift_min_clearance,
            "pick_lift_height": self.pick_lift_height,
            "carry_lift_height": self.carry_lift_height,
            "demo_mode": self.demo_mode,
            "demo_target_color": self.demo_target_color,
            "demo_travel_lift_height": self.demo_travel_lift_height,
            "demo_grasp_lift_height": self.demo_grasp_lift_height,
            "demo_pick_insert_distance": self.demo_pick_insert_distance,
            "demo_stop_after_pick": self.demo_stop_after_pick,
            "open_gripper": self.open_gripper,
            "closed_gripper": self.closed_gripper,
            "drop_hold_gripper": self.drop_hold_gripper,
            "drop_open_gripper": self.drop_open_gripper,
            "drop_release_clearance": self.drop_release_clearance,
            "drop_safe_lift_height": self.drop_safe_lift_height,
            "drop_travel_lift_margin": self.drop_travel_lift_margin,
            "drop_open_seconds": self.drop_open_seconds,
            "drop_release_confirm_seconds": self.drop_release_confirm_seconds,
            "reset_arm_on_start": self.reset_arm_on_start,
            "startup_arm_lift_height": self.startup_arm_lift_height,
            "startup_arm_gripper": self.startup_arm_gripper,
            "startup_arm_reset_seconds": self.startup_arm_reset_seconds,
            "vision_arm_stow_enabled": self.vision_arm_stow_enabled,
            "vision_arm_lift_height": self.vision_arm_lift_height,
            "vision_arm_gripper": self.vision_arm_gripper,
            "vision_arm_settle_seconds": self.vision_arm_settle_seconds,
            "source_travel_arm_enabled": self.source_travel_arm_enabled,
            "source_travel_lift_height": self.source_travel_lift_height,
            "source_travel_gripper": self.source_travel_gripper,
            "approach_arm_guard_enabled": self.approach_arm_guard_enabled,
            "arm_open_seconds": self.arm_open_seconds,
            "arm_close_seconds": self.arm_close_seconds,
            "arm_lift_seconds": self.arm_lift_seconds,
            "arm_place_seconds": self.arm_place_seconds,
            "enable_pick_verification": self.enable_pick_verification,
            "verify_cx_tol": self.verify_cx_tol,
            "verify_depth_tol": self.verify_depth_tol,
            "verify_area_ratio": self.verify_area_ratio,
            "tabletop_drop_mode": self.tabletop_drop_mode,
            "tabletop_drop_yaw_scale": self.tabletop_drop_yaw_scale,
            "tabletop_retreat_after_place": self.tabletop_retreat_after_place,
            "tabletop_use_base_targets": self.tabletop_use_base_targets,
            "tabletop_use_dynamic_stack_targets": self.tabletop_use_dynamic_stack_targets,
            "tabletop_return_to_pick_pose": self.tabletop_return_to_pick_pose,
            "tabletop_return_base_target": self._pose_to_dict(self.tabletop_return_base_target),
            "pose_drive_refine_attempts": self.pose_drive_refine_attempts,
            "pose_drive_pick_dist_tolerance": self.pose_drive_pick_dist_tolerance,
            "pose_drive_pick_yaw_tolerance": self.pose_drive_pick_yaw_tolerance,
            "pose_drive_pick_attempts": self.pose_drive_pick_attempts,
            "pose_drive_drop_dist_tolerance": self.pose_drive_drop_dist_tolerance,
            "pose_drive_drop_yaw_tolerance": self.pose_drive_drop_yaw_tolerance,
            "gazebo_attach_on_pick": self.gazebo_carry.attach_enabled,
            "gazebo_use_robot_model_pose": self.gazebo_carry.use_robot_model_pose,
            "gazebo_verify_physical_pick": self.gazebo_carry.verify_physical_pick,
            "gazebo_physical_pick_min_lift": self.gazebo_carry.physical_pick_min_lift,
            "gazebo_validate_grasp_window": self.gazebo_carry.validate_grasp_window,
            "gazebo_grasp_max_xy_error": self.gazebo_carry.grasp_max_xy_error,
            "gazebo_grasp_max_z_error": self.gazebo_carry.grasp_max_z_error,
            "gazebo_use_source_pick_targets": self.gazebo_carry.use_source_pick_targets,
            "gazebo_source_pick_yaw": self.gazebo_carry.source_pick_yaw,
            "gazebo_source_pick_use_current_yaw": self.gazebo_carry.source_pick_use_current_yaw,
            "gazebo_source_pick_forward_offset": self.gazebo_carry.source_pick_forward_offset,
            "gazebo_source_pick_side_offset": self.gazebo_carry.source_pick_side_offset,
            "gazebo_stabilize_stack_on_release": self.gazebo_carry.stabilize_stack_on_release,
            "gazebo_stable_stack_base_z": self.gazebo_carry.stable_stack_base_z,
            "max_pick_retries": self.max_pick_retries,
            "recover_back_distance": self.recover_back_distance,
            "enable_drop_anchor_correction": self.enable_drop_anchor_correction,
            "drop_anchor_tol": self.drop_anchor_tol,
            "drop_anchor_max_step": self.drop_anchor_max_step,
            "enable_visual_drop_refine": self.enable_visual_drop_refine,
            "visual_refine_cycles": self.visual_refine_cycles,
            "visual_refine_deadband_px": self.visual_refine_deadband_px,
            "visual_refine_depth_target": self.visual_refine_depth_target,
            "visual_refine_depth_tol": self.visual_refine_depth_tol,
            "visual_refine_yaw_gain": self.visual_refine_yaw_gain,
            "visual_refine_lin_gain": self.visual_refine_lin_gain,
            "use_odom_control": self.ctrl.use_odom_control,
            "odom_topic": self.ctrl.odom_topic,
            "odom_dist_tolerance": self.ctrl.dist_tolerance,
            "odom_yaw_tolerance": self.ctrl.yaw_tolerance,
            "motion_pose_recovery_timeout": self.ctrl.pose_recovery_timeout,
            "motion_pose_recovery_poll_hz": self.ctrl.pose_recovery_poll_hz,
            "enable_status_publish": self.status.enabled,
            "status_topic": self.status.status_topic,
            "marker_topic": self.status.marker_topic,
            "visualization_frame": self.status.frame_id,
            "vision_enabled_states": sorted(self.vision_enabled_states),
            "wait_for_localization_inputs": self.wait_for_localization_inputs,
            "localization_watchdog_enabled": self.localization_watchdog_enabled,
            "scan_watchdog_timeout": self.scan_watchdog_timeout,
            "amcl_watchdog_timeout": self.amcl_watchdog_timeout,
            "startup_timeout": self.startup_timeout,
            "confirm_before_start": self.confirm_before_start,
            "require_initial_pose_before_start": self.require_initial_pose_before_start,
            "initial_pose_settle_time": self.initial_pose_settle_time,
            "initial_pose_request_nomotion_update": self.initial_pose_request_nomotion_update,
            "use_wpb_grab_action": self.wpb_grab.enabled,
            "wpb_objects_topic": self.wpb_grab.objects_topic,
            "wpb_grab_action_topic": self.wpb_grab.grab_action_topic,
            "wpb_grab_target_color_topic": self.wpb_grab.grab_target_color_topic,
            "wpb_grab_result_topic": self.wpb_grab.grab_result_topic,
            "wpb_grab_object_wait_timeout": self.wpb_grab.object_wait_timeout,
            "wpb_grab_result_timeout": self.wpb_grab.result_timeout,
            "wpb_stop_object_detect_after_grab_pose": self.wpb_grab.stop_object_detect_after_grab_pose,
        }

    def _handle_lost_target(self):
        self.lost_cycles += 1

        if 0 < self.last_depth < self.lost_depth_fallback and self.lost_cycles >= self.max_target_lost_cycles:
            rospy.logwarn("Target lost near pickup range, forcing PICK")
            self.ctrl.stop_base()
            self._set_state("PICK", "target_lost_near_pick")
            return

        if self.lost_cycles < self.max_target_lost_cycles:
            yaw = 0.16 if self.last_align_error >= 0 else -0.16
            self.ctrl.publish_vel(angular_z=yaw)
            return

        rospy.logwarn("Target lost for too long, abort current cycle")
        if self.target_color is not None:
            self.metrics.finish_cycle(self.target_color, False, "target_lost")
        self._clear_target()
        self._set_state("SEARCH", "target_lost_abort")

    def _handle_alignment_watchdog(self, phase: str, det: Detection, err: float) -> bool:
        self.align_watchdog_cycles += 1
        if (
            self.align_progress_log_cycles > 0
            and self.align_watchdog_cycles % self.align_progress_log_cycles == 0
        ):
            rospy.loginfo(
                "[ALIGN] phase=%s depth=%.2f err=%.1f cycles=%d"
                % (phase, det.depth, err, self.align_watchdog_cycles)
            )

        if self.max_align_watchdog_cycles <= 0:
            return False
        if self.align_watchdog_cycles < self.max_align_watchdog_cycles:
            return False

        self.ctrl.stop_base()
        if det.depth <= self.align_force_pick_depth:
            self._capture_pick_signature(det)
            rospy.logwarn(
                "[ALIGN] watchdog forcing PICK phase=%s depth=%.2f err=%.1f cycles=%d"
                % (phase, det.depth, err, self.align_watchdog_cycles)
            )
            self._set_state("PICK", "align_watchdog_force_pick")
            return True

        rospy.logwarn(
            "[ALIGN] watchdog reacquire phase=%s depth=%.2f err=%.1f cycles=%d"
            % (phase, det.depth, err, self.align_watchdog_cycles)
        )
        self._clear_target()
        self._set_state("SEARCH", "align_watchdog_reacquire")
        return True

    def _publish_status(self):
        self.status.publish(
            state=self.state,
            state_reason=self.state_reason,
            target_color=self.target_color,
            detections=self.perception.get_detections(),
            metrics=self.metrics,
            last_depth=self.last_depth,
            last_align_error=self.last_align_error,
            pick_retry_count=self.pick_retry_count,
            stack_anchors=self.tabletop_stack_anchors,
            drop_base_targets=self.tabletop_drop_base_targets,
            vision_health=self.perception.health(self.vision_topic_timeout),
        )

    def _on_initial_pose(self, msg):
        self.initial_pose_received = True
        self.initial_pose_received_at = rospy.Time.now().to_sec()
        self.initial_pose_nomotion_update_sent = False
        self.initial_pose_localization_ready = False
        self.start_confirmed = False
        self.localization_fault_active = False
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        rospy.loginfo(
            "[LOCALIZING] received initial pose topic=%s pose=(%.3f, %.3f, %.3f)",
            self.initial_pose_topic,
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw,
        )

    def _on_scan_watchdog(self, _msg):
        self.last_scan_received_at = rospy.Time.now().to_sec()

    def _on_amcl_watchdog(self, _msg):
        self.last_amcl_received_at = rospy.Time.now().to_sec()

    def _on_emergency_stop(self, msg):
        reason = msg.data.strip() if msg.data else "external_emergency_stop"
        if not self.emergency_stop_active:
            rospy.logerr("[EMERGENCY-STOP] %s", reason)
        self.emergency_stop_active = True
        self.emergency_stop_reason = reason
        self.localization_fault_active = True
        self.ctrl.stop_base(repeat_seconds=1.0)
        self.wpb_grab.stop(repeat_seconds=1.0)
        self.perception.set_enabled(False)
        self._clear_target()
        self._set_state("ERROR", "emergency_stop")

    def _localization_fault_reason(self) -> str:
        if self.emergency_stop_active:
            return "emergency_stop:%s" % self.emergency_stop_reason
        if not self.localization_watchdog_enabled:
            return ""
        now = rospy.Time.now().to_sec()
        if self.last_scan_received_at > 0.0 and now - self.last_scan_received_at > self.scan_watchdog_timeout:
            return "scan_timeout %.1fs > %.1fs" % (now - self.last_scan_received_at, self.scan_watchdog_timeout)
        if self.last_amcl_received_at > 0.0 and now - self.last_amcl_received_at > self.amcl_watchdog_timeout:
            tf_age = self.ctrl.tf_age()
            if tf_age is not None and tf_age <= self.amcl_watchdog_timeout:
                rospy.logwarn_throttle(
                    5.0,
                    "[LOCALIZATION] /amcl_pose idle %.1fs but map->base TF is fresh %.1fs; keep running",
                    now - self.last_amcl_received_at,
                    tf_age,
                )
                return ""
            if tf_age is None:
                return "amcl_timeout %.1fs > %.1fs and localization_tf_unavailable" % (
                    now - self.last_amcl_received_at,
                    self.amcl_watchdog_timeout,
                )
            return "amcl_timeout %.1fs > %.1fs and localization_tf_age %.1fs > %.1fs" % (
                now - self.last_amcl_received_at,
                self.amcl_watchdog_timeout,
                tf_age,
                self.amcl_watchdog_timeout,
            )
        if self.state not in ("LOCALIZING", "FINISH", "ERROR"):
            if self.last_scan_received_at <= 0.0:
                return "scan_not_received"
            if self.last_amcl_received_at <= 0.0:
                tf_age = self.ctrl.tf_age()
                if tf_age is not None and tf_age <= self.amcl_watchdog_timeout:
                    return ""
                return "amcl_not_received"
        return ""

    def _enter_localization_fault(self, reason: str):
        if not reason:
            return
        self.localization_fault_active = True
        rospy.logerr("[LOCALIZATION-FAULT] %s; stop robot and require RViz 2D Pose Estimate again", reason)
        self.ctrl.stop_base(repeat_seconds=0.8)
        self.wpb_grab.stop(repeat_seconds=0.8)
        self.perception.set_enabled(False)
        self._clear_target()
        self.source_pose_ready = False
        self.start_confirmed = False
        self.initial_pose_received = not self.require_initial_pose_before_start
        self.initial_pose_localization_ready = not self.require_initial_pose_before_start
        self.initial_pose_nomotion_update_sent = False
        self._set_state("LOCALIZING", "localization_fault")

    def _handle_confirm_start(self, _request):
        self.start_confirmed = True
        return TriggerResponse(True, "confirmed")

    def _wait_for_localization_inputs(self):
        if not self.wait_for_localization_inputs:
            return
        rospy.loginfo(
            "[LOCALIZING] waiting for map/scan/odom before starting stack sort"
        )
        rospy.wait_for_message(self.map_topic, OccupancyGrid, timeout=self.startup_timeout)
        rospy.wait_for_message(self.scan_topic, LaserScan, timeout=self.startup_timeout)
        rospy.wait_for_message(self.odom_topic, Odometry, timeout=self.startup_timeout)
        self.last_scan_received_at = rospy.Time.now().to_sec()
        rospy.loginfo("[LOCALIZING] map/scan/odom ready")

    def _request_nomotion_update_after_initial_pose(self):
        if not self.initial_pose_request_nomotion_update or self.initial_pose_nomotion_update_sent:
            return
        self.initial_pose_nomotion_update_sent = True
        service_name = "/request_nomotion_update"
        try:
            rospy.wait_for_service(service_name, timeout=0.2)
            rospy.ServiceProxy(service_name, Empty)()
            rospy.loginfo("[LOCALIZING] requested AMCL no-motion update after initial pose")
        except Exception as exc:
            rospy.logwarn("[LOCALIZING] AMCL no-motion update unavailable after initial pose: %s", exc)

    def _wait_for_localization_after_initial_pose(self) -> bool:
        if self.initial_pose_localization_ready:
            return True
        try:
            rospy.wait_for_message(self.amcl_topic, PoseWithCovarianceStamped, timeout=0.5)
            self.last_amcl_received_at = rospy.Time.now().to_sec()
        except Exception:
            rospy.logwarn_throttle(
                2.0,
                "[LOCALIZING] waiting for %s after RViz 2D Pose Estimate",
                self.amcl_topic,
            )
            return False
        pose = self._drive_pose()
        if pose is None:
            rospy.logwarn_throttle(
                2.0,
                "[LOCALIZING] waiting for pose TF after RViz 2D Pose Estimate",
            )
            return False
        self.initial_pose_localization_ready = True
        rospy.loginfo(
            "[LOCALIZING] AMCL/TF ready after initial pose current=(%.3f, %.3f, %.3f)",
            pose.x,
            pose.y,
            pose.yaw,
        )
        return True

    def _wait_for_start_confirmation(self) -> bool:
        pose = self._drive_pose()
        if pose is not None:
            rospy.loginfo(
                "[LOCALIZING] localization ready current=(%.3f, %.3f, %.3f)",
                pose.x,
                pose.y,
                pose.yaw,
            )
        else:
            rospy.logwarn("[LOCALIZING] localization ready but current pose unavailable")
        rospy.logwarn(
            "[LOCALIZING] Check RViz/localization. Press Enter in this terminal to start stack sort, or call: rosservice call %s",
            self.confirm_start_service_name,
        )
        if sys.stdin is not None and sys.stdin.isatty():
            while not rospy.is_shutdown() and not self.start_confirmed:
                readable, _, _ = select.select([sys.stdin], [], [], 0.2)
                if readable:
                    sys.stdin.readline()
                    self.start_confirmed = True
                    break
            return self.start_confirmed
        while not rospy.is_shutdown() and not self.start_confirmed:
            rospy.sleep(0.2)
        return self.start_confirmed

    def run(self):
        self._set_state("LOCALIZING", "waiting_for_localization_inputs")
        self._publish_status()
        try:
            self._wait_for_localization_inputs()
        except Exception as exc:
            rospy.logerr("[LOCALIZING] failed waiting for map/scan/odom: %s", exc)
            return
        rate = rospy.Rate(15)
        while not rospy.is_shutdown():
            if self.state not in ("LOCALIZING", "FINISH", "ERROR"):
                fault = self._localization_fault_reason()
                if fault:
                    self._enter_localization_fault(fault)
                    self._publish_status()
                    rate.sleep()
                    continue

            if self.planner.total_done() >= self.planner.total_goal():
                self._set_state("FINISH", "goal_reached")
            self.perception.set_enabled(self.state in self.vision_enabled_states)

            if self.require_initial_pose_before_start and not self.initial_pose_received:
                self.ctrl.stop_base()
                self.perception.set_enabled(False)
                self._set_state("LOCALIZING", "waiting_for_initialpose")
                rospy.logwarn_throttle(
                    3.0,
                    "[LOCALIZING] waiting for RViz 2D Pose Estimate on %s before starting stack sort",
                    self.initial_pose_topic,
                )
                self._publish_status()
                rate.sleep()
                continue

            if self.require_initial_pose_before_start and self.initial_pose_received:
                self._request_nomotion_update_after_initial_pose()
                elapsed = rospy.Time.now().to_sec() - self.initial_pose_received_at
                if elapsed < self.initial_pose_settle_time:
                    self.ctrl.stop_base()
                    self.perception.set_enabled(False)
                    self._set_state("LOCALIZING", "initialpose_settling")
                    rospy.logwarn_throttle(
                        1.0,
                        "[LOCALIZING] initial pose received, waiting %.1fs for AMCL/TF before stack sort",
                        max(0.0, self.initial_pose_settle_time - elapsed),
                    )
                    self._publish_status()
                    rate.sleep()
                    continue
                if not self._wait_for_localization_after_initial_pose():
                    self.ctrl.stop_base()
                    self.perception.set_enabled(False)
                    self._set_state("LOCALIZING", "waiting_for_amcl_pose")
                    self._publish_status()
                    rate.sleep()
                    continue
                if self.confirm_before_start and not self.start_confirmed:
                    self.ctrl.stop_base()
                    self.perception.set_enabled(False)
                    self._set_state("LOCALIZING", "waiting_for_start_confirm")
                    self._publish_status()
                    if not self._wait_for_start_confirmation():
                        rate.sleep()
                        continue
                    rate.sleep()
                    continue

            if self.tabletop_use_base_targets and self._drive_pose() is None:
                self.ctrl.stop_base()
                self._set_state("LOCALIZING", "pose_unavailable")
                rate.sleep()
                continue
            if self.state == "LOCALIZING":
                reason = "start_confirmed" if self.start_confirmed else "pose_available"
                self._set_state("SEARCH", reason)

            if self.state == "SEARCH":
                if (
                    self.drive_to_source_before_search
                    and self.tabletop_use_base_targets
                    and not self.source_pose_ready
                    and self.tabletop_return_base_target is not None
                ):
                    self._prepare_pick_demo_travel_pose()
                    self._set_arm_for_source_travel("drive_to_source")
                    if self._drive_to_pose(self.tabletop_return_base_target, "initial_source_base"):
                        self.source_pose_ready = True
                        self._set_arm_for_vision("source_reached", force=True)
                    else:
                        self.ctrl.stop_base()
                        rate.sleep()
                    continue
                self._set_arm_for_vision("search")
                detections = self.perception.get_detections()
                color = self.planner.select_target(detections)
                if self.demo_mode == "pick_only" and self.demo_target_color not in ("", "auto", "any"):
                    color = self.demo_target_color if self.demo_target_color in detections else None
                if color is None:
                    if detections:
                        rospy.logwarn_throttle(
                            3.0,
                            "[SEARCH] detections present but none selectable active_colors=%s detections=%s stack_counts=%s",
                            ",".join(self.planner.active_colors),
                            ",".join(sorted(detections.keys())),
                            str(self.planner.stack_count),
                        )
                    else:
                        rospy.logwarn_throttle(
                            3.0,
                            "[SEARCH] no selectable target: no detections from camera yet; check target placement, color_features, RGB/depth topics, and A pose yaw",
                        )
                    if self.search_spin_when_no_target:
                        self.ctrl.publish_vel(angular_z=self.search_yaw_rate)
                    else:
                        self.ctrl.stop_base()
                else:
                    self._lock_target(color, detections[color])
                    if self.gazebo_carry.enabled and self.gazebo_carry.use_source_pick_targets and self._drive_to_source_pick_target(color):
                        self.source_pose_ready = True
                        self._set_state("PICK", "source_pose_reached")
                    elif self.wpb_grab.enabled and self.wpb_direct_pick_after_color_lock:
                        self.ctrl.stop_base()
                        self._capture_pick_signature(detections[color])
                        self._set_state("PICK", "wpb_direct_color_locked")
                    else:
                        self._set_state("ALIGN", "target_locked")

            elif self.state == "ALIGN":
                det = self._target_detection()
                if det is None:
                    self._handle_lost_target()
                    self._publish_status()
                    rate.sleep()
                    continue

                self.lost_cycles = 0
                self.last_depth = det.depth
                err = self.center_x - det.cx
                self.last_align_error = err

                if det.depth <= self.pick_stop_depth:
                    self.near_pick_align_cycles += 1
                else:
                    self.near_pick_align_cycles = 0

                near_pick_aligned = (
                    det.depth <= self.pick_stop_depth
                    and abs(err) <= self.near_pick_align_deadband_px
                )
                near_pick_timed_out = (
                    det.depth <= self.pick_stop_depth
                    and self.max_near_pick_align_cycles > 0
                    and self.near_pick_align_cycles >= self.max_near_pick_align_cycles
                )

                if near_pick_aligned or near_pick_timed_out:
                    self.ctrl.stop_base()
                    self._capture_pick_signature(det)
                    rospy.loginfo(
                        "[ALIGN] near target accepted depth=%.2f err=%.1f cycles=%d"
                        % (det.depth, err, self.near_pick_align_cycles)
                    )
                    self._set_state("PICK", "near_target_accepted")
                elif self._handle_alignment_watchdog("ALIGN", det, err):
                    pass
                elif abs(err) <= self.deadband_px:
                    self.ctrl.stop_base()
                    if self.approach_arm_guard_enabled:
                        self._set_arm_for_source_travel("approach_guard")
                    self._set_state("APPROACH", "aligned")
                else:
                    yaw = self._clamp(err * self.align_kp, -0.65, 0.65)
                    self.ctrl.publish_vel(angular_z=yaw)

            elif self.state == "APPROACH":
                det = self._target_detection()
                if det is None:
                    self._handle_lost_target()
                    self._publish_status()
                    rate.sleep()
                    continue

                self.lost_cycles = 0
                self.last_depth = det.depth
                err = self.center_x - det.cx
                self.last_align_error = err

                if det.depth <= self.pick_stop_depth:
                    self.ctrl.stop_base()
                    self._capture_pick_signature(det)
                    self._set_state("PICK", "pick_depth_reached")
                elif self._handle_alignment_watchdog("APPROACH", det, err):
                    pass
                else:
                    if self.approach_arm_guard_enabled:
                        self._set_arm_for_source_travel("approach_guard")
                    yaw = self._clamp(err * self.align_kp, -0.50, 0.50)
                    if abs(err) > self.approach_realign_error_px:
                        rospy.logwarn_throttle(
                            1.0,
                            "[APPROACH] large align error %.1fpx > %.1fpx, rotating without forward motion",
                            err,
                            self.approach_realign_error_px,
                        )
                        self.ctrl.publish_vel(angular_z=yaw)
                    else:
                        self.ctrl.publish_vel(linear_x=self.approach_speed, angular_z=yaw)

            elif self.state == "PICK":
                self.ctrl.stop_base()
                grasp_ok = self._pick_sequence()
                localization_fault = self._localization_fault_reason()
                if (not grasp_ok) and localization_fault:
                    self._enter_localization_fault(localization_fault)
                    self._publish_status()
                    rate.sleep()
                    continue
                pick_looks_failed = (not self.wpb_grab.enabled) and self._pick_looks_failed()
                if (not grasp_ok) or pick_looks_failed:
                    if self.pick_retry_count < self.max_pick_retries:
                        self.pick_retry_count += 1
                        self.metrics.mark_retry()
                        rospy.logwarn(
                            "Pick looks failed for %s, retry %d/%d"
                            % (self.target_color, self.pick_retry_count, self.max_pick_retries)
                        )
                        self._set_state("RECOVER_RETRY", "pick_failed_retry")
                    else:
                        if (not grasp_ok) and self.gazebo_carry.enabled:
                            rospy.logwarn(
                                "Grasp window rejected after max retries for %s, reacquire target"
                                % self.target_color
                            )
                        else:
                            rospy.logerr("Pick failed after max retries, abort cycle")
                            if self.target_color is not None:
                                self.metrics.finish_cycle(self.target_color, False, "pick_failed")
                        self._clear_target()
                        self._set_state("SEARCH", "pick_failed_abort")
                else:
                    if self.demo_mode == "pick_only" and self.demo_stop_after_pick:
                        self.metrics.finish_cycle(self.target_color, True, "demo_pick_succeeded")
                        self._set_state("FINISH", "demo_pick_complete")
                    else:
                        self._set_state("DROP", "pick_succeeded")

            elif self.state == "RECOVER_RETRY":
                self._recover_for_retry()
                if self.localization_fault_active:
                    self._publish_status()
                    rate.sleep()
                    continue
                self._set_state("ALIGN", "retry_reacquire")

            elif self.state == "DROP":
                color = self.target_color
                if not self._drop_sequence(color):
                    if self.localization_fault_active:
                        self._publish_status()
                        rate.sleep()
                        continue
                    if color is not None:
                        self.metrics.finish_cycle(color, False, "drop_motion_failed")
                    self.ctrl.stop_base()
                    self._set_state("ERROR", "drop_motion_failed")
                    self._publish_status()
                    rate.sleep()
                    continue
                self.planner.mark_placed(color)
                self.metrics.finish_cycle(color, True, "placed")
                rospy.loginfo("Placed color=%s stack_counts=%s" % (color, str(self.planner.stack_count)))
                self._clear_target()
                self._set_arm_for_vision("after_drop", force=True)
                self._set_state("SEARCH", "placed")

            elif self.state == "ERROR":
                self.ctrl.stop_base()

            elif self.state == "FINISH":
                self.ctrl.stop_base()
                if not self.report_exported:
                    self.reporter.export(
                        metrics=self.metrics,
                        stack_count=self.planner.stack_count,
                        params=self._params_snapshot(),
                    )
                    self.report_exported = True
                rospy.loginfo_throttle(
                    3.0,
                    "All planned stacks completed. %s"
                    % self.metrics.summary(),
                )

            self._publish_status()
            rate.sleep()


def main():
    rospy.init_node("stack_sort_pipeline")
    node = None
    try:
        node = StackSortOrchestrator()
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        if node is not None:
            node.wpb_grab.stop()
            node.ctrl.stop_base()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
