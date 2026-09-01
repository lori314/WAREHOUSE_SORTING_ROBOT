#!/usr/bin/env python3

import os
from datetime import datetime

import rospy
import yaml
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse


class CargoFeatureCaptureNode:
    def __init__(self):
        defaults = rospy.get_param("/warehouse_tuning/cargo_feature_capture", {})
        self.cargo_type = rospy.get_param("~cargo_type", "natural")
        self.output_file = rospy.get_param("~output_file", defaults.get("output_file", "/tmp/warehouse_cargo_features.yaml"))
        self.rgb_topic = rospy.get_param("~rgb_topic", defaults.get("rgb_topic", "/kinect2/sd/image_color_rect"))
        self.depth_topic = rospy.get_param("~depth_topic", defaults.get("depth_topic", "/kinect2/sd/image_depth_rect"))
        self.camera_info_topic = rospy.get_param(
            "~camera_info_topic",
            defaults.get("camera_info_topic", "/kinect2/sd/camera_info"),
        )
        self.roi = rospy.get_param("~roi", defaults.get("roi", {"x": 0.30, "y": 0.42, "width": 0.35, "height": 0.36}))
        self.hsv_padding = rospy.get_param("~hsv_padding", defaults.get("hsv_padding", {"h": 8, "s": 35, "v": 35}))
        self.min_area = int(rospy.get_param("~min_area", defaults.get("min_area", 500)))
        self.use_blob_mask = bool(rospy.get_param("~use_blob_mask", defaults.get("use_blob_mask", True)))
        self.require_blob_mask = bool(rospy.get_param("~require_blob_mask", defaults.get("require_blob_mask", True)))
        self.allow_simulated_fallback = bool(
            rospy.get_param("~allow_simulated_fallback", defaults.get("allow_simulated_fallback", False))
        )
        self.min_saturation = int(rospy.get_param("~min_saturation", defaults.get("min_saturation", 45)))
        self.min_value = int(rospy.get_param("~min_value", defaults.get("min_value", 35)))
        self.hue_ranges_by_type = rospy.get_param(
            "~hue_ranges_by_type",
            defaults.get(
                "hue_ranges_by_type",
                {
                    "red": [{"lower": 0, "upper": 10}, {"lower": 160, "upper": 179}],
                    "green": [{"lower": 35, "upper": 90}],
                    "blue": [{"lower": 95, "upper": 140}],
                },
            ),
        )
        self.default_size = rospy.get_param("~default_size", defaults.get("default_size", {"x": 0.12, "y": 0.12, "z": 0.10}))
        self.wait_timeout = float(rospy.get_param("~wait_timeout", 5.0))
        self.require_fresh_image = bool(rospy.get_param("~require_fresh_image", defaults.get("require_fresh_image", True)))
        self.capture_settle_seconds = float(
            rospy.get_param("~capture_settle_seconds", defaults.get("capture_settle_seconds", 0.2))
        )
        self.save_debug_images = bool(rospy.get_param("~save_debug_images", defaults.get("save_debug_images", False)))
        self.debug_output_dir = rospy.get_param(
            "~debug_output_dir",
            defaults.get("debug_output_dir", "/tmp/warehouse_tuning_debug"),
        )
        self.service_name = rospy.get_param(
            "~service_name",
            defaults.get("service_name", "/warehouse_tuning/capture_cargo_features"),
        )
        self.status_topic = rospy.get_param(
            "~status_topic",
            defaults.get("status_topic", "/warehouse_tuning/cargo_feature_status"),
        )

        self.rgb_image = None
        self.depth_image = None
        self.camera_info = None
        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=5, latch=True)
        rospy.Subscriber(self.rgb_topic, Image, self._on_rgb, queue_size=1)
        rospy.Subscriber(self.depth_topic, Image, self._on_depth, queue_size=1)
        rospy.Subscriber(self.camera_info_topic, CameraInfo, self._on_camera_info, queue_size=1)
        rospy.Service(self.service_name, Trigger, self.handle_capture)
        self._status("ready cargo_type=%s output=%s service=%s" % (self.cargo_type, self.output_file, self.service_name))

    def _on_rgb(self, msg):
        self.rgb_image = msg

    def _on_depth(self, msg):
        self.depth_image = msg

    def _on_camera_info(self, msg):
        self.camera_info = msg

    def handle_capture(self, _request):
        image = None
        debug_path = ""
        try:
            if self.capture_settle_seconds > 0.0:
                rospy.sleep(self.capture_settle_seconds)
            image = self._latest_bgr_image()
            spec = self._measure_spec(image)
            self._write_spec(spec)
        except Exception as exc:
            if image is not None:
                debug_path = self._save_debug_image(image, "failed")
            message = "capture failed: %s" % exc
            if debug_path:
                message += " debug_image=%s" % debug_path
            self._status(message)
            return TriggerResponse(False, message)
        debug_path = self._save_debug_image(image, "captured")
        message = "captured %s hsv=%s..%s size=%s min_area=%s to %s" % (
            self.cargo_type,
            spec.get("hsv_lower"),
            spec.get("hsv_upper"),
            spec.get("size"),
            spec.get("min_area"),
            self.output_file,
        )
        if debug_path:
            message += " debug_image=%s" % debug_path
        self._status(message)
        return TriggerResponse(True, message)

    def _latest_bgr_image(self):
        msg = self.rgb_image
        if self.require_fresh_image or msg is None:
            msg = rospy.wait_for_message(self.rgb_topic, Image, timeout=self.wait_timeout)
            self.rgb_image = msg
        if msg is None:
            raise RuntimeError("no rgb image on %s" % self.rgb_topic)
        from cv_bridge import CvBridge

        return CvBridge().imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def _measure_spec(self, image):
        import cv2
        import numpy as np

        height, width = image.shape[:2]
        x0, y0, x1, y1 = self._roi_bounds(width, height)
        patch = image[y0:y1, x0:x1]
        if patch.size == 0:
            raise RuntimeError("ROI is empty")
        hsv = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV)
        target_mask = self._target_mask(hsv)
        if target_mask is not None:
            pixels = hsv[target_mask > 0].reshape((-1, 3))
        elif self.use_blob_mask and self.require_blob_mask:
            if self.allow_simulated_fallback:
                rospy.logwarn(
                    "[cargo_feature] no colored blob found; using simulated fallback for %s",
                    self.cargo_type,
                )
                return self._fallback_spec()
            raise RuntimeError("no colored blob found in ROI")
        else:
            pixels = hsv.reshape((-1, 3))
        lower = np.percentile(pixels, 5, axis=0)
        upper = np.percentile(pixels, 95, axis=0)
        padding = np.array(
            [
                float(self.hsv_padding.get("h", 8)),
                float(self.hsv_padding.get("s", 35)),
                float(self.hsv_padding.get("v", 35)),
            ]
        )
        hsv_lower = np.maximum([0, 0, 0], lower - padding).astype(int).tolist()
        hsv_upper = np.minimum([179, 255, 255], upper + padding).astype(int).tolist()
        if target_mask is not None:
            bx, by, bw, bh = cv2.boundingRect(target_mask)
            size = self._estimate_size(bw, bh)
        else:
            size = self._estimate_size(x1 - x0, y1 - y0)
        return {
            "hsv_lower": hsv_lower,
            "hsv_upper": hsv_upper,
            "min_area": int(self.min_area),
            "size": {
                "x": round(float(size["x"]), 3),
                "y": round(float(size["y"]), 3),
                "z": round(float(size["z"]), 3),
            },
        }

    def _fallback_spec(self):
        hue_range = self._first_hue_range()
        return {
            "hsv_lower": [
                int(hue_range.get("lower", 0)),
                int(self.min_saturation),
                int(self.min_value),
            ],
            "hsv_upper": [
                int(hue_range.get("upper", 179)),
                255,
                255,
            ],
            "min_area": int(self.min_area),
            "size": {
                "x": round(float(self.default_size.get("x", 0.12)), 3),
                "y": round(float(self.default_size.get("y", 0.12)), 3),
                "z": round(float(self.default_size.get("z", 0.10)), 3),
            },
            "source": "simulated_fallback",
        }

    def _first_hue_range(self):
        raw_ranges = self.hue_ranges_by_type.get(self.cargo_type)
        if isinstance(raw_ranges, list) and raw_ranges:
            first = raw_ranges[0]
            if isinstance(first, dict):
                return first
        if isinstance(raw_ranges, dict):
            return raw_ranges
        return {"lower": 0, "upper": 179}

    def _target_mask(self, hsv):
        if not self.use_blob_mask:
            return None
        import cv2
        import numpy as np

        mask = cv2.inRange(
            hsv,
            np.array([0, self.min_saturation, self.min_value], dtype=np.uint8),
            np.array([179, 255, 255], dtype=np.uint8),
        )
        hue_mask = self._hue_mask(hsv)
        if hue_mask is not None:
            mask = cv2.bitwise_and(mask, hue_mask)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best = None
        best_area = 0.0
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > best_area:
                best = contour
                best_area = area
        if best is None or best_area < self.min_area:
            return None
        out = np.zeros(mask.shape, dtype=np.uint8)
        cv2.drawContours(out, [best], -1, 255, thickness=cv2.FILLED)
        return out

    def _hue_mask(self, hsv):
        raw_ranges = self.hue_ranges_by_type.get(self.cargo_type)
        if not raw_ranges:
            return None
        import cv2
        import numpy as np

        entries = raw_ranges if isinstance(raw_ranges, list) else [raw_ranges]
        combined = None
        for entry in entries:
            if not isinstance(entry, dict):
                continue
            try:
                lower_h = int(entry.get("lower", 0))
                upper_h = int(entry.get("upper", 179))
            except (TypeError, ValueError):
                continue
            part = cv2.inRange(
                hsv,
                np.array([max(0, lower_h), 0, 0], dtype=np.uint8),
                np.array([min(179, upper_h), 255, 255], dtype=np.uint8),
            )
            combined = part if combined is None else cv2.bitwise_or(combined, part)
        return combined

    def _estimate_size(self, width_px, height_px):
        if self.depth_image is None or self.camera_info is None:
            return dict(self.default_size)
        try:
            from cv_bridge import CvBridge
            import numpy as np

            depth = CvBridge().imgmsg_to_cv2(self.depth_image, desired_encoding="passthrough")
            x0, y0, x1, y1 = self._roi_bounds(depth.shape[1], depth.shape[0])
            values = np.asarray(depth[y0:y1, x0:x1], dtype=np.float32)
            values = values[np.isfinite(values)]
            values = values[values > 0.0]
            if values.size == 0:
                return dict(self.default_size)
            if float(np.nanmax(values)) > 10.0:
                values = values / 1000.0
            depth_m = float(np.median(values))
            fx = float(self.camera_info.K[0]) or 1.0
            fy = float(self.camera_info.K[4]) or 1.0
            size = dict(self.default_size)
            size["x"] = abs(float(width_px) * depth_m / fx)
            size["y"] = abs(float(height_px) * depth_m / fy)
            return size
        except Exception as exc:
            rospy.logwarn("size estimate fell back to default: %s", exc)
            return dict(self.default_size)

    def _roi_bounds(self, width, height):
        x = float(self.roi.get("x", 0.0))
        y = float(self.roi.get("y", 0.0))
        w = float(self.roi.get("width", width))
        h = float(self.roi.get("height", height))
        if 0.0 <= x <= 1.0 and 0.0 < w <= 1.0:
            x *= width
            w *= width
        if 0.0 <= y <= 1.0 and 0.0 < h <= 1.0:
            y *= height
            h *= height
        x0 = max(0, min(width, int(round(x))))
        y0 = max(0, min(height, int(round(y))))
        x1 = max(x0, min(width, int(round(x + w))))
        y1 = max(y0, min(height, int(round(y + h))))
        return x0, y0, x1, y1

    def _write_spec(self, spec):
        path = os.path.abspath(os.path.expanduser(self.output_file))
        directory = os.path.dirname(path)
        if directory and not os.path.isdir(directory):
            os.makedirs(directory)
        data = {}
        if os.path.exists(path):
            with open(path, "r", encoding="utf-8") as stream:
                data = yaml.safe_load(stream) or {}
        root = data.setdefault("warehouse_sorting", {})
        cargo_types = root.setdefault("cargo_types", {})
        cargo_types[self.cargo_type] = spec
        stack = data.setdefault("stack_sort_pipeline", {})
        active_colors = stack.setdefault("active_colors", [])
        if not isinstance(active_colors, list):
            active_colors = []
            stack["active_colors"] = active_colors
        if self.cargo_type not in active_colors:
            active_colors.append(self.cargo_type)
        stack.setdefault("color_ranges", {})[self.cargo_type] = [
            {
                "lower": list(spec["hsv_lower"]),
                "upper": list(spec["hsv_upper"]),
            }
        ]
        field = stack.setdefault("field_dimensions", {})
        box_size = field.setdefault("box_size", {"x": 0.0, "y": 0.0, "z": 0.0})
        for axis in ("x", "y", "z"):
            box_size[axis] = round(max(float(box_size.get(axis, 0.0)), float(spec["size"].get(axis, 0.0))), 3)
        with open(path, "w", encoding="utf-8") as stream:
            yaml.safe_dump(data, stream, default_flow_style=False, sort_keys=False)

    def _save_debug_image(self, image, suffix):
        if not self.save_debug_images:
            return ""
        try:
            import cv2

            directory = os.path.abspath(os.path.expanduser(self.debug_output_dir))
            os.makedirs(directory, exist_ok=True)
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            path = os.path.join(directory, "%s_%s_%s.png" % (self.cargo_type, suffix, stamp))
            cv2.imwrite(path, image)
            return path
        except Exception as exc:
            rospy.logwarn("debug image save failed: %s", exc)
            return ""

    def _status(self, message):
        rospy.loginfo("[cargo_feature] %s", message)
        self.status_pub.publish(String(data=message))


if __name__ == "__main__":
    rospy.init_node("cargo_feature_capture")
    CargoFeatureCaptureNode()
    rospy.spin()
