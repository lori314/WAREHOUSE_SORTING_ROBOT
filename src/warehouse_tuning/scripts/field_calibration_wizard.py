#!/usr/bin/env python3

import argparse
import math
import os
import select
import signal
import subprocess
import sys
import time
from datetime import datetime

import rosgraph
import rospy
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from std_srvs.srv import Trigger


class ProcessGroup:
    def __init__(self):
        self.processes = []

    def launch(self, command):
        print("[START] %s" % " ".join(command), flush=True)
        proc = subprocess.Popen(command, preexec_fn=os.setsid)
        self.processes.append(proc)
        return proc

    def stop_all(self):
        for proc in reversed(self.processes):
            if proc.poll() is not None:
                continue
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            except OSError:
                continue
        deadline = time.time() + 8.0
        for proc in reversed(self.processes):
            if proc.poll() is not None:
                continue
            remaining = max(0.1, deadline - time.time())
            try:
                proc.wait(timeout=remaining)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                except OSError:
                    pass


class TopicCache:
    def __init__(self):
        self.map = None
        self.amcl_pose = None

    def on_map(self, msg):
        self.map = msg

    def on_amcl_pose(self, msg):
        self.amcl_pose = msg


def expand_path(path):
    return os.path.abspath(os.path.expanduser(path))


def wait_for_master(timeout):
    master = rosgraph.Master("/warehouse_tuning_field_calibration_wizard")
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            master.getPid()
            return True
        except Exception:
            time.sleep(0.5)
    return False


def master_available():
    try:
        rosgraph.Master("/warehouse_tuning_field_calibration_wizard").getPid()
        return True
    except Exception:
        return False


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def map_stats(msg):
    if msg is None or not msg.data:
        return "no map yet"
    total = float(len(msg.data))
    unknown = sum(1 for value in msg.data if value < 0)
    occupied = sum(1 for value in msg.data if value > 50)
    known = int(total - unknown)
    known_pct = (float(known) / total) * 100.0 if total else 0.0
    width_m = msg.info.width * msg.info.resolution
    height_m = msg.info.height * msg.info.resolution
    age = ""
    if msg.header.stamp:
        stamp = msg.header.stamp.to_sec()
        if stamp > 0.0:
            age = " age=%.1fs" % max(0.0, time.time() - stamp)
    return (
        "size=%dx%d %.2fmx%.2fm res=%.3f known=%.1f%% occupied=%d%s"
        % (
            msg.info.width,
            msg.info.height,
            width_m,
            height_m,
            msg.info.resolution,
            known_pct,
            occupied,
            age,
        )
    )


def load_yaml(path):
    if not os.path.exists(path):
        return {}
    with open(path, "r", encoding="utf-8") as stream:
        return yaml.safe_load(stream) or {}


def parse_zone_plan(raw):
    plan = []
    for item in raw.split(","):
        item = item.strip()
        if not item:
            continue
        parts = [part.strip() for part in item.split(":")]
        while len(parts) < 3:
            parts.append("")
        zone, role, color = parts[:3]
        if not zone or not role:
            raise ValueError("invalid zone entry: %s" % item)
        plan.append({"zone": zone, "role": role, "color": color})
    if not plan:
        raise ValueError("zone plan is empty")
    return plan


def parse_cargo_types(raw):
    values = [value.strip() for value in raw.split(",") if value.strip()]
    if not values:
        raise ValueError("cargo type list is empty")
    return values


class FieldCalibrationWizard:
    def __init__(self, args):
        self.args = args
        self.map_prefix = expand_path(args.map_prefix)
        self.map_file = expand_path(args.map_file or (self.map_prefix + ".yaml"))
        self.zone_file = expand_path(args.zone_file)
        self.feature_file = expand_path(args.feature_file)
        self.debug_dir = expand_path(args.debug_output_dir)
        self.zone_plan = parse_zone_plan(args.zone_plan)
        self.cargo_types = parse_cargo_types(args.cargo_types)
        self.processes = ProcessGroup()
        self.cache = TopicCache()
        self.status_pub = None
        self.started_mapping = False
        self.started_localization = False
        self.started_capture = False

    def run(self):
        self._start_managed_stack_before_ros()
        if not wait_for_master(self.args.master_timeout):
            raise RuntimeError("ROS master is not available; start robot bringup or use --manage-stack")
        rospy.init_node("field_calibration_wizard", anonymous=True, disable_signals=True)
        self.status_pub = rospy.Publisher(
            "/warehouse_tuning/field_calibration_status",
            String,
            queue_size=5,
            latch=True,
        )
        rospy.Subscriber(self.args.map_topic, OccupancyGrid, self.cache.on_map, queue_size=1)
        rospy.Subscriber(self.args.amcl_pose_topic, PoseWithCovarianceStamped, self.cache.on_amcl_pose, queue_size=1)

        self._banner()
        self._preflight()
        if not self.args.skip_mapping:
            self._mapping_phase()
        else:
            self._validate_map_file()
            self._ok("跳过建图，使用已有地图: %s" % self.map_file)

        if not self.args.skip_localization:
            self._localization_phase()

        if not self.args.skip_zones or not self.args.skip_features:
            self._start_capture_services_if_needed()
        if not self.args.skip_zones:
            self._zone_phase()
        if not self.args.skip_features:
            self._feature_phase()
        self._summary()

    def close(self):
        if self.args.keep_managed_stack:
            return
        self.processes.stop_all()

    def _start_managed_stack_before_ros(self):
        if not self.args.manage_stack:
            return
        if not master_available():
            raise RuntimeError(
                "请先启动真机底层: roslaunch warehouse_tuning field_robot_base.launch"
            )
        if not self.args.skip_mapping:
            self.started_mapping = True
            self.processes.launch(
                [
                    "roslaunch",
                    "warehouse_tuning",
                    "mapping_session.launch",
                    "start_gmapping:=%s" % self._bool_arg(self.args.start_gmapping),
                    "map_prefix:=%s" % self.map_prefix,
                    "wait_timeout:=%.1f" % self.args.service_timeout,
                    "rviz:=%s" % self._bool_arg(self.args.rviz),
                ]
            )

    def _start_localization_if_needed(self):
        if not self.args.manage_stack or self.started_localization or self.args.skip_localization:
            return
        self.started_localization = True
        command = [
            "roslaunch",
            "warehouse_tuning",
            "field_localization.launch",
            "map_file:=%s" % self.map_file,
            "scan_topic:=%s" % self.args.scan_topic,
            "map_frame:=%s" % self.args.map_frame,
            "odom_frame:=%s" % self.args.odom_frame,
            "base_frame:=%s" % self.args.base_frame,
            "rviz:=%s" % self._bool_arg(self.args.rviz),
        ]
        self.processes.launch(command)

    def _start_capture_services_if_needed(self):
        if not self.args.manage_stack or self.started_capture:
            return
        self.started_capture = True
        self.processes.launch(
            [
                "roslaunch",
                "warehouse_tuning",
                "stack_sort_capture_services.launch",
                "zone_output_file:=%s" % self.zone_file,
                "feature_output_file:=%s" % self.feature_file,
                "table_height:=%.3f" % self.args.table_height,
                "pose_source:=%s" % self.args.pose_source,
                "map_frame:=%s" % self.args.map_frame,
                "base_frame:=%s" % self.args.base_frame,
                "stack_anchor_forward_offset:=%.3f" % self.args.stack_anchor_forward_offset,
                "save_debug_images:=%s" % self._bool_arg(self.args.save_debug_images),
                "debug_output_dir:=%s" % self.debug_dir,
                "allow_simulated_fallback:=%s" % self._bool_arg(self.args.allow_simulated_fallback),
                "feature_min_area:=%d" % self.args.feature_min_area,
                "feature_roi_x:=%.4f" % self.args.feature_roi_x,
                "feature_roi_y:=%.4f" % self.args.feature_roi_y,
                "feature_roi_width:=%.4f" % self.args.feature_roi_width,
                "feature_roi_height:=%.4f" % self.args.feature_roi_height,
                "rgb_topic:=%s" % self.args.rgb_topic,
                "depth_topic:=%s" % self.args.depth_topic,
                "camera_info_topic:=%s" % self.args.camera_info_topic,
            ]
        )

    def _banner(self):
        self._status("开始现场标定向导")
        print("", flush=True)
        print("========== 现场标定向导 ==========", flush=True)
        print("生成地图: %s" % self.map_file, flush=True)
        print("生成区域: %s" % self.zone_file, flush=True)
        print("生成特征: %s" % self.feature_file, flush=True)
        print("状态话题: /warehouse_tuning/field_calibration_status", flush=True)
        print("操作键: 回车=确认  r=重做  s=跳过  q=退出", flush=True)
        print("==================================", flush=True)

    def _preflight(self):
        self._status("预检查")
        self._info("检查 ROS 依赖，缺失项会直接提示")
        if not self.args.skip_mapping:
            self._wait_topic_feedback(self.args.map_topic, OccupancyGrid, "地图 /map", self.args.topic_timeout)
        if not self.args.manage_stack and not self.args.skip_zones:
            self._wait_service_feedback("/warehouse_tuning/capture_zone_A", "A 区采集服务", optional=True)
        if not self.args.skip_features:
            self._wait_topic_feedback(self.args.rgb_topic, None, "彩色相机", self.args.topic_timeout, optional=True)
        if not self.args.manage_stack and not self.args.skip_features:
            self._wait_service_feedback("/warehouse_tuning/capture_features_green", "绿色特征采集服务", optional=True)

    def _mapping_phase(self):
        self._status("建图")
        self._step("1/4 建图")
        print("请遥控机器人绕 A/B/C 三张桌子和通道行走。终端会持续显示地图覆盖率。", flush=True)
        print("当 RViz 中墙体、桌子边缘稳定后按回车保存地图。", flush=True)
        if self.args.non_interactive:
            self._wait_for_map(self.args.topic_timeout)
            time.sleep(self.args.non_interactive_settle)
        else:
            self._monitor_map_until_confirm()
        result = self._call_trigger("/warehouse_tuning/save_map", "保存地图")
        if not result:
            raise RuntimeError("地图保存失败")
        self._validate_map_file()
        if self.started_mapping:
            self._info("地图已保存，停止 gmapping，后续切换到重载地图定位验证")
            self.processes.stop_all()
            self.processes = ProcessGroup()
            self.started_mapping = False

    def _localization_phase(self):
        self._status("定位验证")
        self._step("2/4 重新加载地图并确认初始位姿")
        self._validate_map_file()
        self._start_localization_if_needed()
        self._wait_topic_feedback("/map", OccupancyGrid, "已加载地图", self.args.topic_timeout, optional=True)
        print("请在 RViz 用 2D Pose Estimate 设置机器人当前位置。", flush=True)
        print("机器人箭头必须贴近真实位置和朝向，否则后面 A/B/C 坐标都会偏。", flush=True)
        while True:
            choice = self._prompt("确认 RViz 里机器人位置正确后按回车")
            if choice == "s":
                self._warn("跳过定位验证；如果 map->base 不准，区域采集会整体偏移")
                return
            if choice == "r":
                continue
            self._wait_for_amcl_pose(self.args.localization_timeout)
            pose = self.cache.amcl_pose.pose.pose
            yaw = yaw_from_quaternion(pose.orientation)
            self._ok(
                "定位已收到: x=%.3f y=%.3f yaw=%.3f"
                % (pose.position.x, pose.position.y, yaw)
            )
            return

    def _zone_phase(self):
        self._status("A/B/C 区域标定")
        self._step("3/4 A/B/C 区域")
        for entry in self.zone_plan:
            zone = entry["zone"]
            role = entry["role"]
            color = entry["color"]
            service = "/warehouse_tuning/capture_zone_%s" % zone
            while True:
                if role.lower() in ("source", "pickup", "pick", "a"):
                    self._info("移动到底盘取货位: %s 桌，车头对准桌面中心，停稳后回车" % zone)
                else:
                    self._info("移动到底盘放置位: %s 桌 (%s)，车头对准堆叠中心，停稳后回车" % (zone, color))
                choice = self._prompt("采集 %s 区" % zone)
                if choice == "s":
                    self._warn("跳过 %s 区" % zone)
                    break
                if choice == "r":
                    continue
                if self._call_trigger(service, "%s 区采集" % zone):
                    self._print_zone_feedback(zone)
                    break
        self._validate_zones()

    def _feature_phase(self):
        self._status("小方块视觉特征标定")
        self._step("4/4 小方块颜色和尺寸")
        for cargo_type in self.cargo_types:
            service = "/warehouse_tuning/capture_features_%s" % cargo_type
            while True:
                self._info("只放一个 %s 小方块到相机 ROI 中心，拿走其他颜色，停 1 秒后回车" % cargo_type)
                choice = self._prompt("采集 %s 特征" % cargo_type)
                if choice == "s":
                    self._warn("跳过 %s 特征" % cargo_type)
                    break
                if choice == "r":
                    continue
                if self._call_trigger(service, "%s 特征采集" % cargo_type):
                    self._print_feature_feedback(cargo_type)
                    break
                self._warn("采集失败时先看 debug 图: %s" % self.debug_dir)
        self._validate_features()

    def _summary(self):
        self._status("标定完成")
        self._step("完成")
        self._ok("地图: %s" % self.map_file)
        self._ok("区域: %s" % self.zone_file)
        self._ok("特征: %s" % self.feature_file)
        print("", flush=True)
        print("实机分拣启动命令:", flush=True)
        print(
            "roslaunch arm_grab_task stack_sort_field.launch "
            "map_file:=%s "
            "use_field_override:=true field_override:=%s "
            "use_feature_override:=true feature_override:=%s "
            "rviz:=true"
            % (self.map_file, self.zone_file, self.feature_file),
            flush=True,
        )

    def _monitor_map_until_confirm(self):
        self._wait_for_map(self.args.topic_timeout)
        last_print = 0.0
        while not rospy.is_shutdown():
            now = time.time()
            if now - last_print >= self.args.feedback_interval:
                self._info("地图状态: %s" % map_stats(self.cache.map))
                last_print = now
            choice = self._read_choice(1.0)
            if choice is None:
                continue
            if choice == "":
                return
            if choice == "q":
                raise RuntimeError("用户退出")
            if choice == "s":
                self._warn("跳过建图保存")
                return
            if choice == "r":
                self._info("继续建图，地图状态会继续刷新")

    def _prompt(self, label):
        if self.args.non_interactive:
            self._info("%s: 非交互模式自动确认" % label)
            return ""
        print("%s [Enter/r/s/q]: " % label, end="", flush=True)
        choice = self._read_choice(None)
        if choice == "q":
            raise RuntimeError("用户退出")
        return choice

    def _read_choice(self, timeout):
        if not sys.stdin.isatty():
            if self.args.non_interactive:
                if timeout is not None:
                    time.sleep(timeout)
                    return None
                return ""
            raise RuntimeError("当前没有交互终端；请用 rosrun 启动向导，或显式加 --non-interactive")
        ready, _, _ = select.select([sys.stdin], [], [], timeout)
        if not ready:
            return None
        line = sys.stdin.readline()
        if line == "":
            return "q"
        return line.strip().lower()

    def _wait_for_map(self, timeout):
        if self.cache.map is not None:
            return
        self._wait_topic_feedback(self.args.map_topic, OccupancyGrid, "地图 /map", timeout)

    def _wait_for_amcl_pose(self, timeout):
        if self.cache.amcl_pose is not None:
            return
        self._wait_topic_feedback(self.args.amcl_pose_topic, PoseWithCovarianceStamped, "AMCL 位姿", timeout)

    def _wait_topic_feedback(self, topic, msg_type, label, timeout, optional=False):
        try:
            if msg_type is None:
                topics = dict(rospy.get_published_topics())
                deadline = time.time() + timeout
                while topic not in topics and time.time() < deadline:
                    time.sleep(0.5)
                    topics = dict(rospy.get_published_topics())
                if topic not in topics:
                    raise RuntimeError("topic not published")
                self._ok("%s 已发布: %s" % (label, topic))
                return True
            msg = rospy.wait_for_message(topic, msg_type, timeout=timeout)
            if isinstance(msg, OccupancyGrid):
                self.cache.map = msg
            elif isinstance(msg, PoseWithCovarianceStamped):
                self.cache.amcl_pose = msg
            self._ok("%s 正常: %s" % (label, topic))
            return True
        except Exception as exc:
            if optional:
                self._warn("%s 暂未就绪: %s (%s)" % (label, topic, exc))
                return False
            raise RuntimeError("%s 未就绪: %s (%s)" % (label, topic, exc))

    def _wait_service_feedback(self, service, label, optional=False):
        try:
            rospy.wait_for_service(service, timeout=self.args.service_timeout)
            self._ok("%s 正常: %s" % (label, service))
            return True
        except Exception as exc:
            if optional:
                self._warn("%s 暂未就绪: %s (%s)" % (label, service, exc))
                return False
            raise RuntimeError("%s 未就绪: %s (%s)" % (label, service, exc))

    def _call_trigger(self, service, label):
        self._wait_service_feedback(service, label)
        proxy = rospy.ServiceProxy(service, Trigger)
        response = proxy()
        if response.success:
            self._ok("%s 成功: %s" % (label, response.message))
            return True
        self._warn("%s 失败: %s" % (label, response.message))
        return False

    def _validate_map_file(self):
        if not os.path.exists(self.map_file):
            raise RuntimeError("找不到地图 yaml: %s" % self.map_file)
        data = load_yaml(self.map_file)
        image = data.get("image", "")
        if image and not os.path.isabs(image):
            image = os.path.join(os.path.dirname(self.map_file), image)
        if image and not os.path.exists(image):
            raise RuntimeError("地图 yaml 存在，但 pgm 不存在: %s" % image)
        self._ok("地图文件校验通过")

    def _validate_zones(self):
        data = load_yaml(self.zone_file)
        zones = data.get("warehouse_tuning", {}).get("abc_zones", {})
        missing = [entry["zone"] for entry in self.zone_plan if entry["zone"] not in zones]
        if missing:
            raise RuntimeError("区域标定缺失: %s" % ", ".join(missing))
        stack = data.get("stack_sort_pipeline", {})
        if "tabletop_return_base_target" not in stack:
            raise RuntimeError("区域文件缺少 source 取货位")
        if "tabletop_drop_base_targets" not in stack:
            raise RuntimeError("区域文件缺少 drop 放置位")
        self._ok("A/B/C 区域文件校验通过")

    def _validate_features(self):
        data = load_yaml(self.feature_file)
        cargo_types = data.get("warehouse_sorting", {}).get("cargo_types", {})
        missing = [cargo_type for cargo_type in self.cargo_types if cargo_type not in cargo_types]
        if missing:
            raise RuntimeError("小方块特征缺失: %s" % ", ".join(missing))
        stack = data.get("stack_sort_pipeline", {})
        if "color_ranges" not in stack:
            raise RuntimeError("特征文件缺少 stack_sort_pipeline.color_ranges")
        self._ok("小方块特征文件校验通过")

    def _print_zone_feedback(self, zone):
        data = load_yaml(self.zone_file)
        zone_data = data.get("warehouse_tuning", {}).get("abc_zones", {}).get(zone, {})
        if not zone_data:
            self._warn("%s 区文件中未找到详细数据" % zone)
            return
        self._ok(
            "%s 区: role=%s color=%s pose=(%.3f, %.3f, %.3f)"
            % (
                zone,
                zone_data.get("role", ""),
                zone_data.get("color", ""),
                float(zone_data.get("x", 0.0)),
                float(zone_data.get("y", 0.0)),
                float(zone_data.get("yaw", 0.0)),
            )
        )

    def _print_feature_feedback(self, cargo_type):
        data = load_yaml(self.feature_file)
        spec = data.get("warehouse_sorting", {}).get("cargo_types", {}).get(cargo_type, {})
        if not spec:
            self._warn("%s 特征文件中未找到详细数据" % cargo_type)
            return
        self._ok(
            "%s: hsv=%s..%s size=%s min_area=%s"
            % (
                cargo_type,
                spec.get("hsv_lower"),
                spec.get("hsv_upper"),
                spec.get("size"),
                spec.get("min_area"),
            )
        )

    def _status(self, message):
        if self.status_pub is not None:
            self.status_pub.publish(String(data=message))
        rospy.loginfo("[field_calibration] %s", message) if rospy.core.is_initialized() else None
        print("[STATUS] %s" % message, flush=True)

    def _step(self, message):
        print("", flush=True)
        print("---- %s ----" % message, flush=True)

    def _ok(self, message):
        print("[OK] %s" % message, flush=True)

    def _warn(self, message):
        print("[WARN] %s" % message, flush=True)

    def _info(self, message):
        print("[INFO] %s" % message, flush=True)

    def _bool_arg(self, value):
        return "true" if value else "false"


def parse_args():
    parser = argparse.ArgumentParser(description="Interactive field calibration wizard.")
    parser.add_argument("--manage-stack", action="store_true", help="start mapping, localization, and capture services")
    parser.add_argument("--keep-managed-stack", action="store_true", help="leave managed roslaunch processes running")
    parser.add_argument("--start-gmapping", action="store_true", default=True)
    parser.add_argument("--no-start-gmapping", dest="start_gmapping", action="store_false")
    parser.add_argument("--rviz", action="store_true")
    parser.add_argument("--map-prefix", default="$HOME/maps/lab")
    parser.add_argument("--map-file", default="")
    parser.add_argument("--zone-file", default="$HOME/maps/abc_zones.yaml")
    parser.add_argument("--feature-file", default="$HOME/maps/cargo_features.yaml")
    parser.add_argument("--debug-output-dir", default="$HOME/maps/debug_images")
    parser.add_argument("--zone-plan", default="A:source:,B:drop:green,C:drop:red")
    parser.add_argument("--cargo-types", default="green,red")
    parser.add_argument("--table-height", type=float, default=0.75)
    parser.add_argument("--stack-anchor-forward-offset", type=float, default=0.56)
    parser.add_argument("--pose-source", default="tf", choices=("tf", "map", "odom", "gazebo", "world"))
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--odom-frame", default="odom")
    parser.add_argument("--base-frame", default="base_footprint")
    parser.add_argument("--map-topic", default="/map")
    parser.add_argument("--scan-topic", default="/scan")
    parser.add_argument("--amcl-pose-topic", default="/amcl_pose")
    parser.add_argument("--rgb-topic", default="/kinect2/sd/image_color_rect")
    parser.add_argument("--depth-topic", default="/kinect2/sd/image_depth_rect")
    parser.add_argument("--camera-info-topic", default="/kinect2/sd/camera_info")
    parser.add_argument("--feature-min-area", type=int, default=500)
    parser.add_argument("--feature-roi-x", type=float, default=0.30)
    parser.add_argument("--feature-roi-y", type=float, default=0.42)
    parser.add_argument("--feature-roi-width", type=float, default=0.35)
    parser.add_argument("--feature-roi-height", type=float, default=0.36)
    parser.add_argument("--save-debug-images", action="store_true", default=True)
    parser.add_argument("--no-save-debug-images", dest="save_debug_images", action="store_false")
    parser.add_argument("--allow-simulated-fallback", action="store_true")
    parser.add_argument("--skip-mapping", action="store_true")
    parser.add_argument("--skip-localization", action="store_true")
    parser.add_argument("--skip-zones", action="store_true")
    parser.add_argument("--skip-features", action="store_true")
    parser.add_argument("--non-interactive", action="store_true")
    parser.add_argument("--non-interactive-settle", type=float, default=3.0)
    parser.add_argument("--master-timeout", type=float, default=30.0)
    parser.add_argument("--topic-timeout", type=float, default=30.0)
    parser.add_argument("--service-timeout", type=float, default=20.0)
    parser.add_argument("--localization-timeout", type=float, default=30.0)
    parser.add_argument("--feedback-interval", type=float, default=2.0)
    return parser.parse_args()


def main():
    args = parse_args()
    wizard = FieldCalibrationWizard(args)
    try:
        wizard.run()
        return 0
    except KeyboardInterrupt:
        print("\n[WARN] 用户中断", flush=True)
        return 130
    except Exception as exc:
        print("[FAIL] %s" % exc, flush=True)
        return 2
    finally:
        wizard.close()


if __name__ == "__main__":
    sys.exit(main())
