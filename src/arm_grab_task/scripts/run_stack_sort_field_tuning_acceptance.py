#!/usr/bin/env python3

import argparse
import json
import math
import os
import random
import shutil
import signal
import subprocess
import sys
import time
from typing import Dict, Optional

import rosgraph
import rospy
import tf
import yaml
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, SetModelState
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger


BASE_BOX_POSES = {
    "green_box_1": {"x": 0.78, "y": 0.18, "z": 0.808, "yaw": 0.0},
    "green_box_2": {"x": 0.92, "y": 0.32, "z": 0.808, "yaw": 0.0},
    "green_box_3": {"x": 1.06, "y": 0.46, "z": 0.808, "yaw": 0.0},
    "blue_box_1": {"x": 0.78, "y": -0.18, "z": 0.808, "yaw": 0.0},
    "blue_box_2": {"x": 0.92, "y": -0.32, "z": 0.808, "yaw": 0.0},
    "blue_box_3": {"x": 1.06, "y": -0.46, "z": 0.808, "yaw": 0.0},
}


class ProcessGroup:
    def __init__(self):
        self.processes = []

    def launch(self, command):
        print("[FIELD-SIM] launching: %s" % " ".join(command), flush=True)
        proc = subprocess.Popen(command, preexec_fn=os.setsid)
        self.processes.append(proc)
        return proc

    def stop_all(self):
        for proc in reversed(self.processes):
            terminate_process_group(proc)
        subprocess.run(["killall", "-q", "gzserver", "gzclient"], check=False)


def terminate_process_group(process: subprocess.Popen):
    if process.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(process.pid), signal.SIGINT)
    except ProcessLookupError:
        return
    try:
        process.wait(timeout=10.0)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
        except ProcessLookupError:
            pass
        try:
            process.wait(timeout=8.0)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass


def wait_for_master(timeout: float) -> bool:
    master = rosgraph.Master("/field_tuning_acceptance_runner")
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            master.getPid()
            return True
        except Exception:
            time.sleep(0.5)
    return False


def wait_service(name: str, timeout: float):
    rospy.loginfo("[FIELD-SIM] waiting service %s", name)
    rospy.wait_for_service(name, timeout=timeout)


def call_trigger(name: str):
    wait_service(name, 20.0)
    response = rospy.ServiceProxy(name, Trigger)()
    if not response.success:
        raise RuntimeError("%s failed: %s" % (name, response.message))
    rospy.loginfo("[FIELD-SIM] %s: %s", name, response.message)


def wait_topic(name: str, topic_type, timeout: float):
    rospy.loginfo("[FIELD-SIM] waiting topic %s", name)
    return rospy.wait_for_message(name, topic_type, timeout=timeout)


def yaw_from_quat(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def normalize(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def table_base_target(get_model_state, table_name: str, approach_distance: float):
    result = get_model_state(table_name, "world")
    if not result.success:
        raise RuntimeError("missing Gazebo table model: %s" % table_name)
    yaw = yaw_from_quat(result.pose.orientation)
    x = result.pose.position.x - math.cos(yaw) * approach_distance
    y = result.pose.position.y - math.sin(yaw) * approach_distance
    return {"x": x, "y": y, "yaw": yaw}


def set_robot_pose(set_model_state, pose: Dict[str, float], model_name: str):
    set_model_pose(set_model_state, pose, model_name, default_z=0.0)
    rospy.sleep(1.0)


def set_model_pose(set_model_state, pose: Dict[str, float], model_name: str, default_z: float):
    state = ModelState()
    state.model_name = model_name
    state.reference_frame = "world"
    state.pose.position.x = pose["x"]
    state.pose.position.y = pose["y"]
    state.pose.position.z = float(pose.get("z", default_z))
    quat = tf.transformations.quaternion_from_euler(0.0, 0.0, pose["yaw"])
    state.pose.orientation.x = quat[0]
    state.pose.orientation.y = quat[1]
    state.pose.orientation.z = quat[2]
    state.pose.orientation.w = quat[3]
    set_model_state(state)


def current_model_pose(get_model_state, model_name: str) -> Dict[str, float]:
    result = get_model_state(model_name, "world")
    if not result.success:
        raise RuntimeError("missing Gazebo model: %s" % model_name)
    return {
        "x": float(result.pose.position.x),
        "y": float(result.pose.position.y),
        "z": float(result.pose.position.z),
        "yaw": yaw_from_quat(result.pose.orientation),
    }


def wait_for_models(get_model_state, model_names, timeout: float):
    deadline = time.time() + timeout
    missing = list(model_names)
    while time.time() < deadline and not rospy.is_shutdown():
        missing = []
        for model_name in model_names:
            result = get_model_state(model_name, "world")
            if not result.success:
                missing.append(model_name)
        if not missing:
            return
        rospy.sleep(0.2)
    raise RuntimeError("models not ready: %s" % ",".join(missing))


def publish_initial_pose(x: float, y: float, yaw: float, timeout: float):
    rospy.loginfo("[FIELD-SIM] publishing /initialpose pose=(%.3f, %.3f, %.3f)", x, y, yaw)
    pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1, latch=True)
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = "map"
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    quat = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)
    msg.pose.pose.orientation.x = quat[0]
    msg.pose.pose.orientation.y = quat[1]
    msg.pose.pose.orientation.z = quat[2]
    msg.pose.pose.orientation.w = quat[3]
    msg.pose.covariance[0] = 0.25
    msg.pose.covariance[7] = 0.25
    msg.pose.covariance[35] = 0.0685
    deadline = time.time() + timeout
    while time.time() < deadline and not rospy.is_shutdown():
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        try:
            amcl = rospy.wait_for_message("/amcl_pose", PoseWithCovarianceStamped, timeout=0.25)
            ayaw = yaw_from_quat(amcl.pose.pose.orientation)
            if (
                math.hypot(amcl.pose.pose.position.x - x, amcl.pose.pose.position.y - y) <= 0.15
                and abs(normalize(ayaw - yaw)) <= 0.25
            ):
                rospy.loginfo("[FIELD-SIM] initial pose verified")
                return
        except Exception:
            pass
    raise RuntimeError("initial pose was not verified")


def box_layout(args) -> Dict[str, Dict[str, float]]:
    poses = {
        name: {"x": pose["x"], "y": pose["y"], "z": pose["z"], "yaw": pose["yaw"]}
        for name, pose in BASE_BOX_POSES.items()
    }
    for pose in poses.values():
        pose["z"] = args.tabletop_model_z
    if args.box_layout == "fixed":
        return poses

    rng = random.Random(args.box_seed)
    for name, pose in poses.items():
        pose["x"] += rng.uniform(-args.box_jitter_x, args.box_jitter_x)
        pose["y"] += rng.uniform(-args.box_jitter_y, args.box_jitter_y)
        pose["yaw"] += rng.uniform(-args.box_yaw_jitter, args.box_yaw_jitter)
        pose["x"] = min(1.12, max(0.72, pose["x"]))
        if name.startswith("green"):
            pose["y"] = min(0.52, max(0.10, pose["y"]))
        elif name.startswith("blue"):
            pose["y"] = max(-0.52, min(-0.10, pose["y"]))
    return poses


def apply_box_layout(set_model_state, poses: Dict[str, Dict[str, float]]):
    for model_name, pose in poses.items():
        set_model_pose(set_model_state, pose, model_name, default_z=0.808)
        rospy.loginfo(
            "[FIELD-SIM] box pose %s=(%.3f, %.3f, %.3f, %.3f)",
            model_name,
            pose["x"],
            pose["y"],
            pose["z"],
            pose["yaw"],
        )
    rospy.sleep(1.0)


def set_stack_sort_box_params(poses: Dict[str, Dict[str, float]]):
    rospy.set_param("/stack_sort_pipeline/gazebo_initial_model_poses", poses)


def recursive_set_param(prefix: str, value):
    if isinstance(value, dict):
        for key, child in value.items():
            recursive_set_param("%s/%s" % (prefix.rstrip("/"), str(key)), child)
        return
    rospy.set_param(prefix, value)


def load_override_yaml(path: str):
    with open(path, "r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}
    for key, value in data.items():
        recursive_set_param("/" + str(key), value)


def wait_for_finish(timeout: float) -> Dict:
    deadline = time.time() + timeout
    last = {}
    while time.time() < deadline and not rospy.is_shutdown():
        try:
            msg = rospy.wait_for_message("/stack_sort/status", String, timeout=1.0)
            last = json.loads(msg.data)
        except Exception:
            continue
        if last.get("state") == "FINISH" and last.get("total_done") == last.get("total_goal"):
            return last
    raise RuntimeError("stack_sort did not reach FINISH, last_status=%s" % last)


def validate_final_positions(get_model_state, expected_per_color: int):
    zones = {
        "green": {"x": 0.0, "y": 1.65, "radius": 0.75, "z_min": 0.76, "z_max": 1.20},
        "blue": {"x": 0.0, "y": -1.65, "radius": 0.75, "z_min": 0.76, "z_max": 1.20},
    }
    errors = []
    for color, zone in zones.items():
        count = 0
        for idx in range(1, expected_per_color + 1):
            model = "%s_box_%d" % (color, idx)
            result = get_model_state(model, "world")
            if not result.success:
                errors.append("missing final model %s" % model)
                continue
            p = result.pose.position
            if (
                math.hypot(p.x - zone["x"], p.y - zone["y"]) <= zone["radius"]
                and zone["z_min"] <= p.z <= zone["z_max"]
            ):
                count += 1
        if count < expected_per_color:
            errors.append("%s final count %d < %d" % (color, count, expected_per_color))
    if errors:
        raise RuntimeError("; ".join(errors))


def parse_args():
    parser = argparse.ArgumentParser(description="Run mapping, localization, tuning, and stack sorting in simulation.")
    parser.add_argument("--output-dir", default="/tmp/warehouse_tuning_sim")
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--timeout", type=float, default=900.0)
    parser.add_argument("--expected-per-color", type=int, default=3)
    parser.add_argument("--source-approach-distance", type=float, default=0.95)
    parser.add_argument("--drop-approach-distance", type=float, default=0.56)
    parser.add_argument("--map-profile", default="empty", choices=("empty", "complex_lab"))
    parser.add_argument("--mapping-mode", default="mock", choices=("mock", "gmapping"))
    parser.add_argument("--mapping-route", default="field_loop", choices=("field_loop", "short_loop"))
    parser.add_argument("--mapping-drive-timeout", type=float, default=100.0)
    parser.add_argument("--box-layout", default="fixed", choices=("fixed", "jittered"))
    parser.add_argument("--box-seed", type=int, default=7)
    parser.add_argument("--box-jitter-x", type=float, default=0.055)
    parser.add_argument("--box-jitter-y", type=float, default=0.045)
    parser.add_argument("--box-yaw-jitter", type=float, default=0.0)
    parser.add_argument("--tabletop-model-z", type=float, default=0.808)
    return parser.parse_args()


def main():
    args = parse_args()
    output_dir = os.path.abspath(os.path.expanduser(args.output_dir))
    if os.path.exists(output_dir):
        shutil.rmtree(output_dir)
    os.makedirs(output_dir, exist_ok=True)
    map_prefix = os.path.join(output_dir, "lab")
    map_file = map_prefix + ".yaml"
    zone_file = os.path.join(output_dir, "abc_zones.yaml")
    feature_file = os.path.join(output_dir, "cargo_features.yaml")

    processes = ProcessGroup()
    ok = False
    try:
        processes.launch(
            [
                "roslaunch",
                "arm_grab_task",
                "stack_sort_abc_tabletop_demo.launch",
                "auto_start_pipeline:=false",
                "gui:=%s" % ("true" if args.gui else "false"),
            ]
        )
        if not wait_for_master(30.0):
            raise RuntimeError("ROS master did not start")
        rospy.init_node("field_tuning_acceptance_runner", anonymous=True, disable_signals=True)

        wait_service("/gazebo/get_model_state", 30.0)
        wait_service("/gazebo/set_model_state", 30.0)
        get_model_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        set_model_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

        poses = box_layout(args)
        wait_for_models(get_model_state, list(poses.keys()) + ["wpb_home"], 30.0)
        rospy.loginfo(
            "[FIELD-SIM] box_layout=%s seed=%s jitter=(%.3f, %.3f, %.3f)",
            args.box_layout,
            args.box_seed,
            args.box_jitter_x,
            args.box_jitter_y,
            args.box_yaw_jitter,
        )
        apply_box_layout(set_model_state, poses)
        set_stack_sort_box_params(poses)

        mapping_launch = None
        map_launch = None
        if args.mapping_mode == "gmapping":
            if args.map_profile != "empty":
                rospy.logwarn("[FIELD-SIM] map_profile=%s ignored when mapping_mode=gmapping", args.map_profile)
            mapping_launch = processes.launch(
                [
                    "roslaunch",
                    "warehouse_tuning",
                    "sim_gmapping_session.launch",
                    "map_prefix:=%s" % map_prefix,
                ]
            )
            wait_topic("/scan", LaserScan, 30.0)
            wait_topic("/map", OccupancyGrid, 45.0)
            drive_proc = processes.launch(
                [
                    "rosrun",
                    "warehouse_tuning",
                    "sim_mapping_drive.py",
                    "_route:=%s" % args.mapping_route,
                    "_timeout:=%.1f" % args.mapping_drive_timeout,
                ]
            )
            try:
                drive_code = drive_proc.wait(timeout=args.mapping_drive_timeout + 20.0)
            except subprocess.TimeoutExpired:
                terminate_process_group(drive_proc)
                raise RuntimeError("mapping drive did not finish")
            if drive_code != 0:
                raise RuntimeError("mapping drive failed with exit code %s" % drive_code)
            wait_topic("/map", OccupancyGrid, 20.0)
            rospy.sleep(2.0)
        else:
            map_launch = processes.launch(
                [
                    "roslaunch",
                    "warehouse_tuning",
                    "sim_map_localization.launch",
                    "publish_mock_map:=true",
                    "use_saved_map:=false",
                    "map_profile:=%s" % args.map_profile,
                ]
            )
            mapping_launch = processes.launch(
                [
                    "roslaunch",
                    "warehouse_tuning",
                    "mapping_session.launch",
                    "start_gmapping:=false",
                    "map_prefix:=%s" % map_prefix,
                ]
            )
        call_trigger("/warehouse_tuning/save_map")
        if not os.path.exists(map_file):
            raise RuntimeError("map file was not saved: %s" % map_file)

        localization_pose = current_model_pose(get_model_state, "wpb_home")
        rospy.loginfo(
            "[FIELD-SIM] localization seed from current robot pose=(%.3f, %.3f, %.3f)",
            localization_pose["x"],
            localization_pose["y"],
            localization_pose["yaw"],
        )
        if mapping_launch is not None:
            terminate_process_group(mapping_launch)
        if map_launch is not None:
            terminate_process_group(map_launch)
        processes.launch(
            [
                "roslaunch",
                "warehouse_tuning",
                "sim_map_localization.launch",
                "publish_mock_map:=false",
                "use_saved_map:=true",
                "map_file:=%s" % map_file,
            ]
        )
        publish_initial_pose(localization_pose["x"], localization_pose["y"], localization_pose["yaw"], timeout=10.0)

        capture_launch = processes.launch(
            [
                "roslaunch",
                "warehouse_tuning",
                "stack_sort_capture_services.launch",
                "zone_output_file:=%s" % zone_file,
                "feature_output_file:=%s" % feature_file,
                "pose_source:=gazebo",
                "allow_simulated_fallback:=true",
                "feature_min_area:=200",
                "feature_roi_x:=0.00",
                "feature_roi_y:=0.00",
                "feature_roi_width:=1.00",
                "feature_roi_height:=1.00",
            ]
        )
        for service in (
            "/warehouse_tuning/capture_zone_A",
            "/warehouse_tuning/capture_zone_B",
            "/warehouse_tuning/capture_zone_C",
            "/warehouse_tuning/capture_features_green",
            "/warehouse_tuning/capture_features_blue",
        ):
            wait_service(service, 30.0)

        targets = {
            "A": table_base_target(get_model_state, "table_a", args.source_approach_distance),
            "B": table_base_target(get_model_state, "table_b", args.drop_approach_distance),
            "C": table_base_target(get_model_state, "table_c", args.drop_approach_distance),
        }
        for zone, service in (
            ("A", "/warehouse_tuning/capture_zone_A"),
            ("B", "/warehouse_tuning/capture_zone_B"),
            ("C", "/warehouse_tuning/capture_zone_C"),
        ):
            set_robot_pose(set_model_state, targets[zone], "wpb_home")
            publish_initial_pose(targets[zone]["x"], targets[zone]["y"], targets[zone]["yaw"], timeout=10.0)
            call_trigger(service)

        set_robot_pose(set_model_state, targets["A"], "wpb_home")
        publish_initial_pose(targets["A"]["x"], targets["A"]["y"], targets["A"]["yaw"], timeout=10.0)
        call_trigger("/warehouse_tuning/capture_features_green")
        call_trigger("/warehouse_tuning/capture_features_blue")
        if not os.path.exists(zone_file) or not os.path.exists(feature_file):
            raise RuntimeError("tuning files were not generated")

        load_override_yaml(zone_file)
        load_override_yaml(feature_file)
        set_stack_sort_box_params(poses)
        processes.launch(["rosrun", "arm_grab_task", "stack_sort_pipeline.py"])
        status = wait_for_finish(args.timeout)
        validate_final_positions(get_model_state, args.expected_per_color)
        ok = True
        print("[FIELD-SIM] PASS status=%s" % status, flush=True)
        print("[FIELD-SIM] map=%s" % map_file, flush=True)
        print("[FIELD-SIM] zones=%s" % zone_file, flush=True)
        print("[FIELD-SIM] features=%s" % feature_file, flush=True)
        return 0
    except Exception as exc:
        print("[FIELD-SIM] FAIL: %s" % exc, flush=True)
        return 2
    finally:
        if not ok:
            print("[FIELD-SIM] output_dir=%s" % output_dir, flush=True)
        processes.stop_all()


if __name__ == "__main__":
    sys.exit(main())
