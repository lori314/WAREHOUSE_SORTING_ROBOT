#!/usr/bin/env python3

import json
import math
import os
import select
import sys
from datetime import datetime

import rospy
from geometry_msgs.msg import Pose, Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from wpb_home_behaviors.msg import Coord


def _bool_param(name, default):
    value = rospy.get_param(name, default)
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def _list_param(name, default):
    value = rospy.get_param(name, default)
    if isinstance(value, str):
        return [item.strip() for item in value.split(",") if item.strip()]
    if isinstance(value, (list, tuple)):
        return [str(item).strip() for item in value if str(item).strip()]
    return list(default)


def _optional_float_param(name):
    value = rospy.get_param(name, "")
    if value is None:
        return None
    if isinstance(value, str) and value.strip() == "":
        return None
    return float(value)


class WpbPickUnitTest:
    def __init__(self):
        self.behaviors_topic = rospy.get_param("~behaviors_topic", "/wpb_home/behaviors")
        self.objects_topic = rospy.get_param("~objects_topic", "/wpb_home/objects_3d")
        self.grab_action_topic = rospy.get_param("~grab_action_topic", "/wpb_home/grab_action")
        self.target_color_topic = rospy.get_param("~target_color_topic", "/wpb_home/grab_target_color")
        self.grab_result_topic = rospy.get_param("~grab_result_topic", "/wpb_home/grab_result")
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
        self.mani_ctrl_topic = rospy.get_param("~mani_ctrl_topic", "/wpb_home/mani_ctrl")

        self.trials = int(rospy.get_param("~trials", 1))
        self.target_color = str(rospy.get_param("~target_color", "green")).strip().lower()
        self.manual_step = _bool_param("~manual_step", True)
        self.dry_run = _bool_param("~dry_run", False)
        self.stop_on_failure = _bool_param("~stop_on_failure", True)
        self.stop_detection_after_grab = _bool_param("~stop_detection_after_grab", False)
        self.stop_runtime_on_exit = _bool_param("~stop_runtime_on_exit", True)
        self.wait_for_connections = _bool_param("~wait_for_connections", True)

        self.detect_warmup_seconds = float(rospy.get_param("~detect_warmup_seconds", 0.6))
        self.object_wait_timeout = float(rospy.get_param("~object_wait_timeout", 8.0))
        self.result_timeout = float(rospy.get_param("~result_timeout", 80.0))
        self.settle_between_trials = float(rospy.get_param("~settle_between_trials", 1.0))
        self.stop_repeat_seconds = float(rospy.get_param("~stop_repeat_seconds", 0.6))

        self.min_probability = float(rospy.get_param("~min_probability", 0.0))
        self.min_x = float(rospy.get_param("~min_x", 0.25))
        self.max_x = float(rospy.get_param("~max_x", 1.60))
        self.min_y = float(rospy.get_param("~min_y", -0.55))
        self.max_y = float(rospy.get_param("~max_y", 0.55))
        self.min_z = float(rospy.get_param("~min_z", 0.55))
        self.max_z = float(rospy.get_param("~max_z", 0.95))
        self.selection_policy = rospy.get_param("~selection_policy", "closest_to_target")
        self.selection_target_x = float(rospy.get_param("~selection_target_x", 1.05))
        self.selection_target_y = float(rospy.get_param("~selection_target_y", 0.0))

        self.action_x_offset = float(rospy.get_param("~action_x_offset", 0.0))
        self.action_y_offset = float(rospy.get_param("~action_y_offset", 0.0))
        self.action_z_offset = float(rospy.get_param("~action_z_offset", 0.0))
        self.action_x_override = _optional_float_param("~action_x_override")
        self.action_y_override = _optional_float_param("~action_y_override")
        self.action_z_override = _optional_float_param("~action_z_override")

        self.result_done_token = rospy.get_param("~result_done_token", "done").strip().lower()
        self.fatal_result_tokens = _list_param("~fatal_result_tokens", ["fail", "abort", "error"])
        self.expected_result_sequence = _list_param(
            "~expected_result_sequence",
            ["object x", "hand up", "forward", "grab", "object up", "backward", "done"],
        )

        self.stow_arm_before_trial = _bool_param("~stow_arm_before_trial", True)
        self.stow_lift = float(rospy.get_param("~stow_lift", 0.35))
        self.stow_gripper = float(rospy.get_param("~stow_gripper", 0.04))
        self.stow_seconds = float(rospy.get_param("~stow_seconds", 1.0))
        self.stow_lift_speed = float(rospy.get_param("~stow_lift_speed", 0.12))
        self.stow_gripper_speed = float(rospy.get_param("~stow_gripper_speed", 5.0))

        self.report_output_dir = rospy.get_param("~report_output_dir", "/tmp/arm_grab_task_reports")
        self.report_prefix = rospy.get_param("~report_prefix", "wpb_pick_unit")

        self.behaviors_pub = rospy.Publisher(self.behaviors_topic, String, queue_size=10)
        self.grab_pub = rospy.Publisher(self.grab_action_topic, Pose, queue_size=1)
        self.target_color_pub = rospy.Publisher(self.target_color_topic, String, queue_size=1, latch=True)
        self.cmd_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=10)
        self.mani_pub = rospy.Publisher(self.mani_ctrl_topic, JointState, queue_size=10)

        self.objects = []
        self.objects_stamp = 0.0
        self.result_events = []
        rospy.Subscriber(self.objects_topic, Coord, self._on_objects, queue_size=1)
        rospy.Subscriber(self.grab_result_topic, String, self._on_result, queue_size=20)

        self.report = {
            "started_at": datetime.now().isoformat(),
            "params": self._params_snapshot(),
            "trials": [],
        }

    def _params_snapshot(self):
        return {
            "objects_topic": self.objects_topic,
            "grab_action_topic": self.grab_action_topic,
            "target_color_topic": self.target_color_topic,
            "grab_result_topic": self.grab_result_topic,
            "trials": self.trials,
            "target_color": self.target_color,
            "manual_step": self.manual_step,
            "dry_run": self.dry_run,
            "object_wait_timeout": self.object_wait_timeout,
            "result_timeout": self.result_timeout,
            "candidate_filter": {
                "x": [self.min_x, self.max_x],
                "y": [self.min_y, self.max_y],
                "z": [self.min_z, self.max_z],
                "min_probability": self.min_probability,
            },
            "selection_policy": self.selection_policy,
            "selection_target": {"x": self.selection_target_x, "y": self.selection_target_y},
            "action_offset": {
                "x": self.action_x_offset,
                "y": self.action_y_offset,
                "z": self.action_z_offset,
            },
            "action_override": {
                "x": self.action_x_override,
                "y": self.action_y_override,
                "z": self.action_z_override,
            },
            "stow_arm_before_trial": self.stow_arm_before_trial,
            "stow_lift": self.stow_lift,
            "stow_gripper": self.stow_gripper,
        }

    def _on_objects(self, msg):
        objects = []
        count = min(len(msg.name), len(msg.x), len(msg.y), len(msg.z))
        for i in range(count):
            probability = float(msg.probability[i]) if i < len(msg.probability) else 1.0
            objects.append(
                {
                    "name": str(msg.name[i]),
                    "x": float(msg.x[i]),
                    "y": float(msg.y[i]),
                    "z": float(msg.z[i]),
                    "probability": probability,
                }
            )
        self.objects = objects
        self.objects_stamp = rospy.Time.now().to_sec()

    def _on_result(self, msg):
        value = msg.data.strip()
        if not value:
            return
        event = {"stamp": rospy.Time.now().to_sec(), "data": value}
        self.result_events.append(event)
        rospy.loginfo("[WPB-PICK-TEST] result=%s", value)

    def _wait_for_publishers(self):
        if not self.wait_for_connections:
            return
        publishers = [
            (self.behaviors_pub, self.behaviors_topic, False),
            (self.grab_pub, self.grab_action_topic, True),
            (self.target_color_pub, self.target_color_topic, False),
            (self.cmd_pub, self.cmd_vel_topic, False),
            (self.mani_pub, self.mani_ctrl_topic, False),
        ]
        deadline = rospy.Time.now() + rospy.Duration(10.0)
        for pub, topic, required in publishers:
            while not rospy.is_shutdown() and pub.get_num_connections() == 0 and rospy.Time.now() < deadline:
                rospy.loginfo_throttle(1.0, "[WPB-PICK-TEST] waiting for subscriber on %s", topic)
                rospy.sleep(0.1)
            if pub.get_num_connections() == 0:
                log = rospy.logerr if required else rospy.logwarn
                log("[WPB-PICK-TEST] no subscriber on %s", topic)

    def _publish_behavior(self, command, repeat_seconds=0.0, hz=10.0):
        msg = String(data=command)
        end_time = rospy.Time.now() + rospy.Duration(max(0.0, repeat_seconds))
        rate = rospy.Rate(hz)
        first = True
        while not rospy.is_shutdown() and (first or rospy.Time.now() < end_time):
            self.behaviors_pub.publish(msg)
            first = False
            rate.sleep()

    def _stop_base(self, repeat_seconds=0.0, hz=20.0):
        msg = Twist()
        end_time = rospy.Time.now() + rospy.Duration(max(0.0, repeat_seconds))
        rate = rospy.Rate(hz)
        first = True
        while not rospy.is_shutdown() and (first or rospy.Time.now() < end_time):
            self.cmd_pub.publish(msg)
            first = False
            rate.sleep()

    def _stop_runtime(self):
        self._publish_behavior("grab stop", repeat_seconds=self.stop_repeat_seconds)
        self._publish_behavior("object_detect stop", repeat_seconds=self.stop_repeat_seconds)
        self._stop_base(repeat_seconds=self.stop_repeat_seconds)

    def _stow_arm(self):
        if not self.stow_arm_before_trial:
            return
        msg = JointState()
        msg.name = ["lift", "gripper"]
        msg.position = [self.stow_lift, self.stow_gripper]
        msg.velocity = [self.stow_lift_speed, self.stow_gripper_speed]
        end_time = rospy.Time.now() + rospy.Duration(max(0.0, self.stow_seconds))
        rate = rospy.Rate(15)
        rospy.loginfo(
            "[WPB-PICK-TEST] stow arm lift=%.3f gripper=%.3f",
            self.stow_lift,
            self.stow_gripper,
        )
        while not rospy.is_shutdown() and rospy.Time.now() < end_time:
            msg.header.stamp = rospy.Time.now()
            self.mani_pub.publish(msg)
            rate.sleep()

    def _prompt(self, message):
        if not self.manual_step:
            return True
        if sys.stdin is None or not sys.stdin.isatty():
            rospy.logwarn("[WPB-PICK-TEST] stdin unavailable; skip manual pause: %s", message)
            return True
        print(message + " [Enter/q]: ", end="", flush=True)
        while not rospy.is_shutdown():
            readable, _, _ = select.select([sys.stdin], [], [], 0.2)
            if not readable:
                continue
            value = sys.stdin.readline().strip().lower()
            return value != "q"
        return False

    def _candidate_ok(self, obj):
        return (
            self.min_x <= obj["x"] <= self.max_x
            and self.min_y <= obj["y"] <= self.max_y
            and self.min_z <= obj["z"] <= self.max_z
            and obj["probability"] >= self.min_probability
        )

    def _candidate_score(self, obj):
        policy = self.selection_policy.strip().lower()
        if policy == "frontmost":
            return (obj["x"], abs(obj["y"] - self.selection_target_y))
        if policy == "highest_probability":
            return (-obj["probability"], abs(obj["y"] - self.selection_target_y))
        if policy == "centered":
            return (abs(obj["y"] - self.selection_target_y), abs(obj["x"] - self.selection_target_x))
        return (
            math.hypot(obj["x"] - self.selection_target_x, obj["y"] - self.selection_target_y),
            -obj["probability"],
        )

    def _latest_candidates_since(self, started_at):
        if self.objects_stamp < started_at:
            return []
        return [obj for obj in self.objects if self._candidate_ok(obj)]

    def _wait_for_candidate(self, started_at):
        deadline = rospy.Time.now().to_sec() + self.object_wait_timeout
        rate = rospy.Rate(10)
        last_objects = []
        while not rospy.is_shutdown() and rospy.Time.now().to_sec() < deadline:
            candidates = self._latest_candidates_since(started_at)
            if candidates:
                candidates.sort(key=self._candidate_score)
                return candidates[0], candidates
            if self.objects_stamp >= started_at:
                last_objects = list(self.objects)
            rate.sleep()
        return None, last_objects

    def _pose_for_object(self, obj):
        pose = Pose()
        pose.position.x = self.action_x_override if self.action_x_override is not None else obj["x"]
        pose.position.y = self.action_y_override if self.action_y_override is not None else obj["y"]
        pose.position.z = self.action_z_override if self.action_z_override is not None else obj["z"]
        pose.position.x += self.action_x_offset
        pose.position.y += self.action_y_offset
        pose.position.z += self.action_z_offset
        pose.orientation.w = 1.0
        return pose

    def _events_since(self, started_at):
        return [event for event in self.result_events if event["stamp"] >= started_at]

    def _wait_for_result(self, started_at):
        deadline = started_at + self.result_timeout
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and rospy.Time.now().to_sec() < deadline:
            events = self._events_since(started_at)
            if events:
                latest = events[-1]["data"].strip().lower()
                if self.result_done_token and self.result_done_token in latest:
                    return True, "done", events
                for token in self.fatal_result_tokens:
                    if token and token.lower() in latest:
                        return False, "fatal_result:%s" % latest, events
            rate.sleep()
        return False, "timeout_waiting_for_%s" % self.result_done_token, self._events_since(started_at)

    def _check_expected_sequence(self, events):
        values = [event["data"].strip().lower() for event in events]
        missing = []
        for expected in self.expected_result_sequence:
            expected_lower = expected.strip().lower()
            if expected_lower and not any(expected_lower in value for value in values):
                missing.append(expected)
        return missing

    def _run_trial(self, trial_index):
        trial = {
            "trial": trial_index,
            "started_at": datetime.now().isoformat(),
            "success": False,
            "reason": "",
            "selected": None,
            "candidates": [],
            "sent_pose": None,
            "result_events": [],
            "missing_expected_results": [],
            "target_color": self.target_color,
        }
        self.report["trials"].append(trial)

        self._stop_runtime()
        self._stow_arm()
        if not self._prompt(
            "[WPB-PICK-TEST] place one test object on the table, keep hands clear, then continue trial %d/%d"
            % (trial_index, self.trials)
        ):
            trial["reason"] = "user_cancelled"
            return False

        self.objects = []
        self.objects_stamp = 0.0
        detect_started = rospy.Time.now().to_sec()
        rospy.loginfo("[WPB-PICK-TEST] trial %d object_detect start", trial_index)
        self._publish_behavior("object_detect start", repeat_seconds=max(0.1, self.detect_warmup_seconds))
        selected, seen = self._wait_for_candidate(detect_started)
        trial["candidates"] = seen
        if selected is None:
            trial["reason"] = "no_candidate"
            rospy.logerr(
                "[WPB-PICK-TEST] no candidate in %.1fs; seen=%s",
                self.object_wait_timeout,
                json.dumps(seen, sort_keys=True),
            )
            self._stop_runtime()
            return False

        trial["selected"] = selected
        rospy.loginfo("[WPB-PICK-TEST] selected=%s", json.dumps(selected, sort_keys=True))
        if self.dry_run:
            trial["success"] = True
            trial["reason"] = "dry_run_candidate_found"
            return True

        if not self._prompt(
            "[WPB-PICK-TEST] selected %s at x=%.3f y=%.3f z=%.3f. Continue to real grab?"
            % (selected["name"], selected["x"], selected["y"], selected["z"])
        ):
            trial["reason"] = "user_cancelled_before_grab"
            self._stop_runtime()
            return False

        pose = self._pose_for_object(selected)
        trial["sent_pose"] = {
            "x": pose.position.x,
            "y": pose.position.y,
            "z": pose.position.z,
        }
        result_started = rospy.Time.now().to_sec()
        self.target_color_pub.publish(String(data=self.target_color))
        rospy.logwarn(
            "[WPB-PICK-TEST] publish grab pose color=%s object=%s xyz=(%.3f, %.3f, %.3f)",
            self.target_color,
            selected["name"],
            pose.position.x,
            pose.position.y,
            pose.position.z,
        )
        self.grab_pub.publish(pose)
        if self.stop_detection_after_grab:
            self._publish_behavior("object_detect stop")

        ok, reason, events = self._wait_for_result(result_started)
        trial["result_events"] = events
        trial["missing_expected_results"] = self._check_expected_sequence(events)
        trial["success"] = bool(ok)
        trial["reason"] = reason
        if ok:
            rospy.loginfo("[WPB-PICK-TEST] trial %d PASS", trial_index)
        else:
            rospy.logerr("[WPB-PICK-TEST] trial %d FAIL: %s", trial_index, reason)
            self._stop_runtime()
        return ok

    def _write_report(self):
        self.report["finished_at"] = datetime.now().isoformat()
        self.report["success_count"] = sum(1 for item in self.report["trials"] if item.get("success"))
        self.report["trial_count"] = len(self.report["trials"])
        os.makedirs(os.path.abspath(os.path.expanduser(self.report_output_dir)), exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = os.path.join(
            os.path.abspath(os.path.expanduser(self.report_output_dir)),
            "%s_%s.json" % (self.report_prefix, stamp),
        )
        with open(path, "w", encoding="utf-8") as stream:
            json.dump(self.report, stream, indent=2, sort_keys=True)
        rospy.loginfo("[WPB-PICK-TEST] report=%s", path)
        return path

    def run(self):
        self._wait_for_publishers()
        success_count = 0
        for trial_index in range(1, max(1, self.trials) + 1):
            ok = self._run_trial(trial_index)
            if ok:
                success_count += 1
            elif self.stop_on_failure:
                break
            if trial_index < self.trials and self.settle_between_trials > 0.0:
                rospy.sleep(self.settle_between_trials)
        if self.stop_runtime_on_exit:
            self._publish_behavior("object_detect stop", repeat_seconds=self.stop_repeat_seconds)
            self._stop_base(repeat_seconds=self.stop_repeat_seconds)
        self._write_report()
        rospy.loginfo(
            "[WPB-PICK-TEST] summary success=%d/%d dry_run=%s",
            success_count,
            max(1, self.trials),
            str(self.dry_run),
        )
        return 0 if success_count == max(1, self.trials) else 2


def main():
    rospy.init_node("wpb_pick_unit_test")
    node = WpbPickUnitTest()
    try:
        return node.run()
    finally:
        node._stop_base(repeat_seconds=0.2)


if __name__ == "__main__":
    sys.exit(main())
