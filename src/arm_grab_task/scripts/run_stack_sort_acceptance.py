#!/usr/bin/env python3
import argparse
import math
import os
import re
import signal
import subprocess
import sys
import threading
import time
from typing import Dict, List, Optional, Tuple

import rosgraph
import rospy
from gazebo_msgs.msg import ModelStates
from rosgraph_msgs.msg import Log


DEFAULT_MODELS_BY_COLOR = {
    "green": ["green_box_1", "green_box_2", "green_box_3"],
    "blue": ["blue_box_1", "blue_box_2", "blue_box_3"],
}

DEFAULT_DESTINATION_ZONES = {
    "green": {"x": 0.0, "y": 1.65, "radius": 0.75, "z_min": 0.76, "z_max": 1.20},
    "blue": {"x": 0.0, "y": -1.65, "radius": 0.75, "z_min": 0.76, "z_max": 1.20},
}


class AcceptanceMonitor:
    def __init__(
        self,
        expected_per_color: int,
        max_jump_distance: float,
        ignore_initial_seconds: float,
        require_finish: bool,
        require_validated_grasp: bool,
        min_stack_height_step: float,
    ):
        self.expected_per_color = expected_per_color
        self.max_jump_distance = max_jump_distance
        self.ignore_initial_seconds = ignore_initial_seconds
        self.require_finish = require_finish
        self.require_validated_grasp = require_validated_grasp
        self.min_stack_height_step = min_stack_height_step
        self.started_at = time.time()
        self.finish_at: Optional[float] = None
        self.lock = threading.Lock()

        self.models_by_color = DEFAULT_MODELS_BY_COLOR
        self.destination_zones = DEFAULT_DESTINATION_ZONES
        self.model_to_color = {
            model: color
            for color, models in self.models_by_color.items()
            for model in models
        }
        self.positions: Dict[str, Tuple[float, float, float]] = {}
        self.last_positions: Dict[str, Tuple[float, float, float]] = {}
        self.jump_violations: List[str] = []
        self.success_counts = {color: 0 for color in self.models_by_color}
        self.grasp_lock_counts = {color: 0 for color in self.models_by_color}
        self.grasp_rejections: List[str] = []
        self.failed_cycles: List[str] = []
        self.report_paths: List[str] = []
        self.sim_proxy_events: List[str] = []

        self.cycle_re = re.compile(r"\[METRICS\] cycle color=(\w+) success=(True|False) reason=([^ ]+)")
        self.grasp_lock_re = re.compile(r"\[SIM-GRASP\] locked model=([^ ]+) color=(\w+) xy_err=([0-9.]+) z_err=([0-9.]+)")
        self.physical_grasp_re = re.compile(r"\[PHYS-GRASP\] lifted model=([^ ]+) color=(\w+) dz=([0-9.]+)")
        self.grasp_reject_re = re.compile(r"\[(?:SIM|PHYS)-GRASP\] rejected(?: model=[^ ]+)? color=(\w+) reason=([^ ]+)")
        self.report_re = re.compile(r"\[REPORT\] exported (.+)")

        rospy.Subscriber("/gazebo/model_states", ModelStates, self._model_cb, queue_size=5)
        rospy.Subscriber("/rosout_agg", Log, self._log_cb, queue_size=50)

    def _model_cb(self, msg: ModelStates):
        now = time.time()
        with self.lock:
            names = set(msg.name)
            for model_name in self.model_to_color:
                if model_name not in names:
                    continue
                idx = msg.name.index(model_name)
                p = msg.pose[idx].position
                pos = (float(p.x), float(p.y), float(p.z))
                last = self.last_positions.get(model_name)
                if last is not None and now - self.started_at >= self.ignore_initial_seconds:
                    jump = math.sqrt(
                        (pos[0] - last[0]) ** 2
                        + (pos[1] - last[1]) ** 2
                        + (pos[2] - last[2]) ** 2
                    )
                    if jump > self.max_jump_distance:
                        self.jump_violations.append(
                            "%s jumped %.3fm from (%.2f, %.2f, %.2f) to (%.2f, %.2f, %.2f)"
                            % (model_name, jump, last[0], last[1], last[2], pos[0], pos[1], pos[2])
                        )
                self.positions[model_name] = pos
                self.last_positions[model_name] = pos

    def _log_cb(self, msg: Log):
        text = msg.msg
        with self.lock:
            cycle = self.cycle_re.search(text)
            if cycle:
                color, success, reason = cycle.groups()
                if success == "True":
                    self.success_counts[color] = self.success_counts.get(color, 0) + 1
                else:
                    self.failed_cycles.append("%s:%s" % (color, reason))

            if "All planned stacks completed" in text:
                self.finish_at = time.time()

            grasp_lock = self.grasp_lock_re.search(text)
            if grasp_lock:
                model, color, _, _ = grasp_lock.groups()
                self.sim_proxy_events.append("attach:%s:%s" % (color, model))
                self.grasp_lock_counts[color] = self.grasp_lock_counts.get(color, 0) + 1
            physical_grasp = self.physical_grasp_re.search(text)
            if physical_grasp:
                _, color, _ = physical_grasp.groups()
                self.grasp_lock_counts[color] = self.grasp_lock_counts.get(color, 0) + 1

            if "[SIM-STACK]" in text:
                self.sim_proxy_events.append(text)

            grasp_reject = self.grasp_reject_re.search(text)
            if grasp_reject:
                color, reason = grasp_reject.groups()
                self.grasp_rejections.append("%s:%s" % (color, reason))

            report = self.report_re.search(text)
            if report:
                self.report_paths.append(report.group(1))

    def expected_reached(self) -> bool:
        with self.lock:
            return all(
                self.success_counts.get(color, 0) >= self.expected_per_color
                for color in self.models_by_color
            )

    def finished_and_settled(self, settle_seconds: float) -> bool:
        with self.lock:
            return self.finish_at is not None and time.time() - self.finish_at >= settle_seconds

    def validate(self) -> Tuple[bool, List[str], List[str]]:
        errors: List[str] = []
        notes: List[str] = []
        with self.lock:
            if self.require_finish and self.finish_at is None:
                errors.append("pipeline did not reach FINISH")

            for color in self.models_by_color:
                count = self.success_counts.get(color, 0)
                if count < self.expected_per_color:
                    errors.append(
                        "%s success count %d < expected %d"
                        % (color, count, self.expected_per_color)
                    )

            if self.failed_cycles:
                errors.append("failed cycles: " + ", ".join(self.failed_cycles))

            if self.sim_proxy_events:
                errors.append("simulation proxy events used: " + "; ".join(self.sim_proxy_events[:5]))

            if self.require_validated_grasp:
                for color in self.models_by_color:
                    count = self.grasp_lock_counts.get(color, 0)
                    if count < self.expected_per_color:
                        errors.append(
                            "%s validated grasp count %d < expected %d"
                            % (color, count, self.expected_per_color)
                        )

            if self.jump_violations:
                errors.append("unexpected model jumps: " + "; ".join(self.jump_violations[:5]))

            for color, models in self.models_by_color.items():
                zone = self.destination_zones[color]
                placed_positions = []
                z_values = []
                for model in models:
                    pos = self.positions.get(model)
                    if pos is None:
                        continue
                    dist = math.hypot(pos[0] - zone["x"], pos[1] - zone["y"])
                    if dist <= zone["radius"] and zone["z_min"] <= pos[2] <= zone["z_max"]:
                        placed_positions.append(pos)
                        z_values.append(pos[2])
                if len(placed_positions) < self.expected_per_color:
                    errors.append(
                        "%s models in destination zone %d < expected %d"
                        % (color, len(placed_positions), self.expected_per_color)
                    )
                if (
                    len(z_values) >= 2
                    and self.min_stack_height_step > 0.0
                    and max(z_values) - min(z_values)
                    < self.min_stack_height_step * (len(z_values) - 1)
                ):
                    errors.append(
                        "%s stack height %.3f < expected %.3f"
                        % (
                            color,
                            max(z_values) - min(z_values),
                            self.min_stack_height_step * (len(z_values) - 1),
                        )
                    )

            notes.append("success_counts=%s" % self.success_counts)
            notes.append("validated_grasp_counts=%s" % self.grasp_lock_counts)
            if self.grasp_rejections:
                notes.append("grasp_rejections=%s" % ", ".join(self.grasp_rejections[:8]))
            final_positions = []
            for model in sorted(self.model_to_color):
                pos = self.positions.get(model)
                if pos is not None:
                    final_positions.append("%s=(%.2f,%.2f,%.2f)" % (model, pos[0], pos[1], pos[2]))
            if final_positions:
                notes.append("final_positions=%s" % ", ".join(final_positions))
            if self.report_paths:
                notes.append("reports=%s" % ", ".join(self.report_paths[-4:]))

        return len(errors) == 0, errors, notes


def wait_for_master(timeout: float) -> bool:
    master = rosgraph.Master("/stack_sort_acceptance_runner")
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            master.getPid()
            return True
        except Exception:
            time.sleep(0.5)
    return False


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


def parse_args():
    parser = argparse.ArgumentParser(description="Run ABC tabletop simulation acceptance.")
    parser.add_argument("--timeout", type=float, default=900.0, help="Maximum runtime in seconds.")
    parser.add_argument("--settle-seconds", type=float, default=3.0, help="Wait after FINISH before checking.")
    parser.add_argument("--expected-per-color", type=int, default=3, help="Required placed boxes per color.")
    parser.add_argument("--max-jump-distance", type=float, default=0.30, help="Reject larger per-sample model jumps.")
    parser.add_argument("--ignore-initial-seconds", type=float, default=12.0, help="Ignore initial spawn/reset motion.")
    parser.add_argument("--min-stack-height-step", type=float, default=0.055, help="Minimum final z separation per stacked box.")
    parser.add_argument("--gui", action="store_true", help="Show Gazebo GUI during acceptance.")
    parser.add_argument("--require-finish", dest="require_finish", action="store_true", default=True)
    parser.add_argument("--no-require-finish", dest="require_finish", action="store_false")
    parser.add_argument("--require-validated-grasp", dest="require_validated_grasp", action="store_true", default=True)
    parser.add_argument("--no-require-validated-grasp", dest="require_validated_grasp", action="store_false")
    return parser.parse_args()


def main():
    args = parse_args()
    launch_cmd = [
        "roslaunch",
        "arm_grab_task",
        "stack_sort_abc_tabletop_demo.launch",
        "auto_start_pipeline:=true",
        "gui:=%s" % ("true" if args.gui else "false"),
    ]
    print("[ACCEPTANCE] launching: %s" % " ".join(launch_cmd), flush=True)
    process = subprocess.Popen(launch_cmd, preexec_fn=os.setsid)

    ok = False
    errors: List[str] = []
    notes: List[str] = []
    try:
        if not wait_for_master(timeout=30.0):
            errors = ["ROS master did not start"]
            return 2

        rospy.init_node("stack_sort_acceptance_monitor", anonymous=True, disable_signals=True)
        monitor = AcceptanceMonitor(
            expected_per_color=args.expected_per_color,
            max_jump_distance=args.max_jump_distance,
            ignore_initial_seconds=args.ignore_initial_seconds,
            require_finish=args.require_finish,
            require_validated_grasp=args.require_validated_grasp,
            min_stack_height_step=args.min_stack_height_step,
        )

        start = time.time()
        while time.time() - start < args.timeout and not rospy.is_shutdown():
            if args.require_finish:
                if monitor.finished_and_settled(args.settle_seconds):
                    break
            elif monitor.expected_reached():
                time.sleep(args.settle_seconds)
                break
            if process.poll() is not None:
                break
            time.sleep(0.25)

        ok, errors, notes = monitor.validate()
        return 0 if ok else 2
    finally:
        terminate_process_group(process)
        subprocess.run(["killall", "-q", "gzserver", "gzclient"], check=False)
        if ok:
            print("[ACCEPTANCE] PASS")
        else:
            print("[ACCEPTANCE] FAIL")
        for item in errors:
            print("[ACCEPTANCE] ERROR: %s" % item)
        for item in notes:
            print("[ACCEPTANCE] NOTE: %s" % item)


if __name__ == "__main__":
    sys.exit(main())
