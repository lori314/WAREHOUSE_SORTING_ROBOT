#!/usr/bin/env python3

import os
import subprocess

import rospy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse


class MappingSessionNode:
    def __init__(self):
        self.map_topic = rospy.get_param("~map_topic", "/map")
        self.map_prefix = rospy.get_param("~map_prefix", "/tmp/warehouse_lab_map")
        self.wait_timeout = float(rospy.get_param("~wait_timeout", 10.0))
        self.status_pub = rospy.Publisher("/warehouse_tuning/mapping_status", String, queue_size=5, latch=True)
        rospy.Service("/warehouse_tuning/save_map", Trigger, self.handle_save_map)
        self._status("ready map_topic=%s map_prefix=%s" % (self.map_topic, self.map_prefix))

    def handle_save_map(self, _request):
        try:
            map_msg = rospy.wait_for_message(self.map_topic, OccupancyGrid, timeout=self.wait_timeout)
        except Exception as exc:
            message = "no map received on %s: %s" % (self.map_topic, exc)
            self._status(message)
            return TriggerResponse(False, message)

        output_dir = os.path.dirname(os.path.abspath(os.path.expanduser(self.map_prefix)))
        if output_dir and not os.path.isdir(output_dir):
            os.makedirs(output_dir)
        prefix = os.path.abspath(os.path.expanduser(self.map_prefix))
        command = ["rosrun", "map_server", "map_saver", "-f", prefix]
        self._status("saving map with: %s" % " ".join(command))
        try:
            completed = subprocess.run(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                check=False,
                text=True,
            )
        except OSError as exc:
            message = "map_saver failed to start: %s" % exc
            self._status(message)
            return TriggerResponse(False, message)

        if completed.returncode != 0:
            message = "map_saver failed: %s" % completed.stdout.strip()
            self._status(message)
            return TriggerResponse(False, message)
        message = "saved %s.yaml and %s.pgm %s" % (prefix, prefix, self._map_stats(map_msg))
        self._status(message)
        return TriggerResponse(True, message)

    def _map_stats(self, msg):
        if msg is None or not msg.data:
            return ""
        total = float(len(msg.data))
        unknown = sum(1 for value in msg.data if value < 0)
        occupied = sum(1 for value in msg.data if value > 50)
        known_pct = ((total - float(unknown)) / total) * 100.0 if total else 0.0
        return "size=%dx%d res=%.3f known=%.1f%% occupied=%d" % (
            msg.info.width,
            msg.info.height,
            msg.info.resolution,
            known_pct,
            occupied,
        )

    def _status(self, message):
        rospy.loginfo("[mapping] %s", message)
        self.status_pub.publish(String(data=message))


if __name__ == "__main__":
    rospy.init_node("mapping_session")
    MappingSessionNode()
    rospy.spin()
