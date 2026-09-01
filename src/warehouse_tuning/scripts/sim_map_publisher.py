#!/usr/bin/env python3

import rospy
from nav_msgs.msg import OccupancyGrid, MapMetaData


class SimMapPublisher:
    def __init__(self):
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.topic = rospy.get_param("~map_topic", "/map")
        self.resolution = float(rospy.get_param("~resolution", 0.05))
        self.width = int(rospy.get_param("~width", 240))
        self.height = int(rospy.get_param("~height", 240))
        self.origin_x = float(rospy.get_param("~origin_x", -6.0))
        self.origin_y = float(rospy.get_param("~origin_y", -6.0))
        self.profile = str(rospy.get_param("~profile", "empty"))
        self.publish_rate = float(rospy.get_param("~publish_rate", 1.0))
        self.pub = rospy.Publisher(self.topic, OccupancyGrid, queue_size=1, latch=True)
        self.map_msg = self._make_map()
        rospy.loginfo(
            "[sim_map] publishing %s profile=%s size=%dx%d resolution=%.3f origin=(%.2f,%.2f) occupied=%d"
            % (
                self.topic,
                self.profile,
                self.width,
                self.height,
                self.resolution,
                self.origin_x,
                self.origin_y,
                sum(1 for cell in self.map_msg.data if cell > 50),
            )
        )

    def _make_map(self):
        msg = OccupancyGrid()
        msg.header.frame_id = self.frame_id
        msg.info = MapMetaData()
        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.orientation.w = 1.0
        msg.data = [0] * (self.width * self.height)
        self._draw_border(msg)
        if self.profile in ("complex", "complex_lab", "lab"):
            self._draw_complex_lab(msg)
        return msg

    def _draw_border(self, msg):
        for x in range(self.width):
            msg.data[x] = 100
            msg.data[(self.height - 1) * self.width + x] = 100
        for y in range(self.height):
            msg.data[y * self.width] = 100
            msg.data[y * self.width + self.width - 1] = 100

    def _draw_complex_lab(self, msg):
        # Keep the robot start and table approach areas open while adding enough
        # structure to exercise map saving, reloading, and localization.
        for rect in (
            (-5.6, -5.6, -5.35, 5.6),   # left outside wall thickness
            (5.35, -5.6, 5.6, 5.6),     # right outside wall thickness
            (-5.6, -5.6, 5.6, -5.35),   # lower outside wall thickness
            (-5.6, 5.35, 5.6, 5.6),     # upper outside wall thickness
            (-2.8, -5.0, -2.65, -1.0),  # vertical partition, lower segment
            (-2.8, 1.0, -2.65, 5.0),    # vertical partition, upper segment
            (-5.0, 2.75, -3.4, 2.9),    # horizontal partition, left segment
            (-1.8, 2.75, 4.8, 2.9),     # horizontal partition, right segment
            (2.7, -4.8, 3.05, -1.8),    # shelf row 1
            (3.65, -4.8, 4.0, -1.8),    # shelf row 2
            (4.6, -4.8, 4.95, -1.8),    # shelf row 3
            (-4.9, -4.8, -3.35, -4.25), # storage block
            (-4.9, -3.65, -3.35, -3.1), # storage block
            (-4.9, -2.5, -3.35, -1.95), # storage block
            (0.70, -0.60, 1.20, 0.60),  # source table A
            (-0.60, 1.40, 0.60, 1.90),  # drop table B
            (-0.60, -1.90, 0.60, -1.40),# drop table C
            (2.0, 1.1, 2.35, 1.45),     # obstacle/pillar
            (3.3, 0.25, 3.65, 0.60),    # obstacle/pillar
            (1.8, -1.25, 2.15, -0.90),  # obstacle/pillar
            (-1.45, -0.55, -1.10, -0.20), # obstacle/pillar
        ):
            self._fill_rect_world(msg, *rect, value=100)

    def _fill_rect_world(self, msg, x_min, y_min, x_max, y_max, value=100):
        gx0, gy0 = self._world_to_grid(min(x_min, x_max), min(y_min, y_max))
        gx1, gy1 = self._world_to_grid(max(x_min, x_max), max(y_min, y_max))
        gx0 = max(0, min(self.width - 1, gx0))
        gx1 = max(0, min(self.width - 1, gx1))
        gy0 = max(0, min(self.height - 1, gy0))
        gy1 = max(0, min(self.height - 1, gy1))
        for gy in range(gy0, gy1 + 1):
            row = gy * self.width
            for gx in range(gx0, gx1 + 1):
                msg.data[row + gx] = value

    def _world_to_grid(self, x, y):
        gx = int(round((float(x) - self.origin_x) / self.resolution))
        gy = int(round((float(y) - self.origin_y) / self.resolution))
        return gx, gy

    def spin(self):
        rate = rospy.Rate(max(0.1, self.publish_rate))
        while not rospy.is_shutdown():
            self.map_msg.header.stamp = rospy.Time.now()
            self.pub.publish(self.map_msg)
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("sim_map_publisher")
    try:
        SimMapPublisher().spin()
    except rospy.ROSInterruptException:
        pass
