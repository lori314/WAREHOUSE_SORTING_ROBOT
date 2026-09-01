#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, JointState
from geometry_msgs.msg import Twist

class VisionGrabPipeline:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/kinect2/qhd/image_color_rect", Image, self.image_cb)
        self.depth_sub = rospy.Subscriber("/kinect2/sd/image_depth_rect", Image, self.depth_cb)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.mani_pub = rospy.Publisher('/wpb_home/mani_ctrl', JointState, queue_size=10)

        self.cv_depth = None
        self.ball_visible = False
        self.ball_cx = 0
        self.ball_cy = 0
        self.ball_depth = 0.0

        self.state = 'SEARCH'

        # 红色在HSV空间中分布在0的左右两边，需要使用两段掩码并且稍微放宽阈值。
        self.lower_color1 = np.array([0, 80, 80])
        self.upper_color1 = np.array([10, 255, 255])
        self.lower_color2 = np.array([160, 80, 80])
        self.upper_color2 = np.array([180, 255, 255])

        self.joint_msg = JointState()
        self.joint_msg.name = ["lift", "gripper"]
        self.joint_msg.position = [0.0, 0.0]
        self.joint_msg.velocity = [0.1, 5.0]

    def move_mani(self, lift, gripper):
        self.joint_msg.position[0] = lift
        self.joint_msg.position[1] = gripper
        self.mani_pub.publish(self.joint_msg)

    def depth_cb(self, data):
        try:
            self.cv_depth = self.bridge.imgmsg_to_cv2(data, "32FC1")
        except Exception as e:
            pass

    def image_cb(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except:
            return

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, self.lower_color1, self.upper_color1)
        mask2 = cv2.inRange(hsv, self.lower_color2, self.upper_color2)
        mask = cv2.bitwise_or(mask1, mask2)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        largest_area = 0
        best_contour = None
        for c in contours:
            area = cv2.contourArea(c)
            # 添加最大面积限制防止离得太近时视野爆炸或噪点过大
            if area > largest_area and area < 400000:
                largest_area = area
                best_contour = c

        # 放宽最小视野，防止走近丢失
        if largest_area > 150 and best_contour is not None:
            M = cv2.moments(best_contour)
            if M["m00"] != 0:
                self.ball_cx = int(M["m10"] / M["m00"])
                self.ball_cy = int(M["m01"] / M["m00"])
                self.ball_visible = True

                if self.cv_depth is not None:
                    dx = int(self.ball_cx * 512 / 960)
                    dy = int(self.ball_cy * 424 / 540)
                    dx = max(0, min(511, dx))
                    dy = max(0, min(423, dy))
                    d = self.cv_depth[dy, dx]
                    if not np.isnan(d) and d > 0:
                        self.ball_depth = d

                cv2.drawContours(cv_image, [best_contour], -1, (0, 255, 0), 2)
                cv2.circle(cv_image, (self.ball_cx, self.ball_cy), 5, (0, 0, 255), -1)
        else:
            self.ball_visible = False
            # 即使丢失视野，我们也保留最后的深度记忆（不重置 self.ball_depth = 0.0）

        cv2.putText(cv_image, f"State: {self.state}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 2)
        cv2.imshow("Vision Grab Pipeline", cv_image)
        cv2.waitKey(3)

    def run(self):
        rate = rospy.Rate(10)
        target_stop_dist = 1.15 # 进一步将刹车距离放宽到大视野的 1.15 米！因为小车的摄像头是 fixed 固定不可低头的
        img_center_x = 960 / 2

        while not rospy.is_shutdown():
            cmd = Twist()

            if self.state == 'SEARCH':
                if self.ball_visible:
                    rospy.loginfo("发现目标！开始对准(ALIGN)。")
                    self.state = 'ALIGN'
                else:
                    cmd.angular.z = 0.4
                    self.cmd_pub.publish(cmd)

            elif self.state == 'ALIGN':
                if not self.ball_visible:
                    # 避免对准一半因为噪点丢失退出
                    if self.ball_depth > 0 and self.ball_depth < 1.3:
                         rospy.logwarn("对准时画面丢失，强制保持盲抓！")
                         self.cmd_pub.publish(Twist())
                         self.state = 'PREPARE_GRAB'
                    else:
                         self.state = 'SEARCH'
                else:
                    err_x = img_center_x - self.ball_cx
                    # 放宽中心死区，防止在这个阶段卡死震荡
                    if abs(err_x) < 30:
                        rospy.loginfo("对准完成！开始靠近(APPROACH)。")
                        self.state = 'APPROACH'
                    else:
                        cmd.angular.z = err_x * 0.003
                        self.cmd_pub.publish(cmd)

            elif self.state == 'APPROACH':
                if not self.ball_visible:
                    # 避免近距离摄像头盲抓时丢失视野退回Search
                    if self.ball_depth > 0 and self.ball_depth < 1.3:
                        rospy.logwarn("由于距离太近视野完全丢失，默认已到达目标区域，强制启动盲抓！")
                        self.cmd_pub.publish(Twist()) # 立即停车
                        self.state = 'PREPARE_GRAB'
                    else:
                         self.state = 'SEARCH'
                else:
                    err_x = img_center_x - self.ball_cx
                    cmd.angular.z = err_x * 0.003

                    if 0 < self.ball_depth <= target_stop_dist:
                        rospy.loginfo(f"距离合适({self.ball_depth:.2f}m)！准备抓取(PREPARE_GRAB)。")
                        self.cmd_pub.publish(Twist())
                        self.state = 'PREPARE_GRAB'
                    else:
                        cmd.linear.x = 0.15 # 加快冲刺速度，防止太近时反而停不下来
                        self.cmd_pub.publish(cmd)

            elif self.state == 'PREPARE_GRAB':
                rospy.loginfo(">> 下降机械臂至地面并张开爪子")
                self.move_mani(lift=0.0, gripper=0.16)
                rospy.sleep(4.0)
                self.state = 'FORWARD_GRAB'

            elif self.state == 'FORWARD_GRAB':
                rospy.loginfo(">> 缓慢前进使目标进入爪臂间(盲抓套圈)")
                cmd.linear.x = 0.15 # 由于我们停得更早(1.15m)，需要适当用快一点的速度冲刺进去
                # 刹车变早了以后，盲开距离变长了，补足时间：v=0.15, d=1.15-0.35=0.8，大约需要 5.0 秒甚至更久
                end_t = rospy.Time.now() + rospy.Duration(5.0)
                while rospy.Time.now() < end_t and not rospy.is_shutdown():
                    self.cmd_pub.publish(cmd)
                    rate.sleep()
                self.cmd_pub.publish(Twist())
                self.state = 'CLOSE_GRIPPER'

            elif self.state == 'CLOSE_GRIPPER':
                rospy.loginfo(">> 闭合机械爪")
                self.move_mani(lift=0.0, gripper=0.032)
                rospy.sleep(3.0)
                self.state = 'LIFTUP'

            elif self.state == 'LIFTUP':
                rospy.loginfo(">> 抬起机械手完成抓取")
                self.move_mani(lift=0.15, gripper=0.032)
                rospy.sleep(3.0)
                self.state = 'DONE'

            elif self.state == 'DONE':
                rospy.loginfo_throttle(5, "任务结束！")

            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('vision_grab_pipeline_node')
    p = VisionGrabPipeline()
    p.run()
