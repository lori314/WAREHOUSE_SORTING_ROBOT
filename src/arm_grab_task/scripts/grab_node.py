#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose

class ArmGrabber:
    def __init__(self):
        self.mani_ctrl_pub = rospy.Publisher('/wpb_home/mani_ctrl', JointState, queue_size=10)
        self.grab_action_pub = rospy.Publisher('/wpb_home/grab_action', Pose, queue_size=10)

        # 预先设置关节状态信息
        self.joint_msg = JointState()
        self.joint_msg.name = ["lift", "gripper"]
        self.joint_msg.position = [0.0, 0.0]  # 初始化
        self.joint_msg.velocity = [0.5, 5.0]  # 速度设置

    def move_arm(self, lift_height, gripper_val):
        self.joint_msg.position[0] = lift_height
        self.joint_msg.position[1] = gripper_val
        self.mani_ctrl_pub.publish(self.joint_msg)
        rospy.loginfo(f"Arm moved: lift= {lift_height:.2f}, gripper= {gripper_val:.2f}")

    def trigger_auto_grab(self, x, y, z):
        # WPB home 有自己预设的抓取Action，可以通过发布位姿来触发内置的抓取状态机
        # 前提是开启了相应的行为节点 (wpb_home_grab_action / wpb_home_behaviors 等)
        # 此处展示通过直接控制关节来实现简单的抓取逻辑
        pose_msg = Pose()
        pose_msg.position.x = x
        pose_msg.position.y = y
        pose_msg.position.z = z
        self.grab_action_pub.publish(pose_msg)
        rospy.loginfo(f"Taking auto grab action to x={x:.2f}, y={y:.2f}, z={z:.2f}")

if __name__ == '__main__':
    rospy.init_node('arm_grab_task_node', anonymous=True)
    grabber = ArmGrabber()

    rospy.sleep(1.0) # 等待发布者注册

    # 根据之前获取到的坐标进行抓取，这里为演示写死坐标或直接订阅。
    # 假设目标物品在此处：
    target_x = 0.8
    target_y = 0.0
    target_z = 0.5

    # --- 流程一：使用手臂直接控制方法 (更底层) ---
    rospy.loginfo("--- Basic Manipulator Control ---")
    # 1. 张开爪子并调整高度到目标位置附近 (-0.1 避开)
    grabber.move_arm(lift_height=target_z, gripper_val=0.16) # 张开，比如0.16
    rospy.sleep(3)

    # 2. (在此可以加入小车移动代码使爪子套住目标，这里只做举起放下演示)

    # 3. 闭合爪子抓取
    grabber.move_arm(lift_height=target_z, gripper_val=0.03) # 闭合抓取
    rospy.sleep(3)

    # 4. 提起物体
    grabber.move_arm(lift_height=target_z + 0.1, gripper_val=0.03)
    rospy.sleep(3)

    # --- 流程二：使用环境封装好的行为接口 ---
    # rospy.loginfo("--- Auto Grab Action Interface ---")
    # grabber.trigger_auto_grab(target_x, target_y, target_z)

    rospy.spin()
