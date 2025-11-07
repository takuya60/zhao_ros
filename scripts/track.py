#!/usr/bin/env python
import rospy
import moveit_commander
import numpy as np
from geometry_msgs.msg import PoseStamped, Twist # Twist 可以用于速度控制，这里我们用PoseStamped实现位移控制
from tf.transformations import euler_from_quaternion

class ArucoTrackingDemo:
    def __init__(self):
        rospy.init_node('aruco_tracking_demo')
        
        self.move_group = moveit_commander.MoveGroupCommander("arm")
        self.move_group.set_planning_time(0.01) # 减小规划时间以提高响应速度
        self.move_group.set_max_velocity_scaling_factor(0.1) # 降低速度，使控制更稳定
        self.move_group.set_max_acceleration_scaling_factor(0.1)
        
        # 跟踪参数
        self.P_gain = 0.5  # 比例增益，需要根据实际情况调整
        self.TOLERANCE = 0.03 # 3 cm 容忍度
        self.MAX_STEP = 0.01 # 每一步最大移动 1 cm

        # 1. 初始回到安全位
        rospy.loginfo("回到初始位姿...")
        self.move_group.set_pose_target("home")
        self.move_group.go(wait=True)
        
        # 2. 订阅 ArUco 位姿
        # 在视觉伺服中，通常保持订阅不取消
        self.aruco_sub = rospy.Subscriber('/aruco_single/pose', PoseStamped, self.track_aruco)
        rospy.loginfo("开始视觉跟踪，等待 ArUco 位姿消息...")


    def track_aruco(self, pose_msg):
        # 目标位姿 (即 ArUco 标记的位姿)
        target_pose = pose_msg.pose
        
        # 获取当前末端执行器位姿
        current_pose_stamped = self.move_group.get_current_pose()
        current_pose = current_pose_stamped.pose

        # ------------------- 1. 计算笛卡尔空间误差 -------------------
        
        # 计算位置误差 (目标 - 当前)
        error_x = target_pose.position.x - current_pose.position.x
        error_y = target_pose.position.y - current_pose.position.y
        error_z = target_pose.position.z - current_pose.position.z
        
        linear_error = np.sqrt(error_x**2 + error_y**2 + error_z**2)
        
        # 检查是否已对齐
        if linear_error < self.TOLERANCE:
            rospy.loginfo(f"对齐完成，误差小于 {self.TOLERANCE} m.")
            # 停止 MoveIt! 运动
            self.move_group.stop() 
            # 如果是单次对齐，可以在这里取消订阅 self.aruco_sub.unregister()
            return

        # ------------------- 2. P 控制器计算位移增量 -------------------

        # 使用 P 控制器计算下一时刻需要的位移增量 (delta_pose)
        delta_x = self.P_gain * error_x
        delta_y = self.P_gain * error_z # 注意！ArUco 标记的 Z 轴通常是垂直于平面指向机械臂的，可能需要调整映射
        delta_z = self.P_gain * error_y # ArUco 的 X/Y/Z 可能与机械臂的 X/Y/Z 存在交叉映射

        # 确保每一步的位移增量不超过最大限制，防止机械臂运动过快
        delta_x = np.clip(delta_x, -self.MAX_STEP, self.MAX_STEP)
        delta_y = np.clip(delta_y, -self.MAX_STEP, self.MAX_STEP)
        delta_z = np.clip(delta_z, -self.MAX_STEP, self.MAX_STEP)
        
        # ------------------- 3. 规划并执行微小位移 -------------------
        
        # 计算新的目标位姿：当前位姿 + 位移增量
        next_pose = PoseStamped()
        next_pose.header.frame_id = current_pose_stamped.header.frame_id
        
        # 仅移动位置，保持方向不变（通常对齐只需要位置调整）
        next_pose.pose.position.x = current_pose.position.x + delta_x
        next_pose.pose.position.y = current_pose.position.y + delta_y
        next_pose.pose.position.z = current_pose.position.z + delta_z
        next_pose.pose.orientation = current_pose.orientation # 保持当前末端的方向
        
        # MoveIt! 规划并执行
        self.move_group.set_pose_target(next_pose)
        
        # 使用 execute 而不是 go(wait=True) 可以更快的响应下一个回调
        plan = self.move_group.plan()
        if plan[0]:
            self.move_group.execute(plan[1], wait=False) # wait=False 允许在执行过程中接收新的反馈
            rospy.loginfo(f"执行微小移动: dx={delta_x:.4f}, dy={delta_y:.4f}, dz={delta_z:.4f}。当前误差: {linear_error:.4f} m.")
        else:
            rospy.logwarn("MoveIt! 规划失败，无法执行微小移动。")

    # def get_predefined_pose(self, name):
    #     # 预定义位姿（需要一个有效的 Pose 对象）
    #     if name == "home":
    #         pose = PoseStamped()
    #         pose.header.frame_id = "my_gen3/base_link" 
    #         pose.pose.position.x = 0.5
    #         pose.pose.position.y = 0.0
    #         pose.pose.position.z = 0.5
    #         pose.pose.orientation.w = 1.0 
    #         return pose.pose

if __name__ == '__main__':
    try:
        demo = ArucoTrackingDemo()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass