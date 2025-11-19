#!/usr/bin/env python
import rospy
import moveit_commander
import sys
import tf2_ros
import tf2_geometry_msgs
import numpy as np
from geometry_msgs.msg import PoseStamped

class ArucoTrackingDemo:
    def __init__(self):
        rospy.init_node('aruco_tracking_demo')
        
        # 初始化 TF 
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # 初始化 MoveIt
        moveit_commander.roscpp_initialize(sys.argv)
        self.move_group = moveit_commander.MoveGroupCommander("arm")
        
        # 提高一点速度限制，否则伺服会跟不上
        self.move_group.set_max_velocity_scaling_factor(0.3) 
        self.move_group.set_max_acceleration_scaling_factor(0.3)
        
        # 参数设置
        self.tracking_distance = 0.20  # 离二维码保持 20cm
        self.tolerance = 0.03          # 3cm 容差
        self.scale_factor = 0.1        # P控制器增益 (系数越小越平滑)

        # 初始化回到初始位置
        rospy.loginfo("回到 Home 点...")
        self.move_group.set_named_target("home")
        self.move_group.go(wait=True)

        # 订阅 ArUco
        # 队列长度设为1，只关心最新的位置，不处理旧的
        self.aruco_sub = rospy.Subscriber('/aruco_single/pose', PoseStamped, self.track_callback, queue_size=1)
        
        rospy.loginfo("等待 ArUco 信号")

    def track_callback(self, msg):
        """
        1. 收到 ArUco 在相机坐标系下的位置
        2. 将其转化为 Base Link 坐标系下的位置
        3. 计算末端应该去哪里
        """
        try:
            # 把相对相机的坐标 转换成 相对基座的坐标
            transform = self.tf_buffer.lookup_transform("base_link", 
                                                      msg.header.frame_id, #  camera_color_optical_frame
                                                      rospy.Time(0), 
                                                      rospy.Duration(1.0))
            
            target_pose_in_base = tf2_geometry_msgs.do_transform_pose(msg, transform)

            # 获取当前状态
            current_pose = self.move_group.get_current_pose().pose

            # 计算误差
            dx = target_pose_in_base.pose.position.x - current_pose.position.x
            dy = target_pose_in_base.pose.position.y - current_pose.position.y
            dz = target_pose_in_base.pose.position.z - current_pose.position.z
            # 计算直线距离
            distance = np.sqrt(dx**2 + dy**2 + dz**2)
            
            # 
            if distance < self.tolerance:
                return

            # ---------------------------------------------------------
            # 步骤 4: 生成控制指令
            # ---------------------------------------------------------
            # 我们不直接去目标点（因为会撞上），我们每次只走一小步（P控制）
            # 这里的逻辑是：当前位置 + (误差向量 * 比例系数)
            
            # 重点：这里我们假设要飞到二维码面前，而不是完全贴合
            # 实际应用中，通常需要根据二维码的法向量来计算“悬停点”。
            # 为了代码简单，这里我们仅仅让机械臂向二维码“靠近”
            
            step_x = dx * self.scale_factor
            step_y = dy * self.scale_factor
            step_z = dz * self.scale_factor
            
            # 限制最大步长 (安全钳位)
            max_step = 0.02 # 每次最多动 2cm
            step_x = np.clip(step_x, -max_step, max_step)
            step_y = np.clip(step_y, -max_step, max_step)
            step_z = np.clip(step_z, -max_step, max_step)

            # 构建新的目标位姿
            next_pose = current_pose
            next_pose.position.x += step_x
            next_pose.position.y += step_y
            next_pose.position.z += step_z
            
            # 执行运动 (使用笛卡尔路径，比 set_pose_target 更快更稳)
            waypoints = [next_pose]
            #使用cartesian路径，沿直线运动，如果直接moveit.go会不按直线走
            (plan, fraction) = self.move_group.compute_cartesian_path(
                                waypoints,   # waypoints to follow
                                0.01,        # eef_step (1cm 插值)
                                0.0)         # jump_threshold
            
            if fraction > 0.9: # 如果规划成功
                # 异步执行，不阻塞回调
                self.move_group.execute(plan, wait=False)
            else:
                rospy.logwarn("规划路径失败")

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("TF 变换失败")

if __name__ == '__main__':
    try:
        ArucoTrackingDemo()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass