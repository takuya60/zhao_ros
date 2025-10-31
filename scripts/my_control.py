#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
import moveit_commander
from geometry_msgs.msg import Pose, Point, Quaternion
from moveit_commander.exception import MoveItCommanderException

class MoveItControlNode(object):
    """一个用于MoveIt!控制的Python节点"""

    def __init__(self):
        # 1. 初始化 moveit_commander 和 rospy 节点
        NS = "my_gen3"
        ROBOT_DESC_PARAM = "/" + NS + "/robot_description"
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('moveit_control_node', anonymous=True)

        rospy.loginfo("MoveIt! Python 节点已启动")

        # 2. 实例化 RobotCommander（提供机器人的运动学模型和状态）
        try:
            self.robot = moveit_commander.RobotCommander(
            robot_description=ROBOT_DESC_PARAM, ns=NS)
        except MoveItCommanderException as e:
            rospy.logerr("无法实例化 RobotCommander: %s", e)
            rospy.logerr("请确保您已加载机器人的 'robot_description' 参数！")
            sys.exit(1)

        # 3. 实例化 PlanningSceneInterface（用于与Rviz中的"世界"交互）
        self.scene = moveit_commander.PlanningSceneInterface(
            ns=NS)

        # 4. 实例化 MoveGroupCommander
        #    这是您要控制的规划组的名称 (例如 "arm" 或 "manipulator")
        #    这个名称必须与您在 MoveIt! Setup Assistant 中定义的一致
        self.group_name = "arm" # <--- !!! 修改为您自己的组名
        try:
            self.move_group = moveit_commander.MoveGroupCommander(
            self.group_name, robot_description=ROBOT_DESC_PARAM, ns=NS)
        except MoveItCommanderException as e:
            rospy.logerr("无法实例化 MoveGroupCommander: %s", e)
            rospy.logerr("错误: 规划组 '%s' 不存在。", self.group_name)
            rospy.logerr("可用的组: %s", self.robot.get_group_names())
            sys.exit(1)

        # 设置一些规划参数（可选）
        self.move_group.set_planner_id("RRTConnect") # 设置规划器
        self.move_group.set_planning_time(5.0)     # 规划时间限制 5s
        
        rospy.loginfo("MoveGroup 已准备就绪。")

    def go_to_named_target(self, target_name):
        """
        移动到在 SRDF 中定义的“命名”位姿
        (例如，在 MoveIt! Setup Assistant 中定义的 "home", "ready" 等)
        """
        rospy.loginfo("正在尝试移动到命名目标: %s", target_name)
        
        # 设置命名目标
        self.move_group.set_named_target(target_name)
        
        # 规划并执行
        # self.move_group.go(wait=True) 是一条合并了 plan() 和 execute() 的指令
        success = self.move_group.go(wait=True)
        
        if success:
            rospy.loginfo("已到达目标: %s", target_name)
        else:
            rospy.logwarn("未能到达目标: %s", target_name)
            
        # 停止任何剩余的运动并清除目标
        self.move_group.stop()

    def go_to_pose_goal(self, x, y, z, roll, pitch, yaw):
        """
        移动到世界坐标系下的特定姿态 (x, y, z, roll, pitch, yaw)
        """
        rospy.loginfo("正在尝试移动到姿态: \n"
                      "  位置 (x, y, z): [%.3f, %.3f, %.3f]\n"
                      "  朝向 (R, P, Y): [%.3f, %.3f, %.3f]",
                      x, y, z, roll, pitch, yaw)

        # 1. 创建一个 Pose 消息
        pose_goal = Pose()
        
        # 2. 设置位置 (Position)
        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z
        
        # 3. 设置朝向 (Orientation)
        # 我们需要将 欧拉角 (Roll, Pitch, Yaw) 转换为 四元数 (Quaternion)
        from tf.transformations import quaternion_from_euler
        q = quaternion_from_euler(roll, pitch, yaw) # RPY
        pose_goal.orientation = Quaternion(*q)

        # 4. 设置目标
        self.move_group.set_pose_target(pose_goal)

        # 5. 规划与执行
        rospy.loginfo("...正在规划路径...")
        # plan = self.move_group.plan() # 仅规划
        # if not plan.joint_trajectory.points:
        #     rospy.logwarn("规划失败！未找到路径。")
        #     return False
        
        success = self.move_group.go(wait=True) # 规划并执行

        if success:
            rospy.loginfo("已到达目标姿态。")
        else:
            rospy.logwarn("未能到达目标姿态。")

        # 6. 清理
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        
        return success

    def shutdown(self):
        """在节点关闭时调用，关闭 moveit_commander"""
        rospy.loginfo("正在关闭 MoveIt! Python 节点...")
        moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        # 实例化控制节点
        controller = MoveItControlNode()

        # 添加一个关闭钩子，确保在退出时正确关闭
        rospy.on_shutdown(controller.shutdown)
        
        # --- 示例：执行一个动作 ---
        
        # 1. (推荐) 移动到一个已知的、安全的“命名”位置
        #    这个 "home" 位置是在 MoveIt! Setup Assistant 中定义的
        controller.go_to_named_target("home") # <--- !!! 修改为您自己的命名位置
        rospy.sleep(1) # 暂停1秒
        
        # 2. 移动到一个具体的世界坐标
        #    !!! 警告: 这些坐标 (x,y,z) 是相对于您URDF中的 "base_link" 的！
        #    !!! 确保这个坐标是您的机械臂可以安全到达的！
        #    !!! (R, P, Y) = (0, 0, 0) 意味着与 base_link 朝向一致
        
        # 示例坐标 (请根据您的机械臂修改!)
        X_POS = 0.3
        Y_POS = 0.1
        Z_POS = 0.5
        
        # 朝向：末端执行器朝向正下方 (绕X轴旋转180度)
        # PI = 3.14159265
        # R_RAD = PI     # Roll
        # P_RAD = 0.0    # Pitch
        # Y_RAD = 0.0    # Yaw
        
        # 朝向：末端执行器朝向正前方 (绕Y轴旋转90度)
        PI_2 = 1.57079632
        R_RAD = 0.0
        P_RAD = PI_2
        Y_RAD = 0.0

        rospy.loginfo("--- 准备移动到自定义姿态 ---")
        controller.go_to_pose_goal(X_POS, Y_POS, Z_POS, R_RAD, P_RAD, Y_RAD)
        
        rospy.sleep(1)
        
        # 3. 回到 "home" 位置
        controller.go_to_named_target("home")

        rospy.loginfo("--- 演示完成 ---")

    except rospy.ROSInterruptException:
        pass
    except MoveItCommanderException as e:
        rospy.logerr("MoveIt Commander 异常: %s", e)
    except Exception as e:
        rospy.logerr("发生未知错误: %s", e)
        import traceback
        traceback.print_exc()