#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import math
import rospy
import moveit_commander
import geometry_msgs.msg
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
import copy # 用于深拷贝姿态(Pose)

# 导入 Actionlib (用于手爪)
import actionlib
import control_msgs.msg # GripperCommand Action

# 用于角度和弧度的转换
DEGREES_TO_RADIANS = math.pi / 180.0
RADIANS_TO_DEGREES = 180.0 / math.pi

class MoveItControlNode(object):
    """一个演示笛卡尔路径和手爪控制的Python节点"""

    def __init__(self):
        # 1. 初始化 moveit_commander 和 rospy 节点
        try:
            moveit_commander.roscpp_initialize(sys.argv)
        except RuntimeError as e:
            rospy.logerr("moveit_commander.roscpp_initialize 失败: %s", e)
            sys.exit(1)
            
        rospy.init_node('moveit_control_node_demo', anonymous=True)

        rospy.loginfo("MoveIt! Python 演示节点已启动")

        # -----------------------------------------------------------------
        # 2. (关键) 初始化 MoveGroup (机械臂)
        # 根据我们之前的调试，我们必须指定正确的命名空间(ns)和参数名
        # -----------------------------------------------------------------
        NS = "my_gen3"
        ROBOT_DESC_PARAM = "/" + NS + "/robot_description"
        
        try:
            self.robot = moveit_commander.RobotCommander(
                robot_description=ROBOT_DESC_PARAM, ns=NS)

            self.scene = moveit_commander.PlanningSceneInterface(ns=NS)
            
            self.group_name = "arm" # "arm" 是 Kortex 的规划组
            self.move_group = moveit_commander.MoveGroupCommander(
                self.group_name, robot_description=ROBOT_DESC_PARAM, ns=NS)
                
        except RuntimeError as e:
            rospy.logerr("初始化 MoveGroupCommander 失败: %s", e)
            sys.exit(1)

        # 设置一些规划参数
        self.move_group.set_planner_id("RRTConnect") 
        self.move_group.set_planning_time(10.0)
        
        # 获取机械臂的基本信息
        self.planning_frame = self.move_group.get_planning_frame()
        self.eef_link = self.move_group.get_end_effector_link()
        rospy.loginfo("Planning frame: %s", self.planning_frame)
        rospy.loginfo("End effector link: %s", self.eef_link)
        
        # -----------------------------------------------------------------
        # 3. (关键) 初始化 Action 客户端 (手爪)
        # -----------------------------------------------------------------
        
        # 将下面的 "gen3_lite_2f" 替换
        # 在 launch 文件中 <arg name="gripper"> 设置的正确型号！
        gripper_model = "robotiq_2f_85" 
        
        PREFIX = ""

        self.gripper_action_name = (
            "/" + NS + "/" + 
            PREFIX +  # <-- 使用修正后的空字符串
            gripper_model + "_gripper_controller/gripper_cmd"
        )
        
        rospy.loginfo("正在等待 Gripper Action Server: %s", self.gripper_action_name)
        
        self.gripper_client = actionlib.SimpleActionClient(
            self.gripper_action_name,
            control_msgs.msg.GripperCommandAction
        )
        
        self.gripper_ready = False
        if self.gripper_client.wait_for_server(timeout=rospy.Duration(5.0)):
            self.gripper_ready = True
            rospy.loginfo("Gripper Action Server 已连接。")
        else:
            rospy.logerr("Gripper Action Server 连接超时！")

        rospy.loginfo("MoveGroup 和 Gripper 已准备就绪。")


    def shutdown(self):
        """在节点关闭时调用"""
        rospy.loginfo("正在关闭 MoveIt! Python 节点...")
        moveit_commander.roscpp_shutdown()

    # -------------------------------------------------
    # 辅助功能 (用于演示)
    # -------------------------------------------------
    def go_to_named_target(self, target_name):
        """
        (辅助) 移动到在 SRDF 中定义的“命名”位姿 (例如 "vertical")
        """
        rospy.loginfo("--- (辅助) 正在移动到命名目标: %s ---", target_name)
        
        self.move_group.set_named_target(target_name)
        success = self.move_group.go(wait=True)
        self.move_group.stop()
        
        if not success:
            rospy.logwarn("未能到达目标: %s", target_name)
        return success

    def get_current_pose(self):
        """(辅助) 获取末端执行器的当前姿态"""
        try:
            return self.move_group.get_current_pose(self.eef_link)
        except Exception as e:
            rospy.logerr("获取当前姿态失败: %s", e)
            return None

    # -------------------------------------------------
    # 核心功能 1: 手爪控制
    # -------------------------------------------------
    def operate_gripper(self, position, max_effort=0.85):
        """
        命令手爪移动到特定位置
        :param position: 目标位置。
                       (Kortex 标准: 1.0 = 完全闭合, 0.0 = 完全张开)
        :param max_effort: 最大力气
        """
        if not self.gripper_ready:
            rospy.logerr("Gripper 客户端未准备就绪，跳过命令。")
            return False

        rospy.loginfo("--- 正在命令手爪移动到位置: %.2f ---", position)
        
        # 1. 创建一个 Action Goal
        goal = control_msgs.msg.GripperCommandGoal()
        
        # 2. 填充命令
        goal.command.position = position
        goal.command.max_effort = max_effort
        
        # 3. 发送 Goal 并等待结果
        try:
            self.gripper_client.send_goal_and_wait(goal, rospy.Duration(5.0))
            result = self.gripper_client.get_result()
            
            if result.reached_goal:
                rospy.loginfo("手爪已到达目标位置。")
                return True
            else:
                rospy.logwarn("手爪未能到达目标位置。")
                return False
        except Exception as e:
            rospy.logerr("手爪 Action 失败: %s", e)
            return False

    # -------------------------------------------------
    # 核心功能 2: 笛卡尔路径 (直线运动)
    # -------------------------------------------------
    def go_to_cartesian_path(self, waypoints_list, eef_step=0.01, jump_threshold=0.0):
        """
        规划并执行一个笛卡尔（直线）路径
        :param waypoints_list: 一个包含多个 geometry_msgs.msg.Pose 的“航点”列表
        :param eef_step: 插值步长(米)。MoveIt! 每隔 1cm 检查一次 IK 解。
        :param jump_threshold: 关节跳变阈值(弧度)。0.0 表示禁用。
        """
        if not waypoints_list:
            rospy.logwarn("笛卡尔路径列表为空，跳过。")
            return False

        rospy.loginfo("--- 正在规划笛卡尔路径 (共 %d 个航点) ---", len(waypoints_list))

        # 1. 计算路径
        try:
            (plan, fraction) = self.move_group.compute_cartesian_path(
                                   waypoints_list,   # 航点列表
                                   eef_step,         # 步长 (e.g., 0.01 米)
                                   True)   # 关节跳变阈值 (0.0 禁用)
        except moveit_commander.exception.MoveItCommanderException as e:
            rospy.logerr("笛卡尔路径规划失败: %s", e)
            return False

        rospy.loginfo("路径规划完成度: %.2f %%", fraction * 100.0)

        # 2. 检查规划是否 100% 成功
        if fraction < 0.99: # (使用 0.99 而不是 1.0 以允许微小的浮点误差)
            rospy.logwarn("未能规划完整的笛卡尔路径 (%.2f < 1.0)。机械臂将不会移动。", fraction)
            return False

        # 3. (如果成功) 执行规划
        rospy.loginfo("规划成功，正在执行直线路径...")
        success = self.move_group.execute(plan, wait=True)
        
        if success:
            rospy.loginfo("笛卡尔路径执行完毕。")
        else:
            rospy.logwarn("笛卡尔路径执行失败。")

        self.move_group.stop()
        return success

# -------------------------------------------------
# 主函数 (演示)
# -------------------------------------------------
if __name__ == '__main__':
    try:
        controller = MoveItControlNode()
        rospy.on_shutdown(controller.shutdown)
        
        # --- 演示开始 ---
        
        # 1. 归位 (安全第一)
        rospy.loginfo("===== 演示: 归位 (vertical) =====")
        controller.go_to_named_target("vertical")
        rospy.sleep(1)

        # 2. 张开手爪
        rospy.loginfo("===== 演示: 张开手爪 =====")
        controller.operate_gripper(0.0) # 1.0 = 完全张开
        rospy.sleep(2)

        # 3. 沿直线向下移动
        rospy.loginfo("===== 演示: 笛卡尔路径 (直线向下 15cm) =====")
        
        # 3.1 获取当前姿态作为起点
        start_pose = controller.get_current_pose()
        if start_pose:
            waypoints_down = []
            
            # 3.2 创建目标航点 (在当前 Z 轴位置下方 15cm)
            wpose = copy.deepcopy(start_pose.pose)
            wpose.position.z -= 0.15 # 向下 15cm
            waypoints_down.append(wpose)
            
            # 3.3 执行
            controller.go_to_cartesian_path(waypoints_down)
        else:
            rospy.logerr("无法获取当前姿态，跳过直线运动。")
        rospy.sleep(2)

        # 4. 闭合手爪
        rospy.loginfo("===== 演示: 闭合手爪 =====")
        controller.operate_gripper(0.5) # 0.85 = 完全闭合
        rospy.sleep(2)

        # 5. 沿直线向上移动 (回到起点)
        rospy.loginfo("===== 演示: 笛卡尔路径 (直线向上 15cm) =====")
        if start_pose: # (我们重用第一次获取的 'start_pose')
            waypoints_up = []
            
            # 目标是回到我们开始的地方
            # (注意: 'start_pose' 是 PoseStamped, 我们用 .pose)
            waypoints_up.append(copy.deepcopy(start_pose.pose))
            
            # 3.3 执行
            controller.go_to_cartesian_path(waypoints_up)
        rospy.sleep(2)
        
        # 6. 归位
        rospy.loginfo("===== 演示结束: 归位 (vertical) =====")
        controller.go_to_named_target("vertical")
        rospy.sleep(1)

        rospy.loginfo("--- 所有演示已完成 ---")

    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("发生未知错误: %s", e)
        import traceback
        traceback.print_exc()