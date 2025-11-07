#!/usr/bin/env python
import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from kortex_driver.srv import PlayCartesianTrajectory

class PickPlaceDemo:
    def __init__(self):
        rospy.init_node('pick_place_demo')
        self.move_group = moveit_commander.MoveGroupCommander("arm")
        self.move_group.set_pose_target("home")  # 回家位
        self.move_group.go(wait=True)
        self.aruco_sub = None
        # 订阅 ArUco 位姿
        self.sub=rospy.Subscriber('/aruco_single/pose', PoseStamped, self.detect_and_pick)


    # 相当于
    def detect_and_pick(self, pose_msg):
        self.sub.unregister()
        self.aruco_sub = None
        # 1. 检测到物体，计算抓取位姿
        pick_pose = PoseStamped()
        pick_pose.header.frame_id = "my_gen3/base_link"
        pick_pose.pose = pose_msg.pose  # ArUco 位姿
        pick_pose.pose.position.z += 0.05  # 抬高 5cm 抓取
        
        # 2. 规划到抓取点
        self.move_group.set_pose_target(pick_pose)
        plan = self.move_group.plan()
        if plan[0]:  # 规划成功
            self.move_group.execute(plan[1], wait=True)
            
            # 3. 夹爪闭合（用服务调用）
            rospy.wait_for_service('/my_gen3/gripper/activate_gripper')
            gripper_srv = rospy.ServiceProxy('/my_gen3/gripper/activate_gripper', String)
            gripper_srv('close')  # 自定义服务
            
            # 4. 抬起并放置
            place_pose = pick_pose
            place_pose.pose.position.x += 0.2  # 移到放置点
            self.move_group.set_pose_target(place_pose)
            self.move_group.go(wait=True)
            gripper_srv('open')  # 释放
            
            rospy.loginfo("Pick & Place 完成！")
            # self.aruco_sub = rospy.Subscriber('/aruco_single/pose', PoseStamped, self.detect_and_pick)

    # def get_predefined_pose(self, name):
    #     # 预定义位姿（可从 SRDF 加载）
    #     if name == "home":
    #         pose = PoseStamped()
    #         pose.pose.position.x = 0.5
    #         pose.pose.position.y = 0.0
    #         pose.pose.position.z = 0.5
    #         return pose.pose

if __name__ == '__main__':
    demo = PickPlaceDemo()
    rospy.spin()