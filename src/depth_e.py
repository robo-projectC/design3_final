#! /usr/bin/env python
# -*- coding: utf-8 -*-
 
import rospy
import moveit_commander
import geometry_msgs.msg
import rosnode
from tf.transformations import quaternion_from_euler
import cv2
import sys
import message_filters
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

def main():
    robot = moveit_commander.RobotCommander()
    arm = moveit_commander.MoveGroupCommander("arm")
    def move_max_velocity(value = 0.5):
        arm.set_max_velocity_scaling_factor(value)
    gripper = moveit_commander.MoveGroupCommander("gripper")

    while len([s for s in rosnode.get_node_names() if 'rviz' in s]) == 0:
        rospy.sleep(1.0)
    rospy.sleep(1.0)

    print("Group names:")
    print(robot.get_group_names())

    print("Current state:")
    print(robot.get_current_state())



    # アーム初期ポーズを表示
    arm_initial_pose = arm.get_current_pose().pose
    print("Arm initial pose:")
    print(arm_initial_pose)
    

    # ハンドを開く/ 閉じる
    def move_gripper(pou):
        gripper.set_joint_value_target([pou, pou])
        gripper.go()


    # アームを移動する
    def move_arm(pos_x, pos_y, pos_z):
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = pos_x
        target_pose.position.y = pos_y
        target_pose.position.z = pos_z
        q = quaternion_from_euler(-3.14/2.0, 0.0, -3.14/2.0)  # 上方から掴みに行く場合
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)  # 目標ポーズ設定
#arm.get_goal_position_tolerance()
        arm.go()  # 実行
        # アーム初期ポーズを表示
        arm_initial_pose = arm.get_current_pose().pose
        print("Arm initial pose:")
        print(arm_initial_pose)
    
    move_gripper(0.01)
    move_gripper(0.9)
    # SRDFに定義されている"home"の姿勢にする
    arm.set_named_target("home")
    arm.go()
    move_arm(-0.2, 0.0, 0.30)
    move_arm(0.2, 0.0, 0.3)
    move_arm(0.2, 0.0, 0.10)
    move_arm(0.2, 0.0, 0.10)
    y_pos=-0.32
    while y_pos <= 0.30:
        y_pos += 0.02
        print(y_pos)
        move_arm(0.2, y_pos, 0.10)


 
class depth_estimater:
    WIDTH = 50
    HEIGHT = 25
 
    def __init__(self):
 
#rospy.init_node('depth_estimater', anonymous=True)
        self.bridge = CvBridge()
        sub_rgb = message_filters.Subscriber("/camera/color/image_raw",Image)
        sub_depth = message_filters.Subscriber("/camera/depth/image_rect_raw",Image)
        self.mf = message_filters.ApproximateTimeSynchronizer([sub_rgb, sub_depth], 100, 10.0)
        self.mf.registerCallback(self.ImageCallback)
 
    def ImageCallback(self, rgb_data , depth_data):
        try:
            color_image = self.bridge.imgmsg_to_cv2(rgb_data, 'passthrough')
            depth_image = self.bridge.imgmsg_to_cv2(depth_data, 'passthrough')
        except CvBridgeError, e:
            rospy.logerr(e)
 
        color_image.flags.writeable = True
        color_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB) 
        h, w, c = color_image.shape
 
        x1 = (w / 2) - self.WIDTH
        x2 = (w / 2) + self.WIDTH
        y1 = (h / 2) - self.HEIGHT
        y2 = (h / 2) + self.HEIGHT
        sum = 0.0
 
        for i in range(y1, y2):
            for j in range(x1, x2):
                color_image.itemset((i, j, 0), 0)
                color_image.itemset((i, j, 1), 0)
                #color_image.itemset((100,100,2), 0)
 
                if depth_image.item(i,j) == depth_image.item(i,j):
                    sum += depth_image.item(i,j)
 
        ave = sum / ((self.WIDTH * 2) * (self.HEIGHT * 2))
        print("%f [cm]" % ave)
 
        cv2.normalize(depth_image, depth_image, 0, 1, cv2.NORM_MINMAX)
        cv2.namedWindow("color_image")
        cv2.namedWindow("depth_image")
        cv2.imshow("color_image", color_image)
        cv2.imshow("depth_image", depth_image)
        cv2.waitKey(10)
 
if __name__ == '__main__':
    try:
        rospy.init_node("crane_x7_pick_and_place_controller")
        depth_estimater()
        main()
        rospy.spin()
    except rospy.ROSInterruptException: pass
