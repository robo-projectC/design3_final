#! /usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import math
import rospy
import moveit_commander
import geometry_msgs.msg
from sensor_msgs.msg import Image,CameraInfo
from std_msgs.msg import String, Float32 
import rosnode
from tf.transformations import quaternion_from_euler
import message_filters


def main():
    
    rospy.init_node("crane_x7_pick_and_place_controller")
    robot = moveit_commander.RobotCommander()
    arm = moveit_commander.MoveGroupCommander("arm")
    def move_max_velocity(value = 0.5):
        arm.set_max_velocity_scaling_factor(value)
    gripper = moveit_commander.MoveGroupCommander("gripper")

    arm_joint_values = arm.get_current_joint_values()
    
    arm.set_named_target("vertical")
    arm.go()

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
    
    def color_view(color_msg):
        color=color_msg.data

    # ハンドを開く/ 閉じる
    def move_gripper(pou):
        gripper.set_joint_value_target([pou, pou])
        gripper.go()

    def chach_motion():
        move_gripper(0.9)
        
        arm_joint_values=[joint_0, -0.75, 0.0, -1.60, 0.0, -0.80, 0.0]
        arm.set_joint_value_target(arm_joint_values)
        arm.go()		
        move_gripper(0.01)



    def unite(depth,color):
        joint_0=arm_joint_values[0]
        joint_0=-2.0
        move_gripper(0.9)
        while joint_0<=1.6:
            joint_0+=0.1
            arm_joint_values=[joint_0, -0.50, 0.0, -1.60, 0.0, -1.0, 0.0]
            arm.set_joint_value_target(arm_joint_values)
            arm.go()		
            if color.data==blue:
                chach_motion()
                break
        length = depth.data
        img = color.data
        out = [length, img]
        print(out)
    # アームを移動する
       
    depth=message_filters.Subscriber('/depth_length', Float32)
    color=message_filters.Subscriber('/color_view', String)
#voice=

    fps = 100. #fpsが整数だと、1/fpsをpython2が評価すると0になってしまう(整数同士の除算は切り捨て除算)
    delay = 1/fps*0.5
    ts = message_filters.ApproximateTimeSynchronizer([depth,color], 10, delay)
    ts.registerCallback(unite)

#deg=90
    
#    deg=0
#    while True:
#        deg += 5.0 
#        print(deg)
#        angle_0 = float(deg/180.0*math.pi)
#        angle_3 = -1.5
#        arm_joint_values = arm.get_current_joint_values()
#        arm_joint_values[0] = angle_0
#        arm_joint_values[1] = angle_1
#        arm_joint_values[3] = angle_3
#
#        arm.set_joint_value_target(arm_joint_values)
#        arm.go()		
#        #print(arm_joint_values[0])
#        print(arm_joint_values[3])
#        #print(arm_joint_values[5])
###もう半周させて顔探査モードへ
#        #    if deg>=170:
#        #        deg=-175
#        #        while deg > -90:
#        #            deg += 5.0 
#        #            print(deg)
#        #arm_joint_values[2] = angle_2
#        #            angle = float(deg/180.0*math.pi)
#        #            arm_joint_values = arm.get_current_joint_values()
#        #            arm_joint_values[joint] = angle
#        #            arm.set_joint_value_target(arm_joint_values)
#     
#    angle_1 = float(0/180.0*math.pi)
       
#deg=90
#    deg=0
#    while True:
#        deg += 5.0 
#        print(deg)
#        angle_0 = float(deg/180.0*math.pi)
#        angle_3 = -1.5
#        arm_joint_values = arm.get_current_joint_values()
#        arm_joint_values[0] = angle_0
#        arm_joint_values[1] = angle_1
#        arm_joint_values[3] = angle_3
#
#        arm.set_joint_value_target(arm_joint_values)
#        arm.go()		
#        #print(arm_joint_values[0])
#        print(arm_joint_values[3])
        #print(arm_joint_values[5])
##もう半周させて顔探査モードへ
        #    if deg>=170:
        #        deg=-175
        #        while deg > -90:
        #            deg += 5.0 
        #            print(deg)
        #arm_joint_values[2] = angle_2
        #            angle = float(deg/180.0*math.pi)
        #            arm_joint_values = arm.get_current_joint_values()
        #            arm_joint_values[joint] = angle
        #            arm.set_joint_value_target(arm_joint_values)
        #            arm.go()
        #else:
        #    print('end')

#    print("done")
#    rospy.Subscriber('/camera/color/image_raw', Image, self.callback)       

if __name__ == '__main__':

    try:
        if not rospy.is_shutdown():
            main()
            rospy.spin()
            sys.exit()
    except rospy.ROSInterruptException:
        pass

