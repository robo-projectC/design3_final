#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import sys
import moveit_commander
import geometry_msgs.msg
import rosnode
from tf.transformations import quaternion_from_euler

rospy.init_node("pose_groupstate_example")
robot = moveit_commander.RobotCommander()
arm = moveit_commander.MoveGroupCommander("arm")
arm.set_max_velocity_scaling_factor(0.1)
gripper = moveit_commander.MoveGroupCommander("gripper")

arm.set_named_target("vertical")
arm.go()

while len([s for s in rosnode.get_node_names() if 'rviz' in s]) == 0:
    rospy.sleep(1.0)

angle_1 = float(0/180.0*math.pi)
   
#deg=90
deg=0
while True:
    deg += 5.0 
    print(deg)
    angle_0 = float(deg/180.0*math.pi)
    angle_3 = -1.5
    arm_joint_values = arm.get_current_joint_values()
    arm_joint_values[0] = angle_0
    arm_joint_values[1] = angle_1
    arm_joint_values[3] = angle_3

    arm.set_joint_value_target(arm_joint_values)
    arm.go()		
    #print(arm_joint_values[0])
    print(arm_joint_values[3])
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
#deg=-170
#while True:
#    deg += 5.0 
#    print(deg)
#    angle = float(deg/180.0*math.pi)
#    arm_joint_values = arm.get_current_joint_values()
#    arm_joint_values[joint] = angle
#    arm.set_joint_value_target(arm_joint_values)
#    arm.go()		
#    if deg>-90:
#        deg=90
#        while deg < 170:
#            deg += 5.0 
#            print(deg)
#            angle = float(deg/180.0*math.pi)
#            arm_joint_values = arm.get_current_joint_values()
#            arm_joint_values[joint] = angle
#            arm.set_joint_value_target(arm_joint_values)
#            arm.go()

#deg=90
#while deg < 170:
#    deg += 1.0 
#    print(deg)
#    angle = float(deg/180.0*math.pi)
#    arm_joint_values = arm.get_current_joint_values()
#    arm_joint_values[joint] = angle
#    arm.set_joint_value_target(arm_joint_values)
#    arm.go()		
#else:
#    deg=-170
##if deg >= -170:
#    while deg < -90:
#        deg += 1.0 
#        print(deg)
#        angle = float(deg/180.0*math.pi)
#        arm_joint_values = arm.get_current_joint_values()
#        arm_joint_values[joint] = angle
#        arm.set_joint_value_target(arm_joint_values)
#        arm.go()		



#else:
#    print('done')
 
#        if deg>=0.1:
#            move_arm(0.25, y_pos, 0.10)
#            move_gripper(0.01)
#            move_arm(0.2, y_pos, 0.10)
#            print(y_pos)
#            move_arm(0, 0.1, 1.0)
#            break
#        else :
#            print('report')
#    else:
#        print("end")

# 移動後の手先ポーズを表示
arm_goal_pose = arm.get_current_pose().pose
print("Arm goal pose:")
print(arm_goal_pose)
print("done")
