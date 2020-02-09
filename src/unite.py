#! /usr/bin/env python
# -*- coding: utf-8 -*-

#from design3_final.msg import unite
import sys
import math
import rospy
import moveit_commander
import geometry_msgs.msg
from sensor_msgs.msg import Image,CameraInfo
from std_msgs.msg import String, Float32 
import rosnode
from tf.transformations import quaternion_from_euler
#import message_filters
from speech_recognition_msgs.msg import SpeechRecognitionCandidates

img="none"
length=0.
voice=""

def color_callback(color):
    global img
    img=color.data
    print("pick "+color.data)
#    print("copy "+img)
    sys.exit()

def depth_callback(depth):
    global length
    length=depth.data
    print(length)


def check_msg(data):
    global voice
    print(data)
    color_check=data.transcript
#voice_list=[' oh', ' o', ' how', ' 00']
    if ' oh' in color_check:
        print("blue???")
        voice="blue"
        sys.exit()
    elif ' Keto' in color_check:
        print("yellow???")
        voice="yellow"
        sys.exit()
#aka=akaido
    elif ' Aikido' in color_check:
        print("red???")
        voice="red"
        sys.exit()
    elif ' commodity' in color_check:
        print("stop???")
        voice="stop"
        sys.exit()
    else:
        pass



def main():
    
    rospy.init_node("crane_x7_pick_and_place_controller")
    robot = moveit_commander.RobotCommander()
    arm = moveit_commander.MoveGroupCommander("arm")
    def move_max_velocity(value = 0.5):
        arm.set_max_velocity_scaling_factor(value)
    gripper = moveit_commander.MoveGroupCommander("gripper")
    arm_joint_values = arm.get_current_joint_values()
    joint_2=arm_joint_values[2]   
#    img="unknown"

#arm_joint_values = arm.get_current_joint_values()
    


    while len([s for s in rosnode.get_node_names() if 'rviz' in s]) == 0:
        rospy.sleep(1.0)
    rospy.sleep(1.0)

    print("Group names:")
    print(robot.get_group_names())

    print("Current state:")
    print(robot.get_current_state())

    arm.set_named_target("vertical")
    arm.go()

    # アーム初期ポーズを表示
    arm_initial_pose = arm.get_current_pose().pose
    print("Arm initial pose:")
    print(arm_initial_pose)
    
    arm_joint_values=[-0.20, -0.50, 0.0, -1.60, 0.0, -1.0, 0.0]
    arm.set_joint_value_target(arm_joint_values)
    arm.go()		
#    rospy.Subscriber('/depth_length', Float32, depth_callback)
#    rospy.Subscriber('/color_view', String, color_callback)
    rospy.Subscriber('/Tablet/voice',SpeechRecognitionCandidates,check_msg)

#def color_callback(color):
#    arm = moveit_commander.MoveGroupCommander("arm")
#    gripper = moveit_commander.MoveGroupCommander("gripper")
# 
#    arm_initial_pose = arm.get_current_pose().pose
    # ハンドを開く/ 閉じる
    def move_gripper(pou):
        gripper.set_joint_value_target([pou, pou])
        gripper.go()

    def catch_motion():
        move_gripper(0.9)
#arm_joint_values = arm.get_current_joint_values()
        arm_joint_values = arm.get_current_joint_values()
        joint_0=arm_joint_values[0]   

        arm_joint_values=[joint_0, -0.75, 0.0, -1.60, 0.0, -0.80, 0.0]
#arm_joint_values=[joint_0, -0.75, 0.0, -1.60, 0.0, -0.80, 1.6]
        arm.set_joint_value_target(arm_joint_values)
        arm.go()		
        move_gripper(0.01)
        print("catch!!!")

    move_gripper(0.01)
    move_gripper(0.9)
    joint_0=-2.0


#global img
    while joint_0<=1.6:
        move_gripper(0.9)
        joint_0+=0.05
        arm_joint_values=[joint_0, -0.50, 0.0, -1.60, 0.0, -1.0, 0.0]
#arm_joint_values=[joint_0, -0.50, 0.0, -1.60, 0.0, -1.0, 1.6]
        arm.set_joint_value_target(arm_joint_values)
        arm.go()		
        print("moving "+img)
#    rospy.Subscriber('/Tablet/voice',SpeechRecognitionCandidates,check_msg)
        rospy.Subscriber('/color_view', String, color_callback)

        if img=="red" and voice=="red":
            print("==red==")
            catch_motion()
            break
        elif img=="blue" and voice=="blue":
            print("==blue==")
            catch_motion()
            break
        elif img=="yellow" and voice=="yellow":
            print("==yellow==")
            catch_motion()
            break
        elif img=="unknown":
            print("==none==")
        elif joint_0>=1.5:
            joint_0=-2.0


    print("==return==")
#    global length
    while joint_0<2.90:
        rospy.Subscriber('/depth_length', Float32, depth_callback)
        move_gripper(0.01)
        joint_0+=0.1
        arm_joint_values=[joint_0, 0.0, 0.0, -1.50, 0.0, 0.0, 0.0]
        arm.set_joint_value_target(arm_joint_values)
        arm.go()		
#        print(length)
        if length>=3.0 and length<=500 or voice=="stop":
             arm_joint_values=[joint_0, 0.0, 0.0, -1.50, 0.0, 0.0, 0.0]
             arm.set_joint_value_target(arm_joint_values)
             arm.go()
             move_gripper(0.9)
             print("complete")
             sys.exit()
        elif joint_0>=2.80:
             while joint_2<=1.4:
                 joint_2+=0.1
                 arm_joint_values=[2.95, 0.0, joint_2, -1.50, 0.0, 0.0, 0.0]
                 arm.set_joint_value_target(arm_joint_values)
                 arm.go()		
#                 print(length)
                 if length>=3.0 and length<=500 or voice=="stop":
                     arm_joint_values=[2.95, 0.0, joint_2, -1.50, 0.0, 0.0, 0.0]
                     arm.set_joint_value_target(arm_joint_values)
                     arm.go()	
                     move_gripper(0.9)
                     print("complete")
                     sys.exit()
             else:
                 joint_0=1.6
                 joint_2=0.0
    else:
        joint_0=1.6
        joint_2=0.0


if __name__ == '__main__':

    try:
        if not rospy.is_shutdown():
            main()
            rospy.spin()
    except rospy.ROSInterruptException:
        pass

