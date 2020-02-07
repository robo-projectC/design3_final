#! /usr/bin/env python
# -*- coding: utf-8 -*-

#from design3_final.msg import unite
import rospy
import cv2
import sys
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import math
import rospy
import moveit_commander
import geometry_msgs.msg
from sensor_msgs.msg import Image,CameraInfo
from std_msgs.msg import String, Float32 
import rosnode
from tf.transformations import quaternion_from_euler
import message_filters
from speech_recognition_msgs.msg import SpeechRecognitionCandidates

img="none"
length=0.
voice=" "

#def color_callback(color):
#    global img
#    img=color.data
#    print("pick "+color.data)
#    print("copy "+img)
#
#def depth_callback(depth):
#    global length
#    length=depth.data
#    print("-----")
#    print(length)
#    print("-----")

class ColorExtract(object):
    def __init__(self):
        self._blue_pub = rospy.Publisher('blue_image', Image, queue_size=1)
        self._red_pub = rospy.Publisher('red_image', Image, queue_size=1)
        self._yellow_pub = rospy.Publisher('yellow_image', Image, queue_size=1)
        self._image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.callback)
        self._bridge = CvBridge()

    def get_colored_area(self, cv_image, lower, upper):
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask_image = cv2.inRange(hsv_image, lower, upper)
        extracted_image = cv2.bitwise_and(cv_image, cv_image, mask=mask_image)
        area = cv2.countNonZero(mask_image)
        return (area, extracted_image)
        
    def callback(self, data):
        try:
            cv_image = self._bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError, e:
            print e
        red_area, red_image = self.get_colored_area(
            cv_image, np.array([0,64,0]), np.array([14,255,255]))
        blue_area, blue_image = self.get_colored_area(
            cv_image, np.array([100,64,0]), np.array([130,255,255]))
        yellow_area, yellow_image = self.get_colored_area(
            cv_image, np.array([15,64,0]), np.array([34,255,255]))
#        try:
#            self._blue_pub.publish(self._bridge.cv2_to_imgmsg(blue_image, 'bgr8'))
#            self._red_pub.publish(self._bridge.cv2_to_imgmsg(red_image, 'bgr8'))
#        except CvBridgeError, e:
#            print e
        rospy.loginfo('red=%d, blue=%d, yellow=%d' % (red_area,blue_area,yellow_area))
        if red_area > 71000:
            img="red"
        elif blue_area > 51000:
            img="blue"
        elif yellow_area > 51000:
            img="yellow"
        else:
            img="unknown"



def check_msg(data):
    global voice
#aka akaido   
    print(data)
    color_check=data.transcript
#voice_list=[' oh', ' o', ' how', ' 00']
    if ' oh' in color_check:
        print("blue???")
        voice="blue"
    elif ' Keto' in color_check:
        print("yellow???")
        voice="yellow"
    elif ' Aikido' in color_check:
        print("red???")
        voice="red"
    elif ' commodity' in color_check:
        print("stop???")
        voice="stop"
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
    r = rospy.Rate(1.0)
#    img="unknown"

#arm_joint_values = arm.get_current_joint_values()
    


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
    
    arm_joint_values=[-0.20, -0.50, 0.0, -1.60, 0.0, -1.0, 0.0]
    arm.set_joint_value_target(arm_joint_values)
    arm.go()		
#    rospy.Subscriber('/color_view', String, color_callback)
#    rospy.Subscriber('/depth_length', Float32, depth_callback)
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

#    global img

    while joint_0<=1.6:
        move_gripper(0.9)
        joint_0+=0.05
        arm_joint_values=[joint_0, -0.50, 0.0, -1.60, 0.0, -1.0, 0.0]
#arm_joint_values=[joint_0, -0.50, 0.0, -1.60, 0.0, -1.0, 1.6]
        arm.set_joint_value_target(arm_joint_values)
        arm.go()		
        print("moving "+img)

        if img=="red" and voice=="red":
            print("==red==")
            catch_motion()
            break
        elif img=="blue" and voice=="blue":
            print("==blue==")
            catch_motion()
#            return_motion()
            break
        elif img=="yellow":
            print("==yellow==")
            catch_motion()
            break
        elif img=="unknown":
            print("==none==")
        elif joint_0>=1.6:
            joint_0=-2.0

    print("==return==")
#    global length
    while joint_0<2.90:
        move_gripper(0.01)
        joint_0+=0.05
        arm_joint_values=[joint_0, 0.0, 0.0, -1.50, 0.0, 0.0, 0.0]
        arm.set_joint_value_target(arm_joint_values)
        arm.go()		
#print(length)
        if voice=="stop":
             move_gripper(0.9)
             sys.exit()
             print("complete")
        elif joint_0>=2.90:
             while joint_2<=1.4:
                 joint_2+=0.05
                 arm_joint_values=[2.95, 0.0, joint_2, -1.50, 0.0, 0.0, 0.0]
                 arm.set_joint_value_target(arm_joint_values)
                 arm.go()		
# print(joint_2)
                 if length>=20 and length<=1000 or voice=="stop":
                     move_gripper(0.9)
                     sys.exit()
                     print("complete")
             else:
                 joint_0=1.6
                 joint_2=0.0
    else:
        joint_0=1.6
        joint_2=0.0


if __name__ == '__main__':

    try:
        if not rospy.is_shutdown():
            color = ColorExtract()
            main()
            rospy.spin()
    except rospy.ROSInterruptException:
        pass

