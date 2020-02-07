#! /usr/bin/env python
# -*- coding: utf-8 -*-

#from design3_final.msg import unite
import sys
import cv2
import math
import rospy
import numpy as np
import moveit_commander
import geometry_msgs.msg
from sensor_msgs.msg import Image,CameraInfo
from std_msgs.msg import String, Float32 
import rosnode
from tf.transformations import quaternion_from_euler
import message_filters
from speech_recognition_msgs.msg import SpeechRecognitionCandidates
from cv_bridge import CvBridge, CvBridgeError

voice=" "
img="none"
length=0.

def color_callback(color):
    global img
    img=color.data
    print("pick "+color.data)
    print("copy "+img)

#def depth_callback(depth):
#    global length
#    length=depth.data
#    print("-----")
#    print(length)
#    print("-----")

def check_msg(data):
    global voice
#aka akaido   
    print(data)
    color_check=data.transcript
    red_voice=' Aikido'
#voice_list=[' oh', ' o', ' how', ' 00']
    if ' oh' in color_check:
        print("blue???")
        voice="blue"
    elif ' Keto' in color_check:
        print("yellow???")
        voice="yellow"
    elif red_voice in color_check:
        print("red???")
        voice="red"
    else:
        pass

#sutete ' state it', ' status',
#nagete  'nugget'
# mottekite        ' Multistate'

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
        global img
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
#        rospy.loginfo('red=%d, blue=%d, yellow=%d' % (red_area,blue_area,yellow_area))
        if red_area > 71000:
            img="red"
        elif blue_area > 51000:
            img="blue"
        elif yellow_area > 51000:
            img="yellow"
        else:
            img="unknown"
        print(img+"~~~")




class depth_estimater:
    WIDTH = 50
    HEIGHT = 25
 
    def __init__(self):
 
#rospy.init_node('depth_estimater', anonymous=True)
        self.bridge = CvBridge()
        sub_rgb = message_filters.Subscriber("/camera/color/image_raw",Image)
        sub_depth = message_filters.Subscriber("/camera/depth/image_rect_raw",Image)
        self.mf = message_filters.ApproximateTimeSynchronizer([sub_rgb, sub_depth], 100, 100.0)
        self.mf.registerCallback(self.ImageCallback)
#        self.pub = rospy.Publisher('/depth_length', Float32, queue_size=10)
#self.pub = rospy.Publisher('/depth_length', unite, queue_size=10)
 
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
 
        global length
        length = sum / ((self.WIDTH * 2) * (self.HEIGHT * 2))
#print("%f [cm]" % ave)

#        msg = unite()
#        msg.header.stamp = rospy.Time.now()
#        msg.length_pic=ave
#
#        self.pub.publish(msg)
#        self.pub.publish(ave)
 
        cv2.normalize(depth_image, depth_image, 0, 1, cv2.NORM_MINMAX)
#        cv2.namedWindow("color_image")
#        cv2.namedWindow("depth_image")
#        cv2.imshow("color_image", color_image)
#        cv2.imshow("depth_image", depth_image)
        cv2.waitKey(10)
 

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


#    def return_motion():
#        arm.set_named_target("vertical")
#        arm.go()
#        arm_joint_values = arm.get_current_joint_values()
#        joint_0=arm_joint_values[0]   
#        joint_2=arm_joint_values[2]   
#        joint_0=1.6
#        joint_2=0.0
#        print("==return==")
#        while joint_0<2.90:
#            move_gripper(0.01)
#            joint_0+=0.05
#            arm_joint_values=[joint_0, 0.0, 0.0, -1.50, 0.0, 0.0, 0.0]
#            arm.set_joint_value_target(arm_joint_values)
#            arm.go()		
#            print(length)
#            if length>=20 and length<=500:
#                 move_gripper(0.9)
#                 sys.exit()
#            elif joint_0>=2.90:
#                 while joint_2<=1.4:
#                     joint_2+=0.05
#                     arm_joint_values=[2.95, 0.0, joint_2, -1.50, 0.0, 0.0, 0.0]
#                     arm.set_joint_value_target(arm_joint_values)
#                     arm.go()		
## print(joint_2)
#                     print(length)
#                     if length>=20 and length<=500:
#                         move_gripper(0.9)
#                         sys.exit()
#                 else:
#                     joint_0=1.6
#                     joint_2=0.0
#        else:
#            joint_0=1.6
#            joint_2=0.0

#    arm.set_named_target("vertical")
#    arm.go()
#    arm_joint_values = arm.get_current_joint_values()
#    joint_0=arm_joint_values[0]   



    move_gripper(0.01)
    move_gripper(0.9)
    joint_0=-2.0

    global img

    while joint_0<=1.6:
        move_gripper(0.9)
        joint_0+=0.05
        arm_joint_values=[joint_0, -0.50, 0.0, -1.60, 0.0, -1.0, 0.0]
#arm_joint_values=[joint_0, -0.50, 0.0, -1.60, 0.0, -1.0, 1.6]
        arm.set_joint_value_target(arm_joint_values)
        arm.go()		
        print("moving "+img)

        if img=="red":
            print("==red==")
        elif img=="blue":
            print("==blue==")
            catch_motion()
#            return_motion()
            break
        elif img=="yellow":
            print("==yellow==")
        elif img=="unknown":
            print("==none==")
        elif joint_0>=1.6:
            joint_0=-2.0

    print("==return==")
    global length
    while joint_0<2.90:
        move_gripper(0.01)
        joint_0+=0.05
        arm_joint_values=[joint_0, 0.0, 0.0, -1.50, 0.0, 0.0, 0.0]
        arm.set_joint_value_target(arm_joint_values)
        arm.go()		
        print(length)
        if length>=20 and length<=1000:
             move_gripper(0.9)
             sys.exit()
        elif joint_0>=2.90:
             while joint_2<=1.4:
                 joint_2+=0.05
                 arm_joint_values=[2.95, 0.0, joint_2, -1.50, 0.0, 0.0, 0.0]
                 arm.set_joint_value_target(arm_joint_values)
                 arm.go()		
# print(joint_2)
                 print(length)
                 if length>=20 and length<=1000:
                     move_gripper(0.9)
                     sys.exit()
             else:
                 joint_0=1.6
                 joint_2=0.0
    else:
        joint_0=1.6
        joint_2=0.0


#    while joint_0<=1.6:
#        arm_joint_values = arm.get_current_joint_values()
#        move_gripper(0.9)
#        joint_0+=0.05
#        arm_joint_values=[joint_0, -0.50, 0.0, -1.60, 0.0, -1.0, 0.0]
##arm_joint_values=[joint_0, -0.50, 0.0, -1.60, 0.0, -1.0, 1.6]
#        arm.set_joint_value_target(arm_joint_values)
#        arm.go()		
#        print("moving "+img)
#        print(joint_0)
#
#        if img=="red":
#            print("==red==")
#        elif img=="blue":
#            print("==blue==")
#            catch_motion()
#            return_motion()
#        elif img=="yellow":
#            print("==yellow==")
#        elif joint_0>=1.6:
#            joint_0=-2.0
#        elif img=="unknown":
#            print("==none==")
#        else:
#            joint_0=-2.0
#    else:
#        joint_0=-2.0

#    rospy.Subscriber('/color_view', String, color_callback)

#    def unite_callback(depth,color):
#
#        length = depth.length_pic
#        img = color.color_pic
#        out = [length, img]
#        print(out)
#  
#             
#        move_gripper(0.9)
#        joint_0+=0.1
#        arm_joint_values=[joint_0, -0.50, 0.0, -1.60, 0.0, -1.0, 0.0]
#        arm.set_joint_value_target(arm_joint_values)
#        arm.go()		
#
#        if img=="blue" and length<=3.0:
#            catch_motion()
#        elif joint_0>=1.6:
#            joint_0=-2.0
           

 
#    sub_1=message_filters.Subscriber('/depth_length', unite)
#    sub_2=message_filters.Subscriber('/color_view', unite)
#voice=

#    fps = 100.
#    delay = 1/fps*0.5
#    ts = message_filters.ApproximateTimeSynchronizer([sub_1,sub_2],100,10)
#    ts.registerCallback(unite_callback)


      
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
            color = ColorExtract()
            depth_estimater()
            main()
            rospy.spin()
            sys.exit()
    except rospy.ROSInterruptException:
        pass

