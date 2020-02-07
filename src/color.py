#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String 


class ColorExtract(object):
    def __init__(self):
        self._vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self._blue_pub = rospy.Publisher('blue_image', Image, queue_size=1)
        self._red_pub = rospy.Publisher('red_image', Image, queue_size=1)
#self._yellow_pub = rospy.Publisher('yellow_image', Image, queue_size=1)
        self._image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.red_color_callback)
        self._image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.blue_color_callback)
#self._depth_sub = rospy.Subscriber('')
#self._voice_sub = rospy.Subscriber('')
        self._bridge = CvBridge()
        self._vel = Twist()

    def get_colored_area(self, cv_image, lower, upper):
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask_image = cv2.inRange(hsv_image, lower, upper)
        extracted_image = cv2.bitwise_and(cv_image, cv_image, mask=mask_image)
        area = cv2.countNonZero(mask_image)
        return (area, extracted_image)
        
    def red_color_callback(self, data):
        try:
            cv_image = self._bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError, e:
            print e
        red_area, red_image = self.get_colored_area(
            cv_image, np.array([0,64,0]), np.array([14,255,255]))
#        yellow_area, yellow_image = self.get_colored_area(
#            cv_image, np.array([15,64,0]), np.array([34,255,255]))

        try:
            self._red_pub.publish(self._bridge.cv2_to_imgmsg(red_image, 'bgr8'))
#self._yellow_pub.publish(self._bridge.cv2_to_imgmsg(yellow_image, 'bgr8'))
        except CvBridgeError, e:
            print e
        rospy.loginfo('red=%d' % (red_area))
        if red_area > 21000:
            #move()
#            self._vel.linear.x = -0.5
#            self._vel_pub.publish(self._vel)
            color="red"
        else:
            color="unknown"

        pub = rospy.Publisher('/color', String, queue_size=10)
        pub.publish(color)


    def blue_color_callback(self, data):
        try:
            cv_image = self._bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError, e:
            print e
        blue_area, blue_image = self.get_colored_area(
            cv_image, np.array([100,64,0]), np.array([130,255,255]))
            
        try:
            self._blue_pub.publish(self._bridge.cv2_to_imgmsg(blue_image, 'bgr8'))
#self._yellow_pub.publish(self._bridge.cv2_to_imgmsg(yellow_image, 'bgr8'))
        except CvBridgeError, e:
            print e
        rospy.loginfo('blue=%d' % (blue_area))
        if blue_area > 21000:
            #move(){realsense}
#            self._vel.linear.x = 0.5
#            self._vel_pub.publish(self._vel)
            color="blue"
        else:
            color="unknown"

        pub = rospy.Publisher('/color', String, queue_size=10)
        pub.publish(color)


if __name__ == '__main__':
    rospy.init_node('color_extract')
    color_func = ColorExtract()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
