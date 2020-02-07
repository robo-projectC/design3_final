#!/usr/bin/env python

from design3_final.msg import unite
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String 

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
            color="red"
        elif blue_area > 51000:
            color="blue"
        elif yellow_area > 51000:
            color="yellow"
        else:
            color="unknown"

#        msg = unite()
#        msg.header.stamp = rospy.Time.now()
#        msg.color_pic=color
##print(msg)
#
#
#        pub = rospy.Publisher('/color_view', unite, queue_size=10)
#        pub.publish(msg)
        r=rospy.Rate(100)
        pub = rospy.Publisher('/color_view', String, queue_size=10)
        pub.publish(color)
        r.sleep()
        
    
if __name__ == '__main__':
    rospy.init_node('color_extract')
    color = ColorExtract()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass

