#!/usr/bin/env python
# -*- coding: utf-8 -*-

# This import is for general library
import os
import threading

# This import is for ROS integration
import rospy
from sensor_msgs.msg import Image,CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2

class PersonDetector():
    def __init__(self):

        # cv_bridge handles
        self.cv_bridge = CvBridge()

#self.person_bbox = BoundingBox()

        # ROS PARAM
        self.m_pub_threshold = rospy.get_param('~pub_threshold', 0.40)

        # Subscribe
#        sub_camera_depth   =  rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, self.CamDepthImageCallback)
        sub_camera_depth   =  rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.CamDepthImageCallback)

        return


    def CamDepthImageCallback(self, depth_image_data):
        try:
            self.m_depth_image = self.cv_bridge.imgmsg_to_cv2(depth_image_data, 'passthrough')
            print(self.m_depth_image)
        except CvBridgeError, e:
            rospy.logerr(e)
        self.m_camdepth_height, self.m_camdepth_width = self.m_depth_image.shape[:2]
        return


if __name__ == '__main__':
    try:
        rospy.init_node('person_detector', anonymous=True)
#       idc = PersonDetector()
        PersonDetector()
        rospy.loginfo('idc Initialized')
#idc.start()
        rospy.spin()
#idc.finish()

    except rospy.ROSInterruptException:
        pass
