#! /usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
import moveit_commander
import geometry_msgs.msg
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String
import rosnode
import numpy as np
from tf.transformations import quaternion_from_euler


def depth_callback(ros_data):
    global rank
    rank+=1
    print(rank)
#np_arr = np.fromstring(ros_data.data,np.uint8,count=rank)
    np_arr = np.fromstring(ros_data.data,np.uint8)

    print(np_arr)
#    list=['0',255,0]
#    print "0" in np_arr
#    if list in np_arr:
#        print('called')


def main():
   
    rospy.init_node('pixel2depth')
    rospy.Subscriber('camera/depth/image_rect_raw',Image,depth_callback)

if __name__ == '__main__':
    rank=500
    main()
    rospy.spin()

