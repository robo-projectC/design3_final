#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class Aruco:
    def __init__(self):
        rospy.init_node('aruco_ros')
        self.pub = rospy.Publisher('aruco_image', Image, queue_size=10)
        self.cv_bridge = CvBridge()
        rospy.Subscriber('/camera/color/image_raw', Image, self.callback)

    def callback(self, data):
        cv_image = self.cv_bridge.imgmsg_to_cv2(data, 'bgr8')
        msg_image = self.cv_bridge.cv2_to_imgmsg(cv_image)
        self.pub.publish(msg_image)

def main():
    Aruco()
    rospy.spin()
    
if __name__ == '__main__':
    main()
