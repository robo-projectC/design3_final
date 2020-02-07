#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
#from sensor_msgs.msg import Imu
from speech_recognition_msgs.msg import SpeechRecognitionCandidates
from geometry_msgs.msg import Vector3, Quaternion, Pose
import rosnode
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from std_msgs.msg import String

voice=" "
def check_msg(data):
    global voice

 
    # ハンドを開く/ 閉じる close 0.01/open 0.9
#aka akaido   
    print(data)
    color_check=data.transcript
    red_voice=' Aikido'
#voice_list=[' oh', ' o', ' how', ' 00']
    if ' how' in color_check:
        print("blue???")
        voice="blue"
    elif ' Keto' in color_check:
        print("yellow???")
        voice="yellow"
    elif red_voice in color_check:
        print("red???")
        voice="red"
    elif ' commodity' in color_check:
        print("stop???")
        voice="stop"
#tsukame
    elif ' comment' in color_check:
        print("catch???")
        voice="catch"
    else:
        pass

#sutete ' state it', ' status',
#nagete  'nugget'
# mottekite        ' Multistate'
#    for l in voice_list:
#        if data.transcript in l:
##if print(voice_list in data.transcript):
#            print("blue???")
#            voice="blue"
#            break
#        else:
#            break

    
#    pub = rospy.Publisher('/voice_pick', String, queue_size=10)
#    pub.publish(voice)
#color=red

#print(arm_initial_pose)
#print(data.linear_acceleration)


def main():
    rospy.init_node("voice_picker")
    rospy.Subscriber('/Tablet/voice',SpeechRecognitionCandidates,check_msg)
#    r = rospy.Rate(0.5)
#    r.sleep()
     
if __name__ == '__main__':
    try:
        if not rospy.is_shutdown():
            main()
            rospy.spin()
    except rospy.ROSInterruptException:
        pass
