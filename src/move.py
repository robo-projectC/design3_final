#! /usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
import moveit_commander
import geometry_msgs.msg
from sensor_msgs.msg import Image,CameraInfo
import rosnode
from tf.transformations import quaternion_from_euler

def depth_callback(depth):
    print(depth.data)
    print('called')


def main():
    rospy.init_node("crane_x7_pick_and_place_controller")
    robot = moveit_commander.RobotCommander()
    arm = moveit_commander.MoveGroupCommander("arm")
    def move_max_velocity(value = 0.5):
        arm.set_max_velocity_scaling_factor(value)
    gripper = moveit_commander.MoveGroupCommander("gripper")

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
    

    # ハンドを開く/ 閉じる
    def move_gripper(pou):
        gripper.set_joint_value_target([pou, pou])
        gripper.go()


    # アームを移動する
    def move_arm(pos_x, pos_y, pos_z):
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = pos_x
        target_pose.position.y = pos_y
        target_pose.position.z = pos_z
        q = quaternion_from_euler(-3.14/2.0, 0.0, -3.14/2.0)  # 上方から掴みに行く場合
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)  # 目標ポーズ設定
#arm.get_goal_position_tolerance()
        arm.go()  # 実行
        # アーム初期ポーズを表示
        arm_initial_pose = arm.get_current_pose().pose
        print("Arm initial pose:")
        print(arm_initial_pose)

#    def y_change_move_arm(pos_y):
#        arm_initial_pose = arm.get_current_pose().pose
#        
#        target_pose = geometry_msgs.msg.Pose()
#        target_pose.position.x=arm_initial_pose.position.x 
#        target_pose.position.y=pos_y
#        target_pose.position.z=arm_initial_pose.position.z
# 
#        q = quaternion_from_euler(-3.14/2.0, 0.0, -3.14/2.0)  # 上方から掴みに行く場合
#        target_pose.orientation.x = q[0]
#        target_pose.orientation.y = q[1]
#        target_pose.orientation.z = q[2]
#        target_pose.orientation.w = q[3]
#        arm.set_pose_target(target_pose)  # 目標ポーズ設定
##arm.get_goal_position_tolerance()
#        arm.go()  # 実行
#        # アーム初期ポーズを表示
#        arm_initial_pose = arm.get_current_pose().pose
#        print("Arm initial pose:")
#        print(arm_initial_pose)


    move_gripper(0.9)
    move_gripper(0.01)
    move_gripper(0.9)
    # SRDFに定義されている"home"の姿勢にする
    arm.set_named_target("home")
    arm.go()
    move_arm(0.2, 0.0, 0.30)
    move_arm(0.2, 0.0, 0.3)
    move_arm(0.2, 0.0, 0.09)
    move_arm(0.2, 0.0, 0.09)
    y_pos=-0.32
#+=0.2<
    while y_pos <= 0.30:
        y_pos += 0.02
        print(y_pos)
        move_arm(0.2, y_pos, 0.1)
        if y_pos>=0.0:
            move_arm(0.25, y_pos, 0.1)
            move_gripper(0.01)
            move_arm(0.2, 0.0, 0.1)
            move_gripper(0.9)
            break
#        else :
#            print('report')
#    else:
#        print("end")
   
#   while(y>0.2;y+=0.1){
#        y+=0.1
#        move_arm(0.2, y, 0.10)
#            if(called){
#                pick_up
#                180return
#                face
#            }
#    }

    #    move_arm(0.2, 0.0, 0.3)
#z_change_move_arm(0.)
#    pose.position.x=-arm_initial_pose.position.x    #重要なのはposition
#    pose.position.y=arm_initial_pose.position.y
#    pose.position.z=arm_initial_pose.position.z
 
#    print("done")


if __name__ == '__main__':

    try:
        if not rospy.is_shutdown():
            main()
            rospy.spin()
    except rospy.ROSInterruptException:
        pass

