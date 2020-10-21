#! /usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
import moveit_commander
import geometry_msgs.msg
import rosnode
from tf.transformations import quaternion_from_euler


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

    # アームを移動しする
    def move_arm_up(pos_x, pos_y, pos_z):
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
        arm.go()  # 実行

    # アームを移動しする
    def move_arm_down(pos_x, pos_y, pos_z):
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = pos_x
        target_pose.position.y = pos_y
        target_pose.position.z = pos_z
        q = quaternion_from_euler(-3.14/2.0 - 0.25, 0.0, -3.14/2.0)  # 上方から掴みに行く場合
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)  # 目標ポーズ設定
        arm.go()  # 実行


    # SRDFに定義されている"home"の姿勢にする
    arm.set_named_target("home")
    arm.go()
 
    #ハンドを開く
    move_gripper(1.3)
    
    #下をつかむ　x = x-1 kara x = x-0.5 y = y z=0.1
    #q[0] -= 0.2
    
    #掴む準備をする-----1
    #move_arm(0.23, 0.1, 0.2)
    
    move_arm_up(0.27, -0.11, -0.005)
    move_arm_down(0.27, -0.11, 0.095)

    """
    move_arm(0.15, 0.2, 0.15)

    #掴みに行く
    arm.set_max_velocity_scaling_factor(0.1)
    move_arm(0.23, 0.2, 0.055)
#HHH
    #ハンドを閉じる
    move_gripper(0.28)
    move_max_velocity()
    
    #持ち上げる
    move_arm(0.15, 0.2, 0.2)

    #homeに戻る
    arm.set_named_target("home")
    arm.go()
 
    #下ろす
    move_arm(0.455, 0.0, 0.068)

    #ハンドを開く
    move_gripper(1.3)

    #少しだけハンドを持ち上げる
    move_arm(0.455, 0.0, 0.15)

    #homeに戻る
    arm.set_named_target("home")
    arm.go()


    #掴む準備をする-----2
    move_arm(0.23, -0.1, 0.2)
    move_arm(0.15, -0.2, 0.15)

    #掴みに行く
    arm.set_max_velocity_scaling_factor(0.1)
    move_arm(0.23, -0.2, 0.055)

    #ハンドを閉じる
    move_gripper(0.29)
    move_max_velocity()

    #持ち上げる
    move_arm(0.2, -0.2, 0.2)

    #homeに戻る
    arm.set_named_target("home")
    arm.go()
 
    #下ろす
    move_arm(0.395, 0.0, 0.063)

    #ハンドを開く
    move_gripper(1.3)

    #少しだけハンドを持ち上げる
    move_arm(0.3, 0.0, 0.17)

    #homeに戻る
    arm.set_named_target("home")
    arm.go()



    #掴む準備をする-----3
    move_arm(0.18, 0.15, 0.2)
    move_arm(0.1, 0.3, 0.17)

    #掴みに行く
    arm.set_max_velocity_scaling_factor(0.1)
    move_arm(0.18, 0.3, 0.055)

    #ハンドを閉じる
    move_gripper(0.29)
    move_max_velocity()

    #持ち上げる
    move_arm(0.1, 0.3, 0.2)

    #homeに戻る
    arm.set_named_target("home")
    arm.go()
 
    #下ろす
    move_arm(0.335, 0.0, 0.062)

    #ハンドを開く
    move_gripper(1.3)

    #少しだけハンドを持ち上げる
    move_arm(0.335, 0.0, 0.15)

    #homeに戻る
    arm.set_named_target("home")
    arm.go()



    #掴む準備をする-----4
    move_arm(0.18, -0.15, 0.2)
    move_arm(0.1, -0.3, 0.17)

    #掴みに行く
    arm.set_max_velocity_scaling_factor(0.1)
    move_arm(0.18, -0.3, 0.055)

    #ハンドを閉じる
    move_gripper(0.29)
    move_max_velocity()

    #持ち上げる
    move_arm(0.1, -0.3, 0.2)

    #homeに戻る
    arm.set_named_target("home")
    arm.go()
 
    #下ろす
    move_arm(0.275, 0.0, 0.062)

    #ハンドを開く
    move_gripper(1.3)

    #少しだけハンドを持ち上げる
    move_arm(0.275, 0.0, 0.15)

    #homeに戻る
    arm.set_named_target("home")
    arm.go()



    #掴む準備をする-----5
    move_arm(0.18, 0.2, 0.2)
    move_arm(0.13, 0.4, 0.15)

    #掴みに行く
    arm.set_max_velocity_scaling_factor(0.1)
    move_arm(0.18, 0.4, 0.065)

    #ハンドを閉じる
    move_gripper(0.29)
    move_max_velocity()

    #持ち上げる
    move_arm(0.13, 0.4, 0.2)

    #homeに戻る
    arm.set_named_target("home")
    arm.go()
 
    #下ろす
    move_arm(0.425, 0.0, 0.13)

    #ハンドを開く
    move_gripper(1.3)

    #少しだけハンドを持ち上げる
    move_arm(0.425, 0.0, 0.15)

    #homeに戻る
    arm.set_named_target("home")
    arm.go()



    #掴む準備をする-----6
    move_arm(0.18, -0.2, 0.2)
    move_arm(0.13, -0.4, 0.15)

    #掴みに行く
    arm.set_max_velocity_scaling_factor(0.1)
    move_arm(0.18, -0.4, 0.065)

    #ハンドを閉じる
    move_gripper(0.29)
    move_max_velocity()

    #持ち上げる
    move_arm(0.13, -0.4, 0.2)

    #homeに戻る
    arm.set_named_target("home")
    arm.go()
 
    #下ろす
    move_arm(0.365, 0.0, 0.13)

    #ハンドを開く
    move_gripper(1.3)

    #少しだけハンドを持ち上げる
    move_arm(0.365, 0.0, 0.15)

    #homeに戻る
    arm.set_named_target("home")
    arm.go()



    #掴む準備をする-----7
    move_arm(0.28, 0.15, 0.2)
    move_arm(0.2, 0.3, 0.15)

    #掴みに行く
    arm.set_max_velocity_scaling_factor(0.1)
    move_arm(0.28, 0.3, 0.065)

    #ハンドを閉じる
    move_gripper(0.29)
    move_max_velocity()

    #持ち上げる
    move_arm(0.2, 0.3, 0.2)

    #homeに戻る
    arm.set_named_target("home")
    arm.go()
 
    #下ろす
    move_arm(0.305, 0.0, 0.12)

    #ハンドを開く
    move_gripper(1.3)

    #少しだけハンドを持ち上げる
    move_arm(0.305, 0.0, 0.15)

    #homeに戻る
    arm.set_named_target("home")
    arm.go()



    #掴む準備をする-----8
    move_arm(0.28, -0.15, 0.15)
    move_arm(0.2, -0.3, 0.15)

    #掴みに行く
    arm.set_max_velocity_scaling_factor(0.1)
    move_arm(0.28, -0.3, 0.065)

    #ハンドを閉じる
    move_gripper(0.29)
    move_max_velocity()

    #持ち上げる
    move_arm(0.2, -0.3, 0.2)

    #homeに戻る
    arm.set_named_target("home")
    arm.go()
 
    #下ろす
    move_arm(0.395, 0.0, 0.20)

    #ハンドを開く
    move_gripper(1.3)

    #少しだけハンドを持ち上げる
    move_arm(0.395, 0.0, 0.23)

    #homeに戻る
    arm.set_named_target("home")
    arm.go()



    #掴む準備をする-----9
    move_arm(0.28, 0.2, 0.2)
    move_arm(0.2, 0.4, 0.13)

    #掴みに行く
    arm.set_max_velocity_scaling_factor(0.1)
    move_arm(0.28, 0.4, 0.065)

    #ハンドを閉じる
    move_gripper(0.29)
    move_max_velocity()

    #持ち上げる
    move_arm(0.2, 0.4, 0.2)

    #homeに戻る
    arm.set_named_target("home")
    arm.go()
 
    #下ろす
    move_arm(0.335, 0.0, 0.18)

    #ハンドを開く
    move_gripper(1.3)

    #少しだけハンドを持ち上げる
    move_arm(0.335, 0.0, 0.23)

    #homeに戻る
    arm.set_named_target("home")
    arm.go()



    #掴む準備をする-----10
    move_arm(0.28, -0.2, 0.2)
    move_arm(0.2, -0.4, 0.15)

    #掴みに行く
    arm.set_max_velocity_scaling_factor(0.1)
    move_arm(0.28, -0.4, 0.065)

    #ハンドを閉じる
    move_gripper(0.29)
    move_max_velocity()

    #持ち上げる
    move_arm(0.2, -0.4, 0.2)

    #homeに戻る
    arm.set_named_target("home")
    arm.go()

    #頂点に振り上げる
    move_arm(0.1, 0.0, 0.5)

    #あおり
    move_arm(0.22, -0.2, 0.2)

    #頂点に振り上げる
    move_arm(0.1, 0.0, 0.5)

    #あおり
    move_arm(0.22, 0.2, 0.2)

    #homeに戻る
    arm.set_named_target("home")
    arm.go()

    #下ろす
    move_arm(0.365, 0.0, 0.25)

    #ハンドを開く
    move_gripper(1.3)

    #少しだけハンドを持ち上げる
    move_arm(0.365, 0.0, 0.3)

    #homeに戻る
    arm.set_named_target("home")
    arm.go()

    rospy.sleep(3.0)

    #破壊
    move_arm(0.22, -0.2, 0.1)
    move_arm(0.38, 0.15, 0.1)

    #homeに戻る
    arm.set_named_target("home")
    arm.go()


    print("done")
"""

if __name__ == '__main__':

    try:
        if not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass
