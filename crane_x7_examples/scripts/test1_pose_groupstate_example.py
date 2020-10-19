#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import geometry_msgs.msg
import rosnode
from tf.transformations import quaternion_from_euler


def main():
    rospy.init_node("pose_groupstate_example")
    robot = moveit_commander.RobotCommander()
    arm = moveit_commander.MoveGroupCommander("arm")
    arm.set_max_velocity_scaling_factor(0.1)
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

    # 何かを掴んでいた時のためにハンドを開く
    gripper.set_joint_value_target([0.9, 0.9])
    gripper.go()

    # SRDFに定義されている"home"の姿勢にする
    print("home")
    arm.set_named_target("home")
    arm.go()

#tuika
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
        arm.go()  # 実行

    #ハンドを開く
    move_gripper(1.3)
    
    
    #掴む準備をする-----1
    move_arm(0.23, 0.1, 0.2)
    move_arm(0.15, 0.2, 0.15)

    #掴みに行く
    arm.set_max_velocity_scaling_factor(0.1)
    move_arm(0.23, 0.2, 0.055)

    #ハンドを閉じる
    move_gripper(0.29)
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
    
#osimai

    # SRDFに定義されている"vertical"の姿勢にする
    print("vertical")
    arm.set_named_target("vertical")
    arm.go()

    # ハンドを少し閉じる
    gripper.set_joint_value_target([0.0, 0.0])
    gripper.go()

    # 手動で姿勢を指定するには以下のように指定
    """
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = 0.0
    target_pose.position.y = 0.0
    target_pose.position.z = 0.624
    q = quaternion_from_euler( 0.0, 0.0, 0.0 )
    target_pose.orientation.x = q[0]
    target_pose.orientation.y = q[1]
    target_pose.orientation.z = q[2]
    target_pose.orientation.w = q[3]
    arm.set_pose_target( target_pose )	# 目標ポーズ設定
    arm.go()							# 実行
    """

    # 移動後の手先ポーズを表示
    arm_goal_pose = arm.get_current_pose().pose
    print("Arm goal pose:")
    print(arm_goal_pose)
    print("done")


if __name__ == '__main__':
    try:
        if not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass
