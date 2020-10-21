#! /usr/bin/env python
# -*- coding: utf-8 -*-

#共有用
"""
流れ
準備
    コップAとそれに対応するボールA、コップBとそれに対応するボールBを用意する
    コップAとコップBは同y、ｚ座標に置く
    コップA中にはボールAを入れておき、ボールBはコップBの横においておく

        #ボールの確認

    1.	コップAの上に移動
    2.	アーム先端を下に向ける
    3.	アームを垂直に下げる
    4.	コップAの下部を握る
    5.	垂直に持ち上げる
    6.	目標座標へ到着後1秒ほど停止
    7.	コップAを下げる
    8.	アームを全開に
    9.	垂直に持ち上げる
    10.	コップBの上に移動
    11.	アーム先端を下に向ける
    12.	アームを垂直に下げる
    13.	コップBの上部を握る
    14.	垂直に持ち上げる
    15.	目標座標へ到着後1秒ほど停止
    16.	横へ移動させボールBの上で停止
    17.	垂直に降下させボールBにコップBをかぶせる
    18.	コップBを横移動させ元の位置へ戻す
    19.	アームを全開に
    20.	垂直に持ち上げる

        #パフォーマンス（消す前の）

    1.	ホームへ戻る
    2.	アームを全開に

        #ボールを瞬間移動

    1.	コップBの上に移動
    2.	アーム先端を下に向ける
    3.	アームを垂直に下げる
    4.	コップBの下部を握る
    5.	垂直に持ち上げる
    6.	目標座標へ到着後1秒ほど停止
    7.	コップBを下げる
    8.	アームを全開に
    9.	垂直に持ち上げる
    10.	コップAの上に移動
    11.	アーム先端を下に向ける
    12.	アームを垂直に下げる
    13.	コップAの上部を握る
    14.	垂直に持ち上げる
    15.	目標座標へ到着後1秒ほど停止
    16.	コップAを下げ、アームを全開に
    17.	垂直に持ち上げる


        #ボールが瞬間移動したことを見せる

    1.	コップAの上に移動
    2.	コップAの上部を握る
    3.	アーム先端を下に向ける
    4.	垂直に持ち上げる
    5.	コップAを手前に移動
    6.	コップAを下す
    7.	アームを全開に
    8.	垂直に持ち上げる
    9.	コップBの上に移動
    10.	アーム先端を下に向ける
    11.	コップBの下部を握る
    12.	垂直に持ち上げる
    13.	コップBを手前に移動
    14.	コップBを下す
    15.	アームを全開に
    16.	垂直に持ち上げる
    17.	ホームへ戻る
"""

import rospy
import moveit_commander
import geometry_msgs.msg
import rosnode
from tf.transformations import quaternion_from_euler


def main():
    rospy.init_node("crane_x7_pick_and_place_controller")
    robot = moveit_commander.RobotCommander()
    arm = moveit_commander.MoveGroupCommander("arm")

    def move_max_velocity(value=0.5):
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
        arm.go()  # 実行

    # SRDFに定義されている"home"の姿勢にする
    arm.set_named_target("home")
    arm.go()

    #ハンドを開く
    move_gripper(1.3)

    #掴む準備をする-----1
    move_arm(0.23, 0.1, 0.2)
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


if __name__ == '__main__':

    try:
        if not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass
