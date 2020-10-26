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

    #紙Aコップのx座標　紙Aコップのy座標　紙Bコップx座標 紙Bコップy座標
    position_base =[[0.37, 0.13, 0.37, -0.11]]
    Acup_tukamu = False
    Bcup_tukamu = False

    """
    使い方
    ハンドをつかんだ時は
    position_manager(False,False,x,y,True)

    ハンドをはなした時は
    position_manager(False,False,x,y,False)

    Aカップをつかみたい時の手先の位置を知りたいときは
    aa = position_manager(True,True,0,0,False)
    x = aa[0]
    y = aa[1]

    BBカップをつかみたい時の手先の位置を知りたいときは
    aa = position_manager(True,False,0,0,False)
    x = aa[0]
    y = aa[1]
    """
    def position_manager(master_judge,paper_cup,x,y,tukami):
        global Acup_tukamu
        global Bcup_tukamu
        position_ret=[0.0,0.0]
        if master_judge == True:
            if len(position_base)>0:
            
            #ホンスワン
                if paper_cup == True:
                    position_ret[0] = position_base[len(position_base)-1][2]
                    position_ret[1] = position_base[len(position_base)-1][3]
                    #position_historyの末尾の紙Aコップのx座標y座標
                    return position_ret
                else:
                    position_ret[0] = position_base[len(position_base)-1][0]
                    position_ret[1] = position_base[len(position_base)-1][1]
                    #position_historyの末尾の紙Bコップのx座標y座標
                    return position_ret
            
            else:
                return position_ret
            
        else:
            #熊谷さん
            #position_base配列の末尾にx,yを追加
            if tukami == True:
                #Aをつかんでいる場合
                if position_base[len(position_base)-1][0] == x and position_base[len(position_base)-1][1] == y:
                    position_base.append([x,y,position_base[len(position_base)-1][2],position_base[len(position_base)-1][3]])
                    Acup_tukamu = True
                #Bをつかんでいる場合
                elif position_base[len(position_base)-1][2] == x and position_base[len(position_base)-1][3] == y:
                    position_base.append([position_base[len(position_base)-1][0],position_base[len(position_base)-1][1],x,y])
                    Bcup_tukamu = True
            else:
                #Aをはなした時
                if Acup_tukamu == True:
                    position_base.append([x,y,position_base[len(position_base)-1][2],position_base[len(position_base)-1][3]])
                    Acup_tukamu = False
                elif Bcup_tukamu == True:
                    position_base.append([position_base[len(position_base)-1][0],position_base[len(position_base)-1][1],x,y])
                    Bcup_tukamu = False
                    
            return position_ret

    # アーム初期ポーズを表示
    arm_initial_pose = arm.get_current_pose().pose
    print("Arm initial pose:")
    print(arm_initial_pose)

    # ハンドを開く/ 閉じる
    def move_gripper(pou):
        gripper.set_joint_value_target([pou, pou])
        gripper.go()

    # コップ上部をつかむ位置へ移動１
    def move_arm_upper(pos_x, pos_y):
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = pos_x - 0.13
        target_pose.position.y = pos_y
        target_pose.position.z = 0.075
        q = quaternion_from_euler(-3.14/2.0, 0.0, -3.14/2.0)  # 上方から掴みに行く場合
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)  # 目標ポーズ設定
        arm.go()  # 実行

    # コップ上部をつかむ位置へ移動２
    def move_arm_upper_catch(pos_x, pos_y):
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = pos_x - 0.06
        target_pose.position.y = pos_y
        target_pose.position.z = 0.075
        q = quaternion_from_euler(-3.14/2.0, 0.0, -3.14/2.0)  # 上方から掴みに行く場合
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)  # 目標ポーズ設定
        arm.go()  # 実行

    # コップ下部をつかむ位置へ移動１
    def move_arm_lower(pos_x, pos_y):
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = pos_x - 0.13
        target_pose.position.y = pos_y
        target_pose.position.z = -0.01
        q = quaternion_from_euler(-3.14/2.0 - 0.2, 0.0, -3.14/2.0)  # 上方から掴みに行く場合
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)  # 目標ポーズ設定
        arm.go()  # 実行

    # コップ下部をつかむ位置へ移動２
    def move_arm_lower_catch(pos_x, pos_y):
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = pos_x - 0.06
        target_pose.position.y = pos_y
        target_pose.position.z = -0.01
        q = quaternion_from_euler(-3.14/2.0 - 0.2, 0.0, -3.14/2.0)  # 上方から掴みに行く場合
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)  # 目標ポーズ設定
        arm.go()  # 実行

    # コップの上を持ち上へ移動
    def move_arm_upeer_up(pos_x, pos_y):
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = pos_x - 0.06
        target_pose.position.y = pos_y
        target_pose.position.z = 0.075 + 0.1
        q = quaternion_from_euler(-3.14/2.0, 0.0, -3.14/2.0)  # 上方から掴みに行く場合
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)  # 目標ポーズ設定
        arm.go()  # 実行

    # コップの下を持ち上へ移動
    def move_arm_lower_up(pos_x, pos_y):
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = pos_x - 0.06
        target_pose.position.y = pos_y
        target_pose.position.z = -0.01 + 0.1
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
    aa = position_manager(True,True, 0.37, -0.11, False)
    move_gripper(1.3)
    move_arm_upper(aa[0], aa[1])
    move_gripper(1.3)
    move_arm_upper_catch(aa[0], aa[1])
    move_gripper(0.28)
    move_arm_upeer_up(aa[0], aa[1])

    move_max_velocity()
    arm.set_named_target("home")
    arm.go()

    aa = position_manager(True,False, 0.37, -0.11, False)
    move_gripper(1.3)
    move_arm_lower(aa[0], aa[1])
    move_gripper(1.3)
    move_arm_lower_catch(aa[0], aa[1])
    move_gripper(0.28)
    move_arm_lower_up(aa[0], aa[1])

    move_max_velocity()
    arm.set_named_target("home")
    arm.go()
    """
    move_arm(0.15, 0.2, 0.15)
    #掴みに行く
    arm.set_max_velocity_scaling_factor(0.1)
    move_arm(0.23, 0.2, 0.055)

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