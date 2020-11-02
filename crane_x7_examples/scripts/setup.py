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

    ###
    ###     b-group
    ###

    #紙Aコップのx座標　紙Aコップのy座標　紙Bコップx座標 紙Bコップy座標
    position_base =[[0.29, 0.13, 0.29, -0.11]]
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

    ###
    ###     b-group-end
    ###

    ###
    ###     a-group
    ###

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

    # コップ上部をつかむ位置へ移動１
    def move_arm_upper(pos_x, pos_y):
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = pos_x - 0.13
        target_pose.position.y = pos_y
        target_pose.position.z = 0.075
        q = quaternion_from_euler(-3.14/2.0, 3.14, -3.14/2.0)  # 上方から掴みに行く場合
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)  # 目標ポーズ設定
        arm.go()  # 実行

    # コップ上部をつかむ位置へ移動２
    def move_arm_upper_catch(pos_x, pos_y):
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = pos_x - 0.005
        target_pose.position.y = pos_y
        target_pose.position.z = 0.075
        q = quaternion_from_euler(-3.14/2.0, 3.14, -3.14/2.0)  # 上方から掴みに行く場合
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
        q = quaternion_from_euler(-3.14/2.0 + 0.15, 3.14, -3.14/2.0)  # 上方から掴みに行く場合
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)  # 目標ポーズ設定
        arm.go()  # 実行

    # コップ下部をつかむ位置へ移動２
    def move_arm_lower_catch(pos_x, pos_y):
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = pos_x - 0.005
        target_pose.position.y = pos_y
        target_pose.position.z = -0.01
        q = quaternion_from_euler(-3.14/2.0 + 0.15, 3.14, -3.14/2.0)  # 上方から掴みに行く場合
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)  # 目標ポーズ設定
        arm.go()  # 実行

    # コップの上を持ち上へ移動
    def move_arm_upper_up(pos_x, pos_y):
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = pos_x - 0.005
        target_pose.position.y = pos_y
        target_pose.position.z = 0.075 + 0.1
        q = quaternion_from_euler(-3.14/2.0, 3.14, -3.14/2.0)  # 上方から掴みに行く場合
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)  # 目標ポーズ設定
        arm.go()  # 実行

    # コップの下を持ち上へ移動
    def move_arm_lower_up(pos_x, pos_y):
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = pos_x - 0.005
        target_pose.position.y = pos_y
        target_pose.position.z = -0.01 + 0.1
        q = quaternion_from_euler(-3.14/2.0 + 0.15, 3.14, -3.14/2.0)  # 上方から掴みに行く場合
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)  # 目標ポーズ設定
        arm.go()  # 実行

    ###
    ###     a-group-end
    ###

    # SRDFに定義されている"home"の姿勢にする
    arm.set_named_target("home")
    arm.go()
 
    #ハンドを開く
    move_gripper(1.3)

    #コップAの初期位置

    #aaにコップAの(x,y)を代入
    aa = position_manager(True, True, 0, 0, False)
    #グリッパを開く
    move_gripper(1.3)
    #アームをコップの下部と水平な部分へ移動
    move_arm_lower(aa[0], aa[1])
    #アームをコップの下部とくっつける
    move_arm_lower_catch(aa[0], aa[1])
    print("グリッパ内にボールを入れた紙コップを設置してください。")
    rospy.sleep(5.0)
    #アームを持ち上げコップから離れたらホームへ戻る
    move_arm_lower_up(aa[0], aa[1])
    move_max_velocity()
    arm.set_named_target("home")
    arm.go()


    #コップBの初期位置

    #aaにコップBの(x,y)を代入
    aa = position_manager(True, False, 0, 0, False)
    #グリッパを開く
    move_gripper(1.3)
    #アームをコップの下部と水平な部分へ移動
    move_arm_upper(aa[0], aa[1])
    #アームをコップの下部とくっつける
    move_arm_upper_catch(aa[0], aa[1])
    print("グリッパ内に空の紙コップを設置してください。")
    rospy.sleep(5.0)
    #アームを持ち上げコップから離れたらホームへ戻る
    move_arm_upper_up(aa[0], aa[1])
    move_max_velocity()
    arm.set_named_target("home")
    arm.go()

    #ボールの初期位置

    #aaにコップBの(x,y)を代入
    aa = position_manager(True, False, 0, 0, False)
    #グリッパを開く
    move_gripper(1.3)
    #アームをボールの位置に
    move_arm_upper_catch(aa[0], aa[1] + 0.1)
    print("グリッパ内にボールを設置してください。")
    rospy.sleep(5.0)
    #アームを持ち上げコップから離れたらホームへ戻る
    move_arm_upper_up(aa[0], aa[1] + 0.1)
    move_max_velocity()
    arm.set_named_target("home")
    arm.go()


if __name__ == '__main__':

    try:
        if not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass