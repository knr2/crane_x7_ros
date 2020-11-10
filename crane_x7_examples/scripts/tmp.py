#! /usr/bin/env python
# -*- coding: utf-8 -*-

"""
引用元
紙コップ　→　https://github.com/GakuKuwano/crane_x7_ros/blob/master/crane_x7_gazebo/worlds/table2.world
アーム動作　→　https://github.com/GakuKuwano/crane_x7_ros/blob/master/crane_x7_examples/scripts/papercup_tower.py
"""

"""
流れ1
準備
    コップAとそれに対応するボールA、コップBとそれに対応するボールBを用意する
    コップAとコップBは同y、ｚ座標に置く
    コップA中にはボールAを入れておき、ボールBはコップBの横においておく
        #ボールの確認
    1.	コップAの手前に移動
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

    ###
    ###     b-group
    ###

    #紙Aコップのx座標　紙Aコップのy座標　紙Bコップx座標 紙Bコップy座標
    position_base = [[0.29, 0.13, 0.29, -0.11]]
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
    def position_manager(master_judge, paper_cup, x, y, tukami):
        global Acup_tukamu
        global Bcup_tukamu
        position_ret = [0.0, 0.0]
        if master_judge == True:
            if len(position_base) > 0:

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
                    position_base.append([x, y, position_base[len(
                        position_base)-1][2], position_base[len(position_base)-1][3]])
                    Acup_tukamu = True
                #Bをつかんでいる場合
                elif position_base[len(position_base)-1][2] == x and position_base[len(position_base)-1][3] == y:
                    position_base.append([position_base[len(
                        position_base)-1][0], position_base[len(position_base)-1][1], x, y])
                    Bcup_tukamu = True
            else:
                #Aをはなした時
                if Acup_tukamu == True:
                    position_base.append([x, y, position_base[len(
                        position_base)-1][2], position_base[len(position_base)-1][3]])
                    Acup_tukamu = False
                elif Bcup_tukamu == True:
                    position_base.append([position_base[len(
                        position_base)-1][0], position_base[len(position_base)-1][1], x, y])
                    Bcup_tukamu = False

            return position_ret

    ###
    ###     b-group-end
    ###

    ###
    ###     a-group
    ###


    stop_time = 2.0  # 停止する時間を指定

    weak_position = 0.13  # 上をつかむ際のz
    weak_power = 0.24  # 上をつかむ際の力
    
    strong_position = 0.11  # 下をつかむ際のｚ
    strong_power = 0.28  # 下をつかむ際の力


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
        q = quaternion_from_euler(-3.14/2.0, 3.14, -3.14/2.0)  # 上方から掴みに行く場合
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)  # 目標ポーズ設定
        arm.go()  # 実行

    # コップ上部をつかむ位置へ移動１
    def move_arm_upper(pos_x, pos_y):
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = pos_x
        target_pose.position.y = pos_y
        target_pose.position.z = 0.25
        q = quaternion_from_euler(0, 3.14, 0)  # 上方から掴みに行く場合
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)  # 目標ポーズ設定
        arm.go()  # 実行

    # コップ上部をつかむ位置へ移動２
    def move_arm_upper_catch(pos_x, pos_y):
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = pos_x
        target_pose.position.y = pos_y
        target_pose.position.z = weak_position
        q = quaternion_from_euler(0, 3.14, 0)  # 上方から掴みに行く場合
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)  # 目標ポーズ設定
        arm.go()  # 実行

    # コップ下部をつかむ位置へ移動１
    def move_arm_lower(pos_x, pos_y):
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = pos_x
        target_pose.position.y = pos_y
        target_pose.position.z = 0.25
        q = quaternion_from_euler(0, 3.14, 0)  # 上方から掴みに行く場合
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)  # 目標ポーズ設定
        arm.go()  # 実行

    # コップ下部をつかむ位置へ移動２
    def move_arm_lower_catch(pos_x, pos_y):
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = pos_x
        target_pose.position.y = pos_y
        target_pose.position.z = strong_position
        q = quaternion_from_euler(0, 3.14, 0)  # 上方から掴みに行く場合
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)  # 目標ポーズ設定
        arm.go()  # 実行

    # コップの上を持ち上へ移動
    def move_arm_upper_up(pos_x, pos_y):
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = pos_x
        target_pose.position.y = pos_y
        target_pose.position.z = 0.25
        q = quaternion_from_euler(0, 3.14, 0)  # 上方から掴みに行く場合
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)  # 目標ポーズ設定
        arm.go()  # 実行

    # コップの下を持ち上へ移動
    def move_arm_lower_up(pos_x, pos_y):
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = pos_x
        target_pose.position.y = pos_y
        target_pose.position.z = 0.25
        q = quaternion_from_euler(0, 3.14, 0)  # 上方から掴みに行く場合
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)  # 目標ポーズ設定
        arm.go()  # 実行

    # ボールをつかむ位置へ移動
    def move_arm_ball(pos_x, pos_y):
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = pos_x
        target_pose.position.y = pos_y
        target_pose.position.z = 0.1
        q = quaternion_from_euler(0, 3.14, 0)  # 上方から掴みに行く場合
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

    #コップA-----1

    #aaにコップAの(x,y)を代入
    aa = position_manager(True, True, 0, 0, False)
    #グリッパを開く
    move_gripper(1.3)
    #アームをコップをくっつける
    move_arm_upper_catch(aa[0], aa[1])
    print("グリッパ下にボールを入れた紙コップを設置してください。")
    rospy.sleep(stop_time)
    print("次の動作へ進みます。")
    #アームを持ち上げコップから離れたらホームへ戻る
    move_arm_lower_up(aa[0], aa[1])
    move_max_velocity()
    arm.set_named_target("home")
    arm.go()

    #コップB-----2

    #aaにコップBの(x,y)を代入
    aa = position_manager(True, False, 0, 0, False)
    #グリッパを開く
    move_gripper(1.3)
    #アームをコップの下部とくっつける
    move_arm_upper_catch(aa[0], aa[1])
    print("グリッパ下に空の紙コップを設置してください。")
    rospy.sleep(stop_time)
    print("次の動作へ進みます。")
    #アームを持ち上げコップから離れたらホームへ戻る
    move_arm_upper_up(aa[0], aa[1])
    move_max_velocity()
    arm.set_named_target("home")
    arm.go()

    #ボール-----3

    #aaにコップBの(x,y)を代入
    aa = position_manager(True, False, 0, 0, False)
    #グリッパを開く
    move_gripper(1.3)
    #アームをボールとくっつける
    move_arm_ball(aa[0], aa[1] + 0.1)
    print("グリッパ下にボールを設置してください。")
    rospy.sleep(stop_time)
    print("次の動作へ進みます。")
    #アームを持ち上げコップから離れたらホームへ戻る
    move_arm_ball(aa[0], aa[1] + 0.1)
    move_max_velocity()
    arm.set_named_target("home")
    arm.go()


if __name__ == '__main__':

    try:
        if not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass
