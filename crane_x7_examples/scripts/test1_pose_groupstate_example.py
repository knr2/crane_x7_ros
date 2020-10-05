#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import geometry_msgs.msg
import rosnode
from tf.transformations import quaternion_from_euler

import sys
import rospy
import time
import actionlib
import math
from std_msgs.msg import Float64
from control_msgs.msg import (
    GripperCommandAction,
    GripperCommandGoal
)


class GripperClient(object):
    def __init__(self):
        self._client = actionlib.SimpleActionClient(
            "/crane_x7/gripper_controller/gripper_cmd", GripperCommandAction)
        self._goal = GripperCommandGoal()

        # Wait 10 Seconds for the gripper action server to start or exit
        self._client.wait_for_server(rospy.Duration(10.0))
        if not self._client.wait_for_server(rospy.Duration(10.0)):
            rospy.logerr("Exiting - Gripper Action Server Not Found.")
            rospy.signal_shutdown("Action Server not found.")
            sys.exit(1)
        self.clear()

    def command(self, position, effort):
        self._goal.command.position = position
        self._goal.command.max_effort = effort
        self._client.send_goal(self._goal, feedback_cb=self.feedback)

    def feedback(self, msg):
        print("feedback callback")
        print(msg)

    def stop(self):
        self._client.cancel_goal()

    def wait(self, timeout=0.1):
        self._client.wait_for_result(timeout=rospy.Duration(timeout))
        return self._client.get_result()

    def clear(self):
        self._goal = GripperCommandGoal()

def main():
    rospy.init_node("pose_groupstate_example")
    rospy.init_node("gipper_action_client")
    gc = GripperClient()
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

    # Open grippers(45degrees)
    print "Open Gripper."
    gripper = 0.0
    gc.command(math.radians(gripper), 0.10)
    result = gc.wait(2.0)
    print result
    time.sleep(1)
    print ""
    print result
    print ""

    """
    # SRDFに定義されている"home"の姿勢にする
    print("home")
    arm.set_named_target("home")
    arm.go()

    # SRDFに定義されている"vertical"の姿勢にする
    print("vertical")
    arm.set_named_target("vertical")
    arm.go()
    """

    """追加"""

    # 手動で姿勢を指定するには以下のように指定
    
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = 0.0
    target_pose.position.y = 0.0
    target_pose.position.z = 0.624
    q = quaternion_from_euler(0.0, 0.0, 0.0)
    target_pose.orientation.x = q[0]
    target_pose.orientation.y = q[1]
    target_pose.orientation.z = q[2]
    target_pose.orientation.w = q[3]
    arm.set_pose_target(target_pose)    # 目標ポーズ設定
    arm.go()    # 実行

    # Open grippers(45degrees)
    print "Open Gripper."
    gripper = 45.0
    gc.command(math.radians(gripper), 0.01)
    print result
    print ""

    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = 1.0
    target_pose.position.y = 1.0
    target_pose.position.z = 0.624
    q = quaternion_from_euler(0.0, 0.0, 0.0)
    target_pose.orientation.x = q[0]
    target_pose.orientation.y = q[1]
    target_pose.orientation.z = q[2]
    target_pose.orientation.w = q[3]
    arm.set_pose_target(target_pose)    # 目標ポーズ設定
    arm.go()    # 実行

    # Close grippers
    print "Close Gripper."
    gripper = 0.0  # 0.0
    gc.command(math.radians(gripper), 0.01)
    print result
    time.sleep(1)

    """ここまで"""

    # ハンドを少し閉じる
    gripper.set_joint_value_target([0.7, 0.7])
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
