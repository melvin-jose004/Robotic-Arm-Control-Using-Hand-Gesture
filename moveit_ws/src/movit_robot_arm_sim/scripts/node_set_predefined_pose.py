#! /usr/bin/env python

# Include the necessary libraries 
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
from math import pi
import time

from std_msgs.msg import Int32


class MyRobot:

    # Default Constructor
    def __init__(self, Group_Name):

        # Initialize the moveit_commander and rospy node
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('node_set_redefined_pose', anonymous=True)

        # Instantiate objects
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        
        self._planning_group = Group_Name
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)

        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path',
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=1
        )

        self._exectute_trajectory_client = actionlib.SimpleActionClient(
            'execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction
        )
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()

        rospy.loginfo('\033[95m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo('\033[95m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo('\033[95m' + "Group Names: {}".format(self._group_names) + '\033[0m')
        rospy.loginfo('\033[95m' + " >>> MyRobot initialization is done." + '\033[0m')

    def set_pose(self, arg_pose_name):
        rospy.loginfo('\033[32m' + "Going to Pose: {}".format(arg_pose_name) + '\033[0m')

        self._group.set_named_target(arg_pose_name)
        plan = self._group.plan()

        goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
        goal.trajectory = plan

        self._exectute_trajectory_client.send_goal(goal)
        self._exectute_trajectory_client.wait_for_result()

        rospy.loginfo('\033[32m' + "Now at Pose: {}".format(arg_pose_name) + '\033[0m')

    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo('\033[95m' + "Object of class MyRobot Deleted." + '\033[0m')


def callback(msg):
    global gesture_id
    gesture_id = msg.data
    rospy.loginfo("Received Gesture ID: %d", gesture_id)


def main():
    global gesture_id
    gesture_id = -1   # No gesture received yet

    # Subscriber
    rospy.Subscriber("/gesture_cmd", Int32, callback)

    # Create robot objects
    arm = MyRobot("arm_group")
    hand = MyRobot("hand")

    while not rospy.is_shutdown():

        rospy.sleep(0.2)

        # Gesture 0 --> Pick object routine
        if gesture_id == 0:
            rospy.loginfo("Gesture 0 detected: Pick object")
            arm.set_pose("pick_object_pose")
            hand.set_pose("hand_open")
            gesture_id = -1

        # Gesture 1 --> Place object
        elif gesture_id == 1:
            rospy.loginfo("Gesture 1 detected: Place object")
            arm.set_pose("place_object_opposit_pose")
            hand.set_pose("hand_closed")
            gesture_id = -1

        # Gesture 2 --> Lift arm & open hand
        elif gesture_id == 2:
            rospy.loginfo("Gesture 2 detected: Lift and open hand")
            arm.set_pose("straight_up")
            hand.set_pose("hand_open")
            gesture_id = -1

    # Cleanup
    del arm
    del hand


if __name__ == '__main__':
    main()

