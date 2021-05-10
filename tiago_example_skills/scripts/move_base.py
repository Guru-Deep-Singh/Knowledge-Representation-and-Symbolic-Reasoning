#!/usr/bin/env python

import actionlib
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class MoveBase(object):
    def __init__(self):
        # Launch Client and wait for server
        self.client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        rospy.loginfo('Waiting for server...')
        self.client.wait_for_server()

    def run(self, goal_pose):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose = goal_pose

        rospy.loginfo('Sending move base goal...')
        self.client.send_goal(goal)
        self.client.wait_for_result()

        rospy.loginfo('Navigation finished')

        