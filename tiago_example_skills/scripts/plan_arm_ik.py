#! /usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from visualization_msgs.msg import Marker




class PlanArmIk(object):
    def __init__(self, side, vis_pub):
        moveit_commander.roscpp_initialize(sys.argv)


        robot = moveit_commander.RobotCommander()
        #scene = moveit_commander.PlanningSceneInterface()

        arm_side = side

        group_name = "arm_" + arm_side + "_torso"
        self.group = moveit_commander.MoveGroupCommander(group_name)

        # We can get the name of the reference frame for this robot:
        # Gripper pose will always be relative to this frame!
        planning_frame = self.group.get_planning_frame()
        print("============ Reference frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = self.group.get_end_effector_link()
        print("============ End effector: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print("============ Robot Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")

        self._vis_pub = vis_pub

    def publish_vis_marker(self, pose):
        # Method to display a marker in RViz
        marker = Marker()
        marker.header.frame_id = "base_footprint"
        marker.header.stamp = rospy.Time()
        marker.ns = "marker_vis"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose = pose
        marker.scale.x = 0.1
        marker.scale.y = 0.01
        marker.scale.z = 0.01
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        rospy.loginfo('publishing marker')
        self._vis_pub.publish(marker)


    def run(self, pose_goal):
        """
        examples:
            pose_goal = geometry_msgs.msg.Pose() # Relative to planning_frame!
            Example orientations: {x, y, z, w}

            Forward(=):   {0.0, 0.0, 0.0, 1.0}
            Forward(||):  {-0.707, 0.0, 0.0, 0.707} 
                    180 degrees rotated: {0.707, 0.0, 0.0, 0.707}

            Up (=):       {0.0, -0.707, 0.0, 0.707}
            Up (||):      {0.5, -0.5, 0.5, 0.5} or {0.5, 0.5, 0.5, -0.5}

            Down (=):     {0.0, 0.707, 0.0, 0.707}
            Down (||):    {-0.5, 0.5, 0.5, 0.5} or {0.5, 0.5, -0.5, 0.5}

            Left (=):     {0.0, 0.0, 0.707, 0.707}
            Left (||):    {0.5, 0.5, 0.5, 0.5}

            Right (=):    {0.707, -0.707, 0.0, 0.0}
            Right (||):   {0.5, -0.5, 0.5, -0.5}, {-0.5, 0.5, -0.5, 0.5}
                180 deg. rot: {0.5, -0.5, -0.5, 0.5}
        """
        if self._vis_pub:
            self.publish_vis_marker(pose_goal)
        self.group.set_pose_target(pose_goal)

        plan = self.group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        if plan:
            self.group.stop()
            # It is always good to clear your targets after planning with poses.
            # Note: there is no equivalent function for clear_joint_value_targets()
            self.group.clear_pose_targets()
            rospy.loginfo('Goal reached')
            return True
        else:
            rospy.logfatal('Failed to find path..')
            return False