#!/usr/bin/env python

import rospy
import actionlib
from actionlib import SimpleActionClient, SimpleActionServer
from tf.listener import TransformerROS
from tf2_ros.transform_listener import TransformListener
from tiago_custom_msgs.msg import PickAction

# For arm control
from plan_arm_ik import PlanArmIk
import moveit_commander
from moveit_commander.conversions import pose_to_list
import geometry_msgs.msg
import std_srvs.srv
import moveit_msgs
import shape_msgs

# For aruco
from aruco_msgs.msg import Marker
from aruco_msgs.msg import MarkerArray

# For gripper control
from gripper_control import GripperControl
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse

# For head control
from look_to_point import LookToPoint
import copy
from std_srvs.srv import Empty

# To tuck the arm
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from sensor_msgs.msg import JointState

import tf2_ros
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math as m

from visualization_msgs.msg import Marker


class PlaceActionClass(object):
    def __init__(self):
        self._action_name = "place_server"
        self._as = actionlib.SimpleActionServer(self._action_name, PickAction, execute_cb=self.execute_cb, auto_start = False)
        self._aruco_sub = rospy.Subscriber("aruco_marker_publisher/markers", MarkerArray, self.aruco_cb)
        self._as.start()
        self._aruco_pose = geometry_msgs.msg.PoseStamped()
        self._aruco_id = 0
        self._use_aruco = False
        self._aruco_found = False


        self._vis_pub = rospy.Publisher("visualization_marker", Marker, queue_size=0)


        self._scene = moveit_commander.PlanningSceneInterface()
        arm_side = "right"
        group_name = "arm_" + arm_side + "_torso"
        self._group = moveit_commander.MoveGroupCommander(group_name)
        self._robot = moveit_commander.RobotCommander()

        self._clear_octomap_srv = rospy.ServiceProxy(
            '/clear_octomap', Empty)
        self._clear_octomap_srv.wait_for_service()

        # Transform gripper_tool_link to frame between fingers
        self._tfBuffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._tfBuffer)
        

    def publish_vis_marker(self, pose):
        # Method to display a marker in RViz. 
        # You can call this method with a goal pose to visualise where the arm is trying to move for debugging purposes.
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
        self._vis_pub.publish(marker)

    def aruco_cb(self, msg):
        # Callback to update pose of detected aruco marker on the object
        if self._use_aruco:
            for marker in msg.markers:
                if marker.id == self._aruco_id:
                    self._aruco_found = True
                    self._aruco_pose = marker.pose.pose


    def execute_cb(self, goal):
        r = rospy.Rate(1)

    # 1. Get pose to place object (either from aruco marker or from goal)
        #### YOUR CODE ####
	self._aruco_id = goal.aruco_id
        if goal.use_aruco:
            self._use_aruco = True
            rospy.loginfo('Placing object on location with aruco marker ID %s...', goal.aruco_id)
	
	else:
            # If we are not using the aruco marker for pose estimation then we simulate the aruco position with data from KB 
            rospy.loginfo('Placing object on location with aruco marker ID %s, using stored pose...', goal.aruco_id)
            self._aruco_pose = geometry_msgs.msg.PoseStamped()
            self._aruco_pose.pose = goal.pose
            #self._aruco_pose.pose.position.z -= self._object_height/2 # Remove object height to simulate position of aruco marker
            #self._aruco_pose.pose.position.z += self._object_height/2 # Add object height to simulate position of aruco marker
            #self._aruco_pose.pose.position.z += 0.13 # Add object height to simulate position of aruco marker
            

            # Rotate object pose to simulate marker pose
            [rotx, roty, rotz] = euler_from_quaternion([self._aruco_pose.pose.orientation.x, self._aruco_pose.pose.orientation.y, self._aruco_pose.pose.orientation.z, self._aruco_pose.pose.orientation.w])
            rotx = rotx + m.pi/2
            [self._aruco_pose.pose.orientation.x, self._aruco_pose.pose.orientation.y, self._aruco_pose.pose.orientation.z, self._aruco_pose.pose.orientation.w] = quaternion_from_euler(rotx, roty, rotz)

            try:
                source_frame = 'map'
                target_frame = 'base_link'
                transform = self._tfBuffer.lookup_transform(target_frame,
                                       source_frame, #source frame
                                       rospy.Time(0), #get the tf at first available time
                                       rospy.Duration(1.0)) #wait for 1 second
                self._aruco_pose = tf2_geometry_msgs.do_transform_pose(self._aruco_pose, transform)
                self._aruco_found = True
                rospy.loginfo("z pos of 'aruco marker': %s", self._aruco_pose.pose.position.z)
            except Exception as e:
                rospy.logerr(e)

	
    # 2. Make TIAGo look around to build an octomap for obstacle avoidance
        #### YOUR CODE ####
	self._clear_octomap_srv()

	head_control = LookToPoint()
        point = geometry_msgs.msg.Point()
        point.x = 1.0
        point.y = 0.0
        point.z = 1.1
        head_control.run(point)

        point.z = 0.0
        head_control.run(point)

        point.y = -0.7
        head_control.run(point)

        point.y = 0.7
        head_control.run(point)

        point.x = 1.0
        point.y = 0.0
        point.z = 0.6
        head_control.run(point)

	#Wait untill aruco marker is found (program can get stuck here!)
	while not self._aruco_found:
            rospy.loginfo('Aruco marker not found!')
            rospy.sleep(1)

	'''
        # Define a virtual table below the object
        table_pose = copy.deepcopy(self._aruco_pose)
        table_height = self._aruco_pose.pose.position.z #- self._object_width/2  
        table_width  = 1.5
        table_depth  = 0.55
        table_pose.pose.position.z += -(2*self._object_width)/2 -table_height/2
        table_pose.pose.orientation.x = 0.0
        table_pose.pose.orientation.y = 0.0
        table_pose.pose.orientation.z = 0.0
        table_pose.pose.orientation.w = 1.0
        table_height -= 0.008 # Remove few millimeters to prevent contact between the object and the table

        self._scene.add_box("table", table_pose, (table_depth, table_width, table_height))
	'''


    # 3. Move TIAGo's arm to the pose that you want to place the object (mind the height)
        #### YOUR CODE ####
	#arm_goal = copy.deepcopy(self._aruco_pose.pose)

        arm_goal = copy.deepcopy(self._aruco_pose.pose)
        
        ## Gripper grasp link in move_group group is not between the grippers, but in the wrist, about 15 cm back)
        [rotx, roty, rotz] = euler_from_quaternion([arm_goal.orientation.x, arm_goal.orientation.y, arm_goal.orientation.z, arm_goal.orientation.w])
 
        # Rotate grasping goal to be between straight forward and from the right side.
        rotz = rotz%(m.pi/2)
        [arm_goal.orientation.x, arm_goal.orientation.y, arm_goal.orientation.z, arm_goal.orientation.w] = quaternion_from_euler(rotx, roty, rotz)

        # Transform position from gripper wrist to gripper fingers (shift back 20 cm)
        arm_goal.position.x -= 0.20*m.cos(rotz)
        arm_goal.position.y -= 0.20*m.sin(rotz)
        #arm_goal.position.z -= 0.03
	#arm_goal.position.x += 0.02*m.cos(rotz)
        #arm_goal.position.y += 0.02*m.sin(rotz)
        arm_goal.position.z += 0.25 #0.25 worked
	
	

        # Publish marker for bedugging purposes
        self.publish_vis_marker(arm_goal)

        rospy.loginfo("moving to armpose: %s", arm_goal)

        self._group.set_pose_target(arm_goal)

        #self._clear_octomap_srv()

        plan = self._group.go(wait=True)

        if plan:
            self._group.stop()
            # It is always good to clear your targets after planning with poses.
            # Note: there is no equivalent function for clear_joint_value_targets()
            self._group.clear_pose_targets()
            rospy.loginfo('Goal reached')
        else:
            rospy.logfatal('Failed to find path..')
            self._as.set_aborted()
	
    # 4. Open the gripper
        #### YOUR CODE ####

	gripper_right = GripperControl('right')
        gripper_right.run('open')
        rospy.sleep(4)
        rospy.loginfo("opened gripper")

    # 5. Detach the 'collision object' in Moveit
        box_name = "aruco_cube_" + str(goal.aruco_id)

        attached_objects = self._scene.get_attached_objects([box_name])
        is_attached = len(attached_objects.keys()) > 0
        rospy.loginfo(attached_objects.keys())

        if is_attached:
            eef_link = self._group.get_end_effector_link()
            self._scene.remove_attached_object(eef_link, name=box_name)
            rospy.loginfo("collision object detached succesfully")


    # 6. Unlink arm an object so that they no longer move together
        detach_srv = rospy.ServiceProxy('link_attacher_node/detach', Attach)
        detach_srv.wait_for_service()
        rospy.loginfo("Detaching Product and gripper")
        req = AttachRequest()
        req.model_name_1 = "aruco_cube_" + str(goal.aruco_id)
        req.link_name_1 = "link"
        req.model_name_2 = "tiago_dual"
        req.link_name_2 = "arm_right_7_link"

        detached = detach_srv.call(req)
        rospy.loginfo("Object detached: %s", detached)

    # 7. Move the arm back to a neutral position
        #### YOUR CODE ####
	
	client = actionlib.SimpleActionClient("play_motion", PlayMotionAction)
        client.wait_for_server()
        rospy.loginfo("...connected.")

        rospy.wait_for_message("joint_states", JointState)
        rospy.sleep(3.0)

        rospy.loginfo("Tuck arm...")
        goal = PlayMotionGoal()
        goal.motion_name = 'home'
        goal.skip_planning = False

        client.send_goal(goal)
        client.wait_for_result(rospy.Duration(10.0))
        rospy.loginfo("Arm tucked.")



        self._scene.remove_world_object(box_name)
        self._as.set_succeeded()

if __name__ == "__main__":
    rospy.init_node("place_server")
    rospy.loginfo("place server started")

    PlaceActionClass()

    while not rospy.is_shutdown():
        rospy.sleep(1)
