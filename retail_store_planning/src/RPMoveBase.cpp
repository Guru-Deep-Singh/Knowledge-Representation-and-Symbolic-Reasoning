#include "RPMoveBase.h"

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>


/* The implementation of RPMoveBase.h */
namespace KCL_rosplan {

	/* constructor */
	RPMoveBaseInterface::RPMoveBaseInterface(ros::NodeHandle &nh) : _nh(nh) {
		// perform setup
	}

	/* action dispatch callback */
	bool RPMoveBaseInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

		// The action implementation goes here.
        typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
		MoveBaseClient ac("move_base", true);

        //wait for the action server to come up
        while(!ac.waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for the move_base action server to come up");
        }

        move_base_msgs::MoveBaseGoal goal;

		goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();

		std::map<std::string, double> position, orientation;
		_nh.getParam("/waypoints/" + msg->parameters[2].value + "/position", position);
		_nh.getParam("/waypoints/"+ msg->parameters[2].value + "/orientation", orientation);

		/* ------------------------------------------------------------------------------------------------------------
    The 'msg' contains valuable information abou what the planner's intentions are.
		Investigate its content and use it to modify the goal definition (lines 42-48)
		HINT: get the parameter from the parameter server according to the message content: "msg->parameters[2].value"
	  --------------------------------------------------------------------------------------------------------------*/

		// This is a hard-coded position, to be changed with your solution to Question 2.1 in assignment 5
		goal.target_pose.pose.position.x = position["x"];
		goal.target_pose.pose.position.y = position["y"];
		goal.target_pose.pose.position.z = position["z"];
		goal.target_pose.pose.orientation.x = orientation["x"];
		goal.target_pose.pose.orientation.y = orientation["y"];
		goal.target_pose.pose.orientation.z = orientation["z"];
		goal.target_pose.pose.orientation.w = orientation["w"];

		ac.sendGoal(goal);
        ac.waitForResult();

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("Hooray, the base moved 1 meter forward");
        else
            ROS_INFO("The base failed to move forward 1 meter for some reason");

		// complete the action
		ROS_INFO("KCL: (%s) MoveBase Action completing.", msg->name.c_str());
		return true;
	}

} // close namespace

	/*-------------*/
	/* Main method */
	/*-------------*/

	int main(int argc, char **argv) {

		ros::init(argc, argv, "rosplan_move_base_action", ros::init_options::AnonymousName);
		ros::NodeHandle nh("~");

		// create PDDL action subscriber
		KCL_rosplan::RPMoveBaseInterface rpti(nh);

		rpti.runActionInterface();

		return 0;
	}
