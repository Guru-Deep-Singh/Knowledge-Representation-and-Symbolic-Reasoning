#include "RPPick.h"

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <tiago_custom_msgs/PickAction.h>


/* The implementation of RPPick.h */
namespace KCL_rosplan {

	/* constructor */
	RPPickInterface::RPPickInterface(ros::NodeHandle &nh) : _nh(nh) {
		// perform setup
	}

	/* action dispatch callback */
	bool RPPickInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

		// The action implementation goes here.
        typedef actionlib::SimpleActionClient<tiago_custom_msgs::PickAction> PickClient;
		PickClient ac("pick_server", true);

        //wait for the action server to come up
        while(!ac.waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for the pick action server to come up");
        }

		tiago_custom_msgs::PickGoal goal;

		// Get parameter to use or not use aruco markers
		bool use_aruco;
		_nh.getParam("/aruco/use_aruco", use_aruco);
		goal.use_aruco = use_aruco;

		std::map<std::string, double> aruco;
		_nh.getParam("/objects/" + msg->parameters[2].value + "/aruco", aruco);
		goal.aruco_id = aruco["id"];

		if(!use_aruco){
			// Use KB info on objects if not using aruco markers
			std::map<std::string, double> position, orientation;
			_nh.getParam("/objects/" + msg->parameters[2].value + "/position", position);
			_nh.getParam("/objects/"+ msg->parameters[2].value + "/orientation", orientation);
			
			goal.pose.position.x = position["x"];
			goal.pose.position.y = position["y"];
			goal.pose.position.z = position["z"];
			goal.pose.orientation.x = orientation["x"];
			goal.pose.orientation.y = orientation["y"];
			goal.pose.orientation.z = orientation["z"];
			goal.pose.orientation.w = orientation["w"];
		}

		ac.sendGoal(goal);
        ac.waitForResult();

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("Hooray, the object is picked up!");
        else
            ROS_INFO("TIAGo failed to pick up the object for some reason...");

		// complete the action
		ROS_INFO("KCL: (%s) Pick Action completing.", msg->name.c_str());
		return true;
	}

} // close namespace

	/*-------------*/
	/* Main method */
	/*-------------*/

	int main(int argc, char **argv) {

		ros::init(argc, argv, "rosplan_pick_action", ros::init_options::AnonymousName);
		ros::NodeHandle nh("~");

		// create PDDL action subscriber
		KCL_rosplan::RPPickInterface rpti(nh);

		rpti.runActionInterface();

		return 0;
	}