#include "RPPlace.h"

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <tiago_custom_msgs/PickAction.h>


/* The implementation of RPPlace.h */
namespace KCL_rosplan {

	/* constructor */
	RPPlaceInterface::RPPlaceInterface(ros::NodeHandle &nh) : _nh(nh) {
		// perform setup
	}

	/* action dispatch callback */
	bool RPPlaceInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

		// The action implementation goes here.
        typedef actionlib::SimpleActionClient<tiago_custom_msgs::PickAction> PlaceClient;
		PlaceClient ac("place_server", true);

        //wait for the action server to come up
        while(!ac.waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for the place action server to come up");
        }

		tiago_custom_msgs::PickGoal goal;

		// Get parameter to use or not use aruco markers
		bool use_aruco;
		_nh.getParam("/aruco/use_aruco", use_aruco);
		goal.use_aruco = use_aruco;

		std::map<std::string, double> aruco;
		_nh.getParam("/objects/" + msg->parameters[2].value + "/location_aruco/aruco", aruco);
		//_nh.getParam("/waypoints/wp_cabinet_2/aruco_marker_original_444_0/aruco", aruco);
		goal.aruco_id = aruco["id"];

		if(!use_aruco){
			// Use KB info on objects if not using aruco markers
			
			// CHANGE THE FOLLOWING THREE  LINES TO GET PLACE POSITION FROM KB
			std::map<std::string, double> position, orientation;
			_nh.getParam("/objects/" + msg->parameters[2].value + "/location_aruco/position", position);
			_nh.getParam("/objects/"+ msg->parameters[2].value + "/location_aruco/orientation", orientation);
			//_nh.getParam("/waypoints/wp_cabinet_2/aruco_marker_original_444_0/position", position);
			//_nh.getParam("/waypoints/wp_cabinet_2/aruco_marker_original_444_0/orientation", orientation);
			
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
            ROS_INFO("Hooray, the object is placed!");
        else
            ROS_INFO("TIAGo failed to place the object for some reason...");

		// complete the action
		ROS_INFO("KCL: (%s) Place Action completing.", msg->name.c_str());
		return true;
	}

} // close namespace

	/*-------------*/
	/* Main method */
	/*-------------*/

	int main(int argc, char **argv) {

		ros::init(argc, argv, "rosplan_place_action", ros::init_options::AnonymousName);
		ros::NodeHandle nh("~");

		// create PDDL action subscriber
		KCL_rosplan::RPPlaceInterface rpti(nh);

		rpti.runActionInterface();

		return 0;
	}
