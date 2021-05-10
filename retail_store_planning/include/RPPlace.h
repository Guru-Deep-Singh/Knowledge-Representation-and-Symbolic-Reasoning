#include <ros/ros.h>
#include <vector>

#include "RPActionInterface.h"



#ifndef KCL_place
#define KCL_place

/**
 * This file defines an action interface for a robot to place an object.
 */
namespace KCL_rosplan {

	class RPPlaceInterface: public RPActionInterface
	{

	private:
		ros::NodeHandle _nh;
		
	public:

		/* constructor */
		RPPlaceInterface(ros::NodeHandle &nh);

		/* listen to and process action_dispatch topic */
		bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);

		
	};
}
#endif