#include <ros/ros.h>
#include <vector>

#include "RPActionInterface.h"



#ifndef KCL_pick
#define KCL_pick

/**
 * This file defines an action interface for a robot with a pan/tilt head to move a point.
 */
namespace KCL_rosplan {

	class RPPickInterface: public RPActionInterface
	{

	private:
		ros::NodeHandle _nh;
		
	public:

		/* constructor */
		RPPickInterface(ros::NodeHandle &nh);

		/* listen to and process action_dispatch topic */
		bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);

		//void RPPickInterface::createPointHeadClient(PointHeadClientPtr& actionClient);
		
	};
}
#endif