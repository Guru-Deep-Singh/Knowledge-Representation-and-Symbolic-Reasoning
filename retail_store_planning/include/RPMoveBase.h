#include <ros/ros.h>
#include <vector>

#include "RPActionInterface.h"



#ifndef KCL_move_base
#define KCL_move_base

/**
 * This file defines an action interface for a robot with a pan/tilt head to move a point.
 */
namespace KCL_rosplan {

	class RPMoveBaseInterface: public RPActionInterface
	{

	private:
		ros::NodeHandle _nh;

	public:

		/* constructor */
		RPMoveBaseInterface(ros::NodeHandle &nh);

		/* listen to and process action_dispatch topic */
		bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);

		//void RPMoveBaseInterface::createPointHeadClient(PointHeadClientPtr& actionClient);

	};
}
#endif
