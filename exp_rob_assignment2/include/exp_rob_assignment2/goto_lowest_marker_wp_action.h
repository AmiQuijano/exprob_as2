#include <ros/ros.h>
#include "rosplan_action_interface/RPActionInterface.h"
#include "exp_rob_assignment2/GetLowestMarkerInfo.h"
#include <std_msgs/Float64.h>


namespace KCL_rosplan {
	
	class GoToLowestMarkerWaypointAction: public RPActionInterface
	{
		private:
            ros::ServiceClient get_lowest_marker_info_client;  // Service client for obtaining info of the lowest marker
            float x;  // X coordinate of the waypoint
            float y;  // Y coordinate of the waypoint

		public:
			/* constructor */
			GoToLowestMarkerWaypointAction(ros::NodeHandle &nh);
			
			/* listen to and process action_dispatch topic */
			bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
	};
}