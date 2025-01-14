#include <ros/ros.h>
#include "rosplan_action_interface/RPActionInterface.h"
#include <std_srvs/SetBool.h>
#include <std_msgs/Int32.h>
#include "exp_rob_assignment2/MarkerWaypointPair.h"

namespace KCL_rosplan {

    class RotateAndDetectAction : public RPActionInterface 
    {
        private:
            ros::ServiceClient toggle_detection_client;     // Service client for toggling marker detection
            ros::Subscriber marker_sub;                     // Subscriber to detected marker ID topic
            ros::Publisher marker_waypoint_pub;             // Publisher for marker-waypoint pairs
            bool marker_detected;                           // Flag to indicate if a marker has been detected
            int detected_marker_id;                         // Stores the detected marker ID

            /* Callback for detected marker ID topic */
            void markerCallback(const std_msgs::Int32::ConstPtr& msg);

        public:
            /* Constructor */
            RotateAndDetectAction(ros::NodeHandle &nh);

            /* Listen to and process action_dispatch topic */
            bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
    };

} 