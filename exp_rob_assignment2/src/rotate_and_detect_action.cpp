#include "exp_rob_assignment2/rotate_and_detect_action.h"
#include "exp_rob_assignment2/MarkerWaypointPair.h"
#include <std_srvs/SetBool.h>
#include <std_msgs/Int32.h>
#include <ros/ros.h>

namespace KCL_rosplan {

    RotateAndDetectAction::RotateAndDetectAction(ros::NodeHandle &nh) {
        // Initialize the service client
        toggle_detection_client = nh.serviceClient<std_srvs::SetBool>("/toggle_marker_detection");

        // Subscribe to the detected marker ID topic
        marker_sub = nh.subscribe("/detected_marker_id", 10, &RotateAndDetectAction::markerCallback, this);

        // Initialize the marker-waypoint pairs publisher
        marker_waypoint_pub = nh.advertise<exp_rob_assignment2::MarkerWaypointPair>("/aruco/marker_waypoint_pairs", 10);

        // Initialize detection state
        marker_detected = false;
    }

    void RotateAndDetectAction::markerCallback(const std_msgs::Int32::ConstPtr& msg) {
        // Set the marker_detected flag to true when a marker ID is received
        marker_detected = true;
        detected_marker_id = msg->data;
        ROS_INFO("Detected marker ID: %d", detected_marker_id);
    }

    bool RotateAndDetectAction::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
        ROS_INFO("RotateAndDetect action started");
        
        // Reset the marker detection flag for next rotate_and_detect actions
        marker_detected = false;
        
        // Step 1: Activate marker detection through Service request
        std_srvs::SetBool srv;
        srv.request.data = true;
        if (!toggle_detection_client.call(srv) || !srv.response.success) {
            ROS_ERROR("Failed to activate marker detection: %s", srv.response.message.c_str());
            return false;  // Return failure to the action dispatcher
        }
        ROS_INFO("Marker detection activated.");

        // Step 2: Wait for marker detection
        ros::Rate loop_rate(10);  // Check for marker detection 10 times per second
        ROS_INFO("Waiting for marker detection...");
        while (ros::ok() && !marker_detected) {
            ros::spinOnce();
            loop_rate.sleep();
        }

        // Step 3: Deactivate marker detection once a marker is found through Service request
        srv.request.data = false;
        if (!toggle_detection_client.call(srv) || !srv.response.success) {
            ROS_ERROR("Failed to deactivate marker detection: %s", srv.response.message.c_str());
            return false;  // Return failure to the action dispatcher
        }
        ROS_INFO("Marker detection deactivated.");


        // Step 4: Publish the waypoint and its associated marker ID
        if (marker_detected) {
            exp_rob_assignment2::MarkerWaypointPair marker_waypoint_msg;
            marker_waypoint_msg.marker_id = detected_marker_id;
            marker_waypoint_msg.waypoint = msg->parameters[1].value;  // Extract waypoint ID from action parameters
            marker_waypoint_pub.publish(marker_waypoint_msg);

            ROS_INFO("Published marker-waypoint pair: marker_id = %d, waypoint = %s",
                     marker_waypoint_msg.marker_id, marker_waypoint_msg.waypoint.c_str());
        }

        // Step 5: Return success to the action dispatcher
        ROS_INFO("RotateAndDetect action completed successfully.");
        return true;
    }

} // namespace KCL_rosplan

int main(int argc, char **argv) {
    ros::init(argc, argv, "rotate_and_detect_action", ros::init_options::AnonymousName);
    ros::NodeHandle nh("~");
    KCL_rosplan::RotateAndDetectAction rad_action(nh);
    rad_action.runActionInterface();

    return 0;
}
