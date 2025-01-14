#include "exp_rob_assignment2/goto_lowest_marker_wp_action.h"
#include <unistd.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <move_base_msgs/MoveBaseAction.h>
#include "exp_rob_assignment2/GetLowestMarkerInfo.h"


namespace KCL_rosplan {
	GoToLowestMarkerWaypointAction::GoToLowestMarkerWaypointAction(ros::NodeHandle &nh) {
        // Initialize the service client
        get_lowest_marker_info_client = nh.serviceClient<exp_rob_assignment2::GetLowestMarkerInfo>("/get_lowest_marker_info");
	}
	
	bool GoToLowestMarkerWaypointAction::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
		// here the implementation of the 'goto_waypoint' action

        ROS_INFO("Action (%s) started.", msg->name.c_str());

        // Create a service instance (empty request)
        exp_rob_assignment2::GetLowestMarkerInfo srv;

        // Create an action client for the move_base action
        actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
        ac.waitForServer();

        // Call the service
        if (get_lowest_marker_info_client.call(srv)) {
            // Process the response
            x = srv.response.x;
            y = srv.response.y;
            ROS_INFO("Received lowest marker info: ID=%d, Waypoint=%s, x=%.2f, y=%.2f",
                     srv.response.marker_id, srv.response.waypoint.c_str(), x, y);
        } else {
            ROS_ERROR("Failed to call service /get_lowest_marker_info");
        }

        // Create a MoveBaseGoal to send to the action server
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "map";  // Set reference frame
        goal.target_pose.header.stamp = ros::Time::now();  // Timestamp for the goal
        goal.target_pose.pose.position.x = x;
        goal.target_pose.pose.position.y = y;
        goal.target_pose.pose.orientation.w = 1.0;

        // Visualize the goal sent
        ROS_INFO("Goal frame_id: %s", goal.target_pose.header.frame_id.c_str());
        ROS_INFO("Goal position: x=%.2f, y=%.2f, orientation.w=%.2f",
                goal.target_pose.pose.position.x,
                goal.target_pose.pose.position.y,
                goal.target_pose.pose.orientation.w);
        

        // Send the goal to the move_base server
        ROS_INFO("Preparing to send goal to move_base...");
        ac.sendGoal(goal);

        // Wait for the result
        ROS_INFO("Goal sent to move_base, waiting for result...");
        ac.waitForResult();
        
        // Check for success
		ROS_INFO("Action (%s) performed: completed!", msg->name.c_str());
		return true;
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "goto_lowest_marker_wp_action", ros::init_options::AnonymousName);
	ros::NodeHandle nh("~");
	KCL_rosplan::GoToLowestMarkerWaypointAction gtlmw_action(nh);
	gtlmw_action.runActionInterface();
	return 0;
}
