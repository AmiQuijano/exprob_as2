#include "exp_rob_assignment2/goto_waypoint_action.h"
#include <unistd.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <move_base_msgs/MoveBaseAction.h>

namespace KCL_rosplan {
	GoToWaypointAction::GoToWaypointAction(ros::NodeHandle &nh) {
	// here the initialization
	}
	
	bool GoToWaypointAction::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
		// here the implementation of the 'goto_waypoint' action

        ROS_INFO("Action dispatch received: %s", msg->name.c_str());

        // Create an action client for the move_base action
        actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
        ac.waitForServer();

        // Create a MoveBaseGoal to send to the action server
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "map";  // Set reference frame
        goal.target_pose.header.stamp = ros::Time::now();  // Timestamp for the goal

        // Define the waypoints based on the input
        if(msg->parameters[2].value == "wp0"){
        goal.target_pose.pose.position.x = 0.0;
        goal.target_pose.pose.position.y = 1.0;
        goal.target_pose.pose.orientation.w = 1.0;
        }
        else if (msg->parameters[2].value == "wp1"){
        goal.target_pose.pose.position.x = -7.0;
        goal.target_pose.pose.position.y = -1.5;
        goal.target_pose.pose.orientation.w = 1.0;
        }
        else if (msg->parameters[2].value == "wp2"){
        goal.target_pose.pose.position.x = -3.0;
        goal.target_pose.pose.position.y = -8.0;
        goal.target_pose.pose.orientation.w = 1.0;
        }
        else if (msg->parameters[2].value == "wp3"){
        goal.target_pose.pose.position.x = 6.0;
        goal.target_pose.pose.position.y = 2.0;
        goal.target_pose.pose.orientation.w = 1.0;
        }
        else if (msg->parameters[2].value == "wp4"){
        goal.target_pose.pose.position.x = 7.0;
        goal.target_pose.pose.position.y = -5.0;
        goal.target_pose.pose.orientation.w = 1.0;
        }

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
	ros::init(argc, argv, "goto_waypoint_action", ros::init_options::AnonymousName);
	ros::NodeHandle nh("~");
	KCL_rosplan::GoToWaypointAction gtw_action(nh);
	gtw_action.runActionInterface();
	return 0;
}
