#!/usr/bin/env python3

# EXPERIMENTAL ROBOTICS LABORATORY
# ASSIGNMENT 2
# By: Ami Sofia Quijano Shimizu

# DESCRIPTION:
# Node subscribes to the marker-waypoint pairs topic and builds a map of detected markers with their corresponding 
# waypoint, x coord. and y coord. Upon service request, it looks in map and replies with the lowest marker ID and 
# its associated information.

import rospy
from exp_rob_assignment2.msg import MarkerWaypointPair
from exp_rob_assignment2.srv import GetLowestMarkerInfo, GetLowestMarkerInfoResponse

class LowestMarkerWaypoint:
    def __init__(self):
        """
        Initialize the node, subscriber, and service.
        """
        # Initialize the node
        rospy.init_node('lowest_marker_waypoint_node', anonymous=True)

        # Dictionary to store marker-waypoint pairs and their coordinates
        self.waypoint_marker_map = {}

        # Predefined waypoint coordinates
        self.waypoint_coordinates = {
            "wp1": {"x": -7.0, "y": -1.5},
            "wp2": {"x": -3.0, "y": -8.0},
            "wp3": {"x": 6.0, "y": 2.0},
            "wp4": {"x": 7.0, "y": -5.0},
        }

        # Subscriber to the marker-waypoint topic
        self.marker_waypoint_subscriber = rospy.Subscriber('/aruco/marker_waypoint_pairs', MarkerWaypointPair, 
                                                        self.marker_waypoint_callback, queue_size=10)

        # Service to provide the lowest marker ID with its waypoint and coordinates
        self.lowest_marker_service = rospy.Service('/get_lowest_marker_info', GetLowestMarkerInfo,
                                                        self.handle_get_lowest_marker_info)

    def marker_waypoint_callback(self, msg):
        """
        Callback function to process MarkerWaypointPair messages and update the map.
        """
        # Update the map with the new marker-waypoint pair
        if msg.waypoint in self.waypoint_coordinates:
            self.waypoint_marker_map[msg.marker_id] = {
                "waypoint": msg.waypoint,
                "x": self.waypoint_coordinates[msg.waypoint]["x"],
                "y": self.waypoint_coordinates[msg.waypoint]["y"],
            }
            rospy.loginfo(f"Updated map: {self.waypoint_marker_map}")
        else:
            rospy.logwarn(f"Waypoint '{msg.waypoint}' not found in predefined coordinates!")

    def handle_get_lowest_marker_info(self, req):
        """
        Service handler to return the lowest marker ID with its waypoint and coordinates.
        """
        if not self.waypoint_marker_map:
            rospy.logwarn("Waypoint-marker map is empty!")
            return GetLowestMarkerInfoResponse(marker_id=-1, waypoint="", x=0.0, y=0.0)

        # Find the lowest marker ID and its associated data
        lowest_marker_id = min(self.waypoint_marker_map.keys())
        lowest_data = self.waypoint_marker_map[lowest_marker_id]

        rospy.loginfo(f"Lowest marker info requested: ID={lowest_marker_id}, "
                      f"Waypoint={lowest_data['waypoint']}, "
                      f"X={lowest_data['x']}, Y={lowest_data['y']}")

        return GetLowestMarkerInfoResponse(
            marker_id=lowest_marker_id,
            waypoint=lowest_data["waypoint"],
            x=lowest_data["x"],
            y=lowest_data["y"]
        )

    def spin(self):
        """
        Keep the node running.
        """
        rospy.loginfo("Lowest Marker-Waypoint Node is running...")
        rospy.spin()


if __name__ == '__main__':
    try:
        # Create an instance of the node and run it
        node = LowestMarkerWaypoint()
        node.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Lowest Marker-Waypoint Node stopped.")
