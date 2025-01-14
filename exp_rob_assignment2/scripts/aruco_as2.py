#!/usr/bin/env python3

# EXPERIMENTAL ROBOTICS LABORATORY
# ASSIGNMENT 2
# By: Ami Sofia Quijano Shimizu

# DESCRIPTION:
# Node subscribes to the camera feed to detect one Aruco marker. The robot rotates until the marker is detected.
# When the marker is found, it highlights the marker, publishes the image with the highlighted marker, 
# publishes its ID, and stops rotating.
# The above funcionality is activated and deactivated through a service request.

# Python libraries
import sys
import numpy as np

# OpenCV libraries
import cv2
import cv2.aruco as aruco
from cv_bridge import CvBridge, CvBridgeError

# ROS libraries
import rospy
from sensor_msgs.msg import CompressedImage, Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from std_srvs.srv import SetBool, SetBoolResponse

class MarkerDetector:
    def __init__(self):
        '''Initialize the ROS node, publishers, subscribers, and other parameters'''
        # Initialize ROS node
        rospy.init_node('marker_detector', anonymous=True)

        # Initialize CvBridge
        self.bridge = CvBridge()
        
        # Publisher for robot motion
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # Publisher for images with Aruco markers highlighted
        self.aruco_pub = rospy.Publisher("/aruco_images", Image, queue_size=10)

        # Publisher for detected marker ID
        self.marker_id_pub = rospy.Publisher("/detected_marker_id", Int32, queue_size=10)

        # Subscriber to camera feed (in compressed image format)
        self.camera_sub = rospy.Subscriber("/robot/camera1/image_raw/compressed",
                                           CompressedImage, self.camera_callback, queue_size=10)

        # Service for activating the funcionalities of this node 
        # (This was necessary for having the marker detection idle unless the ROSplan action dispatcher triggers it)
        self.state_service = rospy.Service('/toggle_marker_detection', SetBool, self.toggle_detection)

        # ArUco dictionary and detector parameters
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
        self.aruco_params = aruco.DetectorParameters_create()

        # Flags
        self.marker_found = False 
        self.activate_marker_detection = False

    def toggle_detection(self, req):
        '''Service callback to activate or deactivate detection'''
        if req.data:  # Activate detection
            rospy.loginfo("Marker detection activated.") 
            self.activate_marker_detection = True
            self.marker_found = False  # Reset flag
            return SetBoolResponse(success=True, message="Marker detection activated.")
        
        else:  # Deactivate detection
            rospy.loginfo("Marker detection deactivated.")
            self.activate_marker_detection = False
            self.marker_found = False  # Reset flag
            return SetBoolResponse(success=True, message="Marker detection deactivated.")


    def camera_callback(self, ros_data):
        '''Callback function for processing images from the camera'''
        if self.activate_marker_detection:
            # Convert ROS compressed image to OpenCV format
            np_arr = np.frombuffer(ros_data.data, np.uint8)
            image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            # Convert to grayscale for ArUco detection
            image_gray = cv2.cvtColor(image_np, cv2.COLOR_BGR2GRAY)

            # Detect ArUco markers in the frame
            corners, ids, _ = aruco.detectMarkers(image_gray, self.aruco_dict, parameters=self.aruco_params)

            if not self.marker_found:
                self.detect_marker(ids, corners, image_np)

    def detect_marker(self, ids, corners, image_np):
        '''Detect and process a single marker'''
        if ids is not None:
            # If any marker is detected, stop rotation and process the first marker
            self.marker_found = True
            new_id = ids[0][0]
            rospy.loginfo(f"Marker detected: ID {new_id}")
            
            # Highlight the marker
            self.highlight_marker(corners[0], new_id, image_np)

            # Publish the image with the highlighted marker
            self.publish_image(image_np)

            # Stop the robot
            self.stop()

            # Publish the marker ID to a ROS topic
            self.marker_id_pub.publish(new_id)
        
        else:
            self.rotate()

    def highlight_marker(self, corner, new_id, image_np):
        '''Draw a red circle around the detected marker'''
        # Calculate the center of the marker
        c_x = int(np.mean(corner[0][:, 0]))
        c_y = int(np.mean(corner[0][:, 1]))

        # Calculate the radius of the circle based on marker size
        radius = int(np.linalg.norm(corner[0][0] - corner[0][1]) / 2)

        # Draw the circle on the image
        cv2.circle(image_np, (c_x, c_y), radius, (0, 0, 255), 2)

        # Add ID number as text on image
        cv2.putText(image_np, str(new_id), 
                    (c_x + 10, c_y + 10),            # Position
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8,   # Font and size
                    (0, 0, 255), 2)                  # Color and thickness

    def publish_image(self, image_np):
        '''Publish the image with the highlighted marker to the ROS topic'''
        try:
            # Convert Cv Image into a ROS Image message
            ros_image = self.bridge.cv2_to_imgmsg(image_np, "bgr8")  

            # Publish to topic
            self.aruco_pub.publish(ros_image)  

        except CvBridgeError as error:
            rospy.loginfo(f"Error publishing image: {error}")

    def rotate(self):
        '''Rotate the robot by publishing angular velocity'''
        vel = Twist()
        vel.angular.z = -0.3  # Rotate to the right
        self.vel_pub.publish(vel)

    def stop(self):
        '''Stop the robot by publishing zero velocity'''
        vel = Twist()
        vel.angular.z = 0
        self.vel_pub.publish(vel)

def main(args):
    '''Main function to initialize the node and start processing'''
    md = MarkerDetector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down Marker Detector node")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
