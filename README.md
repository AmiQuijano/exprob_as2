# exprob_as2
Assignment 2 for the Experimental Robotics Laboratory course.

## Description
4 Aruco markers are placed in a Gazebo world. Under the assumption that around each waypoint there is a marker, the robot is tasked to:

1. Visit a waypoint
2. Find the aruco marker in that waypoint and save its ID
3. After all waypoints have been visited and the 4 markers IDs have been found, go to the waypoint with the marker of lowest ID

## Requirements
The following package runs with ROS in Ubuntu 20.04
It requires previous installation of ROSplan (included in the ```rosplan``` package), OpenCV libraries, aruco marker models, aruco files (included in the ```aruco_ros``` package), and navigation algorithm of move base (included in the ```planning``` package). 

## How to run
Clone this repo, which contains 4 ROS packages and a video folder, in the ```/src``` folder of a ROS workspace. Then build your workspace.

In 3 terminals run the following launch files:

1. Gazebo, RViz, Gmapping and Move Base
   
```roslaunch exp_rob_assignment2 test.launch```

2. ROSplan nodes, Action interfacees, Aruco detection node and Finder of lowest marker-waypoint node
 
```roslaunch exp_rob_assignment2 test2.launch```

3. Call ROSplan services in this order

```rosservice call /rosplan_problem_interface/problem_generation_server ```

```rosservice call /rosplan_planner_interface/planning_server```

```rosservice call /rosplan_parsing_interface/parse_plan```

```rosservice call /rosplan_plan_dispatcher/dispatch_plan ```

Note: the decision of splitting the launch files in 2 is because the Move Base launch file throws repeatedly warnings, which makes impossible to visualize other logs in the terminal regarding other nodes.

## Solution Approach
First, a .xacro and .gazebo files where created to incorporate to a mobile robot a camera and a laser scanner.

For solving the tasks mentioned above, a domain and problem file were creating with PDDL. 3 durative actions where created:
* ```goto_waypoint``` for task 1.
* ```rotate_and_detect``` for task 2.
* ```goto_lowest_marker_wp``` for task 3.

In addition, for each of the three actions, a ```.h``` and ```.cpp``` file were created for the action interface of the given action. Aruco marker detection ```.py``` script was done implementing OpenCV and Aruco libraries.

Demo:
[Watch/Download the video](video/exprob_as2_funcionality_proof_compressed.mp4)

## Visualization
The execution of each action can be followed by the terminal, Gazebo and RViz:

1. **Terminal**: It tells which new marker has been recognized (topic ```/detected_marker_id ```), the marker ID and its associated waypoint (topic ```/aruco/marker_waypoint_pairs```), the dispatched action (topic ```/rosplan_plan_dispatcher/action_dispatch```), and finally the activation of marker detection, goal position, predicate updates in the same terminal where ```test.launch``` is launched.
2. **RViz**: Shows the robot, the camera feed, the highlighted recognized Aruco markers and the robot in the map (add Map from topic) of obstacles, free and unknown space.
3. **Gazebo**: Shows the Gazebo world with the obstacles, the robot and the aruco markers in position.



