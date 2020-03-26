/*
 * Group 7: RWA3 Pick a part from the Conveyer Belt
 * 
 */

// -- CPP INCLUDES
#include <algorithm>
#include <vector>
#include <iostream>
#include <ros/ros.h>
#include <string>
#include <ctime>
#include <ros/service.h>

// -- ARIAC INCLUDES
#include <osrf_gear/LogicalCameraImage.h>
#include <osrf_gear/Order.h>
#include <osrf_gear/Proximity.h>
#include <osrf_gear/VacuumGripperControl.h>
#include <osrf_gear/VacuumGripperState.h>

#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>

#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>


// -- Class PickUp

class PickUp{

public:
	bool busyFlag{false}; // If the arm is busy or not
	int pickUpNumber{-1}; // Part of Interest on the conveyer belt
	
};

