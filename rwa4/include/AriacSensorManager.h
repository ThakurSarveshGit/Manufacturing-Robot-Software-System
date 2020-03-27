/*
 * Group 7 RWA 4
 * AriacSensorManager.h: Contains declarations for AriacSensorManager Class
 */

# pragma once

#include <unordered_map>
#include <string>
#include <cmath>
#include <set>
#include <deque>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> 
#include <tf2_ros/buffer.h>
#include <osrf_gear/LogicalCameraImage.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <osrf_gear/Proximity.h>
#include <osrf_gear/Order.h>

#include "RobotController.h"


class AriacSensorManager{
private:
	ros::NodeHandle sensor_nh_; // Node Handle for AriacSensorManager Class

	// Subscribers
	ros::Subscriber lc_belt_sub; // Subscribe to the '/ariac/lc_belt' topic; Initialized in BB1 callback function
	
	ros::Subscriber bb_1_sub; // Subscribe to the '/ariac/break_beam_1_change' topic
	ros::Subscriber bb_2_sub; // Subscribe to the '/ariac/break_beam_2_change' topic
	
	ros::Subscriber orders_sub; // Orders topic
	
	ros::Subscriber lc_gear_sub; // Subscribe to the /ariac/lc_gear topic
	
	ros::Subscriber qc_1_sub; // Subscribe to the '/ariac/quality_control_sensor_1' topic
	ros::Subscriber qc_2_sub; // Subscribe to the '/ariac/quality_control_sensor_2' topic	

	bool qc_1_redFlag;
	bool qc_2_redFlag;

	// Containers Order/Product/Pose
	std::vector<osrf_gear::Order> received_orders_;
	
	unsigned int order_number;

	std::unordered_map<std::string, geometry_msgs::Pose> part_pose_list; // Frame Name and Its world pose details
	std::unordered_map<std::string, geometry_msgs::Pose> gear_bin_map;
	std::deque<std::pair<std::string, std::string>> part_list; // Queue of key-Value pair of part name & its camera frame name
	std::unordered_map<std::string, unsigned int> part_counter; // No. of times a part has moved on the conveyer belt before
	std::multiset<std::string> desired_parts; // Set of parts that go inside the kit

	RobotController arm1; // arm1 object of class RobotController for accessing UR10 Arm1 
	RobotController arm2; // arm2 object of class RobotController for accessing UR10 Arm2 

public:
    AriacSensorManager();
    ~AriacSensorManager();
    void order_callback(const osrf_gear::Order::ConstPtr &);
    void lc_belt_callback(const osrf_gear::LogicalCameraImage::ConstPtr &);
    void bb_1_callback(const osrf_gear::Proximity::ConstPtr &);
    void bb_2_callback(const osrf_gear::Proximity::ConstPtr &);
    void lc_gear_callback(const osrf_gear::LogicalCameraImage::ConstPtr &);
    void qc_1_callback(const osrf_gear::LogicalCameraImage::ConstPtr &);
    void qc_2_callback(const osrf_gear::LogicalCameraImage::ConstPtr &);

    void setDesiredParts(); // Parts that we want to pick up
	bool checkFaultyArmOne(std::string frameName, const geometry_msgs::Pose& pose); // Check if the picked part is faulty or not

    // Getter: PartList
	std::deque<std::pair<std::string, std::string>> get_part_list(){
		return part_list;
    }
};