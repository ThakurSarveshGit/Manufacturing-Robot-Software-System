/*
 * Group 7 RWA 4
 * MyCompetitionClass.h: Contains declarations for MyCompetitionClass definitions
 */

#include <algorithm>
#include <vector>
#include <deque>
#include <ros/ros.h>
#include <string>
#include <osrf_gear/Order.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <trajectory_msgs/JointTrajectory.h>


class MyCompetitionClass{
private:
	ros::Subscriber orders_subscriber; 	// Subscriber
	std::vector<osrf_gear::Order> received_orders_; // Order/Products

public:
	explicit MyCompetitionClass(ros::NodeHandle &);
	void order_callback(const osrf_gear::Order::ConstPtr &); // Called when a new order message is received
};