/*
 * Group 7 RWA 4
 * main_node.cpp: Build a complete kit
 */


// -- Including Header Files
// #include "MyCompeitionClass.h"
#include "AriacSensorManager.h"

int main(int argc, char ** argv){

	ros::init(argc, argv, "mainNode"); // Initializing a ROS Node
	ros::NodeHandle main_nh; // Create a Node Handle

	// Creating a Multithreaded Object
	ros::AsyncSpinner spinner(0);

	AriacSensorManager sensor; // Create a sensor object

	// Wait for Competition to start
	ROS_INFO_STREAM("Wait for competition to start . . .");

	// enable multithreading
	spinner.start();
	ros::waitForShutdown();

	return 0;
}