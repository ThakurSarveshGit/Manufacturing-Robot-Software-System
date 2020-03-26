/*
 * Group 7 RWA 4
 * main_node.cpp: Build a complete kit
 */


// -- Including Header Files
// #include "MyCompeitionClass.h"
#include "AriacSensorManager.h"

int main(int argc, char ** argv){

	// Initializing a ROS Node: named as 'mainNode; >> rosrun rwa4 mainNode
	ros::init(argc, argv, "mainNode");
	ros::NodeHandle main_nh; // Create a Node Handle

	// Creating a Multithreaded Object
	ros::AsyncSpinner spinner(0);

	// Wait for Competition to start
	ROS_INFO_STREAM("Wait for competition to start . . .");

	/* We initialize subscribers inside constructor of AriacSensorManager and when 
	they recieve msg, corresponding callbacks get called */

	// Create a sensor object
	AriacSensorManager sensor; // Constructor gets called

	// enable multithreading
	spinner.start();

	ros::waitForShutdown();

	return 0;
}