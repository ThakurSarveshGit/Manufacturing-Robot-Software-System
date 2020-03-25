/*
 * Group 7 RWA 4
 * AriacSensorManager.cpp: Class definitions for AriacSensorManager Class
 * Defines all callback functions for every sensor
 */


// -- Including Header Files
#include "AriacSensorManager.h"

using namespace std;

// Constructor AriacSensorManager()
AriacSensorManager::AriacSensorManager():
	part_list{}, arm1{"arm1"}, arm2{"arm2"}{

	// Subscribing to topics
	orders_sub = sensor_nh_.subscribe("/ariac/orders", 10, &AriacSensorManager::order_callback, this);
	bb_1_sub = sensor_nh_.subscribe("/ariac/break_beam_1_change", 10, &AriacSensorManager::bb_1_callback, this);
	bb_2_sub = sensor_nh_.subscribe("/ariac/break_beam_2_change", 10, &AriacSensorManager::bb_2_callback, this);
	lc_gear_sub = sensor_nh_.subscribe("/ariac/lc_gear", 10, &AriacSensorManager::lc_gear_callback, this);
	qc_1_sub = sensor_nh_.subscribe("/ariac/quality_control_sensor", 10, &AriacSensorManager::qc_1_callback, this);

	order_number = 0;

	qc_1_redFlag = false;
	qc_2_redFlag = false;
}

// Deconstructor
AriacSensorManager::~AriacSensorManager(){}

// -- Order Callback
void AriacSensorManager::order_callback(const osrf_gear::Order::ConstPtr & order_msg){
	ROS_INFO_STREAM("[ASM]:Received Order:\n" << *order_msg);
	received_orders_.push_back(*order_msg);
	setDesiredParts();
}

// Helper Function: List the parts we are interested in [Reading the Order Details]
void AriacSensorManager::setDesiredParts(){
	ROS_INFO_STREAM("[ASM]:[setDesiredParts]: Making a List of kit parts to assemble\n");
	
	auto current_order = received_orders_[order_number]; // Complete Order Message
	auto order_id = current_order.order_id; // Order ID
	auto shipments = current_order.shipments; // To access Shipments

	for (const auto &shipment:shipments){
		auto shipment_type = shipment.shipment_type; // ex: order_0_shipment_0
		auto products = shipment.products; // To access Product types

		ROS_INFO_STREAM("[ASM]:[setDesiredParts]: Order ID: " << order_id);
		ROS_INFO_STREAM("[ASM]:[setDesiredParts]: Shipment Type: " << shipment_type);

		for (const auto &product:products)
			desired_parts.insert(product.type);
	}
	ROS_INFO_STREAM("[ASM]:[setDesiredParts]: Current Desired Parts are:\n");
	for (const auto &part:desired_parts)
		std::cout << part << std::endl;

	if (!order_id.empty())
		++order_number; // Everytime order_callback() is called; this will get incremented
}

// -- Logical Camera on the Conveyer Belt Callback Function
void AriacSensorManager::lc_belt_callback(const osrf_gear::LogicalCameraImage::ConstPtr& image_msg){

	// ros::AsyncSpinner spinner(0);
	// spinner.start();

	if (image_msg->models.size() == 0){
		ROS_WARN_THROTTLE(5, "[ASM]:[lc_belt_callback]: LC detects nothing!");
		return;
	}

	ROS_INFO_STREAM("[ASM]:[lc_belt_callback]: LC detects "<< image_msg->models.size() <<" objects!\n");

	ros::Duration timeout(0.2);
	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener tfListener(tfBuffer);

	for (auto &msg:image_msg->models){
		geometry_msgs::TransformStamped transformStamped;

		// Increment the number of times a part has crossed the LC on the belt
		if (part_counter.find(msg.type) == part_counter.end())
			part_counter.insert(make_pair(msg.type, 0));
		else
			part_counter[msg.type]++;

		// Frame name of that part
		std::string camera_frame = "lc_belt_" + msg.type + "_" + to_string(part_counter[msg.type]) + "_frame";

		ROS_INFO_STREAM("[ASM]:[lc_belt_callback]: Frame name: " << camera_frame);

		// Get World Pose of the part
		try{
			transformStamped = tfBuffer.lookupTransform("world", camera_frame, ros::Time(0), timeout);
			std::pair<std::string, std::string> part_pair {msg.type, camera_frame}; // dictionary of key:part name and value:its camera frame name
			part_list.push_back(part_pair);

			geometry_msgs::Pose part_pose;
			part_pose.position.x = transformStamped.transform.translation.x;
			part_pose.position.y = transformStamped.transform.translation.y;
			// part_pose.position.y = 0.9;
			part_pose.position.z = transformStamped.transform.translation.z;
            
            part_pose.orientation.x = transformStamped.transform.rotation.x;
            part_pose.orientation.y = transformStamped.transform.rotation.y;
            part_pose.orientation.z = transformStamped.transform.rotation.z;
            part_pose.orientation.w = transformStamped.transform.rotation.w;

            part_pose_list.insert({camera_frame, part_pose});
		}
		catch (tf2::TransformException &ex){
			ROS_WARN("%s\n", ex.what());
			part_counter[msg.type]--; // If pose could not be extracted, don't include the part
		}
		break; // Just adding the first model in the camera frame
	}
	lc_belt_sub.shutdown(); // subscriber deactivated
}

// -- lc_gear_callback()
void AriacSensorManager::lc_gear_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){
	// ros::AsyncSpinner spinner(0);
	// spinner.start();

	if (image_msg->models.size() == 0){
		ROS_WARN_THROTTLE(5, "[ASM]:[lc_gear_callback]: LC detects nothing!");
		return;
	}
	ROS_INFO_STREAM("[ASM]:[lc_gear_callback]: LC detects "<<image_msg->models.size() << " gears.");

	// Set Arm 2 Joint Angles
	unordered_map<std::string, double> arm2_joint_home_pose;
	
	arm2_joint_home_pose["linear_arm_actuator_joint"] = 0.3;
    arm2_joint_home_pose["shoulder_pan_joint"] = 2;
    arm2_joint_home_pose["shoulder_lift_joint"] = 0;
    arm2_joint_home_pose["elbow_joint"] = 0;
    arm2_joint_home_pose["wrist_1_joint"] = -3.14/2;
    arm2_joint_home_pose["wrist_2_joint"] = -3.14/2;
    arm2_joint_home_pose["wrist_3_joint"] = 0;

    ros::Duration timeout(0.2);
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    size_t gear_counter = 1;

    for (auto & msg : image_msg->models){
        geometry_msgs::TransformStamped transformStamped;
        string camera_frame = "lc_gear_" + msg.type + "_" + to_string(gear_counter) + "_frame";
        ROS_INFO_STREAM("[ASM]:[lc_gear_callback]: Frame name: " << camera_frame);
        try{
            transformStamped = tfBuffer.lookupTransform("world", camera_frame, ros::Time(0), timeout);
            geometry_msgs::Pose part_pose; 
            part_pose.position.x = transformStamped.transform.translation.x;
            part_pose.position.y = transformStamped.transform.translation.y;
            part_pose.position.z = transformStamped.transform.translation.z;
            part_pose.orientation.x = transformStamped.transform.rotation.x;
            part_pose.orientation.y = transformStamped.transform.rotation.y;
            part_pose.orientation.z = transformStamped.transform.rotation.z;
            part_pose.orientation.w = transformStamped.transform.rotation.w;

            ++gear_counter;
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s\n",ex.what());
        }
    }
    lc_gear_sub.shutdown();
}

// -- Quality Control 1 Callback Function
void AriacSensorManager::qc_1_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){
	if (image_msg->models.size() == 0)
		qc_1_redFlag = false;
	else
		qc_1_redFlag = true;
}

// -- Quality Control 2 Callback Function
void AriacSensorManager::qc_2_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){
	if (image_msg->models.size() == 0)
		qc_2_redFlag = false;
	else
		qc_2_redFlag = true;
}

// -- BreakBeam 1 Callback
void AriacSensorManager::bb_1_callback(const osrf_gear::Proximity::ConstPtr &msg){
	// ros::AsyncSpinner spinner(0);
	// spinner.start();

	if (msg->object_detected){
		lc_belt_sub = sensor_nh_.subscribe("/ariac/lc_belt", 10, &AriacSensorManager::lc_belt_callback, this);
	}
}

// -- BreakBeam 2 Callback /* -- This function asks the arm1 to pick up the part from the conveyer belt*/
void AriacSensorManager::bb_2_callback(const osrf_gear::Proximity::ConstPtr &msg){
	// ros::AsyncSpinner spinner(0);
	// spinner.start();

	if (msg->object_detected){
		ROS_INFO_STREAM("[ASM]:[bb_2_callback]: BB Triggered. Part: " << part_list.front().first);
		auto element_itr = desired_parts.find(part_list.front().first);
		if (element_itr != desired_parts.end()){// If element in Desired Part List
			ROS_INFO_STREAM("[ASM]:[bb_2_callback]: Pick this part! ");
			auto part_frame = part_pose_list[part_list.front().second];
			ROS_INFO_STREAM("[ASM]:[bb_2_callback]: Desired part frame name: \n" << part_frame);
			
			// If arm1 was successful in picking up the element, remove it from the desired parts set
			bool if_pick = arm1.PickPart(part_frame);
			if (if_pick)
				desired_parts.erase(element_itr);
		}
		else
			ROS_INFO_STREAM("[ASM]:[bb_2_callback]: Let this part go! ");
		part_list.pop_front();
	}
}