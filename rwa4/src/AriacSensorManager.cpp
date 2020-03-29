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

	// Initializing various Intermediate poses
    arm2_check_qc_pose["linear_arm_actuator_joint"] = -1.1;
    arm2_check_qc_pose["shoulder_pan_joint"] = 4.6;
    arm2_check_qc_pose["elbow_joint"] = 0;
    arm2_check_qc_pose["shoulder_lift_joint"] = 0;
    arm2_check_qc_pose["wrist_1_joint"] = 3.14 * 3 / 2;
    arm2_check_qc_pose["wrist_2_joint"] = 3.14/2;

    go_transition_pose["linear_arm_actuator_joint"] = -0.56;
    go_transition_pose["shoulder_pan_joint"] = 3.14;
    go_transition_pose["shoulder_lift_joint"] = -3.14/2;
    go_transition_pose["elbow_joint"] = 3.14/2;
    go_transition_pose["wrist_1_joint"] = 3.14 * 3 / 2;
    go_transition_pose["wrist_2_joint"] = 3.14/2;

    back_transition_pose["linear_arm_actuator_joint"] = -0.56;
    back_transition_pose["shoulder_pan_joint"] = 3.14;
    back_transition_pose["shoulder_lift_joint"] = -3.14/2;
    back_transition_pose["elbow_joint"] = 3.14/2;
    back_transition_pose["wrist_1_joint"] = -1.5;
    back_transition_pose["wrist_2_joint"] = -3.14/2;

    arm1_bin_pose["linear_arm_actuator_joint"] = 0.28;
    arm1_bin_pose["shoulder_pan_joint"] = 2.13;
    arm1_bin_pose["shoulder_lift_joint"] = -0.8;
    arm1_bin_pose["elbow_joint"] = 1.6;
    arm1_bin_pose["wrist_1_joint"] = 3.9;
    arm1_bin_pose["wrist_2_joint"] = -1.57;

    arm1_check_qc_pose["linear_arm_actuator_joint"] = 1.18;
    arm1_check_qc_pose["shoulder_pan_joint"] = 1.44;
    arm1_check_qc_pose["elbow_joint"] = 1.28;
    arm1_check_qc_pose["shoulder_lift_joint"] = -0.5;
    arm1_check_qc_pose["wrist_1_joint"] = 3.94;
    arm1_check_qc_pose["wrist_2_joint"] = 1.57;
}

// Deconstructor
AriacSensorManager::~AriacSensorManager(){}

// -- Order Callback
void AriacSensorManager::order_callback(const osrf_gear::Order::ConstPtr & order_msg){
	ROS_INFO_STREAM("[ASM]:[order_callback]: Received Order:\n" << *order_msg);
	received_orders_.push_back(*order_msg);
	setDesiredParts();
}

// Helper Function: List the parts we are interested in [Reading the Order Details]
void AriacSensorManager::setDesiredParts(){
	ROS_INFO_STREAM("[ASM]:[setDesiredParts]: Making a List of kit parts to assemble");
	
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
	ROS_INFO_STREAM("[ASM]:[setDesiredParts]: Current Desired Parts are:");
	for (const auto &part:desired_parts)
		std::cout << "\t\t\t" << part << std::endl;

	if (!order_id.empty())
		++order_number; // Everytime order_callback() is called; this will get incremented

    // Call DesiredPartsLocation() Function: Set the location of these parts in a variable
    DesiredPartsLocation();
}

void AriacSensorManager::DesiredPartsLocation(){
    /*
     * Set the location of the desired parts
     */

    // Step 1: Access the desired parts
    auto currentDesiredParts = desired_parts;

    for (auto i:currentDesiredParts)
        ROS_INFO_STREAM("[ASM]:[DesiredPartsLocation]: Part Name: " << i);

    // Step 2: Check whether the part is available on any bins


    // Step 3: Set the location of the part.
}

// -- Logical Camera on the Conveyer Belt Callback Function; Subsciber callback for: lc_belt_sub initialized inside bb_1_callback
void AriacSensorManager::lc_belt_callback(const osrf_gear::LogicalCameraImage::ConstPtr& image_msg){

	ros::AsyncSpinner spinner(0);
	spinner.start();

	if (image_msg->models.size() == 0){
		ROS_WARN_THROTTLE(5, "[ASM]:[lc_belt_callback]: LC detects nothing!");
		return;
	}

	ROS_INFO_STREAM("[ASM]:[lc_belt_callback]: LC detects "<< image_msg->models.size() <<" objects!");

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

		ROS_INFO_STREAM("[ASM]:[lc_belt_callback]: New part frame: " << camera_frame);

		// Get World Pose of the part
		try{
			transformStamped = tfBuffer.lookupTransform("world", camera_frame, ros::Time(0), timeout);
			std::pair<std::string, std::string> part_pair {msg.type, camera_frame}; // dictionary of key:part name and value:its camera frame name
			part_list.push_back(part_pair);

			geometry_msgs::Pose part_pose;
			part_pose.position.x = transformStamped.transform.translation.x;
			// part_pose.position.y = transformStamped.transform.translation.y;
			part_pose.position.y = 1.6; // This is hard-coded! y-coordinate will be same as y-coord for bb2
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
	ros::AsyncSpinner spinner(0);
	spinner.start();

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
	ros::AsyncSpinner spinner(0);
	spinner.start();

	if (msg->object_detected){
		ROS_INFO_STREAM("[ASM]:[bb_1_callback]: BB1 Triggered. Subscribing to LC on Belt.");
		lc_belt_sub = sensor_nh_.subscribe("/ariac/lc_belt", 10, &AriacSensorManager::lc_belt_callback, this);
	}
}


// -- BreakBeam 2 Callback /* -- This function asks the arm1 to pick up the part from the conveyer belt*/
void AriacSensorManager::bb_2_callback(const osrf_gear::Proximity::ConstPtr &msg){
	ros::AsyncSpinner spinner(0);
	spinner.start();

	if (msg->object_detected){
		ROS_INFO_STREAM("[ASM]:[bb_2_callback]: BB2 Triggered. Part: " << part_list.front().first);

		auto element_itr = desired_parts.find(part_list.front().first);

		// Pick up the desired Part ; Check for its quality and place it at the right place
		if (element_itr != desired_parts.end()){

			// Step 1: Check the world pose of the port
			ROS_INFO_STREAM("[ASM]:[bb_2_callback]: Pick this part!");
			auto part_world_pose = part_pose_list[part_list.front().second];
			std::string part_frame_name = part_list.front().second;// Get the frame name of the part; todo: use to check which parts were faulty
			
			// Step 2: Try to pick up the part
			bool if_pick = arm1.PickPart(part_world_pose); // PickPart picks up the part and waits and 'temp_pose' above it

			// Step 3: Check for the quality of the part; Since this [bb_2_callback] initiates arm1 so check at QC1
			auto tempPose = part_world_pose; // todo: remove this intermediate pose; Makes code slow
			tempPose.position.z = part_world_pose.position.z + 0.13; // 'tempPose' -> pose the arm is at currently after pick up.
			bool isFaulty = checkFaultyArmOne(part_frame_name, tempPose);

			/* After Quality is checked the arm is just below the QC*/

			// Step 4: If not faulty, place it on the Bin 1
			if (!isFaulty){
				// Drop the part on the kit tray!
			}
		}
		else
			ROS_INFO_STREAM("[ASM]:[bb_2_callback]: Let this part go! ");
		part_list.pop_front();
	}
}

// Arm 1 check faulty: This is called inside bb_2_callback()
bool AriacSensorManager::checkFaultyArmOne(std::string frameName, const geometry_msgs::Pose& pose){

	/* @arg frameName: name of the part in hand
	 * @arg pose: current pose of that part(It's in the hand of the manipulator)
	 * Step 1: We move from arm1's current position to Quality Control sensor 1.
	 * Step 2: We hear for quality control sensor output
	 * @return bool Step 3: Return the status received.
	 */
	
	ros::AsyncSpinner spinner(0);
	spinner.start();
	ROS_INFO_STREAM("[RobotController]:[checkFaultyArmOne]: part being tested: " << frameName);

	// Step 1: Move the arm from the belt to the QC1
	arm1.SendRobotTo(go_transition_pose);
	ros::Duration(0.2).sleep();
	arm1.SendRobotTo(arm1_check_qc_pose);
	ros::Duration(0.2).sleep();

	// Step 2: Check for the quality of the part
	if (!qc_1_redFlag)
	    ROS_INFO("[ASM]:[checkFaultyArmOne]: Part is good!");
    else
        ROS_INFO("[ASM]:[checkFaultyArmOne]: FAULTY PART");

	return qc_1_redFlag;
}