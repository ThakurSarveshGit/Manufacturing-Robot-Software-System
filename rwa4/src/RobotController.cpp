/*
 * Group 7 RWA 4
 * RobotController.cpp: Class definitions for RobotController Class
 * Definitions of all functions that control and manuever the arms.
 */

#include "RobotController.h"

// -- Constructor for RobotController class
RobotController::RobotController(std::string arm_id):
	robot_controller_nh_("/ariac/"+arm_id),
	robot_controller_options("manipulator","/ariac/"+arm_id+"/robot_description",robot_controller_nh_),
	robot_move_group_(robot_controller_options),
	armSpinner{0},
	arm_name{arm_id}{
    ROS_INFO("Constructor Called");
	ROS_WARN("[RobotController]: Constructor Called\n");

	robot_move_group_.setPlanningTime(20); // Take 20 seconds to plan
	robot_move_group_.setNumPlanningAttempts(10); // Take 10 attempts at max
	robot_move_group_.setPlannerId("RRTConnectkConfigDefault");
	robot_move_group_.setMaxVelocityScalingFactor(1);
	robot_move_group_.setMaxAccelerationScalingFactor(1);
	// robot_move_group_.setEndEffector("moveit_ee");
	robot_move_group_.allowReplanning(true);

	/*Joint Positions for Home Position
    home_joint_pose_ = {0.0, 3.1, -1.1, 1.9, 3.9, 4.7, 0};
	*/

    // Pose 1: Arm 1 Home position: Get in a picking position
    home_joint_pose_1["linear_arm_actuator_joint"] = 0.56; // -- An Li
    home_joint_pose_1["shoulder_pan_joint"] = 0;
    home_joint_pose_1["shoulder_lift_joint"] = -0.8;// -0.95
    home_joint_pose_1["elbow_joint"] = 1.7; // 1.6
    home_joint_pose_1["wrist_1_joint"] = -2.48; // -2.45
    home_joint_pose_1["wrist_2_joint"] = -1.57;
    home_joint_pose_1["wrist_3_joint"] = 0;

	// Pose 2: Arm 2 Home Position: Get over the Bin containing Gears
    home_joint_pose_2["linear_arm_actuator_joint"] = -0.56;
    home_joint_pose_2["shoulder_pan_joint"] = 2;
    home_joint_pose_2["shoulder_lift_joint"] = 0;
    home_joint_pose_2["elbow_joint"] = 0;
    home_joint_pose_2["wrist_1_joint"] = -3.14/2;
    home_joint_pose_2["wrist_2_joint"] = -3.14/2;
    home_joint_pose_2["wrist_3_joint"] = 0;

    // --offset used for picking up parts
    offset_ = 0.0175; // -- An Li:0.02

    // Subscribe to get the status of the gripper
    gripper_subscriber_ = gripper_nh_.subscribe("/ariac/arm1/gripper/state",10,&RobotController::GripperCallback, this);
    
    if (arm_id == "arm1")
    	SendRobotTo(home_joint_pose_1); // Send Arm1 to Pose 1
    else
    	SendRobotTo(home_joint_pose_2); // Send Arm2 to Pose 0
    

    // Get Pose of EE w.r.t Base of the Arm
    robot_tf_listener_.waitForTransform("arm1_linear_arm_actuator", "arm1_ee_link", ros::Time(0), ros::Duration(10));
    robot_tf_listener_.lookupTransform("/arm1_linear_arm_actuator", "/arm1_ee_link", ros::Time(0), robot_tf_transform_);

    fixed_orientation_.x = robot_tf_transform_.getRotation().x();
    fixed_orientation_.y = robot_tf_transform_.getRotation().y();
    fixed_orientation_.z = robot_tf_transform_.getRotation().z();
    fixed_orientation_.w = robot_tf_transform_.getRotation().w();

    tf::quaternionMsgToTF(fixed_orientation_, q);
    tf::Matrix3x3(q).getRPY(roll_def_,pitch_def_,yaw_def_);

    end_position_ = home_joint_pose_1;

    // Get World pose of Arm EE
    robot_tf_listener_.waitForTransform("world", "arm1_ee_link", ros::Time(0),
                                            ros::Duration(10));
    robot_tf_listener_.lookupTransform("/world", "/arm1_ee_link", ros::Time(0),
                                           robot_tf_transform_);   

    // Get Cartesian Pose of EE in world frame
    home_cart_pose_.position.x = robot_tf_transform_.getOrigin().x();
    home_cart_pose_.position.y = robot_tf_transform_.getOrigin().y();
    home_cart_pose_.position.z = robot_tf_transform_.getOrigin().z();
    home_cart_pose_.orientation.x = robot_tf_transform_.getRotation().x();
    home_cart_pose_.orientation.y = robot_tf_transform_.getRotation().y();
    home_cart_pose_.orientation.z = robot_tf_transform_.getRotation().z();
    home_cart_pose_.orientation.w = robot_tf_transform_.getRotation().w();


    // Get world pose of Kitray for AGV1
    agv_tf_listener_.waitForTransform("world", "kit_tray_1", ros::Time(0), ros::Duration(10));
    agv_tf_listener_.lookupTransform("/world", "/kit_tray_1", ros::Time(0), agv_tf_transform_);
    agv_position_.position.x = agv_tf_transform_.getOrigin().x();
    agv_position_.position.y = agv_tf_transform_.getOrigin().y();
    agv_position_.position.z = agv_tf_transform_.getOrigin().z() + 4 * offset_; // Higher Offset for Z-Coordinate

    gripper_client_ = robot_controller_nh_.serviceClient<osrf_gear::VacuumGripperControl>("/ariac/arm1/gripper/control");

    counter_ = 0;
    drop_flag_ = false;
    ROS_WARN("[RobotController]: Constructor Call Ended\n");
}

// -- Deconstructor
RobotController::~RobotController(){}

// -- Planner: Mutlithreading required?
bool RobotController::Planner(){
	ROS_INFO_STREAM("[RobotController]: [Planner]: Planning Started...");
	armSpinner.start(); // This should run in parallel
	
	if (robot_move_group_.plan(robot_planner_) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
		plan_success_ = true;
	else{
		plan_success_ = false;
		ROS_WARN_STREAM("[RobotController]: [Planner]: Planning Failed!\n");
	}
	return plan_success_;
}

// -- Execute(): Mutlithreading required?
void RobotController::Execute(){
	/* When you want to reach a known pose[with known joint angles], use Execute()*/
	armSpinner.start();

	// If planning was successful for the arm, execute
	if (this->Planner()){
		ROS_INFO_STREAM("[RobotController]:[Execute]: Move the arm");
		robot_move_group_.move(); // Move the arm
		ros::Duration(1.0).sleep(); // Giving some time to execute the arm
	}
}

// --GoToTarget(): Reach a Pose
void RobotController::GoToTarget(const geometry_msgs::Pose& pose){
	ROS_INFO_STREAM("[RobotController]:[GoToTarget[1]]: Function Called");
	target_pose_.orientation = fixed_orientation_; // Keep the EE pose as earlier; Dont Change
	target_pose_.position = pose.position; // Just reach the position
	armSpinner.start(); // Is it required? : 

	robot_move_group_.setPoseTarget(target_pose_);

	if (this->Planner()){
		ROS_INFO_STREAM("[RobotController]:[GoToTarget[1]]: Moving the arm");
		robot_move_group_.move();
		ros::Duration(1).sleep();
	}

	ROS_INFO_STREAM("[RobotController]:[GoToTarget[1]]: Position Reached");
}

// --GoToTarget(): Reach a series of poses
void RobotController::GoToTarget(std::initializer_list<geometry_msgs::Pose> list){
	ROS_INFO_STREAM("[RobotController]:[GoToTarget[2]]: Function Called");
	armSpinner.start(); // Is it required? : YESSSS

	std::vector<geometry_msgs::Pose> waypoints;
	
	for (auto i:list){
        i.orientation.x = fixed_orientation_.x;
        i.orientation.y = fixed_orientation_.y;
        i.orientation.z = fixed_orientation_.z;
        i.orientation.w = fixed_orientation_.w;
        waypoints.emplace_back(i);
	}
//	ROS_INFO_STREAM("[RobotController]:[GoToTarget[2]]: Debug: Created waypoints");

	moveit_msgs::RobotTrajectory traj;
	auto fraction = robot_move_group_.computeCartesianPath(waypoints, 0.01, 0.0, traj, true); // Get Trajectory and compute fraction(safety?)
//	ROS_INFO_STREAM("[RobotController]:[GoToTarget[2]]: Debug: Computed cartesian path");

	robot_planner_.trajectory_ = traj;
//	ROS_INFO_STREAM("[RobotController]:[GoToTarget[2]]: Debug: Trajectory planning completed");

	robot_move_group_.execute(robot_planner_); // Asking moveIt to move the arm to the final pose through the waypoints.
	ROS_INFO_STREAM("[RobotController]:[GoToTarget[2]]: Debug: Endpoint reached");

//	ros::Duration(1).sleep();
}

// --SendRobotTo(): Forward Kinematics: Set Joint Angles
void RobotController::SendRobotTo(std::map<std::string, double> desire_joint_states){
	robot_move_group_.setJointValueTarget(desire_joint_states);
    // this->execute();
    armSpinner.start(); // Is it required?
    if (this->Planner()) {
        robot_move_group_.move();
        ros::Duration(1).sleep();
    }
}

// -- GripperToggle(): Activate/Deactivate the gripper
void RobotController::GripperToggle(const bool& state){
	gripper_service_.request.enable = state;
	gripper_client_.call(gripper_service_);
//	ros::Duration(1).sleep();

	if (gripper_service_.response.success)
		ROS_INFO_STREAM("[RobotController]:[GripperToggle]: Gripper activated");
	else
		ROS_WARN_STREAM("[RobotController]:[GripperToggle]: Gripper activation failed!");
}

// --DropPart(): 
bool RobotController::DropPart(geometry_msgs::Pose part_pose){
	
	drop_flag_ = true;

	ros::spinOnce();
	ROS_WARN_STREAM("[RobotController]:[DropPart]: Function called");

	if (gripper_state_){
		ROS_INFO_STREAM("[RobotController]:[DropPart]: Moving towards AGV1 . . .");
		
		// Set Robot Joint Angles
		ROS_INFO_STREAM("[RobotController]:[DropPart]: Setting Joint Angles . . .");
		robot_move_group_.setJointValueTarget(end_position_);
		this->Execute();
		ros::Duration(1.0).sleep();

		// Actuate the gripper
		ROS_INFO_STREAM("[RobotController]:[DropPart]: Actuating gripper . . .");
		this->GripperToggle(false); // Release the part

		// Reach slightly above the part
		ROS_INFO_STREAM("[RobotController]:[DropPart]: Moving to temp_pose . . .");
		auto temp_pose = part_pose;
		temp_pose.position.z += 0.5;
		this->GoToTarget({temp_pose, part_pose});
		ros::Duration(1).sleep();
		ros::spinOnce();

		// . . .
		ROS_INFO_STREAM("[RobotController]:[DropPart]: Actuating Gripper . . .");
		this->GripperToggle(false);
		ros::spinOnce();

		// . . .
		if (!gripper_state_) {
			ROS_INFO_STREAM("[RobotController]:[DropPart]: Going to home position...");
			this->GoToTarget({temp_pose, home_cart_pose_});
			ros::Duration(3.0).sleep();
		}
	}

	drop_flag_ = false;
	ROS_WARN_STREAM("[RobotController]:[DropPart]: Function execution completed");
	return gripper_state_;
}

// -- GripperCallback ; Setter for current gripper state; 'true' if an object is attached.
void RobotController::GripperCallback(const osrf_gear::VacuumGripperState::ConstPtr& grip){
	gripper_state_ = grip->attached;
}


// -- PickPart() is called inside BB2 calback function in AriacSensorManager.
bool RobotController::PickPart(geometry_msgs::Pose& part_pose){

	ROS_INFO_STREAM("[RobotController]:[PickPart]: Recieved Instruction to Pick part!");
    ROS_INFO_STREAM("[RobotController]:[PickPart]: Setting temp_pose . . .");
    
    // Creating Way-points
    part_pose.position.z = part_pose.position.z + offset_; // Go above that part
    auto temp_pose_1 = part_pose;
    temp_pose_1.position.z += 0.2; // Again add an offset above it; create a waypoint

    // todo: Make a conditional block to increment/reduce temp_pose_1 z value if the part
    // is a pulley. Use Gazebo Pose info for pulley and other parts to determine this.
    
    // Step 1: Go above the part then come down on it
    ROS_INFO_STREAM("[RobotController]:[PickPart]: Calling GoToTarget(List)");
    this->GoToTarget({temp_pose_1, part_pose}); // First go to temp_pose then part_pose

    // Step 2: Activate the gripper
    ROS_INFO_STREAM("[RobotController]:[PickPart]: Activating gripper" << part_pose.position.z);
    this->GripperToggle(true);
    ros::spinOnce();
    ros::Duration(0.5).sleep();

    // If not able to pick up once, try again!!!
    while (!gripper_state_){
        ROS_INFO_STREAM("Trying to pick up the part . . .");
        this->GripperToggle(true);
        ros::spinOnce();
        ros::Duration(0.5).sleep();
    }

    // Step 3: Bring the part up
    this->GoToTarget({part_pose, temp_pose_1}); // Come up.
    ros::spinOnce();

    // That's it, RobotController::PickPart() function is complete.

    return gripper_state_;
}