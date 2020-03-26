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
#include "PickUp.h"

// -- Initiating Classes
PickUp pickUpObject; //-- Creating Object to access class PickUp

//-- Global Variables
std::vector<std::string> orderPartList; // For Order Message -> Parts
std::vector<std::string> beltPartList; // Belt Parts
int counter{0}; // Index of part to pick up from the conveyor
clock_t currentTime; // -- Variables to access time for calibration
clock_t duration; // -- Duration of time a part takes from crossing between B1 and B2
int firstPartBreakBeam{0}; // -- 
int break_beam_counter = 0;

bool gripper_state_;
bool grab_now = false;

// Function Definitions
// -- Function 1: Start the competition by waiting for and then calling the start ROS Service.
void start_competition(ros::NodeHandle & node) {
  // Create a Service client for the correct service, i.e. '/ariac/start_competition'.
  ros::ServiceClient start_client = node.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
  // If it's not already ready, wait for it to be ready.
  // Calling the Service using the client before the server is ready would fail.
  if (!start_client.exists()) {
    ROS_INFO("Waiting for the competition to be ready...");
    start_client.waitForExistence();
    ROS_INFO("Competition is now ready.");
  }
  ROS_INFO("Requesting competition start...");
  std_srvs::Trigger srv;  // Combination of the "request" and the "response".
  start_client.call(srv);  // Call the start Service.
  if (!srv.response.success) {  // If not successful, print out why.
    ROS_ERROR_STREAM("Failed to start the competition: " << srv.response.message);
  }
  else {
    ROS_INFO("Competition started!");
  }
}

/// Example class that can hold state and provide methods that handle incoming data.
class MyCompetitionClass
{
public:
  explicit MyCompetitionClass(ros::NodeHandle & node)
  : current_score_(0), arm_1_has_been_zeroed_(false), arm_2_has_been_zeroed_(false)
  {
    // %Tag(ADV_CMD)%
//    arm_1_joint_trajectory_publisher_ = node.advertise<trajectory_msgs::JointTrajectory>(
//      "/ariac/arm1/arm/command", 10);
//
    arm_2_joint_trajectory_publisher_ = node.advertise<trajectory_msgs::JointTrajectory>(
      "/ariac/arm2/arm/command", 10);
//    // %EndTag(ADV_CMD)%
  }

  /// Called when a new message is received.
  void current_score_callback(const std_msgs::Float32::ConstPtr & msg) {
    if (msg->data != current_score_)
    {
      ROS_INFO_STREAM("Score: " << msg->data);
    }
    current_score_ = msg->data;
  }

  /// Called when a new message is received.
  void competition_state_callback(const std_msgs::String::ConstPtr & msg) {
    if (msg->data == "done" && competition_state_ != "done")
    {
      ROS_INFO("Competition ended.");
    }
    competition_state_ = msg->data;
  }

  /// Called when a new Order message is received.
  // -- Making Changes
  void order_callback(const osrf_gear::Order::ConstPtr & order_msg) {
    received_orders_.push_back(*order_msg);
    auto order_details = *order_msg ; 

    for (auto shipment :order_msg->shipments){
    	for (auto product:shipment.products){
    		// std::cout << "Part of Order:" << product.type << std::endl;
    		orderPartList.push_back(product.type);
    	}
    }
  }

  /// Called when a new JointState message is received.
  void arm_1_joint_state_callback(
    const sensor_msgs::JointState::ConstPtr & joint_state_msg){
    ROS_INFO_STREAM_THROTTLE(10,
      "Joint States arm 1 (throttled to 0.1 Hz):\n" << *joint_state_msg);
    // ROS_INFO_STREAM("Joint States:\n" << *joint_state_msg);
    arm_1_current_joint_states_ = *joint_state_msg;
    if (!arm_1_has_been_zeroed_) {
      arm_1_has_been_zeroed_ = true;
      ROS_INFO("Sending arm to zero joint positions...");
      send_arm_to_zero_state(arm_1_joint_trajectory_publisher_);
    }
  }

  void arm_2_joint_state_callback(
    const sensor_msgs::JointState::ConstPtr & joint_state_msg)
  {
    ROS_INFO_STREAM_THROTTLE(10,
      "Joint States arm 2 (throttled to 0.1 Hz):\n" << *joint_state_msg);
    // ROS_INFO_STREAM("Joint States:\n" << *joint_state_msg);
    arm_2_current_joint_states_ = *joint_state_msg;
    if (!arm_2_has_been_zeroed_) {
      arm_2_has_been_zeroed_ = true;
      ROS_INFO("Sending arm 2 to zero joint positions...");
      send_arm_to_zero_state(arm_2_joint_trajectory_publisher_);
    }
  }

  /// Create a JointTrajectory with all positions set to zero, and command the arm.
  void send_arm_to_zero_state(ros::Publisher & joint_trajectory_publisher) {
    // Create a message to send.
    trajectory_msgs::JointTrajectory msg;
    // Fill the names of the joints to be controlled.
    // Note that the vacuum_gripper_joint is not controllable.
    msg.joint_names.clear();
    msg.joint_names.push_back("shoulder_pan_joint");
    msg.joint_names.push_back("shoulder_lift_joint");
    msg.joint_names.push_back("elbow_joint");
    msg.joint_names.push_back("wrist_1_joint");
    msg.joint_names.push_back("wrist_2_joint");
    msg.joint_names.push_back("wrist_3_joint");
    msg.joint_names.push_back("linear_arm_actuator_joint");
    // Create one point in the trajectory.
    msg.points.resize(1);
    // Resize the vector to the same length as the joint names.
    // Values are initialized to 0.
    msg.points[0].positions.resize(msg.joint_names.size(), 0.0);
    // How long to take getting to the point (floating point seconds).
    msg.points[0].time_from_start = ros::Duration(0.001);
    ROS_INFO_STREAM("Sending command:\n" << msg);
    joint_trajectory_publisher.publish(msg);
  }

  /// Called when a new LogicalCameraImage message is received. (For camera1)
  void logical_camera_callback_1(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){
    ROS_INFO_STREAM_THROTTLE(10,
      "Logical camera: '" << image_msg->models.size() << "' objects.");
  }

  /// Called when a new LogicalCameraImage message is received. (For camera4)
  void logical_camera_callback_4(
    const osrf_gear::LogicalCameraImage::ConstPtr & image_msg)
  {
    ROS_INFO_STREAM_THROTTLE(10,
      "Logical camera: '" << image_msg->models.size() << "' objects.");
  }

  /// Logical Camera 5 - Placed over the Conveyer Belt
	void logical_camera_callback_5(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){

      auto imageMessage = *image_msg ;
  	
  	if (imageMessage.models.size() != 0){
  	    if (beltPartList.size() == 0){
  	    	  beltPartList.push_back(imageMessage.models[0].type);
  	    	  counter++;
  	    	  if ((std::find(orderPartList.begin(), orderPartList.end(), imageMessage.models[0].type ) != orderPartList.end())){
  //	    	     std::cout <<imageMessage.models[0].type << "is part of order at index: " << counter << " on belt" << std::endl;
  	    	      if (!(pickUpObject.busyFlag)){
  	    	          pickUpObject.pickUpNumber = counter;
  //	    	         std::cout << "Pick Up task Assigned! Part at: " << pickUpObject.pickUpNumber << std::endl;
  	    	          pickUpObject.busyFlag = true; // Arm gets occupied now.
  	    	      }
  	    	  }
  	    }
  	    if (beltPartList.back() != imageMessage.models[0].type){
  	    		beltPartList.push_back(imageMessage.models[0].type);
  	    		counter++;
  	    		if (std::find(orderPartList.begin(), orderPartList.end(), imageMessage.models[0].type ) != orderPartList.end()){
  //	    		    std::cout <<imageMessage.models[0].type << "is part of order at index: " << counter << " on belt" << std::endl;
  	    	        if (!(pickUpObject.busyFlag)){
  	    	            pickUpObject.pickUpNumber = counter;
  //	    	          std::cout << "Pick Up task Assigned! Part at: " << pickUpObject.pickUpNumber << std::endl;
  	    	            pickUpObject.busyFlag = true; // Arm gets occupied now.
  	    	        }
  	    		}
  	    }
  	}

  	std::cout << "Size of BeltPartList: " << beltPartList.size() << std::endl;
  	for (auto i:beltPartList){
  		std::cout << i << std::endl;
  	}

	}

  /// Called when a new Proximity message is received.
  void break_beam_callback_1(const osrf_gear::Proximity::ConstPtr & msg) {
    if (msg->object_detected) {  // If there is an object in proximity.
      ROS_INFO("Break beam 1 triggered.");
    }
  }
  

private:
  std::string competition_state_;
  double current_score_;
  ros::Publisher arm_1_joint_trajectory_publisher_;
  ros::Publisher arm_2_joint_trajectory_publisher_;
  std::vector<osrf_gear::Order> received_orders_;
  sensor_msgs::JointState arm_1_current_joint_states_;
  sensor_msgs::JointState arm_2_current_joint_states_;
  bool arm_1_has_been_zeroed_;
  bool arm_2_has_been_zeroed_;
};

void proximity_sensor_callback(const sensor_msgs::Range::ConstPtr & msg) {
  if ((msg->max_range - msg->range) > 0.01) {  // If there is an object in proximity.
    ROS_INFO_THROTTLE(1, "Proximity sensor sees something.");
  }
}

void laser_profiler_callback(const sensor_msgs::LaserScan::ConstPtr & msg) {
  size_t number_of_valid_ranges = std::count_if(
    msg->ranges.begin(), msg->ranges.end(), [](const float f) {return std::isfinite(f);});
  if (number_of_valid_ranges > 0) {
    ROS_INFO_THROTTLE(1, "Laser profiler sees something.");
  }
}
/// Called when a new Proximity message is received.
void break_beam_callback_2(const osrf_gear::Proximity::ConstPtr & msg) {
    if (msg->object_detected) {  // If there is an object in proximity.
        ROS_INFO("Break beam 2 triggered.");
        break_beam_counter += 1;
        grab_now = true;
     }
}

ros::ServiceClient gripper_client_;

/// Callback function for gripper actions
void GripperCallback(
        const osrf_gear::VacuumGripperState::ConstPtr& grip) {
    gripper_state_ = grip->attached;
}

/// Function to toggle the gripper activation
void GripperToggle(const bool& state) {
    osrf_gear::VacuumGripperControl gripper_service_;
    gripper_service_.request.enable = state;
    gripper_client_.call(gripper_service_);

    ROS_INFO_STREAM("State is : " << gripper_client_.call(gripper_service_));
    ros::Duration(1.0).sleep();
    //     if (gripper_client_.call(gripper_service_)) {
    if (gripper_service_.response.success) {
        ROS_INFO_STREAM("Gripper activated!");
    } else {
        ROS_WARN_STREAM("Gripper activation failed!");
    }
}


int main(int argc, char ** argv) {
   // Last argument is the default name of the node.
  ros::init(argc, argv, "listener"); //-- Changed the node name to 'listener'
  ros::NodeHandle node;

  ros::NodeHandle node_("/ariac/arm1");
  ros::NodeHandle gripper_nh_;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::MoveGroupInterface::Options loadOptions("manipulator","/ariac/arm1/robot_description",node_);
  moveit::planning_interface::MoveGroupInterface move_group(loadOptions);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface::Plan planner_;

  // Instance of custom class from above.
  MyCompetitionClass comp_class(node);
  // Subscribe to the '/ariac/current_score' topic.
  ros::Subscriber current_score_subscriber = node.subscribe("/ariac/current_score", 10,&MyCompetitionClass::current_score_callback, &comp_class);
  // Subscribe to the '/ariac/competition_state' topic.
  ros::Subscriber competition_state_subscriber = node.subscribe("/ariac/competition_state", 10,&MyCompetitionClass::competition_state_callback, &comp_class);
  // %Tag(SUB_CLASS)%
  // Subscribe to the '/ariac/orders' topic.
  ros::Subscriber orders_subscriber = node.subscribe("/ariac/orders", 10,&MyCompetitionClass::order_callback, &comp_class);
  // %EndTag(SUB_CLASS)%

  // Subscribe to the '/ariac/joint_states' topic.
  //  ros::Subscriber arm_1_joint_state_subscriber = node.subscribe("/ariac/arm1/joint_states", 10, &MyCompetitionClass::arm_1_joint_state_callback, &comp_class);
  //  ros::Subscriber arm_2_joint_state_subscriber = node.subscribe("/ariac/arm2/joint_states", 10, &MyCompetitionClass::arm_2_joint_state_callback, &comp_class);

  // %Tag(SUB_FUNC)%
  // Subscribe to the '/ariac/proximity_sensor_1' topic.
  ros::Subscriber proximity_sensor_subscriber = node.subscribe("/ariac/proximity_sensor_1", 10, proximity_sensor_callback);
  // %EndTag(SUB_FUNC)%
  // Subscribe to the '/ariac/break_beam_1_change' topic.
  ros::Subscriber break_beam_subscriber_1 = node.subscribe("/ariac/break_beam_1_change", 10, &MyCompetitionClass::break_beam_callback_1, &comp_class);
  // Subscribe to the '/ariac/break_beam_2_change' topic.
  //  ros::Subscriber break_beam_subscriber_2 = node.subscribe("/ariac/break_beam_2_change", 10, &MyCompetitionClass::break_beam_callback_2, &comp_class);
  
  // Subscribe to the '/ariac/logical_camera_1' topic.
  ros::Subscriber logical_camera_subscriber_1 = node.subscribe("/ariac/logical_camera_1", 10, &MyCompetitionClass::logical_camera_callback_1, &comp_class);
  // Add an extra subsciber node to logical_camera_4
  ros::Subscriber logical_camera_subscriber_4 = node.subscribe("/ariac/logical_camera_4", 10, &MyCompetitionClass::logical_camera_callback_4, &comp_class);
  // Add an extra subsciber node to logical_camera_5
  ros::Subscriber logical_camera_subscriber_5 = node.subscribe("/ariac/logical_camera_5", 10, &MyCompetitionClass::logical_camera_callback_5, &comp_class);
  // Subscribe to the '/ariac/laser_profiler_1' topic.
  ros::Subscriber laser_profiler_subscriber = node.subscribe("/ariac/laser_profiler_1", 10, laser_profiler_callback);

  ROS_INFO("Setup complete.");
  ros::Subscriber gripper_subscriber_;

  gripper_subscriber_ = gripper_nh_.subscribe(
            "/ariac/arm1/gripper/state", 10, GripperCallback);
  gripper_client_ = node_.serviceClient<osrf_gear::VacuumGripperControl>(
            "/ariac/arm1/gripper/control");
  // Subscribe to the '/ariac/break_beam_2_change' topic.
  ros::Subscriber break_beam_subscriber_2 = node_.subscribe(
            "/ariac/break_beam_2_change", 10,
            break_beam_callback_2);

  // We can print the name of the reference frame for this robot.
  ROS_INFO_NAMED("Lecture5", "Planning frame: %s", move_group.getPlanningFrame().c_str());
  // We can also print the name of the end-effector link for this group.
  ROS_INFO_NAMED("Lecture5", "End effector link: %s", move_group.getEndEffectorLink().c_str());

  geometry_msgs::Pose target_pose1;
  geometry_msgs::Pose temp_pose1;

  //  move_group setup
  move_group.setPlanningTime(20);
  move_group.setNumPlanningAttempts(10);
  move_group.setPlannerId("RRTConnectkConfigDefault");
  move_group.setMaxVelocityScalingFactor(0.9);
  move_group.setMaxAccelerationScalingFactor(0.9);
  // robot_move_group_.setEndEffector("moveit_ee");
  move_group.allowReplanning(true);
  /// starting the competition
  start_competition(node);

  std::vector<double> home_joint_pose_;

  home_joint_pose_ = {0.0, 0, 0, 0, 0, 0, 0};
  move_group.setJointValueTarget(home_joint_pose_);

  ROS_INFO_STREAM("Planning started...");
  bool plan_success_;
  if (move_group.plan(planner_) ==
        moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        plan_success_ = true;
        ROS_INFO_STREAM("Planner succeeded!");
  } else {
        plan_success_ = false;
        ROS_WARN_STREAM("Planner failed!");
  }

  if (plan_success_) {
        move_group.move();
        ros::Duration(0.5).sleep();
  }
  //  ros::Duration(2.0).sleep();

  // temporary pose
  std::map<std::string, double> temp_joint_pose_;
  temp_joint_pose_["shoulder_pan_joint"] = 0;
  temp_joint_pose_["shoulder_lift_joint"] = -0.5;
  temp_joint_pose_["elbow_joint"] = 0.5;
  temp_joint_pose_["wrist_1_joint"] = 0;
  temp_joint_pose_["wrist_2_joint"] = 0;
  temp_joint_pose_["wrist_3_joint"] = 0;
  temp_joint_pose_["linear_arm_actuator_joint"] = 0;

  move_group.setJointValueTarget(temp_joint_pose_);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan1;

  bool success = (move_group.plan(my_plan1) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  move_group.move();
  ros::Duration(0.5).sleep();

  // target pose
  target_pose1.orientation.w = 0.707;
  target_pose1.orientation.y = 0.707;
  target_pose1.position.x = 1.22;
  target_pose1.position.y = 0.7;
  target_pose1.position.z = 0.95;

  move_group.setPoseTarget(target_pose1);

  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking move_group
  // to actually move the robot.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success1 = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success1 ? "" : "FAILED");

  move_group.move();
//  ros::Duration(1.0).sleep();

  geometry_msgs::Pose grab_pose;
  geometry_msgs::Pose throw_away_pose;

  //    grab_pose.orientation.w = 0.707;
  //    grab_pose.orientation.y = 0.707;
  //
  //    grab_pose.position.x = 1.22;
  //    grab_pose.position.y = 0.7;
  //    grab_pose.position.z = 0.938;

  grab_pose = target_pose1;
  throw_away_pose = target_pose1;
  grab_pose.position.z = 0.928;

  while (ros::ok()){
        std::cout << "pickUpObject: " << pickUpObject.pickUpNumber << std::endl;
        std::cout << "break_beam_counter: " << break_beam_counter << std::endl;
        std::cout << "pickup flag: " << pickUpObject.busyFlag << std::endl;

        if(grab_now && pickUpObject.pickUpNumber==break_beam_counter && pickUpObject.busyFlag == true){
            ROS_INFO("Now grabbing.");
            move_group.setPoseTarget(grab_pose);
            moveit::planning_interface::MoveGroupInterface::Plan grab_plan;

            bool success12 = (move_group.plan(grab_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success12 ? "" : "FAILED");
            move_group.move();
            // ros::Duration(0.1).sleep();
            GripperToggle(true);
            grab_now = false;
            pickUpObject.busyFlag = false;

            if (gripper_state_){
                ROS_INFO("gripper attached, will go back now.");


				throw_away_pose.position.z = 1.2;
				move_group.setPoseTarget(throw_away_pose);
				moveit::planning_interface::MoveGroupInterface::Plan throw_plan;

				bool success34 = (move_group.plan(throw_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
				ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success34 ? "" : "FAILED");
				move_group.move();


				throw_away_pose.position.x = 0.25;
				throw_away_pose.position.y = 1;

                move_group.setPoseTarget(throw_away_pose);

                success34 = (move_group.plan(throw_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
                ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success34 ? "" : "FAILED");

                move_group.move();
                ros::Duration(0.5).sleep();
                ROS_INFO("going back.");

                GripperToggle(false);
            }

            throw_away_pose = target_pose1;
            throw_away_pose.position.z =1.2;

            move_group.setPoseTarget(throw_away_pose);
            moveit::planning_interface::MoveGroupInterface::Plan back_plan;

            bool success54 = (move_group.plan(back_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success54 ? "" : "FAILED");

            move_group.move();


            move_group.setPoseTarget(target_pose1);
//            moveit::planning_interface::MoveGroupInterface::Plan back_plan;

            success54 = (move_group.plan(back_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success54 ? "" : "FAILED");

            move_group.move();
            ros::Duration(1.0).sleep();

            grab_now = true;
        }
  }
  ros::spin();  // This executes callbacks on new data until ctrl-c.
  return 0;
}

