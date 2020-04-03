//
// Created by zeid on 2/27/20.
//

#pragma once

#include <unordered_map>
#include <string>
#include <cmath>
#include <set>
#include <queue>
#include <map>
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


class AriacSensorManager {
private:
    ros::NodeHandle sensor_nh_;

    // Subscriber
    ros::Subscriber lc_belt_sub; // Subscribe to the '/ariac/lc_belt' topic
    ros::Subscriber bb_1_sub; // Subscribe to the '/ariac/break_beam_1_change' topic
    ros::Subscriber bb_2_sub; // Subscribe to the '/ariac/break_beam_1_change' topic
    ros::Subscriber orders_sub; 
    ros::Subscriber lc_gear_sub; // Subscribe to the '/ariac/lc_gear' topic
    ros::Subscriber qc_1_sub; // Subscribe to the '/ariac/quality_control_sensor_2' topic
    ros::Subscriber lc_bin_1_sub; // Subscribe to the '/ariac/lc_gear' topic
    ros::Subscriber lc_agv_1_sub;
    ros::Subscriber lc_agv_1_sub_new; // Subscribe to the '/ariac/lc_agv_1'
    bool qc_1_redFlag;
    ros::Subscriber qc_2_sub; // Subscribe to the '/ariac/quality_control_sensor_2' topic
    bool qc_2_redFlag;

    // Order/Product/Pose containers
    std::vector<osrf_gear::Order> received_orders_;
    unsigned int order_number;
    geometry_msgs::Pose partPoseOnKitTray1; // Used to pick up this part using Arm1
    std::unordered_map<std::string, geometry_msgs::Pose> belt_part_map; // map for checked part from the belt
    std::unordered_map<std::string, geometry_msgs::Pose> gear_bin_map; // map for checked part in the bin
    std::queue<std::pair<std::string, std::string>> incoming_partQ; // the queue store (part_type, part_frame_name) from the belt
    std::pair<std::string, std::string> popped_incoming_part;
    std::unordered_map<std::string, unsigned int> belt_part_counter; // map which calculate # of part_type from belt
    // std::multiset<std::string> desired_parts; // mulitset for desired parts in current order
    std::multimap<std::string, geometry_msgs::Pose> desired_parts_info; // mulitset for desired parts in current order
    std::unordered_map<std::string, size_t> task; // task to be done for each oreder
    // std::vector<geometry_msgs::Pose> gear_bin_vector;
    std::vector<std::pair<std::string, geometry_msgs::Pose>> gear_bin_vector;
    bool order_receiving_flag;
    std::set<std::string> parts_to_pickup_belt;


    // Robot related
    RobotController arm1;
    RobotController arm2;
    std::map<std::string, double> arm2_check_qc_pose;
    std::map<std::string, double> arm2_transition_pose;
    // std::map<std::string, double> go_transition_pose;
    // std::map<std::string, double> back_transition_pose;
    std::map<std::string, double> arm1_bin_pose;
    std::map<std::string, double> arm1_check_qc_pose;

    bool arm1_busy;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;

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
    void gear_check(const osrf_gear::LogicalCameraImage::ConstPtr& image_msg);
    void pick_part_from_belt(std::pair<std::string, std::string>);
    void setDesiredParts();
    void lc_bin_1_callback(const osrf_gear::LogicalCameraImage::ConstPtr&);
    void grab_bin1(const osrf_gear::LogicalCameraImage::ConstPtr&);
//    void lc_agv_1_callback(const osrf_gear::LogicalCameraImage::ConstPtr &);
    void lc_agv_1_callback_new(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg);
    void grab_gear();
};

