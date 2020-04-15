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

#include <tf/transform_listener.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> 
#include <tf2_ros/buffer.h>
#include <osrf_gear/LogicalCameraImage.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <osrf_gear/Proximity.h>
#include <osrf_gear/Order.h>
#include "RobotController.h"
#include <algorithm>

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
    ros::Subscriber lc_bin_2_sub; // Subscribe to the '/ariac/lc_gear' topic
    ros::Subscriber lc_bin_3_sub; // Subscribe to the '/ariac/lc_gear' topic
    ros::Subscriber lc_bin_4_sub; // Subscribe to the '/ariac/lc_gear' topic
    ros::Subscriber lc_bin_5_sub; // Subscribe to the '/ariac/lc_gear' topic
    ros::Subscriber lc_bin_6_sub; // Subscribe to the '/ariac/lc_gear' topic

    bool qc_1_redFlag;
    ros::Subscriber qc_2_sub; // Subscribe to the '/ariac/quality_control_sensor_2' topic
    bool qc_2_redFlag;

    // Order/Product/Pose containers
    std::vector<osrf_gear::Order> received_orders_;
    std::vector<osrf_gear::Order> updated_received_orders_;
    unsigned int order_number;
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

    osrf_gear::LogicalCameraImage current_parts_1_;
    osrf_gear::LogicalCameraImage current_parts_2_;
    osrf_gear::LogicalCameraImage current_parts_3_;
    osrf_gear::LogicalCameraImage current_parts_4_;
    osrf_gear::LogicalCameraImage current_parts_5_;
    osrf_gear::LogicalCameraImage current_parts_6_;

    bool  cam_1_, cam_2_,cam_3_, cam_4_, cam_5_,cam_6_;
    bool Flag_updateKit;
    int order_counter;
    int camera1_frame_counter_, camera2_frame_counter_, camera3_frame_counter_;
    int camera4_frame_counter_, camera5_frame_counter_, camera6_frame_counter_;
    std::map<std::string, std::vector<std::string>> product_frame_list_;

    int NumPartsToRemove, NumPartsToModify, NumPartsToAdd;

    tf::TransformListener camera_tf_listener_;
    tf::StampedTransform camera_tf_transform_;

    std::pair<std::string,geometry_msgs::Pose> product_type_pose_;
//    std::vector<geometry_msgs::Pose> empty_vector;
    std::map<std::string, std::vector<geometry_msgs::Pose>> order_update_product_type_pose_;
    std::map<std::string, std::vector<geometry_msgs::Pose>> order_update_copy;

    std::map<std::string, std::vector<geometry_msgs::Pose>> built_kit_product_type_pose_;
    std::map<std::string, std::vector<geometry_msgs::Pose>> parts_to_remove_product_type_pose_;

    RobotController* this_arm;
    RobotController* that_arm;




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

    void lc_bin_1_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg);
    void lc_bin_2_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg);
    void lc_bin_3_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg);
    void lc_bin_4_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg);
    void lc_bin_5_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg);
    void lc_bin_6_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg);

    void BuildProductFrames(int camera_id);
    geometry_msgs::Pose GetPartPose(const std::string& src_frame,
                                                        const std::string& target_frame);
//    bool PickAndPlace(const std::pair<std::string,geometry_msgs::Pose> product_type_pose, int agv_id, RobotController& arm);
    std::string GetProductFrame(std::string product_type);
    void ExecuteOrder();
    void UpdateKit(int agv_id);
    void PickAndThrow(geometry_msgs::Pose part_pose, std::string product_type, RobotController& arm);
    void CorrectPose(geometry_msgs::Pose current_pose, geometry_msgs::Pose updated_pose,
                                         std::string product_type, int agv_id, RobotController& arm);
    bool QualityCheck(int agv_id);
    void buildUpdatedKitMap();
    void WhatToRemove();
    void WhatToModify();
    void WhatToAdd();

    void removeParts(int agv_id, RobotController& arm);
    void modifyPose(int agv_id);
    void addParts(int agv_id);

    void ReExecute(int agv_id);
    void SubmitAGV(int num);

    geometry_msgs::Pose kitToWorld(geometry_msgs::Pose part_pose, int agv_id);
//    bool PickAndPlace(const std::pair<std::string, geometry_msgs::Pose>, int agv_id, RobotController& arm);
    bool PickAndPlace(const std::pair<std::string, geometry_msgs::Pose> product_type_pose,
                                          int agv_id, RobotController& arm);

//    void grab_bin1(const osrf_gear::LogicalCameraImage::ConstPtr&);
//    void lc_agv_1_callback(const osrf_gear::LogicalCameraImage::ConstPtr &);
//    void grab_gear();

    bool init_, everything_ready;
};

