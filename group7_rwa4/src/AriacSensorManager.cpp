
#include <tf2/LinearMath/Quaternion.h>
#include "AriacSensorManager.h"
using namespace std;

AriacSensorManager::AriacSensorManager() :
        task {},
        arm1 {"arm1"},
        arm2 {"arm2"},
        tfListener(tfBuffer)
{
    orders_sub = sensor_nh_.subscribe("/ariac/orders", 10,
                                      & AriacSensorManager::order_callback, this);

    bb_1_sub = sensor_nh_.subscribe("/ariac/break_beam_1_change", 10,
                                    & AriacSensorManager::bb_1_callback, this);

    bb_2_sub = sensor_nh_.subscribe("/ariac/break_beam_2_change", 10,
                                    & AriacSensorManager::bb_2_callback, this);

    lc_bin_1_sub = sensor_nh_.subscribe("/ariac/lc_bin_1", 10,
                                        & AriacSensorManager::lc_bin_1_callback, this);

    lc_agv_1_sub = sensor_nh_.subscribe("/ariac/lc_agv_1", 10,
                                         & AriacSensorManager::lc_agv_1_callback, this);
    
    order_number = 0;

    qc_1_redFlag = false;
    qc_2_redFlag = false;
    order_receiving_flag = false;
    arm1_busy = false;

    arm2_check_qc_pose["linear_arm_actuator_joint"] = -1.1;
    arm2_check_qc_pose["shoulder_pan_joint"] = 4.6;
    arm2_check_qc_pose["elbow_joint"] = 0;
    arm2_check_qc_pose["shoulder_lift_joint"] = 0;
    arm2_check_qc_pose["wrist_1_joint"] = -1.57;
    arm2_check_qc_pose["wrist_2_joint"] = -3.14/2;

    arm2_transition_pose["linear_arm_actuator_joint"] = -0.56;
    arm2_transition_pose["shoulder_pan_joint"] = 3.14;
    arm2_transition_pose["shoulder_lift_joint"] = -3.14/2;
    arm2_transition_pose["elbow_joint"] = 3.14/2;
    arm2_transition_pose["wrist_1_joint"] = -1.5;
    arm2_transition_pose["wrist_2_joint"] = -3.14/2;

    arm1_bin_pose["linear_arm_actuator_joint"] = 0.25;
    arm1_bin_pose["shoulder_pan_joint"] = 2.13;
    arm1_bin_pose["shoulder_lift_joint"] = -0.55;
    arm1_bin_pose["elbow_joint"] = 1.1;
    arm1_bin_pose["wrist_1_joint"] = 4.15;
    arm1_bin_pose["wrist_2_joint"] = -1.57;

    arm1_check_qc_pose["linear_arm_actuator_joint"] = 1.18;
    arm1_check_qc_pose["shoulder_pan_joint"] = 1.44;
    arm1_check_qc_pose["elbow_joint"] = 1.28;
    arm1_check_qc_pose["shoulder_lift_joint"] = -0.5;
    arm1_check_qc_pose["wrist_1_joint"] = 3.94;
    arm1_check_qc_pose["wrist_2_joint"] = -1.57;

    // arm1_trans_pose["linear_arm_actuator_joint"] = -0.56;
    // arm1_trans_pose["shoulder_pan_joint"] = 3.14;
    // arm1_trans_pose["shoulder_lift_joint"] = -3.14/2;
    // arm1_trans_pose["elbow_joint"] = 3.14/2;
    // arm1_trans_pose["wrist_1_joint"] = -1.5;
    // arm1_trans_pose["wrist_2_joint"] = -3.14/2;

}

AriacSensorManager::~AriacSensorManager() {}

void AriacSensorManager::order_callback(const osrf_gear::Order::ConstPtr & order_msg) {
    ROS_INFO_STREAM("[order_callback]:Received order:\n" << *order_msg);
    received_orders_.push_back(*order_msg);
    setDesiredParts();
    order_receiving_flag = true;
    lc_gear_sub = sensor_nh_.subscribe("/ariac/lc_gear", 10,
                                       & AriacSensorManager::lc_gear_callback, this);
}

void AriacSensorManager::setDesiredParts(){
    ROS_INFO_STREAM("[setDesiredParts]:Setting desired parts");
    auto current_order = received_orders_[order_number];
    auto order_id = current_order.order_id;
    auto shipments = current_order.shipments;
    for (const auto &shipment: shipments){
        auto shipment_type = shipment.shipment_type;
        auto products = shipment.products;
        ROS_INFO_STREAM("Order ID: " << order_id);
        ROS_INFO_STREAM("Shipment Type: " << shipment_type);
        for (const auto &product: products){
            desired_parts_info.insert({product.type, product.pose});
            parts_to_pickup_belt.insert(product.type);
            ++(task[product.type]); // Adding product name in task variable
        }
    }

    ROS_INFO_STREAM("[setDesiredParts]:The current desired_parts are:");
    for (const auto & part : desired_parts_info){
        std::cout << part.first << std::endl;
        std::cout << "Pose:\n";
        ROS_INFO_STREAM(part.second);
    }

    ROS_INFO_STREAM("[ASM]:[setDesiredParts]:The Parts_to_pickup_belt set is:");
    for (auto it = parts_to_pickup_belt.begin(); it != parts_to_pickup_belt.end(); ++it)
        std::cout << *it << "\n";

    if (!order_id.empty())
        ++order_number;
}

void AriacSensorManager::lc_gear_callback(const osrf_gear::LogicalCameraImage::ConstPtr& image_msg){
    if (image_msg->models.size() == 0){
        ROS_WARN_THROTTLE(5, "[lc_gear_callback]: lc_gear does not detect things");
        return;
    }
    else{
        parts_to_pickup_belt.erase("gear_part");
    }

    ros::AsyncSpinner spinner(4);
    spinner.start();

    ROS_INFO_STREAM_THROTTLE(5, "[lc_gear_callback]: '" << image_msg->models.size() << "' gears.");
    if (order_receiving_flag){
        lc_gear_sub.shutdown();
        gear_check(image_msg);
    }
}

void AriacSensorManager::gear_check(const osrf_gear::LogicalCameraImage::ConstPtr& image_msg){
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::Duration timeout(0.2);

    size_t desired_gear_num = task["gear_part"]; // count number of gears needed
    size_t gear_counter = image_msg->models.size(); // for labeling number of gear part
    for (auto & msg : image_msg->models){
        geometry_msgs::TransformStamped transformStamped;
        string camera_frame = "lc_gear_" + msg.type + "_" + to_string(gear_counter) + "_frame";
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

            if (task["gear_part"] != 0){
                bool if_pick = arm2.PickPart(part_pose);
                if (if_pick) {
                    /// ---------checking mechanism --------------------
                    qc_2_sub = sensor_nh_.subscribe("/ariac/quality_control_sensor_2", 10,
                                        & AriacSensorManager::qc_2_callback, this);
                    arm2.SendRobotTo(arm2_transition_pose);
                    arm2.SendRobotTo(arm2_check_qc_pose);
                    arm2.GripperToggle(false); // release gripper

                    transformStamped = tfBuffer.lookupTransform("world", "arm2_ee_link", ros::Time(0), timeout);
                    geometry_msgs::Pose ee2_pose;
                    ee2_pose.position.x = transformStamped.transform.translation.x;
                    ee2_pose.position.y = transformStamped.transform.translation.y;
                    ee2_pose.position.z = transformStamped.transform.translation.z;
                    ee2_pose.position.z -= 0.275;  

                    arm2.SendRobotTo("shoulder_pan_joint", 4.3);
                    ros::Duration(0.5).sleep();
                    qc_2_sub.shutdown();
                    arm2.SendRobotTo("shoulder_pan_joint", 4.6);

                    bool if_pick_tray = arm2.PickPart(ee2_pose);
                    ROS_INFO_STREAM("qc_2_redFlag = " << qc_2_redFlag);
                    while (!if_pick_tray) {
                        ee2_pose.position.z -= 0.05;
                        if_pick_tray = arm2.PickPart(ee2_pose);
                    }
                    /// ---------checking mechanism --------------------
                    if (qc_2_redFlag) {
                        // if the one below the camera is a faulty one
                        ROS_WARN_STREAM("[gear_check]: QC 2 detected faulty part, ready to dispose....");
                        arm2.SendRobotTo("shoulder_pan_joint", 3.9);
                        arm2.GripperToggle(false);
                        arm2.SendRobotTo(arm2_transition_pose);
                        arm2.RobotGoHome();
                    }
                    else {
                        // if the one below the camera is a good one
                        arm2.SendRobotTo(arm2_transition_pose);
                        arm2.RobotGoHome();
                        bool attach = arm2.DropPart(part_pose);
                        // If part is dropped now, push it in gear_bin_vector
                        if (!attach){
                            gear_bin_vector.push_back({camera_frame.substr(8), part_pose}); // Storing the pose of the good gears inside gear_bin_vector
                            --(task["gear_part"]);
                            ROS_INFO_STREAM("Need to grab '" << task["gear_part"] << "' gear");
                        }
                    }
                    qc_2_redFlag = false;
                }
            }
            --gear_counter;
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s\n",ex.what());
        }
    }
    arm2.RobotGoHome();
    ros::Duration(0.2).sleep();
    arm2.SendRobotTo(arm2_transition_pose);
    // arm2.SendRobotTo("linear_arm_actuator_joint", -1);
    ros::Duration(1).sleep();
    // ROS_INFO("[gear_check]: Called grab_gear()");
    grab_gear();
}

void AriacSensorManager::qc_2_callback(const osrf_gear::LogicalCameraImage::ConstPtr& image_msg){
    ros::AsyncSpinner spinner(4);
    spinner.start();

    if (image_msg->models.size() == 0){
        qc_2_redFlag = false;
    }
    else {
        ROS_WARN_STREAM_ONCE("[qc_2_callback] detecte faulty part");
        qc_2_redFlag = true;
    }
}

void AriacSensorManager::bb_1_callback(const osrf_gear::Proximity::ConstPtr & msg) {
    ros::AsyncSpinner spinner(4);
    spinner.start();
    if (msg->object_detected){
        lc_belt_sub = sensor_nh_.subscribe("/ariac/lc_belt", 10,
            & AriacSensorManager::lc_belt_callback, this);
    }
}

void AriacSensorManager::lc_belt_callback(const osrf_gear::LogicalCameraImage::ConstPtr& image_msg){
    if (image_msg->models.size() == 0) return;

    ros::AsyncSpinner spinner(4);
    spinner.start();

    ros::Duration timeout(0.2);

    for (auto & msg : image_msg->models){
        geometry_msgs::TransformStamped transformStamped;

        // find if this type of part has passed by before
        if (belt_part_counter.find(msg.type) == belt_part_counter.end())
            belt_part_counter.insert(make_pair(msg.type, 0));
        else
            belt_part_counter[msg.type]++;

        string camera_frame = "lc_belt_" + msg.type + "_" +
                              to_string(belt_part_counter[msg.type]) + "_frame";
        try{
            transformStamped = tfBuffer.lookupTransform("world", camera_frame, ros::Time(0), timeout);
            std::pair<std::string, std::string> part_pair {msg.type, camera_frame.substr(8)};
            incoming_partQ.push(part_pair);
            geometry_msgs::Pose part_pose;
            part_pose.position.x = transformStamped.transform.translation.x;
            // part_pose.position.y = transformStamped.transform.translation.y;
            part_pose.position.y = 1.65;
            part_pose.position.z = transformStamped.transform.translation.z;
            part_pose.orientation.x = transformStamped.transform.rotation.x;
            part_pose.orientation.y = transformStamped.transform.rotation.y;
            part_pose.orientation.z = transformStamped.transform.rotation.z;
            part_pose.orientation.w = transformStamped.transform.rotation.w;
            belt_part_map.insert({camera_frame.substr(8), part_pose});
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s\n",ex.what());
            belt_part_counter[msg.type]--;
        }
        break;
    }
    lc_belt_sub.shutdown();
}

void AriacSensorManager::bb_2_callback(const osrf_gear::Proximity::ConstPtr & msg) {
    ros::AsyncSpinner spinner(4);
    spinner.start();
    if (msg->object_detected) {
        auto incoming_part = incoming_partQ.front(); incoming_partQ.pop();
        ROS_INFO_STREAM("[bb_2_callback]: Break beam triggered. The part is '" << incoming_part.first << "'");
        if (!arm1_busy) {
//            if ((task[incoming_part.first] != 0) &&
//                (incoming_part.first == "piston_rod_part"))
            auto search = parts_to_pickup_belt.find(incoming_part.first);
            if ( (task[incoming_part.first] != 0) &&  search != parts_to_pickup_belt.end())
            {
                ROS_INFO_STREAM("[bb_2_callback]: Pick this part!");
                arm1_busy = true;
                pick_part_from_belt(incoming_part);
            }
        }
        else {
            ROS_INFO_STREAM("[bb_2_callback]: Arm1 is busy...");
        }
    }
}

// void AriacSensorManager::pick_part_from_belt(pair<string, string> incoming_part){
// //    ros::AsyncSpinner spinner(4);
// //    spinner.start();
//     // ROS_INFO_STREAM("[pick_part_from_belt]: Inside pick_part_from_belt");
//     popped_incoming_part = incoming_part;
//     auto part_pose = belt_part_map[incoming_part.second];

//     bool if_pick = arm1.PickPart(part_pose);
//     if (if_pick){
//         qc_1_sub = sensor_nh_.subscribe("/ariac/quality_control_sensor_1", 1,
//                                         & AriacSensorManager::qc_1_callback, this);
//         arm1.SendRobotTo(arm1_check_qc_pose);
//         qc_1_sub.shutdown();
//         if (qc_1_redFlag) { // if the one below the camera is a faulty one
//             ROS_INFO_STREAM("[pick_part_from_belt]: QC 1 detected bad shit, ready to dispose....");
//             arm1_check_qc_pose["shoulder_pan_joint"] = 2.32;
//             arm1.SendRobotTo(arm1_check_qc_pose);
//             arm1.GripperToggle(false);
//             // lc_bin_1_sub = sensor_nh_.subscribe("/ariac/lc_bin_1", 10,
//             //     & AriacSensorManager::lc_bin_1_callback, this);
//         }
//         else { // if the one below the camera is a good one
//             arm1.SendRobotTo(arm1_bin_pose);
//             arm1.GripperToggle(false);
//             --(task[incoming_part.first]);
//         }
//         arm1.RobotGoHome();
//     }
//     arm1_busy = false;
// }

void AriacSensorManager::pick_part_from_belt(pair<string, string> incoming_part){
    // ros::AsyncSpinner spinner(4);
    // spinner.start();
    popped_incoming_part = incoming_part;
    auto part_pose = belt_part_map[incoming_part.second];

    bool if_pick = arm1.PickPart(part_pose);
    if (if_pick){

        /// ---------checking mechanism --------------------
        qc_1_sub = sensor_nh_.subscribe("/ariac/quality_control_sensor_1", 10,
                                        & AriacSensorManager::qc_1_callback, this);
        arm1.SendRobotTo(arm1_check_qc_pose);
        arm1.GripperToggle(false);
        ros::Duration timeout(0.2);

        geometry_msgs::TransformStamped transformStamped;
        transformStamped = tfBuffer.lookupTransform("world", "arm1_ee_link", ros::Time(0), timeout);
        geometry_msgs::Pose ee1_pose;
        ee1_pose.position.x = transformStamped.transform.translation.x;
        ee1_pose.position.y = transformStamped.transform.translation.y;
        ee1_pose.position.z = transformStamped.transform.translation.z;
        ee1_pose.position.z -= 0.15;  

        arm1.SendRobotTo("shoulder_pan_joint", 1.6);
        ros::Duration(0.5).sleep();
        qc_1_sub.shutdown();
        arm1.SendRobotTo("shoulder_pan_joint", 1.44);

        bool if_pick_tray = arm1.PickPart(ee1_pose);
        ROS_INFO_STREAM("qc_1_redFlag = " << qc_1_redFlag);
        while (!if_pick_tray) {
            ee1_pose.position.z -= 0.015;
            if_pick_tray = arm1.PickPart(ee1_pose);
        }
        /// ---------checking mechanism --------------------


        if (qc_1_redFlag) { // if the one below the camera is a faulty one
            ROS_INFO_STREAM("QC 1 detected a faulty part, ready to dispose....");
            arm1.SendRobotTo("shoulder_pan_joint", 2.32);
            arm1.GripperToggle(false);
        }
        else { // if the one below the camera is a good one
            arm1.SendRobotTo(arm1_bin_pose);
            arm1.GripperToggle(false);
            --(task[incoming_part.first]);
            parts_to_pickup_belt.erase(incoming_part.first);

        }
        qc_1_redFlag = false;
        arm1.RobotGoHome();
    }
    arm1_busy = false;
}

void AriacSensorManager::qc_1_callback(const osrf_gear::LogicalCameraImage::ConstPtr& image_msg){
    if (image_msg->models.size() == 0)
        qc_1_redFlag = false;
    else{
        ROS_WARN_STREAM("[qc_1_callback]: faulty part detected");
        qc_1_redFlag = true;
    }
}

void AriacSensorManager::lc_bin_1_callback(const osrf_gear::LogicalCameraImage::ConstPtr& image_msg){
    if (image_msg->models.size() == 0)
        return;

    ros::AsyncSpinner spinner(4);
    spinner.start();
    ROS_INFO_STREAM_THROTTLE(5, "[lc_bin_1_callback]: lc_bin captures '" << image_msg->models.size() << "' item(s).");
    if (!arm1_busy)
        grab_bin1(image_msg);
}

void AriacSensorManager::grab_bin1(const osrf_gear::LogicalCameraImage::ConstPtr& image_msg){
    ros::AsyncSpinner spinner(4);
    spinner.start();

    ros::Duration timeout(0.2);

    size_t part_counter = 0; // for labeling number of gear part // --- Issue

    for (auto & msg : image_msg->models){
        geometry_msgs::TransformStamped transformStamped;
        string frame_name = popped_incoming_part.second;
        string bin_part_camera_frame = "lc_bin_1_" + frame_name;
        try{
            transformStamped = tfBuffer.lookupTransform("world", bin_part_camera_frame, ros::Time(0), timeout);
            geometry_msgs::Pose part_pose;
            part_pose.position.x = transformStamped.transform.translation.x;
            part_pose.position.y = transformStamped.transform.translation.y;
            part_pose.position.z = transformStamped.transform.translation.z;
            part_pose.orientation.x = transformStamped.transform.rotation.x;
            part_pose.orientation.y = transformStamped.transform.rotation.y;
            part_pose.orientation.z = transformStamped.transform.rotation.z;
            part_pose.orientation.w = transformStamped.transform.rotation.w;

            if (!arm1_busy && (desired_parts_info.find(msg.type) != desired_parts_info.end())) {
                arm1_busy = true;
                arm1.SendRobotTo(arm1_bin_pose);
                bool if_pick = arm1.PickPart(part_pose);
                if (if_pick) {
                    ///------------- Get orientation of the bin in kit_tray frame -----------
                    geometry_msgs::PoseStamped StampedPose_in, StampedPose_out;
                    StampedPose_in.header.frame_id = "world";
                    StampedPose_in.pose = part_pose;
                    StampedPose_out = tfBuffer.transform(StampedPose_in, "kit_tray_1");
                    geometry_msgs::Pose part_pose_kit = StampedPose_out.pose;

                    tf2::Quaternion quat_tf;
                    tf2::fromMsg(part_pose_kit.orientation, quat_tf);
                    double part_R, part_P, part_Y;
                    tf2::Matrix3x3(quat_tf).getRPY(part_R, part_P, part_Y);

                    // ROS_INFO_STREAM("Part roll in kit_tray_frame = " << part_R);
                    // ROS_INFO_STREAM("Part pitch in kit_tray_frame = " << part_P);
                    // ROS_INFO_STREAM("Part yall in kit_tray_frame= " << part_Y);

//                    transformStamped = tfBuffer.lookupTransform("kit_tray_1", "arm1_ee_link", ros::Time(0),
//                                                               timeout);
//                    geometry_msgs::Pose ee_pose;
//                    ee_pose.orientation.x = transformStamped.transform.rotation.x;
//                    ee_pose.orientation.y = transformStamped.transform.rotation.y;
//                    ee_pose.orientation.z = transformStamped.transform.rotation.z;
//                    ee_pose.orientation.w = transformStamped.transform.rotation.w;
//
//                    double ee_R, ee_P, ee_Y;
//                    tf2::Quaternion quat_tf_ee;
//                    tf2::fromMsg(ee_pose.orientation, quat_tf_ee);
//                    tf2::Matrix3x3(quat_tf_ee).getRPY(ee_R, ee_P, ee_Y);

                    // ROS_INFO_STREAM("EE roll in kit_tray_frame = " << ee_R);
                    // ROS_INFO_STREAM("EE pitch in kit_tray_frame = " << ee_P);
                    // ROS_INFO_STREAM("EE yall in kit_tray_frame= " << ee_Y);

                    //double part_ee_ang_diff = part_Y - ee_Y;
                    ///----------modification end ----------------------
                    arm1.SendRobotTo(arm1_bin_pose);
                    try {
                        auto itr = desired_parts_info.find(msg.type);
                        auto drop_pose_ = itr->second;
                        geometry_msgs::PoseStamped StampedPose_in, StampedPose_out;
                        StampedPose_in.header.frame_id = "kit_tray_1";
                        StampedPose_in.pose = drop_pose_;
                        StampedPose_out = tfBuffer.transform(StampedPose_in, "world");
                        auto drop_pose = StampedPose_out.pose;
                        drop_pose.position.z += 0.05;
                        ROS_INFO_STREAM("[grab_bin1()======= World frame drop pose***************************** ]:" << drop_pose);
                        //--------Get Desired RPY from kit_tray
                        tf2::Quaternion quat_D;
                        tf2::fromMsg(drop_pose_.orientation, quat_D);
                        double d_R, d_P, d_Y;
                        tf2::Matrix3x3(quat_D).getRPY(d_R, d_P, d_Y);

                        arm1.GoToTarget1(drop_pose);
                        ros::Duration(0.5).sleep();
                        //-----------------//
                        geometry_msgs::PoseStamped StampedPose_in_curr, StampedPose_out_curr;
                        StampedPose_in_curr.header.frame_id = "lc_agv_1_frame";
                        StampedPose_in_curr.pose = qt_pose;
                        StampedPose_out_curr = tfBuffer.transform(StampedPose_in_curr, "world");
                        auto drop_pose_curr = StampedPose_out_curr.pose;

                        tf2::Quaternion quat_D_curr;
                        tf2::fromMsg(drop_pose_curr.orientation, quat_D_curr);
                        double d_R_curr, d_P_curr, d_Y_curr;
                        tf2::Matrix3x3(quat_D_curr).getRPY(d_R_curr, d_P_curr, d_Y_curr);
                        ROS_INFO_STREAM("================ curr_yaw ============="<<d_Y_curr);   // -2.72

                        //double comp = d_Y -  part_ee_ang_diff;
                        //double diff_yaw = d_Y - d_Y_curr;
                        //ROS_INFO_STREAM("================ ================================  diff   ========================== ============="<< diff_yaw);   // -2.76

                        transformStamped = tfBuffer.lookupTransform("world", "arm1_ee_link", ros::Time(0),
                                                                    timeout);
                        geometry_msgs::Pose ee_pose;
                        ee_pose.orientation.x = transformStamped.transform.rotation.x;
                        ee_pose.orientation.y = transformStamped.transform.rotation.y;
                        ee_pose.orientation.z = transformStamped.transform.rotation.z;
                        ee_pose.orientation.w = transformStamped.transform.rotation.w;

                        double ee_R, ee_P, ee_Y;
                        tf2::Quaternion quat_tf_ee;
                        tf2::fromMsg(ee_pose.orientation, quat_tf_ee);
                        tf2::Matrix3x3(quat_tf_ee).getRPY(ee_R, ee_P, ee_Y);

                        double eediff= d_Y_curr - ee_Y;

                        double req = d_Y - eediff;

                        ROS_INFO_STREAM("================  diff ============="<< eediff);

                        //to edit for orientation
                        tf2:: Quaternion myQuat;
                        myQuat.setRPY(0,0,req);
                        std::cout << "Checking..................................." << std::endl;
                        ROS_INFO_STREAM("[grab_bin1()][inside function myQuat ============================= ]:" << *myQuat << *(myQuat+1) << *(myQuat+2) << *(myQuat+3));

                        geometry_msgs::Pose tmp;
                        tmp.orientation.x = *myQuat;
                        tmp.orientation.y = *(myQuat+1);
                        tmp.orientation.z = *(myQuat+2);
                        tmp.orientation.w = *(myQuat+3);

                        geometry_msgs::PoseStamped StampedPose_in1, StampedPose_out1;
                        StampedPose_in1.header.frame_id = "world";
                        StampedPose_in1.pose = tmp;
                        StampedPose_out1 = tfBuffer.transform(StampedPose_in, "arm1_wrist_3_link");
                        geometry_msgs::Pose poseKit = StampedPose_out1.pose;
//
                        tf2::Quaternion quat_tf1;
                        tf2::fromMsg(poseKit.orientation, quat_tf1);
                        double part_R1, part_P1, part_Y1;
                        tf2::Matrix3x3(quat_tf1).getRPY(part_R1, part_P1, part_Y1);
//
//                        ROS_INFO_STREAM("Part roll in kit_tray_frame = " << part_R1);
//                        ROS_INFO_STREAM("Part pitch in kit_tray_frame = " << part_P1);
//                        ROS_INFO_STREAM("Part yell in kit_tray_frame= " << part_Y1);

                        ROS_INFO_STREAM("[grab_bin1() inside function pose ***************************** ]:" << tmp);
                        arm1.SendRobotTo("wrist_3_joint", part_Y1);
                        //ros::Duration(1).sleep();
                        arm1.GripperToggle(false);


                        // to check
                        //geometry_msgs::PoseStamped StampedPose_in_curr, StampedPose_out_curr;
                        StampedPose_in_curr.header.frame_id = "lc_agv_1_frame";
                        StampedPose_in_curr.pose = qt_pose;
                        StampedPose_out_curr = tfBuffer.transform(StampedPose_in_curr, "world");
                        drop_pose_curr = StampedPose_out_curr.pose;

                        ROS_INFO_STREAM("[grab_bin1() =====:========= [CHECKING THE FINAL POSE]==========***************************** ]:" << drop_pose_curr);
                        ///--------------------------------------------------------
                        // bool attach = arm1.DropPart(drop_pose);
                        arm1.RobotGoHome();
                        desired_parts_info.erase(itr);
                    }
                    catch (tf2::TransformException &ex) {
                        ROS_WARN("%s\n", ex.what());
                    }
                }
                arm1_busy = false;
                ++part_counter;
            }
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s\n",ex.what());
        }
    }
}

 void AriacSensorManager::lc_agv_1_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){
     ros::AsyncSpinner spinner(4);
     spinner.start();

     if (image_msg->models.size() == 0)
         return;
     //ROS_INFO_STREAM_THROTTLE(5, "[lc_agv_1_callback]: lc_agv_1 captures '" << image_msg->models.size() << "' item(s).");
     ros::Duration timeout(0.2);
     size_t part_counter = 0;
     auto img = *image_msg;
     //this->qt = img.models[0].pose.orientation;
     this->qt_pose = img.models[0].pose;
     if (image_msg->models.size() > 1)
         this->qt_pose = img.models[1].pose;


     ROS_INFO_STREAM_THROTTLE(5,"[lc_agv_1_callback]: -------------------  " << this->qt_pose.orientation);

//     for (auto & msg : image_msg->models){
//         geometry_msgs::TransformStamped transformStamped;
//         string camera_frame = "lc_agv_1_" + msg.type + "_" + to_string(part_counter) + "_frame";
//         try{
//             transformStamped = tfBuffer.lookupTransform("world", camera_frame, ros::Time(0), timeout);
//             geometry_msgs::Pose part_pose;
//             part_pose.position.x = transformStamped.transform.translation.x;
//             part_pose.position.y = transformStamped.transform.translation.y;
//             part_pose.position.z = transformStamped.transform.translation.z;
//             part_pose.orientation.x = transformStamped.transform.rotation.x;
//             part_pose.orientation.y = transformStamped.transform.rotation.y;
//             part_pose.orientation.z = transformStamped.transform.rotation.z;
//             part_pose.orientation.w = transformStamped.transform.rotation.w;
//
//             geometry_msgs::PoseStamped StampedPose_in, StampedPose_out;
//             StampedPose_in.header.frame_id = "world";
//             StampedPose_in.pose = part_pose;
//             StampedPose_out = tfBuffer.transform(StampedPose_in, "kit_tray_1");
//             auto agv_pose = StampedPose_out.pose;
//
//             ROS_INFO_STREAM("[lc_agv_1_callback]: Part pose on the tray in kit_tray_1 frame is:\n" << part_pose);
//         }
//         catch (tf2::TransformException &ex) {
//             ROS_WARN("%s\n",ex.what());
//         }
     // }
 }

void AriacSensorManager::grab_gear(){
    ros::AsyncSpinner spinner(4);
    spinner.start();

    geometry_msgs::TransformStamped transformStamped;
    ros::Duration timeout(0.2);

    size_t part_counter = 0;
    for (auto & msg : gear_bin_vector){
        try{
            geometry_msgs::Pose part_pose = msg.second;
            if (!arm1_busy && (desired_parts_info.find("gear_part") != desired_parts_info.end())){
                arm1_busy = true;
                arm1.SendRobotTo(arm1_bin_pose);
                arm1.SendRobotTo("linear_arm_actuator_joint",-1.2);

                bool if_pick = arm1.PickPart(part_pose);
                if (if_pick) {
                    // get part pose in kit_tray_frame
                    geometry_msgs::PoseStamped StampedPose_in, StampedPose_out;
                    StampedPose_in.header.frame_id = "world";
                    StampedPose_in.pose = part_pose;
                    StampedPose_out = tfBuffer.transform(StampedPose_in, "kit_tray_1");
                    geometry_msgs::Pose part_pose_kit = StampedPose_out.pose;

                    // get part RPY in kit_tray_frame
                    tf2::Quaternion quat_tf;
                    tf2::fromMsg(part_pose_kit.orientation, quat_tf);
                    double part_R, part_P, part_Y;
                    tf2::Matrix3x3(quat_tf).getRPY(part_R, part_P, part_Y);

                    // get ee pose in kit_tray_frame
                    transformStamped = tfBuffer.lookupTransform("kit_tray_1", "arm1_ee_link", ros::Time(0),
                                                               timeout);
                    geometry_msgs::Pose ee_pose;
                    ee_pose.orientation.x = transformStamped.transform.rotation.x;
                    ee_pose.orientation.y = transformStamped.transform.rotation.y;
                    ee_pose.orientation.z = transformStamped.transform.rotation.z;
                    ee_pose.orientation.w = transformStamped.transform.rotation.w;

                    // get ee RPY in kit_tray_frame
                    double ee_R, ee_P, ee_Y;
                    tf2::Quaternion quat_tf_ee;
                    tf2::fromMsg(ee_pose.orientation, quat_tf_ee);
                    tf2::Matrix3x3(quat_tf_ee).getRPY(ee_R, ee_P, ee_Y);

                    double part_ee_ang_diff = part_Y - ee_Y;
                    ///----------modification end ----------------------

                    arm1.SendRobotTo(arm1_bin_pose);
                    try{
                        auto itr = desired_parts_info.find("gear_part");
                        auto drop_pose_ = itr->second;
                        geometry_msgs::PoseStamped StampedPose_in, StampedPose_out;
                        StampedPose_in.header.frame_id = "kit_tray_1";
                        StampedPose_in.pose = drop_pose_;
                        StampedPose_out = tfBuffer.transform(StampedPose_in, "world");
                        auto drop_pose = StampedPose_out.pose;
                        drop_pose.position.z += 0.05;

                        ///--------Get Desired RPY from kit_tray
                        tf2::Quaternion quat_D;
                        tf2::fromMsg(drop_pose_.orientation, quat_D);
                        double d_R, d_P, d_Y;
                        tf2::Matrix3x3(quat_D).getRPY(d_R, d_P, d_Y);   

                        arm1.GoToTarget1(drop_pose);
                        ros::Duration(0.5).sleep();

                        // ======================================================================== //
                        geometry_msgs::PoseStamped StampedPose_in_curr, StampedPose_out_curr;
                        StampedPose_in_curr.header.frame_id = "lc_agv_1_frame";
                        StampedPose_in_curr.pose = qt_pose;
                        StampedPose_out_curr = tfBuffer.transform(StampedPose_in_curr, "world");
                        auto drop_pose_curr = StampedPose_out_curr.pose;

                        tf2::Quaternion quat_D_curr;
                        tf2::fromMsg(drop_pose_curr.orientation, quat_D_curr);
                        double d_R_curr, d_P_curr, d_Y_curr;
                        tf2::Matrix3x3(quat_D_curr).getRPY(d_R_curr, d_P_curr, d_Y_curr);
                        ROS_INFO_STREAM("================ curr_yaw ============="<<d_Y_curr);   // -2.72

                        transformStamped = tfBuffer.lookupTransform("world", "arm1_ee_link", ros::Time(0),
                                                                    timeout);
                        geometry_msgs::Pose ee_pose;
                        ee_pose.orientation.x = transformStamped.transform.rotation.x;
                        ee_pose.orientation.y = transformStamped.transform.rotation.y;
                        ee_pose.orientation.z = transformStamped.transform.rotation.z;
                        ee_pose.orientation.w = transformStamped.transform.rotation.w;

                        double ee_R, ee_P, ee_Y;
                        tf2::Quaternion quat_tf_ee;
                        tf2::fromMsg(ee_pose.orientation, quat_tf_ee);
                        tf2::Matrix3x3(quat_tf_ee).getRPY(ee_R, ee_P, ee_Y);

                        double eediff= d_Y_curr - ee_Y;

                        double req = d_Y - eediff;

                        ROS_INFO_STREAM("================ ================================  diff   ========================== ============="<< eediff);

                        //to edit for orientation
                        tf2:: Quaternion myQuat;
                        myQuat.setRPY(0,0,req);
                        std::cout << "Checking..................................." << std::endl;
                        ROS_INFO_STREAM("[grab_bin1()][inside function myQuat ============================= ]:" << *myQuat << *(myQuat+1) << *(myQuat+2) << *(myQuat+3));

                        geometry_msgs::Pose tmp;
                        tmp.orientation.x = *myQuat;
                        tmp.orientation.y = *(myQuat+1);
                        tmp.orientation.z = *(myQuat+2);
                        tmp.orientation.w = *(myQuat+3);

                        geometry_msgs::PoseStamped StampedPose_in1, StampedPose_out1;
                        StampedPose_in1.header.frame_id = "world";
                        StampedPose_in1.pose = tmp;
                        StampedPose_out1 = tfBuffer.transform(StampedPose_in, "arm1_wrist_3_link");
                        geometry_msgs::Pose poseKit = StampedPose_out1.pose;
//
                        tf2::Quaternion quat_tf1;
                        tf2::fromMsg(poseKit.orientation, quat_tf1);
                        double part_R1, part_P1, part_Y1;
                        tf2::Matrix3x3(quat_tf1).getRPY(part_R1, part_P1, part_Y1);
                        // ======================================================================== //
                        arm1.SendRobotTo("wrist_3_joint", part_Y1);
                        ros::Duration(1).sleep();
                        arm1.GripperToggle(false);

                        // bool attach = arm1.DropPart(drop_pose);
                        desired_parts_info.erase(itr);
                    }
                    catch (tf2::TransformException &ex) {
                        // ROS_INFO_STREAM("[grab_gear]: grab_bin1 error");
                        ROS_WARN("%s\n",ex.what());
                    }
                }
                arm1_busy = false;
            }
            ++part_counter;
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s\n",ex.what());
        }
    }
    arm1.RobotGoHome();
}