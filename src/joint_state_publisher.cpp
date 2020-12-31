/*******************************************************************************
 *  Copyright (c) Gezp (https://github.com/gezp), All rights reserved.
 *
 *  This program is free software: you can redistribute it and/or modify it 
 *  under the terms of the MIT License, See the MIT License for more details.
 *
 *  You should have received a copy of the MIT License along with this program.
 *  If not, see <https://opensource.org/licenses/MIT/>.
 *
 ******************************************************************************/
#include "universal_robot_ign/joint_state_publisher.hpp"

using namespace std;
using namespace universal_robot_ign;

JointStatePublisher::JointStatePublisher(const rclcpp::Node::SharedPtr& nh,
        const std::vector<std::string>& joint_names,
        const std::string& ros_topic, 
        const std::string& ign_topic,
        const std::vector<int>& joint_idxs_map,
        const unsigned int update_rate)
{
    // ROS and Ignition node
    nh_ = nh;
    ign_node_ = std::make_shared<ignition::transport::Node>();
    //check
    if (joint_names.size() != joint_idxs_map.size()) {
        std::cout << "[JointStatePublisher ERROR]:the size of arrays are not matched!" << std::endl;
        return;
    }
    joint_names_ = joint_names;
    joint_idxs_map_ = joint_idxs_map;
    max_ign_joint_idx_ = 0;
    for (size_t i = 0; i < joint_idxs_map_.size(); i++) {
        if (joint_idxs_map_[i] > max_ign_joint_idx_) {
            max_ign_joint_idx_ = joint_idxs_map_[i];
        }
    }
    //create ros pub and sub
    ros_joint_state_pub_ = nh_->create_publisher<sensor_msgs::msg::JointState>(ros_topic, 10);
    auto period = std::chrono::microseconds(1000000 / update_rate);
    joint_state_timer_ = nh_->create_wall_timer(period, std::bind(&JointStatePublisher::jointStateTimerCb, this));
    //create ignition sub
    ign_node_->Subscribe(ign_topic, &JointStatePublisher::ignJointStateCb, this);
    //init current_joint_msg_
    for (auto i = 0u; i < joint_names_.size(); ++i) {
        auto newJoint = current_joint_msg_.add_joint();
        newJoint->set_name(joint_names_[i]);
        newJoint->mutable_axis1()->set_position(0);
        newJoint->mutable_axis1()->set_velocity(0);
        newJoint->mutable_axis1()->set_force(0);
    }
}

void JointStatePublisher::jointStateTimerCb()
{
    std::lock_guard<std::mutex> lock(current_joint_msg_mut_);
    //create  JointState msg
    sensor_msgs::msg::JointState ros_msg;
    ros_msg.header.stamp = rclcpp::Clock().now();
    ros_msg.header.frame_id = "robot";
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        ros_msg.name.push_back(joint_names_[i]);
        auto idx = joint_idxs_map_[i];
        ros_msg.position.push_back(current_joint_msg_.joint(idx).axis1().position());
        ros_msg.velocity.push_back(current_joint_msg_.joint(idx).axis1().velocity());
        ros_msg.effort.push_back(current_joint_msg_.joint(idx).axis1().force());
    }
    ros_joint_state_pub_->publish(ros_msg);
}

void JointStatePublisher::ignJointStateCb(const ignition::msgs::Model& msg)
{
    std::lock_guard<std::mutex> lock(current_joint_msg_mut_);
    //check
    if (msg.joint_size() <= max_ign_joint_idx_) {
        //std::cout << "the size of joint state from ignition is less idx of joint" << std::endl;
        return;
    }
    current_joint_msg_ = msg;
}