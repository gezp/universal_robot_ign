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
        const unsigned int update_rate)
{
    // ROS and Ignition node
    nh_ = nh;
    ign_node_ = std::make_shared<ignition::transport::Node>();

    joint_names_ = joint_names;
    for (size_t i = 0; i < joint_names_.size(); i++) {
        joint_names_map_[joint_names_[i]]=i;
    }
    //create ros pub and sub
    ros_joint_state_pub_ = nh_->create_publisher<sensor_msgs::msg::JointState>(ros_topic, 10);
    auto period = std::chrono::microseconds(1000000 / update_rate);
    joint_state_timer_ = nh_->create_wall_timer(period, std::bind(&JointStatePublisher::jointStateTimerCb, this));
    //create ignition sub
    ign_node_->Subscribe(ign_topic, &JointStatePublisher::ignJointStateCb, this);
    //init current_joint_msg_
    for (auto i = 0u; i < joint_names_.size(); ++i) {
        current_joint_msg_.name.push_back(joint_names_[i]);
        current_joint_msg_.position.push_back(0);
        current_joint_msg_.velocity.push_back(0);
        current_joint_msg_.effort.push_back(0);
    }
}

void JointStatePublisher::jointStateTimerCb()
{
    ignition::msgs::Model model_msg;
    {
        std::lock_guard<std::mutex> lock(current_joint_msg_mut_);
        model_msg=current_ign_joint_msg_;
    }
    
    //create  JointState msg
    current_joint_msg_.header.stamp = rclcpp::Clock().now();
    current_joint_msg_.header.frame_id = model_msg.name();
    for(int i = 0; i < model_msg.joint_size () ; ++i){
        if (joint_names_map_.find(model_msg.joint(i).name()) != joint_names_map_.end()) {
            int idx=joint_names_map_[model_msg.joint(i).name()];
            current_joint_msg_.position[idx]=model_msg.joint(i).axis1().position();
            current_joint_msg_.velocity[idx]=model_msg.joint(i).axis1().velocity();
            current_joint_msg_.effort[idx]=model_msg.joint(i).axis1().force();
        }
    }
    ros_joint_state_pub_->publish(current_joint_msg_);
}

void JointStatePublisher::ignJointStateCb(const ignition::msgs::Model& msg)
{
    std::lock_guard<std::mutex> lock(current_joint_msg_mut_);
    current_ign_joint_msg_ = msg;
}