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
#include "universal_robot_ign/joint_position_controller.hpp"

using namespace std;
using namespace universal_robot_ign;

JointPositionController::JointPositionController(const rclcpp::Node::SharedPtr& nh,
    const std::vector<std::string>& joint_names,
    const std::string& ros_cmd_topic,
    const std::vector<std::string>& ign_cmd_topics)
{
    // ROS and Ignition node
    nh_ = nh;
    ign_node_ = std::make_shared<ignition::transport::Node>();
    //check
    if (joint_names.size() != ign_cmd_topics.size()) {
        std::cout << "[JointPositionController ERROR]:the size of arrays are not matched!" << std::endl;
        return;
    }
    joint_names_ = joint_names;
    //init joint names map
    for (size_t i = 0; i < joint_names_.size(); i++) {
        joint_names_map_[joint_names_[i]] = i;
    }
    //create ros pub and sub
    ros_cmd_joint_state_sub_ = nh_->create_subscription<sensor_msgs::msg::JointState>(ros_cmd_topic, 10,
        std::bind(&JointPositionController::setJointPositionCb, this, std::placeholders::_1));
    //create ignition pub
    for (size_t i = 0; i < ign_cmd_topics.size(); i++) {
        auto pub = std::make_shared<ignition::transport::Node::Publisher>(
            ign_node_->Advertise<ignition::msgs::Double>(ign_cmd_topics[i]));
        ign_cmd_joint_pubs_.push_back(pub);
    }
}

void JointPositionController::setJointPositionCb(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    for (auto i = 0u; i < msg->position.size(); ++i) {
        if (joint_names_map_.find(msg->name[i]) != joint_names_map_.end()) {
            //find joint name in `joint_names_` .
            int idx = joint_names_map_[msg->name[i]];
            ignition::msgs::Double ign_msg;
            ign_msg.set_data(msg->position[i]);
            ign_cmd_joint_pubs_[idx]->Publish(ign_msg);
        }
    }
}
