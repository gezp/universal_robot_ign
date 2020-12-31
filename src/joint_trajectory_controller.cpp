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
#include "universal_robot_ign/joint_trajectory_controller.hpp"

using namespace std;
using namespace universal_robot_ign;

jointTrajectoryController::jointTrajectoryController(const rclcpp::Node::SharedPtr& nh,
    const std::vector<std::string>& joint_names,
    const std::string& ros_cmd_topic,
    const std::vector<std::string>& ign_cmd_topics,
    const unsigned int update_rate)
{
    // ROS and Ignition node
    nh_ = nh;
    ign_node_ = std::make_shared<ignition::transport::Node>();
    //check
    if (joint_names.size() != ign_cmd_topics.size()) {
        std::cout << "[jointTrajectoryController ERROR]:the size of arrays are not matched!" << std::endl;
        return;
    }
    joint_names_ = joint_names;
    joint_num_=joint_names_.size();
    //init joint_names_map_ and target_positions_
    for (size_t i = 0; i < joint_num_; i++) {
        joint_names_map_[joint_names_[i]] = i;
        target_positions_.push_back(0);
    }
    //create ros pub and sub
    ros_cmd_joint_trajectory_sub_ = nh_->create_subscription<trajectory_msgs::msg::JointTrajectory>(ros_cmd_topic, 10,
        std::bind(&jointTrajectoryController::setJointTrajectoryCb, this, std::placeholders::_1));
    auto period = std::chrono::microseconds(1000000 / update_rate);
    update_position_timer_ = nh_->create_wall_timer(period, std::bind(&jointTrajectoryController::updatePositionTimerCb, this));
    //create ignition pub
    for (size_t i = 0; i < ign_cmd_topics.size(); i++) {
        auto pub = std::make_shared<ignition::transport::Node::Publisher>(
            ign_node_->Advertise<ignition::msgs::Double>(ign_cmd_topics[i]));
        ign_cmd_joint_pubs_.push_back(pub);
    }
}

void jointTrajectoryController::updatePositionTimerCb()
{
    std::lock_guard<std::mutex> lock(trajectory_mut_);
    //wait trajectory
    if (!has_trajectory_) {
        return;
    }
    //check index of trajectory points
    if (trajectory_index_ >= points_.size()) {
        has_trajectory_ = false;
        return;
    }
    // trajectory roll-out based on time, this is not the most efficient way to set things
    // joint position interpolation (simple linear interpolation)
    if (trajectory_index_ < points_.size() - 1) {
        rclcpp::Time current_time = rclcpp::Clock().now();
        rclcpp::Duration cur_time_from_start = current_time - trajectory_start_time_;
        rclcpp::Duration time_from_start = points_[trajectory_index_].time_from_start;
        rclcpp::Duration next_time_from_start = points_[trajectory_index_ + 1].time_from_start;
        double c = (cur_time_from_start - time_from_start).seconds() / (next_time_from_start - time_from_start).seconds();
        c = c > 1 ? 1 : c;
        for (size_t i = 0; i < joint_names_.size(); ++i) {
            target_positions_[i] = (1 - c) * points_[trajectory_index_].positions[i] + c * points_[trajectory_index_ + 1].positions[i];
        }
        // increment to next trajectory point
        if (cur_time_from_start >= next_time_from_start) {
            trajectory_index_++;
        }
    } else {
        for (size_t i = 0; i < joint_names_.size(); ++i) {
            target_positions_[i] = points_[trajectory_index_].positions[i];
        }
        trajectory_index_++;
    }
    //publish control msg to ignition
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        ignition::msgs::Double ign_msg;
        ign_msg.set_data(target_positions_[i]);
        ign_cmd_joint_pubs_[i]->Publish(ign_msg);
    }
}

void jointTrajectoryController::setJointTrajectoryCb(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{
    //check
    if (msg->joint_names.size() < joint_names_.size()) {
        return;
    }
    for (size_t k = 0; k < joint_names_.size(); k++) {
        if (joint_names_[k] != msg->joint_names[k]) {
            return;
        }
    }
    //get points
    {
        std::lock_guard<std::mutex> lock(trajectory_mut_);

        auto chain_size = static_cast<unsigned int>(joint_names_.size());
        auto points_size = static_cast<unsigned int>(msg->points.size());
        //std::cout<<"get trajectory msg:"<<points_size<<std::endl;
        points_.resize(points_size);
        for (unsigned int i = 0; i < points_size; ++i) {
            points_[i].positions.resize(chain_size);
            points_[i].time_from_start = msg->points[i].time_from_start;
            for (unsigned int j = 0; j < chain_size; ++j) {
                points_[i].positions[j] = msg->points[i].positions[j];
            }
        }
        // trajectory start time
        trajectory_start_time_ = rclcpp::Clock().now();
        has_trajectory_ = true;
        trajectory_index_ = 0;
    }
}
