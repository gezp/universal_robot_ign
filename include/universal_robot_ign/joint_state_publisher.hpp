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
#ifndef UNIVERSAL_ROBOT_IGN_JOINT_STATE_PUBLISHER_H
#define UNIVERSAL_ROBOT_IGN_JOINT_STATE_PUBLISHER_H

#include <ignition/transport/Node.hh>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <map>

namespace universal_robot_ign {

class JointStatePublisher {
public:
    JointStatePublisher(const rclcpp::Node::SharedPtr& nh,
        const std::vector<std::string>& joint_names,
        const std::string& ros_topic, 
        const std::string& ign_topic,
        const unsigned int update_rate);
    ~JointStatePublisher() {};

private:
    void jointStateTimerCb();
    //callback for Ignition
    void ignJointStateCb(const ignition::msgs::Model& msg);

private:
    rclcpp::Node::SharedPtr nh_;
    std::shared_ptr<ignition::transport::Node> ign_node_;
    // ros pub and sub
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr ros_joint_state_pub_;
    rclcpp::TimerBase::SharedPtr joint_state_timer_;
    // joint names and map
    std::vector<std::string> joint_names_;
    std::map<std::string,int> joint_names_map_;
    //joint state info recieved form Ignition
    ignition::msgs::Model current_ign_joint_msg_;
    sensor_msgs::msg::JointState current_joint_msg_;
    std::mutex current_joint_msg_mut_;
};

}

#endif //UNIVERSAL_ROBOT_IGN_JOINT_STATE_PUBLISHER_H