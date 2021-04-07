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
#include <rclcpp/rclcpp.hpp>
#include <universal_robot_ign/joint_state_publisher.hpp>

int main(int argc, char* argv[])
{
    // create ros2 node
    rclcpp::init(argc, argv);
    auto ros_node = std::make_shared<rclcpp::Node>("joint_state_publisher");
    // variable
    std::vector<std::string> joint_names;
    std::string joint_states_topic;
    std::string ign_joint_states_topic;
    std::vector<int> ign_joint_idxs;
    int update_rate;
    // get parameter
    ros_node->declare_parameter("joint_names");
    ros_node->declare_parameter("ign_joint_states_topic");
    ros_node->declare_parameter("ign_joint_idxs");
    ros_node->declare_parameter("rate", 30);
    joint_names = ros_node->get_parameter("joint_names").as_string_array();
    ign_joint_states_topic = ros_node->get_parameter("ign_joint_states_topic").as_string();
    std::vector<int64_t> ign_joint_idxs_64 = ros_node->get_parameter("ign_joint_idxs").as_integer_array();
    update_rate = ros_node->get_parameter("rate").as_int();
    if (ign_joint_idxs_64.size() == 0) {
        for (size_t i = 0; i < joint_names.size(); i++) {
            ign_joint_idxs.push_back(i);
        }
    }else{
        for (size_t i = 0; i < ign_joint_idxs_64.size(); i++) {
            ign_joint_idxs.push_back(ign_joint_idxs_64[i]);
        }  
    }
    // create publisher
    auto joint_publisher = std::make_shared<universal_robot_ign::JointStatePublisher>(ros_node,
        joint_names, "joint_states", ign_joint_states_topic, ign_joint_idxs ,update_rate);
    // run node until it's exited
    rclcpp::spin(ros_node);
    //clean up
    rclcpp::shutdown();
    return 0;
}
