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
#include <universal_robot_ign/joint_position_controller.hpp>

int main(int argc, char* argv[])
{
    // create ros2 node
    rclcpp::init(argc, argv);
    auto ros_node = std::make_shared<rclcpp::Node>("joint_position_controller");
    // variables
    std::vector<std::string> joint_names;
    std::string cmd_joint_states_topic;
    std::vector<std::string> ign_cmd_joint_topics;
    // parameters
    ros_node->declare_parameter("joint_names");
    ros_node->declare_parameter("cmd_joint_states_topic","cmd_joint_states");
    ros_node->declare_parameter("ign_cmd_joint_topics");
    joint_names = ros_node->get_parameter("joint_names").as_string_array();
    cmd_joint_states_topic = ros_node->get_parameter("cmd_joint_states_topic").as_string();
    ign_cmd_joint_topics = ros_node->get_parameter("ign_cmd_joint_topics").as_string_array();
    // create controller 
    auto joint_controller = std::make_shared<universal_robot_ign::JointPositionController>(ros_node,
        joint_names, cmd_joint_states_topic, ign_cmd_joint_topics);
    // run node until it's exited
    rclcpp::spin(ros_node);
    //clean up
    rclcpp::shutdown();
    return 0;
}
