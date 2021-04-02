'''Launch ur10 ignition_simulator with ros joint position controller and state publisher'''

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_ros_ign_gazebo = get_package_share_directory('ros_ign_gazebo')
    pkg_universal_robot_ign = get_package_share_directory('universal_robot_ign')
    #data
    world_sdf_path=os.path.join(pkg_universal_robot_ign, 'worlds', 'ur10_robotiq140_world.sdf') 
    ign_config_path=os.path.join(pkg_universal_robot_ign, 'ign', 'gui.config')
    # ignition_simulator launch
    ignition_simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py'),
        ),
        launch_arguments={
            'ign_args': world_sdf_path + ' -v 4 --gui-config ' + ign_config_path,
        }.items()
    )
    # parameter for ur10 controller
    joint_names_list=["shoulder_pan_joint","shoulder_lift_joint","elbow_joint",
                    "wrist_1_joint","wrist_2_joint","wrist_3_joint"]
    ign_cmd_joint_topics_list=[]
    for joint_name in joint_names_list:
        ign_cmd_joint_topics_list.append("/model/ur10/joint/%s/0/cmd_pos"%joint_name)
    
    # ros<-ign, joint state publisher for ur10
    joint_state_publisher=Node(package='universal_robot_ign', 
                executable='joint_state_publisher',
                name="ur10_joint_state_publisher",
                parameters=[{"joint_names": joint_names_list},
                            {"ign_joint_states_topic": "/world/demo/model/ur10/joint_state"},
                            {"ign_joint_idxs": [0,1,2,3,4,5]},
                        ],
                output='screen') 
    #  ros->ign,  joint controller for ur10
    joint_position_controller=Node(package='universal_robot_ign', 
                executable='joint_position_controller',
                name="ur10_joint_position_controller",
                parameters=[{"joint_names": joint_names_list},
                            {"ign_cmd_joint_topics": ign_cmd_joint_topics_list},
                           ],
                output='screen') 
                
    return LaunchDescription([
        ignition_simulator,joint_state_publisher,joint_position_controller
    ])
