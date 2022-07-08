'''Launch ur10 ignition_simulator with ros joint trajectory controller and state publisher'''

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    pkg_ros_ign_gazebo = get_package_share_directory('ros_ign_gazebo')
    pkg_universal_robot_ign = get_package_share_directory('universal_robot_ign')
    #data
    world_sdf_path=os.path.join(pkg_universal_robot_ign, 'resource', 'worlds', 'empty_world.sdf') 
    robot_sdf_path=os.path.join(pkg_universal_robot_ign, 'resource', 'models', 'ur10', 'model.sdf') 
    ign_config_path=os.path.join(pkg_universal_robot_ign, 'ign', 'gui.config')
    # ignition_simulator launch
    ignition_simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py'),
        ),
        launch_arguments={
            'ign_args': world_sdf_path + ' -r -v 2 --gui-config ' + ign_config_path,
        }.items()
    )
    ld.add_action(ignition_simulator)
    # Spawn robot
    # robot_macro = XMLMacro4sdf()
    # robot_macro.set_xml_file(robot_xmacro_path)
    # robot_macro.generate({"global_initial_color":"red"})
    # robot_xml = robot_macro.to_string()
    spawn_robot = Node(package='ros_ign_gazebo', executable='create',
        arguments=['-name', 'ur10' ,'-z', '1.4', '-file', robot_sdf_path],
        output='screen')
    ld.add_action(spawn_robot)
    # parameter for ur10 controller
    joint_names_list=["shoulder_pan_joint","shoulder_lift_joint","elbow_joint",
                    "wrist_1_joint","wrist_2_joint","wrist_3_joint"]
    ign_joint_topics_list=[]
    for joint_name in joint_names_list:
        ign_joint_topics_list.append("/model/ur10/joint/%s/0/cmd_pos"%joint_name)
    
    # ros<-ign, joint state publisher for ur10
    joint_state_publisher=Node(package='universal_robot_ign', 
                executable='joint_state_publisher',
                name="ur10_joint_state_publisher",
                parameters=[{"joint_names": joint_names_list},
                            {"ign_topic": "/world/default/model/ur10/joint_state"},
                        ],
                output='screen')
    ld.add_action(joint_state_publisher)
    #  ros->ign,  joint controller for ur10
    joint_controller=Node(package='universal_robot_ign', 
                executable='joint_controller',
                name="ur10_joint_controller",
                parameters=[{"joint_names": joint_names_list},
                            {"ign_joint_topics": ign_joint_topics_list},
                            {"rate":200},
                           ],
                output='screen') 
    ld.add_action(joint_controller)          
    return ld
