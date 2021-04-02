
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_universal_robot_ign = get_package_share_directory('universal_robot_ign')
    # parameter
    urdf = os.path.join(pkg_universal_robot_ign,"res","robotiq140_config","robotiq140.urdf")
    #
    joint_names_list=["finger_joint","left_inner_knuckle_joint","left_inner_finger_joint",
                    "right_outer_knuckle_joint","right_inner_knuckle_joint","right_inner_finger_joint"]
    ign_cmd_joint_topics_list=[]
    for joint_name in joint_names_list:
        ign_cmd_joint_topics_list.append("/model/ur10/joint/%s/0/cmd_pos"%joint_name)
    
    joint_position_controller=Node(package='universal_robot_ign', 
            executable='joint_position_controller',
            name="robotiq_joint_position_controller",
            parameters=[{"joint_names": joint_names_list},
                        {"ign_cmd_joint_topics": ign_cmd_joint_topics_list},
                         ],
            remappings=[('/cmd_joint_states', '/cmd_robotiq_joint_states')],
            output='screen') 
    # cmd_joint_states GUI publisher (position controller)  
    cmd_gui_publisher = Node(package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name="robotiq_joint_state_publisher_gui",
            arguments=[ urdf ],
            remappings=[('/joint_states', '/cmd_robotiq_joint_states')],
            output='screen') 
    return LaunchDescription([
        cmd_gui_publisher,joint_position_controller
    ])
