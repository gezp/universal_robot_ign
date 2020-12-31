 #  Copyright (c) 2020 robomaster-oss, All rights reserved.
 #
 #  This program is free software: you can redistribute it and/or modify it 
 #  under the terms of the MIT License, See the MIT License for more details.
 #
 #  You should have received a copy of the MIT License along with this program.
 #  If not, see <https://opensource.org/licenses/MIT/>.


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
            'ign_args': world_sdf_path + ' -v 2 --gui-config ' + ign_config_path,
        }.items()
    )
    return LaunchDescription([
        ignition_simulator
    ])
