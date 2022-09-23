"""Launch MoveIt2 move_group action server and the required bridges between Ignition and ROS 2"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from sdformat_tools.urdf_generator import UrdfGenerator


def generate_launch_description():
    pkg_universal_robot_ign = get_package_share_directory('universal_robot_ign')
    # Launch Arguments
    robot_sdf_path=os.path.join(pkg_universal_robot_ign, 'resource', 'models', 'ur10', 'model.sdf')
    urdf_generator = UrdfGenerator()
    urdf_generator.parse_from_sdf_file(robot_sdf_path)
    urdf_generator.remove_joint('world_ur10_joint')
    robot_urdf_xml = urdf_generator.to_string()
    rviz2_config = os.path.join(pkg_universal_robot_ign,"launch", "test.rviz")

    # Robot state publisher
    robot_state_publisher = Node(package="robot_state_publisher",
             executable="robot_state_publisher",
             name="robot_state_publisher",
             parameters=[{'robot_description': robot_urdf_xml}],
             output="screen")

    #static Robot state publisher
    static_transform_publisher=Node(package="tf2_ros",
             executable="static_transform_publisher",
             name="static_transform_publisher",
             output="screen",
             arguments=["0.0", "0.0", "1.4",
                        "0.0", "0.0", "0.0",
                        "world", "base_link"])
    # MoveIt2 move_group action server
    move_group=IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_universal_robot_ign,"launch", "ur10_move_group_server.launch.py")
            ),
            # Simulation time does not function properly (as of Nov 2020), see https://github.com/AndrejOrsula/ign_moveit2/issues/4
        )
    # RViz2
    rviz2=Node(package="rviz2",
             executable="rviz2",
             name="rviz2",
             output="log",
             arguments=["--display-config", rviz2_config])   

    return LaunchDescription([
        robot_state_publisher,static_transform_publisher,move_group,rviz2
    ])
