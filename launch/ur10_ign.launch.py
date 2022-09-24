'''Launch ur10 ignition_simulator with ros2_control'''

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from sdformat_tools.xmacro4sdf import XMLMacro4sdf
from sdformat_tools.urdf_generator import UrdfGenerator


def generate_launch_description():
    ld = LaunchDescription()
    pkg_ros_ign_gazebo = get_package_share_directory('ros_ign_gazebo')
    pkg_universal_robot_ign = get_package_share_directory('universal_robot_ign')
    # path
    world_sdf_path=os.path.join(pkg_universal_robot_ign, 'resource', 'worlds', 'empty_world.sdf') 
    robot_sdf_xmacro=os.path.join(pkg_universal_robot_ign, 'resource', 'sdf', 'ur10_ros.sdf.xmacro')
    ros_control_urdf=os.path.join(pkg_universal_robot_ign, 'resource', 'urdf', 'ur10_ros_control.urdf')
    controller_config=os.path.join(pkg_universal_robot_ign, 'resource', 'controller_config', 'ur10_position.yaml')
    ign_config=os.path.join(pkg_universal_robot_ign, 'ign', 'gui.config')
    rviz2_config = os.path.join(pkg_universal_robot_ign,"launch", "test.rviz")
    # sdf
    robot_macro = XMLMacro4sdf()
    robot_macro.set_xml_file(robot_sdf_xmacro)
    robot_macro.generate({"ros2_control_config" : controller_config})
    robot_sdf_string = robot_macro.to_string()
    # urdf
    urdf_generator = UrdfGenerator()
    urdf_generator.parse_from_sdf_string(robot_sdf_string)
    urdf_generator.remove_joint('world_ur10_joint')
    urdf_generator.merge_urdf_file(ros_control_urdf)
    robot_urdf_string = urdf_generator.to_string()

    # ignition_simulator launch
    ignition_simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py'),
        ),
        launch_arguments={
            'ign_args': world_sdf_path + ' -r -v 2 --gui-config ' + ign_config,
        }.items()
    )

    # Spawn robot
    spawn_robot = Node(package='ros_ign_gazebo', executable='create',
        arguments=['-name', 'ur10' ,'-z', '1.4', '-string', robot_sdf_string],
        output='screen')

    # robot state publisher
    robot_state_publisher = Node(package="robot_state_publisher",
             executable="robot_state_publisher",
             name="robot_state_publisher",
             parameters=[{'robot_description': robot_urdf_string}],
             output="screen")

    # static Robot state publisher
    static_transform_publisher=Node(package="tf2_ros",
             executable="static_transform_publisher",
             name="static_transform_publisher",
             output="screen",
             arguments=["0.0", "0.0", "1.4",
                        "0.0", "0.0", "0.0",
                        "world", "base_link"])

    # RViz2
    rviz2=Node(package="rviz2",
             executable="rviz2",
             name="rviz2",
             output="screen",
             arguments=["--display-config", rviz2_config])

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start', 'joint_trajectory_controller'],
        output='screen'
    )

    return LaunchDescription([
        ignition_simulator,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_robot,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_joint_trajectory_controller],
            )
        ),
        spawn_robot,
        robot_state_publisher,
        static_transform_publisher,
        rviz2
    ])

