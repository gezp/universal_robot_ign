
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
    urdf = os.path.join(pkg_universal_robot_ign,"resource","urdf","ur10.urdf")
    # cmd_joint_states GUI publisher (position controller) 
    cmd_gui_publisher = Node(package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name="joint_state_publisher_gui",
            arguments=[ urdf ],
            remappings=[('/joint_states', '/set_joint_state')],
            output='screen') 
    
    return LaunchDescription([
        cmd_gui_publisher,
    ])
