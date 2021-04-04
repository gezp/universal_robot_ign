cmake_minimum_required(VERSION 3.5)
project(universal_robot_ign)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# ros package
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

# ignition package
# find_package(ignition-plugin1 REQUIRED)
# set(IGN_PLUGIN_VER 1)
find_package(ignition-gazebo4 REQUIRED)
set(IGN_GAZEBO_VER 4)
find_package(ignition-transport9 REQUIRED)
set(IGN_TRANSPORT_VER 9)
find_package(ignition-msgs6 REQUIRED)
set(IGN_MSGS_VER 6)

# include
include_directories(include)

# generate universal_robot_ign lib
AUX_SOURCE_DIRECTORY(${PROJECT_SOURCE_DIR}/src DIR_SRCS)
add_library(${PROJECT_NAME} ${DIR_SRCS})

ament_target_dependencies(${PROJECT_NAME}
    rclcpp 
    std_msgs
    sensor_msgs
    trajectory_msgs
)

target_link_libraries(${PROJECT_NAME}
    ignition-msgs${IGN_MSGS_VER}::core
    ignition-transport${IGN_TRANSPORT_VER}::core
)

#executable node
add_executable(joint_state_publisher nodes/joint_state_publisher_node.cpp)
target_link_libraries(joint_state_publisher ${PROJECT_NAME})

add_executable(joint_position_controller nodes/joint_position_controller_node.cpp)
target_link_libraries(joint_position_controller ${PROJECT_NAME})

add_executable(joint_trajectory_controller nodes/joint_trajectory_controller_node.cpp)
target_link_libraries(joint_trajectory_controller ${PROJECT_NAME})

# for ignition-gazebo plugin RobotiqController
add_library(RobotiqController SHARED plugins/robotiq_controller/RobotiqController.cpp)

target_link_libraries(RobotiqController
  ignition-gazebo${IGN_GAZEBO_VER}::ignition-gazebo${IGN_GAZEBO_VER}
)

#install
install( DIRECTORY
    ign launch models worlds res
  DESTINATION share/${PROJECT_NAME}
)

# Install include directories
install(DIRECTORY include/
  DESTINATION include
)

# Install libraries
install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

# Install executables
install(TARGETS joint_state_publisher joint_position_controller joint_trajectory_controller 
  DESTINATION lib/${PROJECT_NAME} 
)

install(TARGETS RobotiqController 
    DESTINATION $ENV{HOME}/.ignition/gazebo/plugins
)

# Install executable scripts
install(PROGRAMS 
        scripts/test_pose_goal.py
        scripts/test_gripper.py
    DESTINATION lib/${PROJECT_NAME})

#environment
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/ignition.dsv.in")

#test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
