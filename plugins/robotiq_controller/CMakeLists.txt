# ignition package
# Citadel
if("$ENV{IGNITION_VERSION}" STREQUAL "citadel")
  find_package(ignition-transport8 REQUIRED)
  set(IGN_TRANSPORT_VER ${ignition-transport8_VERSION_MAJOR})

  find_package(ignition-msgs5 REQUIRED)
  set(IGN_MSGS_VER ${ignition-msgs5_VERSION_MAJOR})

  message(STATUS "Compiling against Ignition Citadel")
# Dome
elseif("$ENV{IGNITION_VERSION}" STREQUAL "dome")
  find_package(ignition-transport9 REQUIRED)
  set(IGN_TRANSPORT_VER ${ignition-transport9_VERSION_MAJOR})

  find_package(ignition-msgs6 REQUIRED)
  set(IGN_MSGS_VER ${ignition-msgs6_VERSION_MAJOR})

  message(STATUS "Compiling against Ignition Dome")
# Fortress
elseif("$ENV{IGNITION_VERSION}" STREQUAL "fortress")
  find_package(ignition-transport11 REQUIRED)
  set(IGN_TRANSPORT_VER ${ignition-transport11_VERSION_MAJOR})

  find_package(ignition-msgs8 REQUIRED)
  set(IGN_MSGS_VER ${ignition-msgs8_VERSION_MAJOR})

  message(STATUS "Compiling against Ignition Fortress")
# Default to Edifice
else()
  find_package(ignition-transport10 REQUIRED)
  set(IGN_TRANSPORT_VER ${ignition-transport10_VERSION_MAJOR})

  find_package(ignition-msgs7 REQUIRED)
  set(IGN_MSGS_VER ${ignition-msgs7_VERSION_MAJOR})
  message(STATUS "Compiling against Ignition Edifice")
endif()

# for ignition-gazebo plugin RobotiqController
add_library(RobotiqController SHARED RobotiqController.cpp)

target_link_libraries(RobotiqController
    ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
    ignition-gazebo${IGN_GAZEBO_VER}::ignition-gazebo${IGN_GAZEBO_VER})
)




