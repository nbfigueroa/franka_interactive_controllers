cmake_minimum_required(VERSION 3.4)
project(franka_interactive_controllers)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  dynamic_reconfigure
  eigen_conversions
  franka_hw
  franka_gripper
  geometry_msgs
  hardware_interface
  tf
  tf_conversions
  message_generation
  pluginlib
  realtime_tools
  roscpp
  rospy
)

find_package(Eigen3 REQUIRED)
find_package(Franka 0.7.0 REQUIRED)

generate_messages()

generate_dynamic_reconfigure_options(
  cfg/compliance_param.cfg
  cfg/compliance_full_param.cfg
  cfg/desired_mass_param.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES franka_interactive_controllers
  CATKIN_DEPENDS
    controller_interface
    dynamic_reconfigure
    eigen_conversions
    franka_hw
    franka_gripper
    geometry_msgs
    hardware_interface
    tf
    tf_conversions
    message_runtime
    pluginlib
    realtime_tools
    roscpp
  DEPENDS Franka
)

# AS IN THE DS_MOTION_GENRATOR CMAKELIST
set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(H_FILES ${INCLUDE_DIR}/franka_cartesian_controllers/cartesian_pose_franka_controller.h
            ${INCLUDE_DIR}/franka_cartesian_controllers/cartesian_velocity_franka_controller.h
            ${INCLUDE_DIR}/franka_cartesian_controllers/cartesian_impedance_controller_pose_command.h
#             ${INCLUDE_DIR}/franka_cartesian_controllers/cartesian_impedance_controller_velocity_command.h
#             ${INCLUDE_DIR}/franka_cartesian_controllers/cartesian_passiveDS_impedance_controller.h
            ${INCLUDE_DIR}/franka_cartesian_controllers/cartesian_force_controller.h
            ${INCLUDE_DIR}/franka_joint_controllers/joint_position_franka_controller.h
            ${INCLUDE_DIR}/franka_joint_controllers/joint_velocity_franka_controller.h
            ${INCLUDE_DIR}/franka_joint_controllers/joint_impedance_franka_controller.h
            ${INCLUDE_DIR}/franka_utils/pseudo_inversion.h
            ${INCLUDE_DIR}/franka_utils/controllers_common.h)

## Specify locations of header files
## Your package locations should be listed before other locations
include_directories(include/franka_cartesian_controllers include/franka_joint_controllers include/franka_utils ${catkin_INCLUDE_DIRS})
include_directories(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)


## Declare a C++ executable
set(SRCS
  src/franka_cartesian_controllers/cartesian_pose_franka_controller.cpp
  src/franka_cartesian_controllers/cartesian_velocity_franka_controller.cpp
  src/franka_cartesian_controllers/cartesian_impedance_controller_pose_command.cpp
#   src/franka_cartesian_controllers/cartesian_impedance_controller_velocity_command.cpp
#   src/franka_cartesian_controllers/cartesian_passiveDS_impedance_controller.cpp
  src/franka_cartesian_controllers/cartesian_force_controller.cpp
  src/franka_joint_controllers/joint_position_franka_controller.cpp
  src/franka_joint_controllers/joint_velocity_franka_controller.cpp
  src/franka_joint_controllers/joint_impedance_franka_controller.cpp
  src/franka_utils/controllers_common.cpp)

add_library(franka_interactive_controllers ${H_FILES} ${SRCS})

add_dependencies(franka_interactive_controllers
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  ${PROJECT_NAME}_gencpp
  ${PROJECT_NAME}_gencfg
)

target_link_libraries(franka_interactive_controllers PUBLIC
  ${Franka_LIBRARIES}
  ${catkin_LIBRARIES}
)

target_include_directories(franka_interactive_controllers SYSTEM PUBLIC
  ${Franka_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
target_include_directories(franka_interactive_controllers PUBLIC
  include
)

## Installation
install(TARGETS franka_interactive_controllers
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES franka_interactive_controllers_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
catkin_install_python(
  PROGRAMS scripts/interactive_marker.py scripts/move_to_start.py scripts/franka_cartesian_impedance_pose_command.py scripts/franka_to_geometry_messages.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Tools
include(${CMAKE_CURRENT_LIST_DIR}/../cmake/ClangTools.cmake OPTIONAL
  RESULT_VARIABLE CLANG_TOOLS
)
if(CLANG_TOOLS)
  file(GLOB_RECURSE SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
  file(GLOB_RECURSE HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.h
  )
  add_format_target(franka_interactive_controllers FILES ${SOURCES} ${HEADERS})
  add_tidy_target(franka_interactive_controllers
    FILES ${SOURCES}
    DEPENDS franka_interactive_controllers
  )
endif()
