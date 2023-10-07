cmake_minimum_required(VERSION 3.7.2)
project(fetch_follow_target)

find_package(Boost REQUIRED)
find_package(gazebo REQUIRED)

find_package(catkin
  REQUIRED
    angles
    control_toolbox
    gazebo_ros
    robot_controllers
    robot_controllers_interface
    roscpp
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
    tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

catkin_package(
  CATKIN_DEPENDS
    control_toolbox
    gazebo_plugins
    gazebo_ros
    robot_controllers
    robot_controllers_interface
    roscpp 
    std_msgs 
    sensor_msgs 
    geometry_msgs 
    nav_msgs 
    tf
  LIBRARIES
    fetch_gazebo_plugin
  INCLUDE_DIRS
    include
)

################################################################################
# Install
################################################################################
install(TARGETS turtlebot3_drive
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch models rviz worlds 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(
  PROGRAMS
    scripts/prepare_simulated_robot.py
    scripts/prepare_simulated_robot_pick_place.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY config include launch robots worlds models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

