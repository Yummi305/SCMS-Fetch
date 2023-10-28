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
    rospy	
    std_msgs
    sensor_msgs
    roslib
    control_msgs
    geometry_msgs
    cv_bridge
    image_transport
    message_generation
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)



# Check for c++11 / c++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

################################################
## Declare ROS messages, services and actions ##
################################################

# ## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   file.msg
# )

# # Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   file.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
#   geometry_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################

catkin_package(
  # INCLUDE_DIRS include
  # LIBRARIES fetch_follow_target
  # CATKIN_DEPENDS roscpp rospy std_msgs message_runtime geometry_msgs
  # DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
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
  PATTERN ".svn" EXCLUDE
)

catkin_install_python(PROGRAMS
  nodes/turtlebot3_teleop_key
  nodes/teleop_twist_key.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
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

add_library(${PROJECT_NAME}_laserprocessing src/laserprocessing.cpp)

add_executable(followTarget src/main.cpp src/followTarget.cpp src/laserprocessing.cpp)
add_dependencies(followTarget ${catkin_EXPORTED_TARGETS})
target_link_libraries(followTarget ${catkin_LIBRARIES})

add_executable(joy_teleop src/joy_teleop.cpp)
add_dependencies(joy_teleop ${catkin_EXPORTED_TARGETS})
target_link_libraries(joy_teleop ${catkin_LIBRARIES})
