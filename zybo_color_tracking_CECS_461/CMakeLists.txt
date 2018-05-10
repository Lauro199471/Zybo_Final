cmake_minimum_required(VERSION 2.8.3)
project(zybo_color_tracking_CECS_461)
cmake_policy(SET CMP0046 OLD)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  interactive_markers
  pluginlib
  cv_bridge
  image_transport
  roscpp
  std_msgs
  message_generation
)

find_package(OpenCV REQUIRED)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES seven_dof_arm_test
#  CATKIN_DEPENDS cmake_modules interactive_markers moveit_core moveit_ros_perception moveit_ros_planning_interface pluginlib roscpp std_msgs
#  DEPENDS system_lib
  CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

###################################################
## Declare a cpp executable
add_executable(usb_cam_driver src/usb_cam_driver.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(usb_cam_driver USB_Camera_Driver_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(usb_cam_driver
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
 )

###################################################
## Declare a cpp executable
add_executable(color_tracking src/color_tracking.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(color_tracking USB_Camera_Driver_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(color_tracking
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
 )




