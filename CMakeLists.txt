cmake_minimum_required(VERSION 3.0.2)
project(ssd_nodelet)

#魔法の1行
add_compile_options(-std=c++11 -O2 -g -Wall)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  message_filters
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  sobit_common_msg
  std_msgs
  tf
)

set(OpenCV_DIR /opt/opencv/opencv_3.4.2/share/OpenCV/)  # for ros melodic
find_package(OpenCV 3.4.2 REQUIRED #4.0.0
  NO_MODULE
  PATHS /usr/local
  NO_DEFAULT_PATH
)


## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
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
#   geometry_msgs#   sensor_msgs#   std_msgs
# )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ssd_nodelet
#  CATKIN_DEPENDS cv_bridge geometry_msgs message_filters pcl_conversions pcl_ros roscpp rospy sensor_msgs sobit_common_msg std_msgs tf
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
