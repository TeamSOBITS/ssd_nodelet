cmake_minimum_required(VERSION 3.8)
project(ssd_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(OpenCV 4.9.0 REQUIRED PATHS "/opt/opencv/opencv_4.9.0/")
# find_package(OpenCV 4 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(sobits_msgs REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)


#--- for C++
# add_library(${PROJECT_NAME} src/nodelets/image_subscriber_nodelet.cpp
#                             src/nodelets/image_cloud_subscriber_nodelet.cpp
#                             src/single_shot_multibox_detector.cpp)
############################################
include_directories(include)
add_executable(single_shot_multibox_detector src/single_shot_multibox_detector.cpp)
ament_target_dependencies(single_shot_multibox_detector OpenCV cv_bridge rclcpp std_msgs geometry_msgs sensor_msgs pcl_conversions pcl_ros tf2_ros sobits_msgs rosidl_default_runtime)
# add_executable(bbox_to_tf_fast_shot src/bbox_to_tf_fast_shot.cpp)
# ament_target_dependencies(bbox_to_tf_fast_shot rclcpp std_msgs geometry_msgs sensor_msgs pcl_conversions pcl_ros tf2_ros sobits_msgs rosidl_default_runtime)

install(TARGETS
  single_shot_multibox_detector
  DESTINATION lib/${PROJECT_NAME}
)
#--- end C++

#-- for Launch
install(DIRECTORY launch/
        DESTINATION share/${PROJECT_NAME}/launch)
#--- end Launch

#-- for Models
install(DIRECTORY models/
DESTINATION share/${PROJECT_NAME}/models)
#--- end Models



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


ament_package()