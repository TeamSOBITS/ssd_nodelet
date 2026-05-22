cmake_minimum_required(VERSION 3.8)
project(ssd_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
# find_package(OpenCV 4.5.4 REQUIRED PATHS "/opt/opencv/opencv_4.5.4/")
find_package(OpenCV 4 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_srvs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(vision_msgs REQUIRED)


#--- for C++
############################################
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
)
add_executable(single_shot_multibox_detector src/single_shot_multibox_detector.cpp)
ament_target_dependencies(single_shot_multibox_detector OpenCV cv_bridge rclcpp std_msgs std_srvs sensor_msgs std_srvs rosidl_default_runtime vision_msgs)
target_link_libraries(single_shot_multibox_detector yaml-cpp)

install(TARGETS
  single_shot_multibox_detector
  DESTINATION lib/${PROJECT_NAME}
)
#--- end C++

#-- for Launch
install(DIRECTORY launch/
        DESTINATION share/${PROJECT_NAME}/launch)
#--- end Launch


#-- for Config
install(DIRECTORY config/
DESTINATION share/${PROJECT_NAME}/config)
#--- end Config


#-- for Models
install(DIRECTORY models/face_model/
DESTINATION share/${PROJECT_NAME}/models/face_model)
install(DIRECTORY models/objects_model/
DESTINATION share/${PROJECT_NAME}/models/objects_model)
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