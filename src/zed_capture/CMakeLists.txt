cmake_minimum_required(VERSION 3.8)
project(zed_capture)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ZED 3 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(CUDA 11.2 REQUIRED)
find_package(perception_msgs REQUIRED)

add_executable(zed_capture src/zed_capture.cpp)
ament_target_dependencies(zed_capture rclcpp OpenCV sensor_msgs perception_msgs ZED CUDA)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  TARGETS zed_capture
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
