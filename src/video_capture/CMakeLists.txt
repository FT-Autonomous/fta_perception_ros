cmake_minimum_required(VERSION 3.8)
project(video_capture)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(perception_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(video_capture src/video_capture.cpp)
ament_target_dependencies(video_capture rclcpp sensor_msgs perception_msgs OpenCV)

install(TARGETS video_capture
        DESTINATION lib/${PROJECT_NAME})

ament_package()
