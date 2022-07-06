cmake_minimum_required(VERSION 3.8)
project(center_estimation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(eufs_msgs REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(center_estimation src/center_estimation.cpp)
ament_target_dependencies(center_estimation rclcpp geometry_msgs eufs_msgs OpenCV)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  TARGETS center_estimation
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
