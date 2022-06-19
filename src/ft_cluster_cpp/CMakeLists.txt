cmake_minimum_required(VERSION 3.8)
project(ft_cluster_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(perception_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(ft_cluster_cpp src/cluster.cpp)
ament_target_dependencies(ft_cluster_cpp rclcpp perception_msgs sensor_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  TARGETS ft_cluster_cpp
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()