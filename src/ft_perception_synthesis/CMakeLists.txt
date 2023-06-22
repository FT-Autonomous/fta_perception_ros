cmake_minimum_required(VERSION 3.8)
project(ft_perception_synthesis)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(eufs_msgs REQUIRED)
find_package(ft_msgs REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(perception src/perception.cpp)
ament_target_dependencies(perception rclcpp ft_msgs eufs_msgs OpenCV)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS perception
	DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config bags
        DESTINATION share/${PROJECT_NAME})

ament_package()
