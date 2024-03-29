cmake_minimum_required(VERSION 3.8)
project(ft_cluster_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(eufs_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ft_msgs REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(ft_cluster_cpp src/cluster.cpp src/gl.c src/egl.c src/cluster_gl.cpp)
target_include_directories(ft_cluster_cpp PUBLIC include)
target_link_libraries(ft_cluster_cpp EGL)
ament_target_dependencies(ft_cluster_cpp ft_msgs rclcpp eufs_msgs sensor_msgs OpenCV)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  TARGETS ft_cluster_cpp
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY lib/shaders
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
