cmake_minimum_required(VERSION 3.5)
project(filter_visualization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(data_publisher src/data_publisher.cpp)
ament_target_dependencies(data_publisher rclcpp std_msgs sensor_msgs)

add_executable(filter_node src/filter_node.cpp)
ament_target_dependencies(filter_node rclcpp std_msgs sensor_msgs)

install(TARGETS
  data_publisher
  filter_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
