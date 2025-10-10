cmake_minimum_required(VERSION 3.5)
project(hz_pkg) 

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# 信号发生器节点
add_executable(signal_generator src/signal_generator.cpp)
ament_target_dependencies(signal_generator rclcpp std_msgs)

# 信号处理器节点
add_executable(signal_processor src/signal_processor.cpp)
ament_target_dependencies(signal_processor rclcpp std_msgs)

# 安装可执行文件
install(TARGETS
  signal_generator
  signal_processor
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
    