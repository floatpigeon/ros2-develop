cmake_minimum_required(VERSION 3.8)
project(pub_sub_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# 编译发布节点
add_executable(publisher src/publisher_node.cpp)
ament_target_dependencies(publisher rclcpp std_msgs)

# 编译订阅节点
add_executable(subscriber src/subscriber_node.cpp)
ament_target_dependencies(subscriber rclcpp std_msgs)

# 安装可执行文件
install(TARGETS
  publisher
  subscriber
  DESTINATION lib/${PROJECT_NAME})

ament_package()
