cmake_minimum_required(VERSION 3.8)
project(signal_processor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 声明自定义消息
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Signal.msg"
  DEPENDENCIES std_msgs
)

# 编译生成器节点
add_executable(generator src/signal_generator.cpp)
ament_target_dependencies(generator rclcpp std_msgs)
rosidl_target_interfaces(generator
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

# 编译处理器节点
add_executable(processor src/signal_processor_node.cpp)
ament_target_dependencies(processor rclcpp std_msgs)
rosidl_target_interfaces(processor
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

# 安装可执行文件
install(TARGETS
  generator
  processor
  DESTINATION lib/${PROJECT_NAME})

ament_package()
