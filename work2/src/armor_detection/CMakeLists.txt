cmake_minimum_required(VERSION 3.8)
project(armor_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ament_index_cpp REQUIRED)

# 添加可执行文件
add_executable(armor_detector src/armor_detector.cpp)

# 链接依赖库
ament_target_dependencies(armor_detector
  rclcpp
  OpenCV
  ament_index_cpp
)
target_link_libraries(armor_detector ${OpenCV_LIBS})

# 安装可执行文件
install(TARGETS
  armor_detector
  DESTINATION lib/${PROJECT_NAME}
)

# 安装图片资源
install(DIRECTORY
  images
  DESTINATION share/${PROJECT_NAME}
)

ament_package()