cmake_minimum_required(VERSION 3.8)
project(hik_camera_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# --- 核心：链接海康 MVS 库 ---
include_directories(/opt/MVS/include)
link_directories(/opt/MVS/lib/64)

add_executable(hik_camera_node src/hik_camera_node.cpp)

ament_target_dependencies(hik_camera_node rclcpp sensor_msgs)

# 链接海康动态库
target_link_libraries(hik_camera_node MvCameraControl)
# -----------------------------

install(TARGETS hik_camera_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()