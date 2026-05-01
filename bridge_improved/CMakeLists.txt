cmake_minimum_required(VERSION 3.8)
project(cyberdog_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-std=c++17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(action_msgs REQUIRED)

# motion_msgs / interaction_msgs 不在 foxy 默认路径，必须手动指定
set(motion_msgs_DIR "/opt/ros2/cyberdog/share/motion_msgs/cmake/motion_msgsConfig.cmake")
find_package(motion_msgs REQUIRED)
add_definitions(-DCYBERDOG_INTERFACES_FOUND)

set(interaction_msgs_DIR "/opt/ros2/cyberdog/share/interaction_msgs/cmake/interaction_msgsConfig.cmake")
find_package(interaction_msgs REQUIRED)

add_executable(bridge_node bridge_node.cpp)

target_include_directories(bridge_node PRIVATE
  /opt/ros2/cyberdog/include
  /opt/ros2/foxy/include
)

ament_target_dependencies(bridge_node
  rclcpp
  rclcpp_action
  std_msgs
  builtin_interfaces
  action_msgs
  motion_msgs
  interaction_msgs
)

install(TARGETS bridge_node DESTINATION lib/${PROJECT_NAME})
ament_package()
