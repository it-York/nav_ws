cmake_minimum_required(VERSION 3.8)
project(go2w_nav2_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(unitree_api REQUIRED)

add_executable(cmd_vel_to_sport src/cmd_vel_to_sport.cpp)
ament_target_dependencies(cmd_vel_to_sport
  action_msgs geometry_msgs nav_msgs rclcpp sensor_msgs std_msgs unitree_api)

add_executable(go2w_mode_manager src/go2w_mode_manager.cpp)
ament_target_dependencies(go2w_mode_manager rclcpp std_msgs unitree_api)

install(TARGETS cmd_vel_to_sport go2w_mode_manager
  DESTINATION lib/${PROJECT_NAME})

ament_package()
