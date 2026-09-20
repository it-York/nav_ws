cmake_minimum_required(VERSION 3.8)
project(go2w_nav2_navigation)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fast_lio REQUIRED)
find_package(nav2_collision_monitor REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(fastlio_odom_bridge src/fastlio_odom_bridge.cpp)
target_include_directories(fastlio_odom_bridge PRIVATE include)
ament_target_dependencies(fastlio_odom_bridge
  geometry_msgs nav_msgs rclcpp std_msgs tf2 tf2_geometry_msgs tf2_ros)

add_executable(fastlio_imu_odometry src/fastlio_imu_odometry.cpp)
target_include_directories(fastlio_imu_odometry PRIVATE include)
target_link_libraries(fastlio_imu_odometry Eigen3::Eigen)
ament_target_dependencies(fastlio_imu_odometry fast_lio geometry_msgs nav_msgs rclcpp sensor_msgs std_msgs tf2 tf2_geometry_msgs tf2_ros)
add_executable(safe_collision_monitor src/safe_collision_monitor.cpp)
target_include_directories(safe_collision_monitor PRIVATE include)
# Geometry rollout is latency-sensitive on Jetson.
target_compile_options(safe_collision_monitor PRIVATE -O2)
ament_target_dependencies(safe_collision_monitor nav2_collision_monitor nav_msgs visualization_msgs rclcpp sensor_msgs std_msgs tf2_geometry_msgs)

add_executable(cloud_to_scan src/cloud_to_scan.cpp)
ament_target_dependencies(cloud_to_scan rclcpp sensor_msgs tf2)

add_executable(cloud_obstacle_filter src/cloud_obstacle_filter.cpp)
ament_target_dependencies(cloud_obstacle_filter rclcpp sensor_msgs tf2)

install(TARGETS fastlio_imu_odometry safe_collision_monitor fastlio_odom_bridge cloud_to_scan cloud_obstacle_filter
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY behavior_trees config launch rviz
  DESTINATION share/${PROJECT_NAME})
install(FILES README.md
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_dynamic_slowdown test/test_dynamic_slowdown.cpp)
  target_include_directories(test_dynamic_slowdown PRIVATE include)
  target_compile_options(test_dynamic_slowdown PRIVATE -O2)
  ament_add_gtest(test_imu_prediction test/test_imu_prediction.cpp)
  target_include_directories(test_imu_prediction PRIVATE include)
  target_link_libraries(test_imu_prediction Eigen3::Eigen)
  ament_add_gtest(test_odom_kinematics test/test_odom_kinematics.cpp)
  target_include_directories(test_odom_kinematics PRIVATE include)
  ament_target_dependencies(test_odom_kinematics geometry_msgs tf2)
endif()

ament_package()
