cmake_minimum_required(VERSION 3.5)
project(mek_lilrobot)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wfatal-errors)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2)
find_package(tf2_ros)
find_package(geometry_msgs)
find_package(tf2_geometry_msgs)
find_package(std_srvs)
find_package(mek_lilrobot_interfaces REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(mapodomtfpub src/MapOdomTfPub.cpp)
ament_target_dependencies( mapodomtfpub 
  rclcpp
  std_msgs
  tf2
  tf2_ros
  geometry_msgs
  tf2_geometry_msgs
  std_srvs
)

add_executable(robot src/Robot.cpp)
ament_target_dependencies( robot
  rclcpp
  std_msgs
  tf2
  tf2_ros
  geometry_msgs
  tf2_geometry_msgs
  std_srvs
  mek_lilrobot_interfaces
  nav2_msgs
  sensor_msgs
)

install(TARGETS mapodomtfpub robot
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY description launch config worlds
  DESTINATION share/${PROJECT_NAME})

ament_package()
