cmake_minimum_required(VERSION 3.5)
project(mek_drive)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(mek_drive_interfaces REQUIRED)


add_executable(mek_drive minimal_action_server.cpp)
ament_target_dependencies(mek_drive
  "rclcpp"
  "rclcpp_action"
  "example_interfaces"
  "mek_drive_interfaces")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  mek_drive
  DESTINATION lib/${PROJECT_NAME})

ament_package()