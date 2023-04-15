cmake_minimum_required(VERSION 3.8)
project(range_sensor_broadcaster)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(range_sensor_broadcaster SHARED
  src/range_sensor_broadcaster.cpp
)

target_include_directories(range_sensor_broadcaster PRIVATE include)
ament_target_dependencies(range_sensor_broadcaster
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
)

pluginlib_export_plugin_description_file(controller_interface range_sensor_broadcaster.xml)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS range_sensor_broadcaster
  DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  # Todo: add tests
endif()

ament_export_libraries(range_sensor_broadcaster)
ament_export_dependencies(
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
)

ament_package()
