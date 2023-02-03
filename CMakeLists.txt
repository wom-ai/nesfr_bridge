cmake_minimum_required(VERSION 3.8)
project(nesfr_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(nesfr_ros_bridge src/nesfr_ros_bridge.cpp)
ament_target_dependencies(
    nesfr_ros_bridge
    geometry_msgs
    nav_msgs
    rclcpp
    tf2
    tf2_ros
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
    nesfr_ros_bridge
    DESTINATION lib/${PROJECT_NAME})

ament_package()
