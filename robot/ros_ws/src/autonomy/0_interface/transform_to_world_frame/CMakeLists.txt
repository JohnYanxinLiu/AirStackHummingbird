cmake_minimum_required(VERSION 3.8)
project(transform_to_world_frame)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(transform_to_world_frame src/transform_to_world_frame.cpp)
target_include_directories(transform_to_world_frame PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(transform_to_world_frame rclcpp nav_msgs tf2 tf2_ros tf2_geometry_msgs)

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  transform_to_world_frame
  DESTINATION lib/${PROJECT_NAME})

ament_package()
