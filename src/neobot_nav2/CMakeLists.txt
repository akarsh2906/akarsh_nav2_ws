cmake_minimum_required(VERSION 3.8)
project(neobot_nav2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)

add_executable(nav2_control src/nav2_control.cpp)
ament_target_dependencies(nav2_control rclcpp geometry_msgs lifecycle_msgs tf2 tf2_geometry_msgs nav2_msgs visualization_msgs rclcpp_action)

add_executable(multi_goal src/multi_goal.cpp)
ament_target_dependencies(multi_goal rclcpp geometry_msgs lifecycle_msgs tf2 tf2_geometry_msgs nav2_msgs visualization_msgs rclcpp_action)

install(TARGETS
nav2_control
multi_goal
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY config launch maps rviz src
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
