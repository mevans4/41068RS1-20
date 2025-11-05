cmake_minimum_required(VERSION 3.8)
project(path_planner_cpp)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_srvs REQUIRED) 

include_directories(include)

add_executable(planner_node src/planner_node.cpp)

ament_target_dependencies(planner_node
  rclcpp
  rclcpp_action
  nav2_msgs
  geometry_msgs
  nav_msgs
  std_srvs
)

install(TARGETS
  planner_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

