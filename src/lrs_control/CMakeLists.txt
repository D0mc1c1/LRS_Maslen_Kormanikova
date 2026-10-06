cmake_minimum_required(VERSION 3.8)
project(lrs_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(lrs_planning REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(ROS_DEPS
  rclcpp mavros_msgs geometry_msgs sensor_msgs std_msgs std_srvs
  nav_msgs visualization_msgs tf2 tf2_geometry_msgs)

# ---- indoor misia (A1.2) -----------------------------------------------
add_executable(indoor_mission
  src/indoor_mission_node.cpp
  # src/mavros_interface.cpp
  # src/mission_loader.cpp
  # src/controller.cpp
  # src/heading.cpp
)
target_include_directories(indoor_mission PRIVATE include)
ament_target_dependencies(indoor_mission ${ROS_DEPS})
target_link_libraries(indoor_mission lrs_planning::lrs_planning)

# ---- outdoor misia (A1.3) - odkomentuj vo faze 3 ---------------------------
# add_executable(outdoor_mission src/outdoor_mission_node.cpp ...)
# target_include_directories(outdoor_mission PRIVATE include)
# ament_target_dependencies(outdoor_mission ${ROS_DEPS})

install(TARGETS indoor_mission DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config missions rviz DESTINATION share/${PROJECT_NAME})

ament_package()
