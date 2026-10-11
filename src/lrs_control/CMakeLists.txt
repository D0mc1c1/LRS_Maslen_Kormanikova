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
find_package(std_srvs REQUIRED)

# ---- jadro bez ROS: CSV misia + stavovy automat ----------------------------
add_library(lrs_control_core STATIC
  src/mission.cpp
  src/mission_executor.cpp
)
target_include_directories(lrs_control_core PUBLIC include)
target_link_libraries(lrs_control_core PUBLIC lrs_planning::lrs_planning)

# ---- A1.2 indoor misia (ROS node) ----------------------------------------
add_executable(indoor_mission
  src/indoor_mission_node.cpp
  src/mavros_vehicle.cpp
)
target_link_libraries(indoor_mission lrs_control_core)
ament_target_dependencies(indoor_mission rclcpp mavros_msgs geometry_msgs std_srvs)

# ---- offline test stavoveho automatu (bez ROS, bez simulatora) ----------------
add_executable(mission_sim test/mission_sim.cpp)
target_link_libraries(mission_sim lrs_control_core)

# ---- A1.3 outdoor misia - odkomentuj vo faze 3 -----------------------------
# add_executable(outdoor_mission src/outdoor_mission_node.cpp src/mavros_vehicle.cpp)
# target_link_libraries(outdoor_mission lrs_control_core)
# ament_target_dependencies(outdoor_mission rclcpp mavros_msgs geometry_msgs sensor_msgs std_srvs)

install(TARGETS indoor_mission mission_sim DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config missions DESTINATION share/${PROJECT_NAME})

ament_package()
