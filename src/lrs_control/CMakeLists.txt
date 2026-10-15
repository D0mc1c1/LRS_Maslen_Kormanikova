cmake_minimum_required(VERSION 3.8)
project(lrs_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
# bez build type kompiluje colcon BEZ optimalizacie -> A* je ~10x pomalsi
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(lrs_planning REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# ---- jadro bez ROS: CSV misia + stavovy automat ----------------------------
add_library(lrs_control_core STATIC
  src/mission.cpp
  src/mission_executor.cpp
  src/outdoor_executor.cpp
)
target_include_directories(lrs_control_core PUBLIC include)
target_link_libraries(lrs_control_core PUBLIC lrs_planning::lrs_planning)

# ---- A1.2 indoor misia (ROS node) ----------------------------------------
add_executable(indoor_mission
  src/indoor_mission_node.cpp
  src/mavros_vehicle.cpp
  src/viz.cpp
)
target_link_libraries(indoor_mission lrs_control_core)
ament_target_dependencies(indoor_mission rclcpp mavros_msgs geometry_msgs std_srvs
  nav_msgs visualization_msgs sensor_msgs std_msgs)

# ---- vizualizacia planu v RViz2 bez simulatora -------------------------------
add_executable(plan_viz
  src/plan_viz_node.cpp
  src/viz.cpp
)
target_link_libraries(plan_viz lrs_control_core)
ament_target_dependencies(plan_viz rclcpp geometry_msgs visualization_msgs)

# ---- offline test stavoveho automatu (bez ROS, bez simulatora) ----------------
add_executable(mission_sim test/mission_sim.cpp)
target_include_directories(mission_sim PRIVATE test)   # sim_vehicle.hpp
target_link_libraries(mission_sim lrs_control_core)

# ---- A1.3 cast 1: diagnostika GPS, home a kompasu ----------------------------
add_executable(gps_info
  src/gps_info_node.cpp
  src/mavros_vehicle.cpp
)
target_link_libraries(gps_info lrs_control_core)
ament_target_dependencies(gps_info rclcpp mavros_msgs geometry_msgs sensor_msgs std_msgs)

# ---- A1.3: test prevodu LLA <-> ENU a kompasu (bez ROS) -----------------------
add_executable(geo_test test/geo_test.cpp)
target_link_libraries(geo_test lrs_control_core)

# ---- A1.3 cast 2: outdoor misia (ROS node) -------------------------------------
add_executable(outdoor_mission
  src/outdoor_mission_node.cpp
  src/mavros_vehicle.cpp
)
target_link_libraries(outdoor_mission lrs_control_core)
ament_target_dependencies(outdoor_mission rclcpp mavros_msgs geometry_msgs sensor_msgs
  std_msgs std_srvs)

# ---- A1.3: offline test outdoor automatu (bez ROS) ---------------------------
add_executable(outdoor_sim test/outdoor_sim.cpp)
target_include_directories(outdoor_sim PRIVATE test)
target_link_libraries(outdoor_sim lrs_control_core)

install(TARGETS indoor_mission mission_sim plan_viz gps_info geo_test
  outdoor_mission outdoor_sim
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config missions rviz DESTINATION share/${PROJECT_NAME})

ament_package()
