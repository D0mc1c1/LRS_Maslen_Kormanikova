cmake_minimum_required(VERSION 3.8)
project(lrs_vision)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(aruco_detector src/aruco_detector_node.cpp)
target_include_directories(aruco_detector PRIVATE include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(aruco_detector ${OpenCV_LIBS})
ament_target_dependencies(aruco_detector rclcpp sensor_msgs geometry_msgs cv_bridge)

add_executable(marker_pose_test test/marker_pose_test.cpp)
target_include_directories(marker_pose_test PRIVATE include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(marker_pose_test ${OpenCV_LIBS})

install(TARGETS aruco_detector marker_pose_test DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
