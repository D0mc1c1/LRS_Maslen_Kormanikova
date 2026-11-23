cmake_minimum_required(VERSION 3.8)
project(lrs_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/sitl_2x.sh scripts/check_bringup.sh scripts/record.sh
  scripts/verify_pose.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
