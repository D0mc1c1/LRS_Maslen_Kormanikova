cmake_minimum_required(VERSION 3.8)
project(lrs_planning)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

# ---- kniznica (bez ROS) -------------------------------------------------
# SHARED + PCL ako PRIVATE: lrs_control nemusi o PCL nic vediet.
add_library(${PROJECT_NAME} SHARED
  src/voxel_map.cpp
  # src/inflation.cpp
  # src/astar.cpp
  # src/path_simplifier.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(${PROJECT_NAME} PRIVATE ${PCL_INCLUDE_DIRS})
target_compile_definitions(${PROJECT_NAME} PRIVATE ${PCL_DEFINITIONS})
target_link_libraries(${PROJECT_NAME} PRIVATE ${PCL_LIBRARIES})

# ---- standalone CLI (deliverable A1.1) ---------------------------------
add_executable(plan_cli apps/plan_cli.cpp)
target_link_libraries(plan_cli ${PROJECT_NAME})

# ---- install -------------------------------------------------------------
install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS plan_cli DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

# ---- testy (odkomentuj ked pribudnu) --------------------------------------
# if(BUILD_TESTING)
#   find_package(ament_cmake_gtest REQUIRED)
#   ament_add_gtest(test_voxel_map test/test_voxel_map.cpp)
#   target_link_libraries(test_voxel_map ${PROJECT_NAME})
# endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()
