cmake_minimum_required(VERSION 3.8)
project(map_updater)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O3 -Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(interface REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

set(SRC_LIST
  src/ikd_tree/ikd_Tree.cpp
  src/map_updater_core.cpp
  src/voxel_hash_map.cpp
)

add_library(map_updater_component SHARED
  src/map_updater_node.cpp
  ${SRC_LIST}
)
ament_target_dependencies(map_updater_component
  rclcpp
  rclcpp_components
  sensor_msgs
  nav_msgs
  pcl_conversions
  interface
)
target_link_libraries(map_updater_component
  ${PCL_LIBRARIES}
  yaml-cpp
)
target_include_directories(map_updater_component PRIVATE src)

rclcpp_components_register_node(map_updater_component
  PLUGIN "MapUpdaterNode"
  EXECUTABLE map_updater_node
)

install(TARGETS map_updater_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
