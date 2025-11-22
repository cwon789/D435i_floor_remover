cmake_minimum_required(VERSION 3.5)
project(D435i_floor_remover)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# Add PCL definitions
add_definitions(${PCL_DEFINITIONS})

# Build floor remover library
add_library(floor_remover_core
  src/floor_remover.cpp
)

ament_target_dependencies(floor_remover_core
  rclcpp
  sensor_msgs
  pcl_conversions
)

target_link_libraries(floor_remover_core
  ${PCL_LIBRARIES}
)

# Build server node executable
add_executable(floor_remover_node
  src/server_node.cpp
)

ament_target_dependencies(floor_remover_node
  rclcpp
  sensor_msgs
  visualization_msgs
  pcl_conversions
  pcl_ros
)

target_link_libraries(floor_remover_node
  floor_remover_core
  ${PCL_LIBRARIES}
)

# Install targets
install(TARGETS
  floor_remover_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  floor_remover_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
