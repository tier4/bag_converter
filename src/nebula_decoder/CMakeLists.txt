cmake_minimum_required(VERSION 3.14)
project(nebula_decoder)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nebula_msgs REQUIRED)
find_package(seyond_nebula_decoder REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nebula_decoders REQUIRED)
find_package(nebula_common REQUIRED)

# Include directories
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# Component library
add_library(${PROJECT_NAME}_component SHARED
  src/nebula_decoder_node.cpp
)

target_include_directories(${PROJECT_NAME}_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}_component
  ${PCL_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}_component
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  nebula_msgs
  seyond_nebula_decoder
  pcl_conversions
  nebula_decoders
  nebula_common
)

# Register component
rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "nebula_decoder::NebulaDecoderNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

# Standalone executable
add_executable(${PROJECT_NAME}_node_exe
  src/nebula_decoder_node_main.cpp
)

target_include_directories(${PROJECT_NAME}_node_exe PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}_node_exe
  ${PROJECT_NAME}_component
)

ament_target_dependencies(${PROJECT_NAME}_node_exe
  rclcpp
)

# Install component library
install(
  TARGETS ${PROJECT_NAME}_component
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install executables
install(
  TARGETS ${PROJECT_NAME}_node_exe
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Export dependencies
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  nebula_msgs
  seyond_nebula_decoder
  pcl_conversions
  nebula_decoders
  nebula_common
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
