cmake_minimum_required(VERSION 3.14)
project(seyond_nebula_decoder)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)

# Set C++ standard (must be after ament_cmake)
# Ensure C++17 or higher (default to 17 if not set)
if(NOT DEFINED CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
elseif(CMAKE_CXX_STANDARD LESS 17)
  message(FATAL_ERROR "CMAKE_CXX_STANDARD must be 17 or higher (got ${CMAKE_CXX_STANDARD})")
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(Eigen3 REQUIRED)
find_package(nebula_msgs REQUIRED)
find_package(nebula_decoders REQUIRED)
find_package(nebula_common REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(sensor_msgs REQUIRED)

# Include directories
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${nebula_decoders_INCLUDE_DIRS}
  ${nebula_common_INCLUDE_DIRS}
)

# Create library
add_library(${PROJECT_NAME}_lib SHARED
  src/seyond_nebula_decoder.cpp
)

target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../bag_converter/include>
)

target_link_libraries(${PROJECT_NAME}_lib
  ${PCL_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}_lib
  nebula_msgs
  nebula_decoders
  nebula_common
  builtin_interfaces
  rclcpp
  pcl_conversions
  sensor_msgs
)

# Export library
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(
  PCL
  Eigen3
  nebula_msgs
  nebula_decoders
  nebula_common
  builtin_interfaces
  rclcpp
)

# Install library
install(
  TARGETS ${PROJECT_NAME}_lib
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()