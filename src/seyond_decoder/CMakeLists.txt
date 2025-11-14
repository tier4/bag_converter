cmake_minimum_required(VERSION 3.14)
project(seyond_decoder)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Check and build Seyond SDK if needed
set(SEYOND_SDK_PATH ${CMAKE_CURRENT_SOURCE_DIR}/src/seyond_sdk/)
set(SEYOND_SDK_LIB_PATH ${SEYOND_SDK_PATH}/lib/libinnolidarsdkclient.a)

if(NOT EXISTS ${SEYOND_SDK_LIB_PATH})
  message(STATUS "Seyond SDK not built yet. Building it now...")
  
  execute_process(
    COMMAND bash -c "cd ${SEYOND_SDK_PATH}/build && ./build_unix.sh"
    RESULT_VARIABLE SDK_BUILD_RESULT
    OUTPUT_VARIABLE SDK_BUILD_OUTPUT
    ERROR_VARIABLE SDK_BUILD_ERROR
  )
  
  if(NOT SDK_BUILD_RESULT EQUAL 0)
    message(FATAL_ERROR "Failed to build Seyond SDK: ${SDK_BUILD_ERROR}")
  else()
    message(STATUS "Seyond SDK built successfully")
  endif()
else()
  message(STATUS "Seyond SDK already built")
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)

# Set C++ standard (must be after ament_cmake)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(Eigen3 REQUIRED)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SeyondPacket.msg"
  "msg/SeyondScan.msg"
  DEPENDENCIES builtin_interfaces std_msgs
)

# Include directories
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# Add SDK include directories
include_directories(
  ${SEYOND_SDK_PATH}/src
)

# Create library
add_library(${PROJECT_NAME}_lib SHARED
  src/seyond_decoder.cpp
)

target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link SDK libraries directly
target_link_libraries(${PROJECT_NAME}_lib
  ${SEYOND_SDK_PATH}/lib/libinnolidarsdkclient.a
  ${SEYOND_SDK_PATH}/lib/libinnolidarsdkcommon.a
  ${SEYOND_SDK_PATH}/lib/libinnolidarutils.a
  ${PCL_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  sensor_msgs
  pcl_ros
  pcl_conversions
)

# Link with message generation
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(${PROJECT_NAME}_lib "${cpp_typesupport_target}")

# Export library
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_include_directories(
  include
  ${SEYOND_SDK_PATH}/src
)
ament_export_dependencies(
  rosidl_default_runtime
  rclcpp
  sensor_msgs
  pcl_ros
  pcl_conversions
  PCL
  Eigen3
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

# Build example if requested
option(BUILD_EXAMPLES "Build example programs" ON)
if(BUILD_EXAMPLES)
  add_executable(seyond_decoder_example
    example/decoder_example.cpp
  )
  
  target_link_libraries(seyond_decoder_example
    ${PROJECT_NAME}_lib
  )
  
  ament_target_dependencies(seyond_decoder_example
    rclcpp
    sensor_msgs
  )
  
  
  install(TARGETS
    seyond_decoder_example
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()