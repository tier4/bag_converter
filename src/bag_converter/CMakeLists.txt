cmake_minimum_required(VERSION 3.8)
project(bag_converter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Set C++ standard (must be after ament_cmake)
# Ensure C++17 or higher (default to 17 if not set)
if(NOT DEFINED CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
elseif(CMAKE_CXX_STANDARD LESS 17)
  message(FATAL_ERROR "CMAKE_CXX_STANDARD must be 17 or higher (got ${CMAKE_CXX_STANDARD})")
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rosbag2_storage_default_plugins REQUIRED)
find_package(rcpputils REQUIRED)
find_package(rcutils REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nebula_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nebula_decoders REQUIRED)
find_package(nebula_common REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(Eigen3 REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

# Generate Seyond messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SeyondPacket.msg"
  "msg/SeyondScan.msg"
  DEPENDENCIES builtin_interfaces std_msgs
)

# Seyond SDK paths
set(SEYOND_SDK_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../dependencies/seyond_sdk)
set(SEYOND_SDK_LIB_PATH ${SEYOND_SDK_PATH}/lib/libinnolidarsdkclient.a)

# Build Seyond SDK if needed
if(NOT EXISTS ${SEYOND_SDK_LIB_PATH})
  message(STATUS "Seyond SDK not built yet. Building it now...")
  execute_process(
    COMMAND bash -c "cd ${SEYOND_SDK_PATH}/build && ./build_unix.sh"
    RESULT_VARIABLE SDK_BUILD_RESULT
  )
  if(NOT SDK_BUILD_RESULT EQUAL 0)
    message(FATAL_ERROR "Failed to build Seyond SDK")
  endif()
endif()


# Get message typesupport target
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

# Build unified bag_converter executable
add_executable(bag_converter_bin
  src/bag_converter.cpp
  src/seyond_decoder.cpp
  src/nebula_decoder.cpp
  src/timescale.cpp
  src/tf_transformer.cpp
)

set_target_properties(bag_converter_bin PROPERTIES OUTPUT_NAME bag_converter)

# Set version from build argument (default: "latest")
if(NOT DEFINED BAG_CONVERTER_VERSION)
  set(BAG_CONVERTER_VERSION "latest")
endif()
target_compile_definitions(bag_converter_bin PRIVATE BAG_CONVERTER_VERSION="${BAG_CONVERTER_VERSION}")

target_include_directories(bag_converter_bin PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${SEYOND_SDK_PATH}/src
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${nebula_decoders_INCLUDE_DIRS}
  ${nebula_common_INCLUDE_DIRS}
)

target_link_libraries(bag_converter_bin
  ${SEYOND_SDK_PATH}/lib/libinnolidarsdkclient.a
  ${SEYOND_SDK_PATH}/lib/libinnolidarsdkcommon.a
  ${SEYOND_SDK_PATH}/lib/libinnolidarutils.a
  ${PCL_LIBRARIES}
)

ament_target_dependencies(bag_converter_bin
  rclcpp
  sensor_msgs
  nebula_msgs
  std_msgs
  builtin_interfaces
  rosbag2_cpp
  rosbag2_storage
  pcl_conversions
  nebula_decoders
  nebula_common
  tf2
  tf2_msgs
  tf2_sensor_msgs
)

# Link with message generation
target_link_libraries(bag_converter_bin "${cpp_typesupport_target}")

install(TARGETS bag_converter_bin
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
