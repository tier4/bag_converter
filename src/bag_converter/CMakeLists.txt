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
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rosbag2_storage_default_plugins REQUIRED)
find_package(rcpputils REQUIRED)
find_package(rcutils REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nebula_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(seyond_decoder REQUIRED)
find_package(nebula_decoders REQUIRED)
find_package(nebula_common REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(Eigen3 REQUIRED)
find_package(builtin_interfaces REQUIRED)


# Build seyond_bag_decoder executable
add_executable(seyond_bag_decoder
  src/seyond_bag_decoder.cpp
  src/seyond_decoder.cpp
)

target_include_directories(seyond_bag_decoder PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${seyond_decoder_INCLUDE_DIRS}
)

target_link_libraries(seyond_bag_decoder
  seyond_decoder::seyond_decoder_lib
)

ament_target_dependencies(seyond_bag_decoder
  rclcpp
  sensor_msgs
  rosbag2_cpp
  rosbag2_storage
  seyond_decoder
)

install(TARGETS seyond_bag_decoder
  DESTINATION lib/${PROJECT_NAME}
)

# Build seyond_nebula_bag_decoder executable
add_executable(seyond_nebula_bag_decoder
  src/seyond_nebula_bag_decoder.cpp
  src/nebula_decoder.cpp
)

target_include_directories(seyond_nebula_bag_decoder PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${nebula_decoders_INCLUDE_DIRS}
  ${nebula_common_INCLUDE_DIRS}
)

target_link_libraries(seyond_nebula_bag_decoder
  ${PCL_LIBRARIES}
)

ament_target_dependencies(seyond_nebula_bag_decoder
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
)

install(TARGETS seyond_nebula_bag_decoder
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