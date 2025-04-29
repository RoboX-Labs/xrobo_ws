################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(robocon_detector)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenVINO REQUIRED)


################################################################################
# Build
################################################################################

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  /usr/include/intel/openvino/runtime/include
  ${OpenVINO_INCLUDE_DIRS}
  include
)
  
add_executable(openvino_node src/openvino_node.cpp src/inference.cc)
ament_target_dependencies(openvino_node rclcpp vision_msgs sensor_msgs OpenCV cv_bridge)

add_executable(distance_estimation src/distance_estimation.cpp)
ament_target_dependencies(distance_estimation OpenCV)

target_link_libraries(openvino_node
  ${OpenCV_LIBS}
  /usr/lib/libopenvino.so # OpenVINO shared library
)

################################################################################
# Install
################################################################################

install(TARGETS
  openvino_node
  distance_estimation
  DESTINATION lib/${PROJECT_NAME})

ament_package()