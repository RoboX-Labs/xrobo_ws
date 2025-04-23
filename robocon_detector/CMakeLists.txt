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
include_directories(include)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  /usr/include/intel/openvino/runtime/include
  ${OpenVINO_INCLUDE_DIRS}
)

add_executable(object_detection src/object_detection/object_detection.cpp src/object_detection/inference.cc)
ament_target_dependencies(object_detection rclcpp vision_msgs sensor_msgs OpenCV cv_bridge)

target_link_libraries(object_detection
  ${OpenCV_LIBS}
  /usr/lib/libopenvino.so # OpenVINO shared library
)

add_executable(web_cam src/object_detection/web_cam.cpp)
ament_target_dependencies(web_cam rclcpp sensor_msgs cv_bridge OpenCV)

# Add OpenCV using target_link_libraries to link the OpenCV libraries or can use ament_target_dependencies
target_link_libraries(web_cam
  ${OpenCV_LIBS}
)


################################################################################
# Install
################################################################################

install(TARGETS
  object_detection
  web_cam
  DESTINATION lib/${PROJECT_NAME})

ament_package()