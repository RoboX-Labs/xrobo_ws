cmake_minimum_required(VERSION 3.5)
project(robocon_camera)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(realsense2 REQUIRED)


add_executable(web_cam src/web_cam.cpp)
ament_target_dependencies(web_cam rclcpp sensor_msgs cv_bridge OpenCV)

# Add OpenCV using target_link_libraries to link the OpenCV libraries or can use ament_target_dependencies
target_link_libraries(web_cam
  ${OpenCV_LIBS}
)

include_directories(include ${realsense_INCLUDE_DIR})

add_executable(rs_d455_node src/rs_d455_node.cpp)
ament_target_dependencies(rs_d455_node rclcpp std_msgs sensor_msgs cv_bridge OpenCV tf2_ros)
target_include_directories(rs_d455_node PRIVATE ${realsense_INCLUDE_DIR})
target_link_libraries(rs_d455_node ${realsense2_LIBRARY})

add_executable(rs_t265_node src/rs_t265_node.cpp)
ament_target_dependencies(rs_t265_node rclcpp std_msgs sensor_msgs cv_bridge OpenCV tf2_ros)
target_include_directories(rs_t265_node PRIVATE ${realsense_INCLUDE_DIR})
target_link_libraries(rs_t265_node ${realsense2_LIBRARY})

install(TARGETS
  web_cam  
  rs_t265_node
  rs_d455_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

ament_package()