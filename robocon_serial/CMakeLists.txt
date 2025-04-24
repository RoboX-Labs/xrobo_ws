cmake_minimum_required(VERSION 3.8)
project(robocon_serial)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# Find Threads
set(THREADS_HAVE_PTHREAD_ARG 1)
find_package(Threads REQUIRED)

# Find LibSerial using pkg-config
find_package(PkgConfig)
pkg_check_modules(SERIAL libserial)

add_executable(serialcom_hex "src/robocon_serial.cpp" "src/Command.cpp" "src/SerialComHex.cpp")
ament_target_dependencies(serialcom_hex rclcpp std_msgs SERIAL)

# Include directories
# target_include_directories(serialcom_hex PRIVATE ${SERIAL_INCLUDE_DIRS})

# Link libraries
# target_link_libraries(serialcom_hex PRIVATE ${SERIAL_LDFLAGS} ${CMAKE_THREAD_LIBS_INIT})

install(TARGETS
  serialcom_hex
  DESTINATION lib/${PROJECT_NAME})

ament_package()