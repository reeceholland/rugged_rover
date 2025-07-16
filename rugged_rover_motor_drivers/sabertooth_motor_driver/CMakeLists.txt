################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.8)
project(sabertooth_motor_driver)

if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif ()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Required dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_cmake_gtest REQUIRED)

# === Driver Library ===
add_library(sabertooth_driver_lib
  src/driver.cpp
)

target_include_directories(sabertooth_driver_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

ament_target_dependencies(sabertooth_driver_lib
  rclcpp
)

# === Node Executable ===
add_executable(sabertooth_driver_node
  src/sabertooth_driver_node.cpp
  src/main.cpp
)

target_link_libraries(sabertooth_driver_node
  sabertooth_driver_lib
)

ament_target_dependencies(sabertooth_driver_node
  rclcpp
  sensor_msgs
  std_msgs
)

# === Unit Tests ===
ament_add_gtest(test_driver
  test/test_driver.cpp
)

target_link_libraries(test_driver
  sabertooth_driver_lib
  pthread
)

ament_target_dependencies(test_driver
  rclcpp
)

# === Install ===
install(TARGETS
  sabertooth_driver_node
  sabertooth_driver_lib
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# === Export ===
ament_export_include_directories(include)
ament_export_dependencies(rclcpp)

ament_package()