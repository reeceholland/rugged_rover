cmake_minimum_required(VERSION 3.8)
project(rugged_rover_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(GPIODCXX REQUIRED IMPORTED_TARGET libgpiodcxx)

add_library(rover_manager_component SHARED
  src/rover_manager_node.cpp
)

target_compile_features(rover_manager_component PUBLIC cxx_std_17)

target_include_directories(rover_manager_component
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${GPIODCXX_INCLUDE_DIRS}
)

target_compile_options(rover_manager_component
  PRIVATE
    ${GPIODCXX_CFLAGS_OTHER}
)

target_link_libraries(rover_manager_component
  PkgConfig::GPIODCXX
)

ament_target_dependencies(rover_manager_component
  rclcpp
  rclcpp_components
  std_msgs
)

rclcpp_components_register_node(rover_manager_component
  PLUGIN "rugged_rover_manager::RoverManagerNode"
  EXECUTABLE rover_manager_node
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS rover_manager_component
  EXPORT export_rover_manager_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_rover_manager_node HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  rclcpp_components
  std_msgs
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_mode_switch test/test_mode_switch.cpp)
  target_include_directories(test_mode_switch PRIVATE include)
  target_compile_features(test_mode_switch PUBLIC cxx_std_17)
endif()

ament_package()