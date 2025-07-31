cmake_minimum_required(VERSION 3.8)
project(rugged_rover_hardware_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rugged_rover_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(controller_interface REQUIRED)

# Include headers
include_directories(
  include
  ${rugged_rover_interfaces_INCLUDE_DIRS}
)

# Add the hardware interface library
add_library(${PROJECT_NAME} SHARED
  src/sabertooth/sabertooth_system_interface.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  hardware_interface
  pluginlib
  rugged_rover_interfaces
)

# Export the plugin description file
pluginlib_export_plugin_description_file(hardware_interface src/sabertooth/hardware_plugin.xml)

# Export library
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  hardware_interface
  pluginlib
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)

# Install library
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install plugin description
install(FILES src/sabertooth/hardware_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_sabertooth_interface
    test/test_sabertooth_interface.cpp
  )
  target_link_libraries(test_sabertooth_interface
    ${PROJECT_NAME}
  )
endif()

ament_package()