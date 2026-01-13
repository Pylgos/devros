cmake_minimum_required(VERSION 3.14)
project(pkg_a_lib)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Create a simple library
add_library(pkg_a_lib SHARED src/pkg_a_lib.cpp)
target_include_directories(pkg_a_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(pkg_a_lib rclcpp)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install library
install(TARGETS pkg_a_lib
  EXPORT export_pkg_a_lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Export targets
ament_export_targets(export_pkg_a_lib HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp)

ament_package()
