cmake_minimum_required(VERSION 3.14)
project(pkg_e_standalone)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Create a standalone executable
add_executable(standalone_node src/standalone_node.cpp)
ament_target_dependencies(standalone_node rclcpp)

# Install executable
install(TARGETS standalone_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
