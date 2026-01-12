cmake_minimum_required(VERSION 3.14)
project(example_node)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Create executable
add_executable(example_publisher src/example_publisher.cpp)
ament_target_dependencies(example_publisher rclcpp std_msgs)

# Install targets
install(TARGETS
  example_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
