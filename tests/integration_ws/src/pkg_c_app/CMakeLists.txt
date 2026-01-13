cmake_minimum_required(VERSION 3.14)
project(pkg_c_app)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pkg_a_lib REQUIRED)
find_package(pkg_b_lib REQUIRED)

# Create executable that uses both libraries
add_executable(pkg_c_app src/main.cpp)
target_link_libraries(pkg_c_app
  pkg_a_lib::pkg_a_lib
  pkg_b_lib::pkg_b_lib
)
ament_target_dependencies(pkg_c_app rclcpp)

# Install executable
install(TARGETS pkg_c_app
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
