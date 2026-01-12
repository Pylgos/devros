cmake_minimum_required(VERSION 3.14)
project(pkg_b)

find_package(ament_cmake REQUIRED)
find_package(pkg_a REQUIRED)

ament_package()
