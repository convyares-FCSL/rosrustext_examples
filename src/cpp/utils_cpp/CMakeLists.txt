cmake_minimum_required(VERSION 3.8)
project(utils_cpp)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_dependencies(rclcpp)

ament_package()
