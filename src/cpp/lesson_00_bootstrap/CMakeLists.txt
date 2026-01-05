cmake_minimum_required(VERSION 3.8)
project(lesson_00_bootstrap_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(lesson_00_bootstrap src/bootstrap_node.cpp)
target_include_directories(lesson_00_bootstrap PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(lesson_00_bootstrap rclcpp)
target_compile_features(lesson_00_bootstrap PUBLIC cxx_std_17)

install(TARGETS lesson_00_bootstrap
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)

ament_package()
