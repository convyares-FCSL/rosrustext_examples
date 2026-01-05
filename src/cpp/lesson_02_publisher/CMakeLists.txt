cmake_minimum_required(VERSION 3.8)
project(lesson_02_publisher_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(utils_cpp REQUIRED)

add_executable(lesson_02_publisher src/publisher_node.cpp)
target_include_directories(lesson_02_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(lesson_02_publisher rclcpp std_msgs utils_cpp)
target_compile_features(lesson_02_publisher PUBLIC cxx_std_17)

install(TARGETS lesson_02_publisher
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)

ament_package()
