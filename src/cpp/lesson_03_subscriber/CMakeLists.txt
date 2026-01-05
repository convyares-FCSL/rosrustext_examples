cmake_minimum_required(VERSION 3.8)
project(lesson_03_subscriber_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(utils_cpp REQUIRED)

add_executable(lesson_03_subscriber src/subscriber_node.cpp)
target_include_directories(lesson_03_subscriber PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(lesson_03_subscriber rclcpp std_msgs utils_cpp)
target_compile_features(lesson_03_subscriber PUBLIC cxx_std_17)

install(TARGETS lesson_03_subscriber
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)

ament_package()
