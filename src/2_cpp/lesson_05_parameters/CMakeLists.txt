cmake_minimum_required(VERSION 3.8)
project(lesson_05_parameters_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(utils_cpp REQUIRED)
find_package(lesson_interfaces REQUIRED)

# ---------------------------------------------------------
# 1. Logic Library (Header-Only)
# ---------------------------------------------------------
add_library(stream_validator INTERFACE)
target_include_directories(stream_validator INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ---------------------------------------------------------
# 2. Publisher Node
# ---------------------------------------------------------
# Naming convention: lesson_05_publisher_cpp
add_executable(lesson_05_publisher_cpp src/publisher.cpp)
ament_target_dependencies(lesson_05_publisher_cpp rclcpp utils_cpp lesson_interfaces)
target_include_directories(lesson_05_publisher_cpp PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ---------------------------------------------------------
# 3. Subscriber Node
# ---------------------------------------------------------
# Naming convention: lesson_05_subscriber_cpp
add_executable(lesson_05_subscriber_cpp src/subscriber.cpp)
ament_target_dependencies(lesson_05_subscriber_cpp rclcpp utils_cpp lesson_interfaces)
# Link the logic library
target_link_libraries(lesson_05_subscriber_cpp stream_validator)

# ---------------------------------------------------------
# 4. Install & Export
# ---------------------------------------------------------
install(TARGETS lesson_05_publisher_cpp lesson_05_subscriber_cpp
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)

# ---------------------------------------------------------
# 5. Tests
# ---------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  
  ament_add_gtest(test_stream_validator test/test.cpp)
  target_link_libraries(test_stream_validator stream_validator)
endif()

ament_package()