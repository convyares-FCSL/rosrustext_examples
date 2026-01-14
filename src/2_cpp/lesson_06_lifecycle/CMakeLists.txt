cmake_minimum_required(VERSION 3.8)
project(lesson_06_lifecycle_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
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
# 2. Lifecycle Publisher Node
# ---------------------------------------------------------
add_executable(lesson_06_lifecycle_publisher src/publisher.cpp)
ament_target_dependencies(lesson_06_lifecycle_publisher 
  rclcpp 
  rclcpp_lifecycle 
  lifecycle_msgs
  utils_cpp 
  lesson_interfaces
)
target_include_directories(lesson_06_lifecycle_publisher PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ---------------------------------------------------------
# 3. Lifecycle Subscriber Node
# ---------------------------------------------------------
add_executable(lesson_06_lifecycle_subscriber src/subscriber.cpp)
ament_target_dependencies(lesson_06_lifecycle_subscriber 
  rclcpp 
  rclcpp_lifecycle 
  lifecycle_msgs
  utils_cpp 
  lesson_interfaces
)
target_link_libraries(lesson_06_lifecycle_subscriber stream_validator)
target_include_directories(lesson_06_lifecycle_subscriber PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ---------------------------------------------------------
# 4. Install & Export
# ---------------------------------------------------------
install(TARGETS lesson_06_lifecycle_publisher lesson_06_lifecycle_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)

# ---------------------------------------------------------
# 5. Tests
# ---------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  
  # Unit Tests
  ament_add_gtest(test_stream_validator test/test_unit.cpp)
  
  ament_target_dependencies(test_stream_validator rclcpp) 
  
  target_link_libraries(test_stream_validator stream_validator)

  # Integration Test
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(
    test/test_integration.py
    TARGET lesson_06_lifecycle_publisher
  )
endif()