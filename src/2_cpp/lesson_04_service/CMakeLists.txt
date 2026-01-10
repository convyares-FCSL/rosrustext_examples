cmake_minimum_required(VERSION 3.8)
project(lesson_04_service_cpp)

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
add_library(stats_logic INTERFACE)
target_include_directories(stats_logic INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ---------------------------------------------------------
# 2. Service Server Node
# ---------------------------------------------------------
add_executable(service_server src/service_node.cpp)
ament_target_dependencies(service_server rclcpp utils_cpp lesson_interfaces)
target_link_libraries(service_server stats_logic)

# ---------------------------------------------------------
# 3. Service Client Node
# ---------------------------------------------------------
add_executable(service_client src/client_node.cpp)
ament_target_dependencies(service_client rclcpp utils_cpp lesson_interfaces)
target_include_directories(service_client PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ---------------------------------------------------------
# 4. Install & Export
# ---------------------------------------------------------
install(TARGETS service_server service_client
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)

# ---------------------------------------------------------
# 5. Tests
# ---------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  
  # Create the test executable
  ament_add_gtest(test_stats_logic test/test_stats_logic.cpp)
  
  # Link it to our header-only logic library
  target_link_libraries(test_stats_logic stats_logic)
endif()

ament_package()