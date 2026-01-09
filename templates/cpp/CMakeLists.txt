cmake_minimum_required(VERSION 3.8)
project(lesson_cpp_template)

# Default to C++17 (Standard for ROS 2 Humble/Jazzy)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Find Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# find_package(std_msgs REQUIRED) # Uncomment if needed

# 2. Build Executable
# We include the source file which contains both the class implementation and main()
add_executable(lesson_node src/lesson_node.cpp)

target_include_directories(lesson_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(lesson_node
  rclcpp
  # std_msgs
)

# 3. Install Rules
# Install the executable to lib/<package_name>/
install(TARGETS lesson_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install headers (preserves the include/lesson_cpp_template/ structure)
install(DIRECTORY include/
  DESTINATION include
)

ament_package()