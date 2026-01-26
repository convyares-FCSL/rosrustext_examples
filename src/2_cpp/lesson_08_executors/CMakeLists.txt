cmake_minimum_required(VERSION 3.8)
project(lesson_08_executors_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(action_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(utils_cpp REQUIRED)
find_package(lesson_interfaces REQUIRED)
find_package(rclcpp_components REQUIRED)

# ---------------------------------------------------------
# 1. Header-only logic (interface target for include paths)
# ---------------------------------------------------------
add_library(lesson_08_logic INTERFACE)
target_include_directories(lesson_08_logic INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ---------------------------------------------------------
# 2. Action Server (Lifecycle node)
# ---------------------------------------------------------
add_executable(lesson_08_actions_server src/action_server_node.cpp src/action_server.cpp)
ament_target_dependencies(lesson_08_actions_server
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  action_msgs
  lifecycle_msgs
  utils_cpp
  lesson_interfaces
  rclcpp_components
)
target_link_libraries(lesson_08_actions_server lesson_08_logic)
target_include_directories(lesson_08_actions_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ---------------------------------------------------------
# 3. Action Client (demo tool)
# ---------------------------------------------------------
add_executable(lesson_08_actions_client src/action_client_node.cpp)
ament_target_dependencies(lesson_08_actions_client
  rclcpp
  rclcpp_action
  action_msgs
  utils_cpp
  lesson_interfaces
)
target_link_libraries(lesson_08_actions_client lesson_08_logic)
target_include_directories(lesson_08_actions_client PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ---------------------------------------------------------
# 4. Install & Export
# ---------------------------------------------------------
install(TARGETS lesson_08_actions_server lesson_08_actions_client
  DESTINATION lib/${PROJECT_NAME}
)

# ---------------------------------------------------------
# Topic 09 (Composition): Component Registration
#
# Build a shared library containing composable node entrypoints so the nodes are
# discoverable/loadable via canonical `ros2 component types/load/unload`.
# The existing standalone executables remain available.
# ---------------------------------------------------------
add_library(${PROJECT_NAME}_component SHARED
  src/action_server_node.cpp
  src/action_server.cpp
)
target_compile_definitions(${PROJECT_NAME}_component PRIVATE "COMPOSITION_BUILD")
ament_target_dependencies(${PROJECT_NAME}_component
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  action_msgs
  lifecycle_msgs
  rclcpp_components
  utils_cpp
  lesson_interfaces
)
target_link_libraries(${PROJECT_NAME}_component lesson_08_logic)
target_include_directories(${PROJECT_NAME}_component PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

rclcpp_components_register_nodes(${PROJECT_NAME}_component
  "lesson_08_executors_cpp::ActionServerNode"
)

install(TARGETS ${PROJECT_NAME}_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)

# ---------------------------------------------------------
# 5. Tests
# ---------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Unit Tests (ROS-free)
  ament_add_gtest(test_lesson_08_unit test/test_unit.cpp)
  target_link_libraries(test_lesson_08_unit lesson_08_logic)
  target_include_directories(test_lesson_08_unit PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )

  # Integration Test (launch_testing)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_integration.py
    TARGET lesson_08_actions_server
  )
endif()

ament_package()
