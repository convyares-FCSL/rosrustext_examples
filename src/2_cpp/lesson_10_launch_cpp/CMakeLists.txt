cmake_minimum_required(VERSION 3.8)
project(lesson_10_launch_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-style=file -Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
# No source code, so mostly install rules for launch files

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install test scripts
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  
  # Install verification script
  install(PROGRAMS
    test/verify_fault_tolerance.py
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

ament_package()
