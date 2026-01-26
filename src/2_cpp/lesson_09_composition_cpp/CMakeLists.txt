cmake_minimum_required(VERSION 3.8)
project(lesson_09_composition_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(FILES README.md THEORY.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
