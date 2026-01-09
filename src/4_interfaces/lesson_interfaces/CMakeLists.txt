cmake_minimum_required(VERSION 3.8)
project(lesson_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Add interface files here as you introduce new lessons.
# Example:
set(msg_files
  "msg/MsgCount.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)

ament_package()
