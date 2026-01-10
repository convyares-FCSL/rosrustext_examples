cmake_minimum_required(VERSION 3.8)
project(lesson_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_generator_rs REQUIRED)

# 1. Define Messages
set(msg_files
  "msg/MsgCount.msg"
)

# 2. Define Services
set(srv_files
  "srv/ComputeStats.srv"
)

# 3. Generate Code
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
)

ament_package()