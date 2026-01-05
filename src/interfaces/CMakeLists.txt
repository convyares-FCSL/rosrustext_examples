cmake_minimum_required(VERSION 3.8)
project(lesson_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Add interface files here as you introduce new lessons.
# Example:
# set(msg_files
#   "msg/Example.msg"
# )
# set(srv_files
#   "srv/Example.srv"
# )
# set(action_files
#   "action/Example.action"
# )
#
# rosidl_generate_interfaces(${PROJECT_NAME}
#   ${msg_files}
#   ${srv_files}
#   ${action_files}
# )

ament_package()
