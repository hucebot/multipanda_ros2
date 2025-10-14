cmake_minimum_required(VERSION 3.8)
project(custom_msgs)

find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GripperWidth.msg"
  DEPENDENCIES std_msgs
)

ament_package()