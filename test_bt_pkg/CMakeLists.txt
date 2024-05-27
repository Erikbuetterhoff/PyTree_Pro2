cmake_minimum_required(VERSION 3.5)
project(test_bt_pkg)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Test.action"
)

ament_package()
