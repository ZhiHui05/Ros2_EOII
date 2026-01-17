cmake_minimum_required(VERSION 3.5)
project(turtle_tracker_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/TurtleInfo.srv"
  "action/TurtleInfo.action"
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
