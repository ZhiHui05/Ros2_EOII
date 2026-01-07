cmake_minimum_required(VERSION 3.8)
project(turtle_tracker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclpy REQUIRED)
find_package(turtlesim REQUIRED)
find_package(geometry_msgs REQUIRED)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/TurtleInfo.srv"
)

# Install Python modules
# ament_python_install_package(${PROJECT_NAME})

# Install entry points
install(PROGRAMS
  turtle_tracker/explorer_node.py
  turtle_tracker/client_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
