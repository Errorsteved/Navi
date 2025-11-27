cmake_minimum_required(VERSION 3.8)
project(path_follower)


find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(extract_step_msgs REQUIRED)


ament_python_install_package(${PROJECT_NAME})


install(
  PROGRAMS
    src/path_follower/src/path_follower_node.py   
  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)


ament_export_dependencies(
  rclpy
  sensor_msgs
  std_msgs
  geometry_msgs
  builtin_interfaces
  extract_step_msgs
)

ament_package()
