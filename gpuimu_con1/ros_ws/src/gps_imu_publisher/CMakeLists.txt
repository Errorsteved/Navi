cmake_minimum_required(VERSION 3.8)
project(gps_imu_publisher)


find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(gps_imu_msgs REQUIRED)


ament_python_install_package(${PROJECT_NAME})


install(
  PROGRAMS
    src/gps_imu_publisher/src/gps_imu_node.py
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
  gps_imu_msgs
)

ament_package()
