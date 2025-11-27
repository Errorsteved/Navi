cmake_minimum_required(VERSION 3.8)
project(udp_sender)

find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(yolov5_vehicle_detector_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS
    src/udp_sender/src/udp_sender_node.py   
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(
  rclpy
  yolov5_vehicle_detector_msgs
)

ament_python_install_package()

ament_package()
