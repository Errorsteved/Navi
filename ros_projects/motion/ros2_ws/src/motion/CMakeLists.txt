cmake_minimum_required(VERSION 3.8)
project(yolov5_vehicle_detector)


find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(yolov5_vehicle_detector_msgs REQUIRED)


ament_python_install_package(${PROJECT_NAME})


install(
  PROGRAMS
    src/yolov5_vehicle_detector/src/yolov5_tracker_node.py   
  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)


install(FILES
  /src/yolov5_vehicle_detector/weights/yolov5s.pt
  DESTINATION share/${PROJECT_NAME}/weights
)


ament_export_dependencies(
  rclpy
  sensor_msgs
  std_msgs
  geometry_msgs
  builtin_interfaces
  yolov5_vehicle_detector_msgs
)

ament_package()
