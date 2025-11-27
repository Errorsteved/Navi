cmake_minimum_required(VERSION 3.8)
project(tcp_sender)

# 通过 ament_cmake_python 找到 Python 相关的依赖
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(yolov5_vehicle_detector_msgs REQUIRED)


ament_python_install_package(${PROJECT_NAME})


install(
  PROGRAMS
    src/tcp_sender/src/tcp_sender_node.py   # ← 根据实际路径修改
  DESTINATION lib/${PROJECT_NAME}
)

# 安装launch文件
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

src/tcp_sender/src/tcp_sender_node.py

ament_export_dependencies(
  rclpy
  yolov5_vehicle_detector_msgs
)

# 生成包
ament_python_install_package()

ament_package()
