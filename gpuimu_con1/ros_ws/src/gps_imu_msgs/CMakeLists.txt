cmake_minimum_required(VERSION 3.8)
project(gps_imu_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GpsImuData.msg"
  "msg/GpsImuDatas.msg"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
)


ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)
ament_package()
