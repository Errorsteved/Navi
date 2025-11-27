rm -rf build/ install/ log/
source /opt/ros/foxy/setup.bash
colcon build
source /opt/ros/foxy/setup.bash
source /ros2_ws/install/setup.bash
ros2 launch gps_imu_publisher gps_imu.launch.py
# ros2 launch gps_imu_publisher gps_imu.launch.py &
# ros2 launch gps_imu_mqtt mqtt.launch.py