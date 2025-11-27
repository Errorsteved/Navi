rm -rf build/ install/ log/
source /opt/ros/foxy/setup.bash
colcon build
source /opt/ros/foxy/setup.bash
source /ros2_ws/install/setup.bash