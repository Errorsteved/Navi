rm -rf build/ install/ log/
source /opt/ros/foxy/setup.bash
colcon build
source /opt/ros/foxy/setup.bash
source /ros2_ws/install/setup.bash
ros2 launch extract_step extract.launch.py # &
# ros2 launch path_follower path_follower.launch.py