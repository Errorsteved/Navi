from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
import os
def generate_launch_description():

    return LaunchDescription([
        Node(
            package="gps_imu_mqtt",
            executable="mqtt_node",
            name="mqtt_node",
            env={
                'HOME': '/root',
                'LD_LIBRARY_PATH': '/opt/ros/foxy/lib'
                                   ':/ros2_ws/install/gps_imu_mqtt/lib'
                                   ':/ros2_ws/install/gps_imu_msgs/lib',
                "PYTHONPATH": os.environ.get("PYTHONPATH", "") + ":" +
                              "/ros2_ws/install/gps_imu_msgs/lib/python3.8/site-packages:" +
                              "/ros2_ws/install/gps_imu_mqtt/lib/python3.8/site-packages",
            },
        )
    ])