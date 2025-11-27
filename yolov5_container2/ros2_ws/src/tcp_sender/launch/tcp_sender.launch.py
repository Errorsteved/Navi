from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
import os
def generate_launch_description():

    return LaunchDescription([
        Node(
            package="tcp_sender",
            executable="tcp_sender_node",
            name="tcp_sender_node",
            output="screen",
            env={
                'HOME': '/root',
                'LD_LIBRARY_PATH': '/opt/ros/foxy/lib'
                                   ':/ros2_ws/install/tcp_sender/lib'
                                   ':/ros2_ws/install/yolov5_vehicle_detector_msgs/lib',
                "PYTHONPATH": os.environ.get("PYTHONPATH", "") + ":" +
                              "/ros2_ws/install/yolov5_vehicle_detector_msgs/lib/python3.8/site-packages:" +
                              "/ros2_ws/install/tcp_sender/lib/python3.8/site-packages",
                'DISPLAY': ':1',
            },
        )
    ])