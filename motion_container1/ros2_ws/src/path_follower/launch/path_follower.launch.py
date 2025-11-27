from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
import os

ament_prefix_path = os.environ.get("AMENT_PREFIX_PATH", "/ros2_ws/install")

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="path_follower",
            executable="path_follower_node",
            name="path_follower_node",
            env={
                'HOME': '/root',
                'AMENT_PREFIX_PATH': ament_prefix_path,
                'LD_LIBRARY_PATH': '/opt/ros/foxy/lib' +
                                   ':/ros2_ws/install/path_follower/lib' +
                                   ':/ros2_ws/install/extract_step_msgs/lib',
                "PYTHONPATH": os.environ.get("PYTHONPATH", "") + ":" +
                              "/ros2_ws/install/extract_step_msgs/lib/python3.8/site-packages:" + 
                              "/ros2_ws/install/path_follower/lib/python3.8/site-packages"
            },
        )
    ])