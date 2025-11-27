from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
import os
def generate_launch_description():
    weights_path = PathJoinSubstitution([
        FindPackageShare("yolov5_vehicle_detector"),
        "weights", 
        "yolov5s.engine"
    ])

    return LaunchDescription([
        Node(
            package="yolov5_vehicle_detector",
            executable="yolov5_tracker_node",
            name="yolov5_tracker_node",
            output="screen",
            env={
                'HOME': '/root',
                'LD_LIBRARY_PATH': '/opt/ros/foxy/lib'
                                   ':/ros2_ws/install/yolov5_vehicle_detector/lib'
                                   ':/ros2_ws/install/yolov5_vehicle_detector_msgs/lib',
                "PYTHONPATH": os.environ.get("PYTHONPATH", "") + ":" +
                              "/ros2_ws/install/yolov5_vehicle_detector_msgs/lib/python3.8/site-packages:" +
                              "/ros2_ws/install/yolov5_vehicle_detector/lib/python3.8/site-packages",
                'DISPLAY': ':1',
            },
            parameters=[{
                "weights": weights_path,
                "device": "cuda:0",
                "imgsz": [640, 640],
                "conf_thre": 0.5,
                "iou_thre": 0.5,
                "camera_source": 0,
                "visualize": True,
            }],
        )
    ])