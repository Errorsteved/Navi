from pathlib import Path
from glob import glob
from setuptools import setup, find_packages

package_name = "yolov5_vehicle_detector"

weight_file = Path("src") / "yolov5_vehicle_detector" / "weights" / "yolov5s.engine"

data_files = [
    ("share/ament_index/resource_index/packages",
     [f"resource/{package_name}"]),
    (f"share/{package_name}", ["package.xml"]),
    (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    (f"share/{package_name}/weights", [str(weight_file)]),
]

package_data = {package_name: ["weights/yolov5s.engine"]}

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(where='src', include=[package_name, f"{package_name}.*"]),
    package_dir={'': 'src'},
    package_data=package_data,
    data_files=data_files,
    install_requires=[
        "setuptools",
        "numpy",
        "pandas",
        "Pillow",
    ],
    zip_safe=True,
    maintainer="",
    maintainer_email="",
    description="ROS2 Node for YOLOv5 Vehicle Detection and Tracking",
    license="Apache License 2.0",
    entry_points={
        'console_scripts': [
            'yolov5_tracker_node = yolov5_vehicle_detector.yolov5_tracker_node:main',
        ],
    },
)