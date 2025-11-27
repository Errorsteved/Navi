from pathlib import Path
from glob import glob
from setuptools import setup, find_packages

package_name = "path_follower"

data_files = [
    ("share/ament_index/resource_index/packages",
     [f"resource/{package_name}"]),
    (f"share/{package_name}", ["package.xml"]),
    (f"share/{package_name}/launch", glob("launch/*.launch.py")),
]

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(where='src', include=[package_name, f"{package_name}.*"]),
    package_dir={'': 'src'},
    data_files=data_files,
    install_requires=[
        "setuptools",
    ],
    zip_safe=True,
    maintainer="",
    maintainer_email="",
    description="ROS2 Node for Driving Vehicle",
    license="Apache License 2.0",
    entry_points={
        'console_scripts': [
            'path_follower_node = path_follower.path_follower_node:main',
        ],
    },
)