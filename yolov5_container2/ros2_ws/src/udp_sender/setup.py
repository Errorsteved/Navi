from pathlib import Path
from glob import glob
from setuptools import setup, find_packages

package_name = "udp_sender"

weight_file = Path("src") / "udp_sender"

data_files = [
    ("share/ament_index/resource_index/packages",
     [f"resource/{package_name}"]),
    (f"share/{package_name}", ["package.xml"]),
    (f"share/{package_name}/launch", glob("launch/*.launch.py"))
]

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(where='src', include=[package_name, f"{package_name}.*"]),
    package_dir={'': 'src'},
    data_files=data_files,
    entry_points={
        'console_scripts': [
            'udp_sender_node = udp_sender.udp_sender_node:main',
        ],
    },
)