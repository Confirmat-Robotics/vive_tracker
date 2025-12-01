from glob import glob
from setuptools import setup, find_packages

package_name = "vive_ros2"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test", "test.*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
    ],
    install_requires=["setuptools", "pydantic", "scipy"],
    zip_safe=True,
    maintainer="michael",
    maintainer_email="michaelrequi@gmail.com",
    description="ROS 2 interface for streaming Vive Tracker pose data.",
    license="MIT",
    python_requires=">=3.8",
    entry_points={
        "console_scripts": [
            "vive_tracker_node = vive_ros2.vive_tracker_node:main",
            "vive_tracker_client = vive_ros2.vive_tracker_client:main",
        ],
    },
)
