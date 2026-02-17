import os
from glob import glob
from setuptools import find_packages, setup

package_name = "mavlink_bridge"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "pymavlink"],
    zip_safe=True,
    maintainer="root",
    maintainer_email="ridh.choudhury@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "mavlink_publisher = mavlink_bridge.mavlink_publisher:main",
            "output_monitor = mavlink_bridge.output_monitor:main",
            "ros2_receiver = mavlink_bridge.ros2_receiver:main",
        ],
    },
)
