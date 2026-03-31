import os
from glob import glob

from setuptools import find_packages, setup

package_name = "imu_orientation_pkg"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="root",
    maintainer_email="todo@example.com",
    description="IMU quaternion to RPY topics",
    license="TODO: License declaration",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "imu_to_rpy_node = imu_orientation_pkg.imu_to_rpy_node:main",
        ]
    },
)
