import os
from glob import glob

from setuptools import find_packages, setup

package_name = "pressure_pose_pkg"

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
    description="Fluid pressure to z-only pose for depth aiding",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "pressure_z_ned_to_pose_node = pressure_pose_pkg.pressure_z_ned_to_pose_node:main",
        ]
    },
)
