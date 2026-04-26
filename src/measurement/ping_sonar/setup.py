from setuptools import find_packages, setup
import os
from glob import glob

package_name = "ping_sonar"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "bluerobotics-ping", "pyserial", "future"],
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
        "console_scripts": ["ice_measurement = ping_sonar.ice_measurement:main"],
    },
)
