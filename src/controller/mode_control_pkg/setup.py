from setuptools import find_packages, setup
from glob import glob

package_name = "mode_control_pkg"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="root",
    maintainer_email="buehlern@student.ethz.ch",
    description="TODO: Package description",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "joy_handler_node = mode_control_pkg.joy_handler_node:main",
            "manual_control_node = mode_control_pkg.manual_control_node:main",
            "manual_altitude_hold_control_node = mode_control_pkg.manual_altitude_hold_control_node:main",
            "emergency_stop_mode_node = mode_control_pkg.emergency_stop_mode_node:main",
            "mode_control_node = mode_control_pkg.mode_control_node:main",
            "collision_avoidance_node = mode_control_pkg.collision_avoidance:main",
        ],
    },
)
