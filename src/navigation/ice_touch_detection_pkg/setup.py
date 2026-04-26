import os
from glob import glob
from setuptools import find_packages, setup

package_name = "ice_touch_detection_pkg"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="root",
    maintainer_email="todo@example.com",
    description="Detects contact between the AUV tower and the ice ceiling.",
    license="TODO: License declaration",
    extras_require={
        "test": ["pytest"],
    },
    entry_points={
        "console_scripts": [
            "ice_touch_detection_node = ice_touch_detection_pkg.ice_touch_detection_node:main",
        ]
    },
)
