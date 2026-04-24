import os
from glob import glob

from setuptools import find_packages, setup

package_name = "water_properties_pkg"

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
    maintainer="Gleb Platonov",
    maintainer_email="gleb.platonov@aris-space.ch",
    description="On-demand speed-of-sound computation for fresh-water DVL and SBL calibration.",
    license="Apache-2.0",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "water_sos_node = water_properties_pkg.water_sos_node:main",
        ]
    },
)
