from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'keller_26x_pkg'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='polaris_pz',
    maintainer_email='paulzambelli@student.ethz.ch',
    description='A package for reading the keller_26x pressure sensor and publishing the data as a ROS2 topic.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'keller_26x = keller_26x_pkg.keller_26x_node:main'
        ],
    },
)
