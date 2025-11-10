from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'prototype_gelb_package'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*'))
    
    ],
    install_requires=['setuptools'
                      ],
    zip_safe=True,
    maintainer='polaris-rc',
    maintainer_email='ridh.choudhury@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pwm_node = prototype_gelb_package.pwm_node:main',
            'pwm_node_ext = prototype_gelb_package.pwm_node_ext:main',
        ],
    },
)
