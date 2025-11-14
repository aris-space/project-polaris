from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'pt_gelb_pkg'

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
            'thrust_control_fw = pt_gelb_pkg.thrust_control_fw:main',
            'thrust_control_fwbw = pt_gelb_pkg.thrust_control_fwbw:main',
        ],
    },
)
