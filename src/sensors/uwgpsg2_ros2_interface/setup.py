from setuptools import setup
import os
from glob import glob

package_name = 'uwgpsg2_ros2_interface'
#submodules = 'examples' # waterlinked Python API

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        #(os.path.join('share', package_name, 'config'), glob('config/*.perspective'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nadir',
    maintainer_email='nadir.kapetanovic@fer.hr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'uwgpsg2_ros2_interface = uwgpsg2_ros2_interface.uwgpsg2_ros2_interface:main'                 
        ],
    },
)
