from setuptools import find_packages, setup

package_name = 'semira_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='semira.kir@epfl.ch',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'my_node = semira_ros.my_node:main',
        'simulation_node = semira_ros.simulation_node:main',
        'ice_node = semira_ros.ice_node:main',
        ],
    },
)
