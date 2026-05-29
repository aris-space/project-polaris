import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'autonomy_bringup_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'params'),
            glob('params/*.yaml')),
        (os.path.join('share', package_name, 'missions'),
            glob('missions/*')),
        (os.path.join('share', package_name, 'behavior_trees'),
            glob('behavior_trees/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='polaris_pz',
    maintainer_email='paulzambelli@student.ethz.ch',
    description='Autonomy bringup: launch files, params, missions, BTs, and helper nodes.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'export_tracking_bag_csv     = autonomy_bringup_pkg.export_tracking_bag_csv:main',
            'mission_waypoint_loader     = autonomy_bringup_pkg.mission_waypoint_loader:main',
            'mission_waypoints_publisher = autonomy_bringup_pkg.mission_waypoints_publisher:main',
            'nav2_activate               = autonomy_bringup_pkg.nav2_activate:main',
            'nav2_arm_watchdog           = autonomy_bringup_pkg.nav2_arm_watchdog:main',
            'nav2_deactivate             = autonomy_bringup_pkg.nav2_deactivate:main',
            'nav2_lifecycle_diagnostics  = autonomy_bringup_pkg.nav2_lifecycle_diagnostics:main',
            'wgs84_mission_starter       = autonomy_bringup_pkg.WGS84_mission_starter:main',
        ],
    },
)
