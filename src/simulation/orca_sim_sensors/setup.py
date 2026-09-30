from setuptools import find_packages, setup

package_name = 'orca_sim_sensors'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Noel Buehler',
    maintainer_email='noel.buehler@aris-space.ch',
    description='Synthetic sensors for the Polaris simulation.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'sim_gnss_node = orca_sim_sensors.sim_gnss_node:main',
        ],
    },
)
