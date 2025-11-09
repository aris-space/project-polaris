from setuptools import setup

package_name = 'demo_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=False,
    author='noel',
    author_email='noel@example.com',
    description='Demo ROS2 Python package (talker/listener)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'talker = demo_package.talker:main',
            'listener = demo_package.listener:main',
        ],
    },
)
