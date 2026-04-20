from setuptools import setup

package_name = "recorder_controller_node"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Polaris",
    maintainer_email="polaris@example.com",
    description="ROS2 rosbag2 recorder controller for POLARIS Foxglove panel",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "recorder_controller = recorder_controller_node.recorder_controller:main",
        ],
    },
)
