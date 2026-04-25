from setuptools import find_packages, setup

package_name = "ice_estimates"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/launch_archimedes_measurement.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="root",
    maintainer_email="semkir@ethz.ch",
    description="TODO: Package description",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "my_node = ice_estimates.my_node:main",
            "sim_archi_node = ice_estimates.sim_archi_node:main",
            "ice_node = ice_estimates.ice_node:main",
            "hybrid_valnode = ice_estimates.hybrid_valnode:main",
            "sim_ultrasonic_node = ice_estimates.sim_ultrasonic_node:main",
            "ultrasonic_node = ice_estimates.ultrasonic_node:main",
            "pool_testing_icethickness = ice_estimates.pool_testing_icethickness:main",
            "fixed_pool_testing = ice_estimates.fixed_pool_testing:main",
            "archimedes_touch = ice_estimates.archimedes_touch:main",
        ],
    },
)
