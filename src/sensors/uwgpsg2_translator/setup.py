from glob import glob
from setuptools import find_packages, setup

package_name = 'uwgpsg2_translator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='polaris-rc',
    maintainer_email='ridh.choudhury@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'to_navsatfix_translator = uwgpsg2_translator.tonavsatfix_translate:main',
            'selector = uwgpsg2_translator.selector:main',
        ],
    },
)
