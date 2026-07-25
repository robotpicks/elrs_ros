import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'elrs_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nir Mor',
    maintainer_email='nir@nadirwave.com',
    description='Generic ExpressLRS / CRSF receiver driver for ROS 2 (Joy + handset telemetry).',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'elrs_driver = elrs_driver.elrs_driver_node:main',
        ],
    },
)
