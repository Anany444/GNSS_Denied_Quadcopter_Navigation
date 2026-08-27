from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'vo_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Todo',
    maintainer_email='Todo',
    description='Full RGB-D Visual Odometry pipeline — RealSense D455F + RTAB-Map odometry/SLAM + PX4 VehicleOdometry bridge for GNSS-denied navigation',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vo_bridge = vo_nav.vo_bridge_node:main',
            'foxglove_relay = vo_nav.foxglove_relay_node:main',
        ],
    },
)
