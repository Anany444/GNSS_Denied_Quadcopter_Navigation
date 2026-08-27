from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'lio_nav'

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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='robot@drone.dev',
    description='Full LiDAR-Inertial Odometry pipeline — Point-LIO odometry/mapping + PX4 VehicleOdometry bridge for GNSS-denied navigation',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lio_bridge = lio_nav.lio_bridge_node:main',
        ],
    },
)
