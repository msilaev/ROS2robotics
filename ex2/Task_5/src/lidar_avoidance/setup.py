import os
from glob import glob
from setuptools import setup

package_name = 'lidar_avoidance'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[
        'lidar_avoidance.lidarNode',
        'lidar_avoidance.keyboardNode',
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Lidar-based wall avoidance node for a robot.',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'lidarNode = lidar_avoidance.lidarNode:main',
            'keyboardNode = lidar_avoidance.keyboardNode:main',
        ],
    },
)
