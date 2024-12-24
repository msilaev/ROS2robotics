import os
from glob import glob
from setuptools import setup

package_name = 'move2cord'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[
        'move2cord.TurtleNode'
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='movig robot to ccordinates',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'MovePoint = move2cord.MovePoint:main',
            'MovePose = move2cord.MovePose:main',
            'MovePath = move2cord.MovePath:main',
            'OdometryLogger = move2cord.OdometryLogger:main'
        ],
    },
)
