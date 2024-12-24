from setuptools import find_packages, setup

package_name = 'webcam_tracking'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='atte',
    maintainer_email='atte@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'webcam_publisher = webcam_tracking.webcam_publisher:main',
            'webcam_subscriber = webcam_tracking.webcam_subscriber:main',
        ],
    },
)
