from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'arm_robot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hiwe',
    maintainer_email='hiwe@todo.todo',
    description='Package for controlling arm robot with camera detection',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detection_node = arm_robot_control.detection:main',
            'camera_sub = arm_robot_control.camera_sub:main',
            'servo_sender_node = arm_robot_control.servo_sender_node:main',
            'coordinate_sub = arm_robot_control.coordinate_sub:main',
            'state_transforms = arm_robot_control.state_transforms:main'
        ],
    },
)
