from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'simulation'

def package_files(directory):
    paths = []
    for (path, _, filenames) in os.walk(directory):
        for filename in filenames:
            paths.append(os.path.join(path, filename))
    return paths

launch_files = glob('launch/*.py')
urdf_files = glob('urdf/*.urdf') + glob('urdf/*.xacro')
asset_files = package_files('arm_robot')

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch', launch_files),
    ('share/' + package_name + '/urdf', urdf_files),
]
for f in asset_files:
    data_files.append((os.path.join('share', package_name, os.path.dirname(f)), [f]))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hiwe',
    maintainer_email='hieuai0305@gmail.com',
    description='Simulation package (ament_python)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'joint_state_publisher = simulation.joint_state_publisher:main',
            'robot_description_publisher = simulation.robot_description_publisher:main',
        ],
    },
)
