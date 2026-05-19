import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'recorder'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jiaxuan Lin',
    maintainer_email='jiaxuan0lin0@gmail.com',
    description='ROS 2 segmented recorder for UR robot state and RealSense images.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'record = recorder.record:main',
            'keyboard_control = recorder.keyboard_control:main',
            'fix_gripper_targets = recorder.fix_gripper_targets:main',
        ],
    },
)
