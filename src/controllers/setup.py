from glob import glob

from setuptools import find_packages, setup

package_name = 'controllers'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('controllers/launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='JiaxuanLin',
    maintainer_email='jiaxuan0lin0@gmail.com',
    description='controller for UR and AG controller',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'gripper = controllers.gripper:main',
            (
                'ur_cartesian_delta_control = '
                'controllers.ur_cartesian_delta_control:main'
            ),
            'ur_joint_control = controllers.ur_joint_control:main',
        ],
    },
)
