import os
from glob import glob

from setuptools import find_packages, setup


package_name = 'policy_client'

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
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
        ('share/' + package_name, ['README.md']),
    ],
    install_requires=['setuptools', 'numpy', 'requests', 'pyserial'],
    zip_safe=True,
    maintainer='Jiaxuan Lin',
    maintainer_email='jiaxuan0lin0@gmail.com',
    description='Minimal local policy service client for direct UR RTDE control.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'policy_control = policy_client.node:main',
        ],
    },
)
