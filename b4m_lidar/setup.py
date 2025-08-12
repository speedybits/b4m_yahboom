from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'b4m_lidar'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='B4M Robot System',
    maintainer_email='b4m@example.com',
    description='B4M LiDAR-based intelligent navigation using B4M API',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'b4m_lidar_navigator = b4m_lidar.b4m_lidar_navigator:main',
        ],
    },
)