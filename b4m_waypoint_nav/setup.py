from setuptools import setup
import os
from glob import glob

package_name = 'b4m_waypoint_nav'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Include RViz configuration files
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yahboom',
    maintainer_email='user@todo.todo',
    description='Waypoint Navigation with Orientation for Yahboom Robot',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'b4m_waypoint_nav = b4m_waypoint_nav.b4m_waypoint_nav:main',
            'waypoint_manager = b4m_waypoint_nav.waypoint_manager:main',
        ],
    },
)
