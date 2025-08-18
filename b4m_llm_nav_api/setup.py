import os
from glob import glob
from setuptools import setup

package_name = 'b4m_llm_nav_api'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name, f'{package_name}.utils'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='B4M Team',
    maintainer_email='team@b4m.dev',
    description='LLM Navigation API for B4M robot system',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'api_server = b4m_llm_nav_api.api_server:main',
        ],
    },
)
