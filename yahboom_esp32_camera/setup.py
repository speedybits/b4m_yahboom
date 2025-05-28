from setuptools import setup
import os
from glob import glob

package_name = 'yahboom_esp32_camera'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'launch'),glob(os.path.join('launch','*launch.py'))),
        #(os.path.join('share','yahboomcar_description','urdf'),glob(os.path.join('urdf','*.*'))),
		#(os.path.join('share','yahboomcar_description','meshes'),glob(os.path.join('meshes','*.*'))),
        (os.path.join('share','yahboomcar_description','rviz'),glob(os.path.join('rviz','*.rviz*'))),
        (os.path.join('share', package_name, 'param'), glob(os.path.join('param', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nx-ros2',
    maintainer_email='nx-ros2@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [ 
        'sub_img = yahboom_esp32_camera.sub_img:main',  
        ],
    },
)
