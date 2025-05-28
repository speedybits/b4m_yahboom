from setuptools import find_packages, setup

package_name = 'yahboomcar_laser'

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
    maintainer='yahboom',
    maintainer_email='yahboom@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'laser_Avoidance = yahboomcar_laser.laser_Avoidance:main',
        'laser_Tracker = yahboomcar_laser.laser_Tracker:main',
        'laser_Warning = yahboomcar_laser.laser_Warning:main',
        ],
    },
)
