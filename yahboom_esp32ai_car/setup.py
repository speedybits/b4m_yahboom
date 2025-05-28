from setuptools import find_packages, setup

package_name = 'yahboom_esp32ai_car'

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
            'qrTracker = yahboom_esp32ai_car.qrTracker:main',
            'control_shape = yahboom_esp32ai_car.control_shape:main',
            'FingerCtrl = yahboom_esp32ai_car.FingerCtrl:main',
            'RobotCtrl = yahboom_esp32ai_car.RobotCtrl:main',
            'HandCtrl = yahboom_esp32ai_car.HandCtrl:main',
            'colorHSV = yahboom_esp32ai_car.colorHSV:main',
            'colorTracker = yahboom_esp32ai_car.colorTracker:main',
            'mono_Tracker = yahboom_esp32ai_car.mono_Tracker:main',
            'face_fllow = yahboom_esp32ai_car.face_fllow:main',
            'follow_line = yahboom_esp32ai_car.follow_line:main',
            
            
            
            
        ],
    },
)
