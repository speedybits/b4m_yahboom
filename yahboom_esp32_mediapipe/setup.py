from setuptools import find_packages, setup

package_name = 'yahboom_esp32_mediapipe'

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
            '01_HandDetector = yahboom_esp32_mediapipe.01_HandDetector:main',
            '02_PoseDetector = yahboom_esp32_mediapipe.02_PoseDetector:main',
            '03_Holistic = yahboom_esp32_mediapipe.03_Holistic:main',
            '04_FaceMesh = yahboom_esp32_mediapipe.04_FaceMesh:main',
            '05_FaceEyeDetection = yahboom_esp32_mediapipe.05_FaceEyeDetection:main',
            '06_FaceLandmarks = yahboom_esp32_mediapipe.06_FaceLandmarks:main',
            '07_FaceDetection = yahboom_esp32_mediapipe.07_FaceDetection:main',
            '08_Objectron = yahboom_esp32_mediapipe.08_Objectron:main',
            '09_VirtualPaint = yahboom_esp32_mediapipe.09_VirtualPaint:main',
            '10_HandCtrl = yahboom_esp32_mediapipe.10_HandCtrl:main',
            '11_GestureRecognition = yahboom_esp32_mediapipe.11_GestureRecognition:main',
            'control_servo = yahboom_esp32_mediapipe.control_servo:main',
            
            
            
        ],
    },
)
