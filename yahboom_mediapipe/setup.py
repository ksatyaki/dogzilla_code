from setuptools import setup
import os
from glob import glob 

package_name = 'yahboom_mediapipe'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share','yahboom_mediapipe','rviz'),glob(os.path.join('rviz','*.rviz*'))),
        (os.path.join('share',package_name,'launch'),glob(os.path.join('launch','*launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='pi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        '01_HandDetector = yahboom_mediapipe.01_HandDetector:main',
        '02_PoseDetector = yahboom_mediapipe.02_PoseDetector:main',
        '03_Holistic = yahboom_mediapipe.03_Holistic:main',
        '04_FaceMesh = yahboom_mediapipe.04_FaceMesh:main',
        '05_FaceEyeDetection = yahboom_mediapipe.05_FaceEyeDetection:main',
        'test_msg = yahboom_mediapipe.test_msg:main'
        ],
    },
)
