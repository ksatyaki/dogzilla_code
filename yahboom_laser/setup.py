from setuptools import setup
import os
from glob import glob

package_name = 'yahboom_laser'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'launch'),glob(os.path.join('launch','*launch.py')))
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
        'laser_Avoidance_a1_X3 = yahboomcar_laser.laser_Avoidance_a1_X3:main',
        'laser_Avoidance_4ROS_R2 = yahboomcar_laser.laser_Avoidance_4ROS_R2:main',
        'laser_Avoidance_a1_R2 = yahboomcar_laser.laser_Avoidance_a1_R2:main',
        'laser_Avoidance_xgo_RS200 = yahboom_laser.laser_Avoidance_xgo_RS200:main',
        'laser_Tracker_a1_X3 = yahboom_laser.laser_Tracker_a1_X3:main',
        'laser_Tracker_4ROS_R2 = yahboom_laser.laser_Tracker_4ROS_R2:main',
        'laser_Tracker_a1_R2 = yahboom_laser.laser_Tracker_a1_R2:main',
        'laser_Warning_a1_X3 = yahboom_laser.laser_Warning_a1_X3:main',
        'laser_Warning_RS200 = yahboom_laser.laser_Warning_xgo_RS200:main',
        'laser_Tracke_RS200 = yahboom_laser.laser_Tracker_xgo_RS200:main',
        'laser_Patrol_RS200 = yahboom_laser.laser_Patrol_xgo_RS200:main',
        ],
    },
)
