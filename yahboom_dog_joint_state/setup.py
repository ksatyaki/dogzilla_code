from setuptools import setup

package_name = 'yahboom_dog_joint_state'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' +package_name, ['launch/yahboom_dog_joint_state.launch.py']),
        
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
            'yahboomcar_joint_state = yahboom_dog_joint_state.yahboomcar_joint_state:main'
        ],
    },
)
