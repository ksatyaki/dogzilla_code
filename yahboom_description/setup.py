from setuptools import setup

package_name = 'yahboom_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/urdf', ['urdf/yahboom_xgo_rviz.xacro']),
        ('share/' +package_name, ['launch/yahboom_urdf.launch.py']),
        ('share/' + package_name + '/meshes/XGO/', ['meshes/XGO/base_link.STL','meshes/XGO/camera_link.STL', 'meshes/XGO/laser_link.STL', 'meshes/XGO/lf_hip_link.STL', 'meshes/XGO/lf_lower_leg_link.STL', 'meshes/XGO/lf_upper_leg_link.STL', 'meshes/XGO/lh_hip_link.STL', 'meshes/XGO/lh_lower_leg_link.STL', 'meshes/XGO/lh_upper_leg_link.STL', 'meshes/XGO/rf_hip_link.STL', 'meshes/XGO/rf_lower_leg_link.STL', 'meshes/XGO/rf_upper_leg_link.STL', 'meshes/XGO/rh_hip_link.STL', 'meshes/XGO/rh_lower_leg_link.STL', 'meshes/XGO/rh_upper_leg_link.STL']),
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
        ],
    },
)
