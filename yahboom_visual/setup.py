from setuptools import setup

package_name = 'yahboom_visual'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
        'simple_AR = yahboom_visual.simple_AR:main',
        'laser_to_image = yahboom_visual.laser_to_image:main',
        'pub_image = yahboom_visual.pub_image:main',
        'astra_rgb_image = yahboom_visual.astra_rgb_image:main',
        'astra_depth_image = yahboom_visual.astra_depth_image:main',
        'astra_image_flip = yahboom_visual.astra_image_flip:main',
        'astra_color_point = yahboom_visual.astra_color_point:main'
        ],
    },
)
