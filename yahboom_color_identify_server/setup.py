from setuptools import setup

package_name = 'yahboom_color_identify_server'

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
    maintainer='yahboom',
    maintainer_email='yahboom@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "yahboom_color_identify_server=yahboom_color_identify_server.yahboom_color_identify_server:main",
            "yahboom_color_identify_client=yahboom_color_identify_server.yahboom_color_identify_client:main"
        ],
    },
)
