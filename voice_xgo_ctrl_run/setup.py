from setuptools import setup

package_name = 'voice_xgo_ctrl_run'

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
        "voice_xgo_ctrl_run = voice_xgo_ctrl_run.voice_xgo_ctrl_run:main",
        "voice_xgo_ctrl_color_identify = voice_xgo_ctrl_run.voice_xgo_ctrl_color_identify:main",
        "voice_xgo_ctrl_mult_goal = voice_xgo_ctrl_run.voice_xgo_ctrl_mutl_goal:main",
        "voice_xgo_ctrl_action = voice_xgo_ctrl_run.voice_xgo_ctrl_action:main",
        "voice_xgo_ctrl_mutl_goal_identify = voice_xgo_ctrl_run.voice_xgo_ctrl_mutl_goal_identify:main",
        "voice_xgo_cmd_re = voice_xgo_ctrl_run.voice_xgo_cmd_re:main",
        "voice_follow_line = voice_xgo_ctrl_run.voice_xgo_follow_line:main",
        "follow_line = voice_xgo_ctrl_run.follow_line:main"
        ],
    },
)
