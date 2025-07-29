from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression

#trot, walk, high_walk
def generate_launch_description():

    gait_type = LaunchConfiguration('gait')
    mark_type = LaunchConfiguration('mark')
    return LaunchDescription([
        Node(
            package='yahboom_gait',
            executable='yahboom_gait',
            name='yahboom_gait',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'gait': gait_type},
                {'marking': mark_type}
            ]
        )
    ])
