from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():

    xGoHeight_value = LaunchConfiguration('xGoHeight')
    attitude_p_value = LaunchConfiguration('attitude_p')
    move_x_value = LaunchConfiguration('move_x')
    move_y_value = LaunchConfiguration('move_y')
    print(xGoHeight_value)
    return LaunchDescription([
        Node(
            package='yahboom_set_height',
            executable='yahboom_set_height',
            name='yahboom_set_height',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'xGoHeight': xGoHeight_value},
                {'attitude_p': attitude_p_value},
                {'move_x': move_x_value},
                {'move_y': move_y_value}
            ]
        )
    ])
