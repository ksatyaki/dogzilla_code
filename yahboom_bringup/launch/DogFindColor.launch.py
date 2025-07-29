from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    voice_xgo_ctrl_run_node = Node(
        package='voice_xgo_ctrl_run',
        executable='voice_xgo_ctrl_mutl_goal_identify',
        output='screen'
    )

    xgo_image_publisher_c_node = Node(
        package='yahboom_publish',
        executable='pub_color',
        output='screen'
    )

    xgo_color_status_redis_node = Node(
        package='yahboom_color_staus_redis',
        executable='yahboom_color_status_redis',
        output='screen'
    )
    
    return LaunchDescription(
        [
            voice_xgo_ctrl_run_node,
            xgo_image_publisher_c_node,
            xgo_color_status_redis_node,
        ]
    )
