from launch import LaunchDescription
#from launch.actions
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource



def generate_launch_description():
    yahboom_base_node = Node(
        package='yahboom_base',
        executable='base',
        name='yahboom_base',
        output="screen"

    )

   

    return LaunchDescription(
        [
            yahboom_base_node,
        ]
    )
