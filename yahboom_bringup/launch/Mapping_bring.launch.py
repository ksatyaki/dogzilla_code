from launch import LaunchDescription
#from launch.actions
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():


    
    
    carto = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("xgo_bringup"),
                "",
                "Catographer.launch.py",
            )),
        )
    
    laser_to_point_node = Node(
        package='laserscan_to_point_pulisher',
        node_executable='laserscan_to_point_pulisher',
        output="screen"
    )

    tf_basefootprint = NODE(
        package='tf_publisher',
        node_executable="tf_publisher",
        output="screen"
    )
    
    rviz_display_node = Node(
        package='rviz2',
        node_executable="rviz2",
        output="screen"
    )

    return LaunchDescription(
        [
            carto,
            laser_to_point_node,
            tf_basefootprint
        ]
    )
