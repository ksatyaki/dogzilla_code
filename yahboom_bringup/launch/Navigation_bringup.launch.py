from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os



def generate_launch_description():
    
    rf2o = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("rf2o_laser_odometry"),
                "",
                "rf2o_laser_odometry.launch.py",
            )),
        )
        
    Catographer_localization = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("xgo_bringup"),
                "",
                "Catographer_localization.launch.py",
            )),
        )
    
    localization_server = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("localization_server"),
                "",
                "localization.launch.py",
            )),
        )
        
    navigation_server = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("localization_server"),
                "",
                "navigation.launch.py",
            )),
        )


    path_planner_server = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("path_planner_server"),
                "",
                "pathplanner.launch.py",
            )),
        )

    rviz_display_node = Node(
        package='rviz2',
        node_executable="rviz2",
        output="screen"
    )

    return LaunchDescription(
        [
            #rf2o,
            Catographer_localization,
            localization_server,
            navigation_server
            #path_planner_server
            #rviz_display_node
        ]
    )
