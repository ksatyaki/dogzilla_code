from launch import LaunchDescription
from launch.actions
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource



def generate_launch_description():
    rviz_and_imu_node = Node(
        package='wit_ros2_imu',
        node_executable='wit_ros2_imu',
        node_name='imu',
        remappings=[('/wit/imu', '/imu/data')],
        parameters=[{'port': '/dev/imu_usb'},
                    {"baud": 9600}],
        output="screen"

    )

    laser = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("oradar_lidar"),
                "launch",
                "ms200_scan.launch.py",
            )),
        )

    

    rviz_display_node = Node(
        package='rviz2',
        node_executable="rviz2",
        output="screen"
    )

    return LaunchDescription(
        [
            rviz_and_imu_node,
            laser
            #rviz_display_node
        ]
    )