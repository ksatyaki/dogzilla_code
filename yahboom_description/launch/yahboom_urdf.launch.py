import os
from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro

def generate_launch_description():

  # Constants for paths to different files and folders
  # Set the path to different files and folders.
  #指定路径
  default_xacro_model_path = os.path.join(get_package_share_directory('yahboom_description'), 'urdf/', 'yahboom_xgo_rviz.xacro') 
  #default_rviz_config_path = os.path.join(get_package_share_directory('champ_config'), 'config/rviz2/', 'xgo.rviz') 

        
  # Launch configuration variables specific to simulation
  #配置变量
  #gui = LaunchConfiguration('gui')
  #rviz_config_file = LaunchConfiguration('rviz_config_file')
  
  # Declare the launch arguments  
  #声明启动参数
	
  # declare_use_joint_state_publisher_cmd = DeclareLaunchArgument(
  #   name='gui',
  #   default_value='True',
  #   description='Flag to enable joint_state_publisher_gui')
            
  # declare_rviz_config_file_cmd = DeclareLaunchArgument(
  #   name='rviz_config_file',
  #   default_value=default_rviz_config_path,
  #   description='Full path to the RVIZ config file to use')
  

  declare_xacro_model_path_cmd = DeclareLaunchArgument(
    name='xacro_model', 
    default_value=default_xacro_model_path, 
    description='Absolute path to robot xacro file')

    
  declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
    name='use_robot_state_pub',
    default_value='True',
    description='Whether to start the robot state publisher')

  # declare_use_rviz_cmd = DeclareLaunchArgument(
  #   name='use_rviz',
  #   default_value='True',
  #   description='Whether to start RVIZ')
    
  robot_description_config = xacro.process_file(default_xacro_model_path)
  robot_desc = robot_description_config.toxml()
  # Subscribe to the joint states of the robot, and publish the 3D pose of each link.    
  #机器人状态发布
  start_robot_state_publisher_cmd = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[
                {"robot_description": robot_desc}],
            output="screen")

  
            
  # Publish the joint state values for the non-fixed joints in the URDF file.
  #机器人关节状态发布节点
  # start_joint_state_publisher_cmd = Node(
  #   condition=UnlessCondition(gui),
  #   package='joint_state_publisher',
  #   executable='joint_state_publisher',
  #   name='joint_state_publisher')

  yahboom_dog_joint_state_cmd = Node(
            package="yahboom_dog_joint_state",
            executable="yahboomcar_joint_state",
            name="yahboom_dog_joint_state_publisher",
            output="screen")
  

  # A GUI to manipulate the joint state values
  #机器人关节状态发布节点 带gui 
  # start_joint_state_publisher_gui_node = Node(
  #   condition=IfCondition(gui),
  #   package='joint_state_publisher_gui',
  #   executable='joint_state_publisher_gui',
  #   name='joint_state_publisher_gui')

  # Launch RViz2
  #启动RViz2
  # start_rviz_cmd = Node(
  #   package='rviz2',
  #   executable='rviz2',
  #   name='rviz2',
  #   output='screen',
  #   arguments=['-d', rviz_config_file])


  return LaunchDescription(
      [
          # declare_use_joint_state_publisher_cmd,
          #declare_rviz_config_file_cmd,
          declare_xacro_model_path_cmd,
          #declare_use_robot_state_pub_cmd,
          #declare_use_rviz_cmd,

          start_robot_state_publisher_cmd,
          #yahboom_dog_joint_state_cmd
          #start_joint_state_publisher_cmd,
          # start_joint_state_publisher_gui_node,
          #start_rviz_cmd,
      ]    
    )
