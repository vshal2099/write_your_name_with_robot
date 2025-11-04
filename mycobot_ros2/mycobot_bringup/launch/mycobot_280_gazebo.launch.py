from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # Launch arguments
    load_controllers = LaunchConfiguration('load_controllers', default='true')
    world_file = LaunchConfiguration('world_file', default='pick_and_place_demo.world')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub', default='true')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x = LaunchConfiguration('x', default='0.0')
    y = LaunchConfiguration('y', default='0.0')
    z = LaunchConfiguration('z', default='0.05')
    roll = LaunchConfiguration('roll', default='0.0')
    pitch = LaunchConfiguration('pitch', default='0.0')
    yaw = LaunchConfiguration('yaw', default='0.0')

    # Path to the actual Gazebo launch file
    mycobot_gazebo_launch = os.path.join(
        get_package_share_directory('mycobot_gazebo'),
        'launch',
        'mycobot_280_gazebo.launch.py'
    )

    # Include the Gazebo launch file
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mycobot_gazebo_launch),
        launch_arguments={
            'load_controllers': load_controllers,
            'world_file': world_file,
            'use_rviz': use_rviz,
            'use_robot_state_pub': use_robot_state_pub,
            'use_sim_time': use_sim_time,
            'x': x,
            'y': y,
            'z': z,
            'roll': roll,
            'pitch': pitch,
            'yaw': yaw
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument('load_controllers', default_value='true'),
        DeclareLaunchArgument('world_file', default_value='pick_and_place_demo.world'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_robot_state_pub', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.05'),
        DeclareLaunchArgument('roll', default_value='0.0'),
        DeclareLaunchArgument('pitch', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        gazebo_launch
    ])
