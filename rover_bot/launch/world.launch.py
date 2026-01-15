import os
from os import pathsep
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable,TimerAction
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

my_pkg_path = os.path.join(get_package_share_directory('rover_bot'))
world_path_1 = os.path.join(my_pkg_path, 'worlds', 'tent_world.sdf')
def generate_launch_description():

    gazebo = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([os.path.join(
                        get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"]),
                    launch_arguments={
                        "gz_args": PythonExpression(["'", world_path_1, " -v 4 -r'"])
                    }.items()
                )
    
    return LaunchDescription([gazebo])