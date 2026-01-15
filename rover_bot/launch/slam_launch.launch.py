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

def generate_launch_description():
        
        slam_toolbox = Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            remappings=[
            ('scan','/scan')
            ],
            parameters=[{
                'use_sim_time': True
            },
            PathJoinSubstitution([
                get_package_share_directory('rover_bot'),
                'config', 'mapper_params_online_async.yaml'
            ])]
        )
        
        return LaunchDescription([slam_toolbox])
