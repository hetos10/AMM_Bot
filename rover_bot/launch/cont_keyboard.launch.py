#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'teleop_twist_keyboard', 'teleop_twist_keyboard',
                '--ros-args', '-r', 'cmd_vel:=diff_drive_controller/cmd_vel_unstamped'
            ],
            output='screen',
            shell=False
        )
    ])