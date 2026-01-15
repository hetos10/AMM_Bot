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
    
    my_pkg_path = os.path.join(get_package_share_directory('rover_bot'))
    xacro_file = os.path.join(my_pkg_path,'description','rover.urdf.xacro')
    rviz_config_file = os.path.join(my_pkg_path,'rviz','sim_model.rviz')
    world_path = os.path.join(my_pkg_path, 'worlds', 'final_world.sdf')
    robot_description_config = Command(['xacro ', xacro_file])
    ros2_gz_bridge_config = os.path.join(my_pkg_path,'config','bridge.yaml')
    description_params_file = {'robot_description': robot_description_config}
    controller_params_file = os.path.join(get_package_share_directory('rover_bot'),'config','my_controllers.yaml')

    gazebo = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([os.path.join(
                        get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"]),
                    launch_arguments={
                        "gz_args": PythonExpression(["'", world_path, " -v 4 -r'"])
                    }.items()
                )

    gazebo_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=['--ros-args','-p',f'config_file:={ros2_gz_bridge_config}'],
        output="screen",
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic",
            "/robot_description",
            "-name",
            "rover_bot",
            "-allow_renaming",
            "true",
        ],
    )
    
    
    control_node = TimerAction(
        period=5.0,  # wait 2 seconds for robot_description to be published
        actions=[
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[controller_params_file],
                output="both",
                remappings=[
                    ("~/robot_description", "/robot_description"),
                    ("/diff_drive_controller/cmd_vel_unstamped", "/cmd_vel"),
                ]
            )
        ]
    )
    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[description_params_file],
        
    )

    joint_state_broadcaster_spawner= TimerAction(
        period=4.0,
        actions=[
            Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
            output='screen'
            )
        ]
    )
    joint_trajectory_controller_spawner= TimerAction(
        period=8.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_trajectory_controller', '--controller-manager', '/controller_manager'],
                output='screen'
            )
        ]
    )
    diff_drive_controller_spawner = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
                output='screen'
            )
        ]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    nodes = [
        gazebo,
        gazebo_bridge,
        node_robot_state_publisher, 
        gz_spawn_entity, 
        control_node,
        joint_state_broadcaster_spawner,
        joint_trajectory_controller_spawner,
        diff_drive_controller_spawner,
        rviz_node 
    ]

    return LaunchDescription(nodes)
