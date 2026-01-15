from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    

    joy_params = os.path.join(get_package_share_directory('rover_bot'),'config','joystick.yaml')

    joy_node = Node(
            package='joy',
            executable='joy_node',
            parameters=[joy_params],
         )
    gazebo_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments= ["/joy@sensor_msgs/msg/Joy[gz.msgs.Joy",
                     "/joy/set_feedback@sensor_msgs/msg/JoyFeedback[gz.msgs.JoyFeedback"],
        output="screen",
    )
    teleop_node = Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_node',
            parameters=[joy_params],
            remappings=[('/cmd_vel','/diff_drive_controller/cmd_vel_unstamped')]
         )

    return LaunchDescription([
        
        joy_node,
        teleop_node,
      
    ])