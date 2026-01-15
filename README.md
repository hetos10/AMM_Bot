# AMM_Bot

AMM_Bot (Autonomous Mobile Manipulator Bot) is a ROS 2 Humble based mobile robot with a robotic arm.
The project uses Gazebo for simulation and RViz for visualization.
It supports SLAM mapping, localization, autonomous navigation, and arm motion planning.

This project is developed for learning and experimentation with ROS 2, Nav2, and MoveIt 2.

## Requirements

Ubuntu 22.04  
ROS 2 Humble  
Gazebo Fortress  
Nav2  
SLAM Toolbox  
MoveIt 2  

## Installation

mkdir -p ~/rover_ws/src  
cd ~/rover_ws/src  
git clone https://github.com/hetos10/AMM_Bot.git  
cd ~/rover_ws  
rosdep install --from-paths src --ignore-src -r -y  
colcon build  
source install/setup.bash  

## Run Simulation

ros2 launch rover_bot sim_gazebo.launch.py  

## RViz

ros2 launch rover_bot view_rviz.launch.py  

## SLAM Mapping

ros2 launch rover_bot slam.launch.py  

## Navigation

ros2 launch rover_bot navigation.launch.py  

## Arm Planning

ros2 launch rover_bot moveit.launch.py  

## Folder Structure

rover_ws/src  
├── moveit_setup  
└── rover_bot  
    ├── config  
    ├── description  
    ├── launch  
    ├── maps  
    ├── meshes  
    ├── models  
    ├── rviz  
    └── worlds  

## Author

Het Chauhan  
GitHub: https://github.com/hetos10
