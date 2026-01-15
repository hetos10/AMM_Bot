# 🤖 AMM_Bot

AMM_Bot (Autonomous Mobile Manipulator Bot) is a ROS 2 Humble based mobile robot integrated with a robotic arm.
The robot is simulated in Gazebo and visualized using RViz.
It supports SLAM mapping, localization, autonomous navigation, and arm motion planning.

This project focuses on understanding mobile manipulation using ROS 2, Nav2, and MoveIt 2.

---

## ✨ Features

🚗 Differential drive mobile base  
🦾 Robotic arm integration  
🌍 Gazebo simulation  
👀 RViz visualization  
🗺️ SLAM mapping  
📍 Autonomous navigation (Nav2)  
🧠 Motion planning (MoveIt 2)  

---

## 🧰 Requirements

🖥️ Ubuntu 22.04  
🤖 ROS 2 Humble Hawksbill  
🌐 Gazebo Fortress  
🧭 Navigation2  
🗺️ SLAM Toolbox  
🦾 MoveIt 2  

---

## ⚙️ Installation

mkdir -p ~/rover_ws/src  
cd ~/rover_ws/src  
git clone https://github.com/hetos10/AMM_Bot.git  
cd ~/rover_ws  
rosdep install --from-paths src --ignore-src -r -y  
colcon build  
source install/setup.bash  

---
## 👁️ RViz Visualization

ros2 launch rover_bot view_rviz.launch.py  

---

## ▶️ Run Simulation

ros2 launch rover_bot sim_gazebo.launch.py  

---

## 🗺️ SLAM Mapping

ros2 launch rover_bot slam.launch.py  

---
## 🗺️ SLAM Localisation
####just change the mapping mode to localisation in yaml file

ros2 launch rover_bot slam.launch.py  

---
#OR
## AMCL Localisation

ros2 launch rover_bot localisation.launch.py map:=/home/hetos_10/rover_ws/src/rover_bot/maps/map_save0.yaml

---
## 🚀 Navigation

ros2 launch rover_bot navigation.launch.py  

---

## 🦾 Arm Motion Planning

ros2 launch rover_bot moveit.launch.py  

---

## 📁 Folder Structure

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

---

## 👤 Author

Het Chauhan  
🔗 GitHub: https://github.com/hetos10
