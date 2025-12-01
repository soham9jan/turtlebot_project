# turtlebot_project
A complete TurtleBot3 simulation and SLAM setup guide using ROS 2 Humble and Gazebo Classic. Includes installation steps, workspace setup, teleoperation, and map-saving commands.


## 🚀 Project Overview  
This repository contains a step-by-step setup guide for running TurtleBot3 simulations using Gazebo Classic under ROS 2 Humble.  
It covers installation, workspace setup, simulation launch, teleop control, and SLAM (map building) instructions.

---

## 📦 Prerequisites  
- Ubuntu (22.04 or compatible)  
- ROS 2 Humble installed  
- Internet connection to download packages & repositories  

---

## 🛠️ Installation & Setup  

### 1. Install Gazebo Classic 11  
```bash
sudo apt update  
sudo apt install gazebo  
gazebo --version  
gazebo

2. Install TurtleBot3 simulation packages
sudo apt update  
sudo apt install ros-humble-turtlebot3 ros-humble-turtlebot3-simulations -y

3. Create and Initialize Workspace

mkdir -p ~/turtlebot3_ws/src  
cd ~/turtlebot3_ws/src

4. Clone Required Reposit
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3.git  
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git  
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git

5. Build Workspace
cd ~/turtlebot3_ws  
colcon build --symlink-install  
source install/setup.bash

6. Set TurtleBot3 Mod
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc  
source ~/.bashrc

//Running Simulation & Teleop

Launch Gazebo Simulation
source /opt/ros/humble/setup.bash  
ros2 launch turtlebot3_gazebo empty_world.launch.py

Control Robot via Keyboard
source /opt/ros/humble/setup.bash  
ros2 run turtlebot3_teleop teleop_keyboard

SLAM (Mapping) Instruction

1. Launch Simulation World
ource /opt/ros/humble/setup.bash  
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

2. Start SLAM Node
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True

3. eleop — Move the Robot to Build Map
ros2 run turtlebot3_teleop teleop_keyboard

4. Save the Map
ros2 run nav2_map_server map_saver_cli -f ~/map

