# turtlebot_project
A complete TurtleBot3 simulation and SLAM setup guide using ROS 2 Humble and Gazebo Classic. Includes installation steps, workspace setup, teleoperation, and map-saving commands.


## 🚀 Project Overview  
This repository contains a step-by-step setup guide for running TurtleBot3 simulations using Gazebo Classic under ROS 2 Humble.  
It covers installation, workspace setup, simulation launch, teleop control, and SLAM (map building) instructions.

###############################################
#   INSTALLATION & SETUP
###############################################

# Update system
sudo apt update


###############################################
#   INSTALL GAZEBO CLASSIC 11
###############################################
sudo apt install gazebo
gazebo --version
gazebo


###############################################
#   INSTALL TURTLEBOT3 PACKAGES
###############################################
sudo apt update
sudo apt install ros-humble-turtlebot3 ros-humble-turtlebot3-simulations -y


###############################################
#   CREATE WORKSPACE
###############################################
mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws/src


###############################################
#   CLONE TURTLEBOT3 REPOSITORIES (HUMBLE BRANCH)
###############################################
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git


###############################################
#   BUILD WORKSPACE
###############################################
cd ~/turtlebot3_ws
colcon build --symlink-install
source install/setup.bash


###############################################
#   SET TURTLEBOT3 MODEL (BURGER)
###############################################
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc


###############################################
#   RUN TURTLEBOT3 SIMULATION (EMPTY WORLD)
###############################################
source /opt/ros/humble/setup.bash
ros2 launch turtlebot3_gazebo empty_world.launch.py


###############################################
#   TELEOP (MOVE ROBOT WITH KEYBOARD)
###############################################
source /opt/ros/humble/setup.bash
ros2 run turtlebot3_teleop teleop_keyboard


###############################################
#   SLAM (MAPPING)
###############################################

# 1. Launch World (choose one)
source /opt/ros/humble/setup.bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
# OR
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py


# 2. Start SLAM - Cartographer
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True


# 3. Launch Teleop for SLAM Movement
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 run turtlebot3_teleop teleop_keyboard


# 4. Save Map
source /opt/ros/humble/setup.bash
ros2 run nav2_map_server map_saver_cli -f ~/map

###############################################

# Clean build if issues:
rm -rf build install log
colcon build --symlink-install

# Full workspace reset:
rm -rf ~/turtlebot3_ws

# Remove installed packages:
sudo apt remove --purge ros-humble-turtlebot3* -y
sudo apt autoremove -y

###############################################
#   FULL CODE
###############################################

sudo apt update
sudo apt install gazebo
gazebo --version
gazebo

sudo apt update
sudo apt install ros-humble-turtlebot3 ros-humble-turtlebot3-simulations -y

mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws/src

git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git

cd ~/turtlebot3_ws
colcon build --symlink-install
source install/setup.bash

echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc

source /opt/ros/humble/setup.bash
ros2 launch turtlebot3_gazebo empty_world.launch.py

source /opt/ros/humble/setup.bash
ros2 run turtlebot3_teleop teleop_keyboard

source /opt/ros/humble/setup.bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True

source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 run turtlebot3_teleop teleop_keyboard

source /opt/ros/humble/setup.bash
ros2 run nav2_map_server map_saver_cli -f ~/map

rm -rf build install log
colcon build --symlink-install

rm -rf ~/turtlebot3_ws

sudo apt remove --purge ros-humble-turtlebot3* -y
sudo apt autoremove -y
