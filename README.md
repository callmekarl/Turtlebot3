# Turtlebot3 Use-case 

In this project, the use-case explores AGVs handling goods within a warehouse, being subject to remote control and requiring real-time support. The pysical Turtlebot3 Burger is running under Ubuntu 22.04 and ROS2(Robot Operation System) Humble. There are two ways to setup the application. We go through local setup firstly and then docker setup.

# **1. Local Setup**

We run the turtlebot application through the remote pc, therefore, we need to setup remote pc and turtlebot respectively.

## 1.1 PC setup

### 1.1.1 Install Ubuntu 22.04 LTS Destop for Remote PC

### 1.1.2 Install ROS2 Humble on Remote PC
[Debian package](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) installation is recommended.

### 1.1.3 Install Dependent ROS2 Package
1. Install Gazebo
```
  sudo apt install ros-humble-gazebo-*
```
2. Install Cartographer
```
  sudo apt install ros-humble-cartographer
  sudo apt install ros-humble-cartographer-ros
```
3. Install Navigation2
```
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
```
### 1.1.4 Install Turtlebot Packages from Source
```
  sudo apt remove ros-humble-turtlebot3-msgs
  sudo apt remove ros-humble-turtlebot3
  git clone https://github.com/callmekarl/Turtlebot3_ws.git
  colcon build --symlink-install
  echo 'source ~/turtlebot3_ws/install/setup.bash' >> ~/.bashrc
  source ~/.bashrc
```
### 1.1.5 Config ROS Environment for Remote PC
```
  echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.bashrc
  source ~/.bashrc
```

