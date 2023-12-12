# Turtlebot3 Use-case 

In this project, the use-case explores AGVs handling goods within a warehouse, being subject to remote control and requiring real-time support. The pysical Turtlebot3 Burger is running under Ubuntu 22.04 and ROS2(Robot Operation System) Humble. There are two ways to setup the application. We go through local setup firstly and then docker setup.

# 1. Local Setup

We run the turtlebot application through the remote pc, therefore, we need to setup remote pc and turtlebot respectively.

## 1.1 PC Setup

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
  echo 'source ~/Turtlebot3_ws/install/setup.bash' >> ~/.bashrc
  source ~/.bashrc
```
### 1.1.5 Config ROS Environment for Remote PC
```
  echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.bashrc
  source ~/.bashrc
```
## 1.2 Turtlebot(Raspberry Pi) Setup

### 1.2.1 Install Ubuntu Server 22.04.5 LTS (64-bit) on Turtlebot
### 1.2.2 Connect to Turtlebot via SSH
```
  ssh turtlebot1@10.0.33.30
```
### 1.2.3 Install ROS2 Humble on Turtlebot
### 1.2.4 Install and Build ROS Packages
```
  sudo apt install python3-argcomplete python3-colcon-common-extensions libboost-system-dev build-essential
  sudo apt install ros-humble-hls-lfcd-lds-driver
  sudo apt install ros-humble-turtlebot3-msgs
  sudo apt install ros-humble-dynamixel-sdk
  sudo apt install libudev-dev
  git clone https://github.com/callmekarl/Turtlebot3_ws.git
  echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
  source ~/.bashrc
  colcon build --symlink-install --parallel-workers 1
  echo 'source ~/Turtlebot3_ws/install/setup.bash' >> ~/.bashrc
  source ~/.bashrc
```
### 1.2.5 USB Port Setting for OpenCR
```
  sudo cp `ros2 pkg prefix turtlebot3_bringup`/share/turtlebot3_bringup/script/99-turtlebot3-cdc.rules /etc/udev/rules.d/
  sudo udevadm control --reload-rules
  sudo udevadm trigger
```
### 1.2.6 Config ROS Environment for Remote PC
```
  echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.bashrc
  source ~/.bashrc
```
### 1.2.7 LDS Configuration
```
  echo 'export LDS_MODEL=LDS-02' >> ~/.bashrc
  source ~/.bashrc
```
# 2. Docker Setup



# 3. Running the Application
Nowadays, AGVs are supporting the heterogeneous and growing demand of material handling and logistics in flexible factory environments. In our Use-case design, Turtlebot is configured to perform three workloads tasks respectively.
<center>
  <img width="569" alt="Workload" src="https://github.com/callmekarl/Turtlebot3_ws/assets/105017251/371b3f99-5ef1-422e-9d3c-d6e514763715">
</center>

- Workload1: AGV departs from Docking Station, picking up at Station 1, and then returns to the Docking Dtation to drop off. 
- Workload2: AGV departs from Docking Station, picking up at Station 3, and then returns to the Docking Dtation to drop off.
- Workload3: AGV departs from Docking Station, picking up at Station 2, droping off at Station 3, and then returns to the Docking Station.

### 3.1 Launch the Bringup
```
  ssh turtlebot1@10.0.33.30
  export TURTLEBOT3_MODEL=burger
  ros2 launch turtlebot3_bringup robot.launch.py
```
### 3.2 Launch the Navigation from Remote PC
```
  source ~/Turtlebot3_ws/install/setup.bash
  export TURTLEBOT3_MODEL=burger
  ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=Turtlebot3_ws/maps/map.yaml
```
### 3.3 Run the Workload from Remote PC
```
  source ~/Turtlebot3_ws/install/setup.bash
  export TURTLEBOT3_MODEL=burger
  python3 ~/Turtlebot3_ws/src/workload/scripts/workload1.py
```
