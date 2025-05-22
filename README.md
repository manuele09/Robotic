# Project Name

**Description:** A brief description of your project and its purpose.

## Table of Contents
- [Installation](#installation)
  - [Installing ROS Melodic](#installing-ros-melodic)
  - [Setting up Velodyne](#setting-up-velodyne)
- [Usage](#usage)
  - [Launching Velodyne Node](#launching-velodyne-node)

---

## Installation

### Installing ROS Melodic

To install ROS Melodic on Ubuntu, follow these steps:

1. **Set up the ROS repository:**  
   Add the ROS package repository to your sources list:
   ```bash
   sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
   ```

2. **Install `curl` (if not already installed):**  
   ```bash
   sudo apt install curl
   ```

3. **Add the ROS key for package verification:**  
   ```bash
   curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
   ```

4. **Update the package index:**  
   ```bash
   sudo apt update
   ```

5. **Install ROS Melodic (Desktop Full version):**  
   ```bash
   sudo apt install ros-melodic-desktop-full
   ```

6. **Set up environment variables:**  
   ```bash
   echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```

7. **Install additional dependencies:**  
   ```bash
   sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
   ```

### Setting up Velodyne

To set up the Velodyne drivers, run the following commands:

```bash
mkdir -p velodyne/src
cd velodyne/src
git clone -b melodic-devel https://github.com/ros-drivers/velodyne.git
cd ..
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro melodic -y
catkin_make -j2
```

## Usage

### Launching Velodyne Node

To launch the Velodyne node:

```bash
source devel/setup.bash
roslaunch velodyne_pointcloud VLP16_points.launch
```
